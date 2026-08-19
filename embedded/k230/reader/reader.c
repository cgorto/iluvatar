/* reader.c — Little-core DATAFIFO reader and TCP forwarder.
 *
 * Runs on the K230 little core (Linux). Reads motion frames from the
 * DATAFIFO shared memory ring buffer (written by the big core's
 * RT-Smart camera pipeline) and forwards them to the Iluvatar server
 * over TCP.
 *
 * Architecture: three threads.
 *   - Main thread: reads DATAFIFO slots, copies data to per-destination
 *     buffers, signals the send threads.
 *   - Server thread: sends motion frames to the Iluvatar server.
 *   - Viewer thread: sends diff masks to the debug viewer.
 *
 * The server and viewer threads are fully independent. A slow server
 * send cannot block viewer delivery, and vice versa.
 *
 * Build: see build_reader.sh
 * Usage: set ILUVATAR_DATAFIFO_PHYS_ADDR and ILUVATAR_REGISTRATION_HEX,
 *        then run ./reader <server_ip:port> [viewer_ip:port]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include "k_datafifo.h"

/* Must match the big core's DATAFIFO_SLOT_SIZE and DATAFIFO_SLOT_COUNT. */
#define SLOT_SIZE  (256 * 1024)
#define SLOT_COUNT 4

/* ---------- TCP helpers ------------------------------------------------ */

static int tcp_connect(const char *host, int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    int flag = 1;
    struct timeval timeout = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        fprintf(stderr, "Invalid address: %s\n", host);
        close(fd); return -1;
    }
    int old_flags = fcntl(fd, F_GETFL, 0);
    if (old_flags < 0 || fcntl(fd, F_SETFL, old_flags | O_NONBLOCK) < 0) {
        perror("fcntl"); close(fd); return -1;
    }

    int rc = connect(fd, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) {
        perror("connect"); close(fd); return -1;
    }
    if (rc < 0) {
        struct pollfd candidate = { .fd = fd, .events = POLLOUT };
        rc = poll(&candidate, 1, 5000);
        if (rc <= 0) {
            if (rc == 0) errno = ETIMEDOUT;
            perror("connect"); close(fd); return -1;
        }
        int socket_error = 0;
        socklen_t error_size = sizeof(socket_error);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &error_size) < 0
            || socket_error != 0) {
            if (socket_error != 0) errno = socket_error;
            perror("connect"); close(fd); return -1;
        }
    }
    if (fcntl(fd, F_SETFL, old_flags) < 0) {
        perror("fcntl"); close(fd); return -1;
    }
    return fd;
}

static int tcp_send_all(int fd, const void *buf, size_t len)
{
    const char *p = (const char *)buf;
    while (len > 0) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) { perror("send"); return -1; }
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static int tcp_recv_exact(int fd, void *buf, size_t len)
{
    char *p = (char *)buf;
    while (len > 0) {
        ssize_t n = recv(fd, p, len, 0);
        if (n <= 0) {
            if (n == 0) fprintf(stderr, "Connection closed by server\n");
            else perror("recv");
            return -1;
        }
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static int tcp_recv_frame_discard(int fd)
{
    unsigned char header[4];
    if (tcp_recv_exact(fd, header, 4) < 0) return -1;
    uint32_t len = ((uint32_t)header[0] << 24) | ((uint32_t)header[1] << 16)
                 | ((uint32_t)header[2] << 8)  | ((uint32_t)header[3]);
    if (len > 1024 * 1024) { fprintf(stderr, "Frame too large: %u\n", len); return -1; }
    char discard[4096];
    while (len > 0) {
        size_t chunk = len < sizeof(discard) ? len : sizeof(discard);
        if (tcp_recv_exact(fd, discard, chunk) < 0) return -1;
        len -= (uint32_t)chunk;
    }
    return 0;
}

/* ---------- Startup metadata ------------------------------------------- */

static int parse_phys_addr(const char *text, k_u64 *out)
{
    char *end = NULL;
    errno = 0;
    unsigned long long value = strtoull(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0') return -1;
    *out = (k_u64)value;
    return 0;
}

static int hex_nibble(char value)
{
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

static int parse_registration_hex(const char *text, unsigned char *target,
                                  size_t target_size)
{
    size_t text_size = strlen(text);
    if (text_size == 0 || text_size % 2 != 0 || text_size / 2 > target_size)
        return -1;

    for (size_t index = 0; index < text_size / 2; index++) {
        int high = hex_nibble(text[index * 2]);
        int low = hex_nibble(text[index * 2 + 1]);
        if (high < 0 || low < 0) return -1;
        target[index] = (unsigned char)((high << 4) | low);
    }
    return (int)(text_size / 2);
}

static int parse_endpoint(const char *s, char *host, size_t host_size, int *port)
{
    const char *colon = strrchr(s, ':');
    if (!colon) return -1;
    size_t len = (size_t)(colon - s);
    if (len >= host_size) len = host_size - 1;
    memcpy(host, s, len);
    host[len] = '\0';
    *port = atoi(colon + 1);
    return 0;
}

/* ---------- Thread-safe single-slot buffer ----------------------------- */

/* Each send thread has its own buffer. The main thread copies data in,
 * signals the condition variable. The send thread wakes, sends, and
 * goes back to waiting. If the main thread produces faster than the
 * send thread consumes, the main thread overwrites the buffer (the
 * send thread always gets the latest frame). */

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    unsigned char   data[SLOT_SIZE];
    uint32_t        size;       /* Bytes of valid data. 0 = empty. */
    int             ready;      /* New data available. */
    int             quit;       /* Signal thread to exit. */
    int             fd;         /* TCP socket. */
    const char     *name;       /* For log messages. */
    uint64_t        sent;       /* Frames successfully sent. */
    uint64_t        dropped;    /* Frames overwritten before transmission. */

    /* Reconnect state (server thread only). */
    char            host[64];
    int             port;
    unsigned char   reg_buf[4096];
    int             reg_len;
} send_buf_t;

static void send_buf_init(send_buf_t *sb, int fd, const char *name)
{
    memset(sb, 0, sizeof(*sb));
    pthread_mutex_init(&sb->mutex, NULL);
    pthread_cond_init(&sb->cond, NULL);
    sb->fd = fd;
    sb->name = name;
}

/* Called by main thread: copy new data into the buffer. */
static void send_buf_push(send_buf_t *sb, const void *data, uint32_t size)
{
    pthread_mutex_lock(&sb->mutex);
    if (sb->ready) sb->dropped++;
    memcpy(sb->data, data, size);
    sb->size = size;
    sb->ready = 1;
    pthread_cond_signal(&sb->cond);
    pthread_mutex_unlock(&sb->mutex);
}

static void send_buf_record(send_buf_t *sb, int sent, int dropped)
{
    pthread_mutex_lock(&sb->mutex);
    sb->sent += (uint64_t)sent;
    sb->dropped += (uint64_t)dropped;
    pthread_mutex_unlock(&sb->mutex);
}

static void send_buf_stats(send_buf_t *sb, uint64_t *sent, uint64_t *dropped)
{
    pthread_mutex_lock(&sb->mutex);
    *sent = sb->sent;
    *dropped = sb->dropped;
    pthread_mutex_unlock(&sb->mutex);
}

static int send_buf_should_quit(send_buf_t *sb)
{
    pthread_mutex_lock(&sb->mutex);
    int quit = sb->quit;
    pthread_mutex_unlock(&sb->mutex);
    return quit;
}

/* Reconnect to the server: close old socket, connect, re-register.
 * Returns new fd on success, -1 on failure. Uses exponential backoff. */
static int server_reconnect(send_buf_t *sb)
{
    if (sb->fd >= 0) { close(sb->fd); sb->fd = -1; }

    int delay_ms = 500;
    int max_delay_ms = 30000;

    for (int attempt = 0; attempt < 20; attempt++) {
        if (send_buf_should_quit(sb)) return -1;

        fprintf(stderr, "%s: reconnecting to %s:%d (attempt %d, backoff %dms)...\n",
                sb->name, sb->host, sb->port, attempt + 1, delay_ms);
        usleep((unsigned)(delay_ms * 1000));

        int fd = tcp_connect(sb->host, sb->port);
        if (fd < 0) {
            delay_ms = delay_ms * 2;
            if (delay_ms > max_delay_ms) delay_ms = max_delay_ms;
            continue;
        }

        /* Re-send registration. */
        if (tcp_send_all(fd, sb->reg_buf, (size_t)sb->reg_len) < 0) {
            close(fd); continue;
        }

        /* Receive and discard RegisteredWithPrefs + GridConfig. */
        if (tcp_recv_frame_discard(fd) < 0 ||
            tcp_recv_frame_discard(fd) < 0) {
            close(fd); continue;
        }

        fprintf(stderr, "%s: reconnected and re-registered.\n", sb->name);
        sb->fd = fd;
        return fd;
    }
    fprintf(stderr, "%s: reconnect failed after 20 attempts.\n", sb->name);
    return -1;
}

/* Send thread entry point. */
static void *send_thread(void *arg)
{
    send_buf_t *sb = (send_buf_t *)arg;
    unsigned char local[SLOT_SIZE];
    uint32_t local_size;

    for (;;) {
        pthread_mutex_lock(&sb->mutex);
        while (!sb->ready && !sb->quit)
            pthread_cond_wait(&sb->cond, &sb->mutex);

        if (sb->quit && !sb->ready) {
            pthread_mutex_unlock(&sb->mutex);
            break;
        }

        memcpy(local, sb->data, sb->size);
        local_size = sb->size;
        sb->ready = 0;
        pthread_mutex_unlock(&sb->mutex);

        int rc = tcp_send_all(sb->fd, local, local_size);
        if (rc < 0) {
            /* Send failed. If this is the server thread (has reconnect
             * state), try to reconnect. Viewer thread just exits. */
            if (sb->host[0] != '\0') {
                if (server_reconnect(sb) < 0) break;
                /* Retry the current frame after reconnect. */
                if (tcp_send_all(sb->fd, local, local_size) < 0) {
                    fprintf(stderr, "%s: send failed after reconnect.\n",
                            sb->name);
                    break;
                }
            } else {
                fprintf(stderr, "%s: send failed, exiting thread.\n",
                        sb->name);
                break;
            }
        }
        send_buf_record(sb, 1, 0);
    }
    return NULL;
}

static void send_buf_stop(send_buf_t *sb)
{
    pthread_mutex_lock(&sb->mutex);
    sb->quit = 1;
    pthread_cond_signal(&sb->cond);
    pthread_mutex_unlock(&sb->mutex);
}

/* ---------- Main ------------------------------------------------------- */

int main(int argc, char *argv[])
{
    if (argc < 2 || argc > 3) {
        fprintf(stderr, "Usage: %s <server_ip:port> [viewer_ip:port]\n", argv[0]);
        return 1;
    }

    char host[64]; int port = 0;
    if (parse_endpoint(argv[1], host, sizeof(host), &port) < 0) {
        fprintf(stderr, "Expected host:port, got: %s\n", argv[1]); return 1;
    }

    char viewer_host[64] = {0}; int viewer_port = 0;
    if (argc == 3) {
        if (parse_endpoint(argv[2], viewer_host, sizeof(viewer_host),
                           &viewer_port) < 0) {
            fprintf(stderr, "Bad viewer endpoint: %s\n", argv[2]); return 1;
        }
    }

    /* Force line-buffered stdout so logs appear in nohup. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("=== DATAFIFO Reader (little core) ===\n");

    /* The RT-Smart process prints both values on its serial console. This
     * avoids relying on ShareFS, which hangs on some dual-system images. */
    const char *fifo_text = getenv("ILUVATAR_DATAFIFO_PHYS_ADDR");
    const char *registration_text = getenv("ILUVATAR_REGISTRATION_HEX");
    if (fifo_text == NULL || registration_text == NULL) {
        fprintf(stderr,
                "Set ILUVATAR_DATAFIFO_PHYS_ADDR and "
                "ILUVATAR_REGISTRATION_HEX from the RT-Smart console.\n");
        return 1;
    }

    k_u64 fifo_phys = 0;
    if (parse_phys_addr(fifo_text, &fifo_phys) < 0) {
        fprintf(stderr, "Invalid DATAFIFO physical address: %s\n", fifo_text);
        return 1;
    }
    printf("DATAFIFO phys_addr: 0x%lx\n", fifo_phys);

    unsigned char reg_buf[4096];
    int reg_len = parse_registration_hex(
        registration_text, reg_buf, sizeof(reg_buf));
    if (reg_len < 0) {
        fprintf(stderr, "Invalid registration hex.\n");
        return 1;
    }
    printf("Registration message: %d bytes\n", reg_len);

    /* Step 3: Connect to server and perform registration handshake. */
    printf("Connecting to %s:%d...\n", host, port);
    int tcp_fd = tcp_connect(host, port);
    if (tcp_fd < 0) return 1;

    if (tcp_send_all(tcp_fd, reg_buf, (size_t)reg_len) < 0) {
        fprintf(stderr, "Failed to send registration\n");
        close(tcp_fd); return 1;
    }
    if (tcp_recv_frame_discard(tcp_fd) < 0) {
        fprintf(stderr, "Failed to receive RegisteredWithPrefs\n");
        close(tcp_fd); return 1;
    }
    if (tcp_recv_frame_discard(tcp_fd) < 0) {
        fprintf(stderr, "Failed to receive GridConfig\n");
        close(tcp_fd); return 1;
    }
    printf("Registration complete.\n");

    /* Connect to viewer (optional). */
    int viewer_fd = -1;
    if (viewer_host[0] != '\0') {
        printf("Connecting to viewer at %s:%d...\n", viewer_host, viewer_port);
        viewer_fd = tcp_connect(viewer_host, viewer_port);
        if (viewer_fd < 0) {
            printf("WARNING: viewer connection failed, continuing without.\n");
        } else {
            printf("Viewer connected.\n");
        }
    }

    /* Step 4: Open DATAFIFO as reader. */
    k_datafifo_handle fifo_handle = K_DATAFIFO_INVALID_HANDLE;
    k_datafifo_params_s fifo_params = {
        .u32EntriesNum = SLOT_COUNT,
        .u32CacheLineSize = SLOT_SIZE,
        .bDataReleaseByWriter = K_TRUE,
        .enOpenMode = DATAFIFO_READER,
    };

    k_s32 ret = kd_datafifo_open_by_addr(
        &fifo_handle, &fifo_params, fifo_phys);
    if (ret != 0) {
        fprintf(stderr, "datafifo_open_by_addr failed: 0x%x\n", ret);
        close(tcp_fd); return 1;
    }
    printf("DATAFIFO opened.\n");

    /* Step 5: Start send threads. */
    send_buf_t server_buf, viewer_buf;
    pthread_t server_thread, viewer_thread;

    send_buf_init(&server_buf, tcp_fd, "server");
    /* Populate reconnect state so the thread can reconnect on failure. */
    memcpy(server_buf.host, host, sizeof(server_buf.host));
    server_buf.port = port;
    memcpy(server_buf.reg_buf, reg_buf, (size_t)reg_len);
    server_buf.reg_len = reg_len;
    int thread_rc = pthread_create(&server_thread, NULL, send_thread, &server_buf);
    if (thread_rc != 0) {
        fprintf(stderr, "server thread: %s\n", strerror(thread_rc));
        kd_datafifo_close(fifo_handle);
        close(tcp_fd);
        return 1;
    }

    int has_viewer = (viewer_fd >= 0);
    if (has_viewer) {
        send_buf_init(&viewer_buf, viewer_fd, "viewer");
        thread_rc = pthread_create(&viewer_thread, NULL, send_thread, &viewer_buf);
        if (thread_rc != 0) {
            fprintf(stderr, "viewer thread: %s; continuing without viewer\n",
                    strerror(thread_rc));
            close(viewer_fd);
            has_viewer = 0;
        }
    }

    printf("Forwarding motion frames (%s viewer)...\n",
           has_viewer ? "with" : "without");

    /* Step 6: Main read loop — DATAFIFO → per-thread buffers. */
    uint64_t frames_read = 0;

    for (;;) {
        k_u32 avail_read = 0;
        ret = kd_datafifo_cmd(fifo_handle,
            DATAFIFO_CMD_GET_AVAIL_READ_LEN, &avail_read);
        if (ret != 0) {
            fprintf(stderr, "get_avail_read_len error: 0x%x\n", ret);
            break;
        }

        if (avail_read == 0) {
            usleep(1000);
            continue;
        }

        void *slot_data = NULL;
        ret = kd_datafifo_read(fifo_handle, &slot_data);
        if (ret != 0) {
            fprintf(stderr, "datafifo_read error: 0x%x\n", ret);
            continue;
        }

        unsigned char *slot = (unsigned char *)slot_data;

        /* Extract motion payload (for server). */
        uint32_t motion_size = (uint32_t)slot[0]
                             | ((uint32_t)slot[1] << 8)
                             | ((uint32_t)slot[2] << 16)
                             | ((uint32_t)slot[3] << 24);

        if (motion_size > 0 && motion_size <= SLOT_SIZE - 4) {
            send_buf_push(&server_buf, slot + 4, motion_size);
        }

        /* Extract viewer payload (for viewer). */
        if (has_viewer) {
            uint32_t viewer_offset = 4 + motion_size;
            if (viewer_offset + 4 <= SLOT_SIZE) {
                uint32_t viewer_size =
                    (uint32_t)slot[viewer_offset]
                    | ((uint32_t)slot[viewer_offset + 1] << 8)
                    | ((uint32_t)slot[viewer_offset + 2] << 16)
                    | ((uint32_t)slot[viewer_offset + 3] << 24);
                if (viewer_size > 0 &&
                    viewer_offset + 4 + viewer_size <= SLOT_SIZE) {
                    send_buf_push(&viewer_buf,
                        slot + viewer_offset + 4, viewer_size);
                }
            }
        }

        kd_datafifo_cmd(fifo_handle, DATAFIFO_CMD_READ_DONE, slot_data);
        frames_read++;

        if (frames_read <= 10 || frames_read % 100 == 0) {
            uint64_t server_sent = 0, server_dropped = 0;
            uint64_t viewer_sent = 0, viewer_dropped = 0;
            send_buf_stats(&server_buf, &server_sent, &server_dropped);
            if (has_viewer)
                send_buf_stats(&viewer_buf, &viewer_sent, &viewer_dropped);

            printf("read=%lu srv_sent=%lu srv_drop=%lu",
                   frames_read, server_sent, server_dropped);
            if (has_viewer)
                printf(" view_sent=%lu view_drop=%lu",
                       viewer_sent, viewer_dropped);
            printf("\n");
        }
    }

    /* Cleanup. */
    send_buf_stop(&server_buf);
    pthread_join(server_thread, NULL);
    if (has_viewer) {
        send_buf_stop(&viewer_buf);
        pthread_join(viewer_thread, NULL);
    }

    uint64_t server_sent = 0, server_dropped = 0;
    uint64_t viewer_sent = 0, viewer_dropped = 0;
    send_buf_stats(&server_buf, &server_sent, &server_dropped);
    if (has_viewer)
        send_buf_stats(&viewer_buf, &viewer_sent, &viewer_dropped);

    kd_datafifo_close(fifo_handle);
    if (server_buf.fd >= 0) close(server_buf.fd);
    if (has_viewer && viewer_buf.fd >= 0) close(viewer_buf.fd);
    printf("=== Reader exit: read=%lu srv=%lu/%lu view=%lu/%lu ===\n",
           frames_read, server_sent, server_dropped,
           viewer_sent, viewer_dropped);
    return 0;
}
