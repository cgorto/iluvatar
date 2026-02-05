/**
 * K230 VICAP Camera Capture Implementation
 *
 * Wraps the K230 SDK's VICAP API for zero-copy camera capture.
 * Based on K230 Linux SDK v0.6.9 API patterns.
 */

#include "k230_capture.h"

#include <cstdlib>
#include <cstring>

// K230 SDK headers
#include "k230_sdk/k230_mpi_sys.h"
#include "k230_sdk/k230_mpi_vb.h"
#include "k230_sdk/k230_mpi_vicap.h"

/** Internal capture context */
struct K230CaptureContext {
    K230CaptureConfig config;
    k_vicap_dev dev_num;
    k_vicap_chn chn_num;
    k_bool is_streaming;
    k_video_frame_info frame_info;
    void* mapped_addr;
};

/** Map sensor type enum to K230 SDK sensor type */
static k_vicap_sensor_type get_sdk_sensor_type(K230SensorType type) {
    switch (type) {
        case K230_SENSOR_OV5647:
            return OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR;
        case K230_SENSOR_IMX335:
            return IMX335_MIPI_CSI0_2LANE_1920X1080_30FPS_12BIT_LINEAR;
        case K230_SENSOR_GC2093:
            return GC2093_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR;
        default:
            return OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR;
    }
}

K230CaptureContext* k230_capture_init(const K230CaptureConfig* config, K230Error* err) {
    if (!config || !err) {
        if (err) *err = K230_ERR_INVALID_ARG;
        return nullptr;
    }

    // Allocate context
    K230CaptureContext* ctx = static_cast<K230CaptureContext*>(
        calloc(1, sizeof(K230CaptureContext)));
    if (!ctx) {
        *err = K230_ERR_ALLOC;
        return nullptr;
    }

    ctx->config = *config;
    ctx->dev_num = static_cast<k_vicap_dev>(config->dev_num);
    ctx->chn_num = static_cast<k_vicap_chn>(config->chn_num);
    ctx->is_streaming = K_FALSE;
    ctx->mapped_addr = nullptr;

    k_s32 ret;

    // ========================================
    // 1. Configure Video Buffer Pool
    // ========================================
    k_vb_config vb_config;
    memset(&vb_config, 0, sizeof(vb_config));

    // NV12 format: Y plane (width*height) + UV plane (width*height/2)
    k_u32 frame_size = config->width * config->height * 3 / 2;

    vb_config.max_pool_cnt = 2;
    vb_config.comm_pool[0].blk_cnt = 4;  // 4 frame buffers
    vb_config.comm_pool[0].blk_size = frame_size;
    vb_config.comm_pool[0].mode = VB_REMAP_MODE_CACHED;

    ret = kd_mpi_vb_set_config(&vb_config);
    if (ret != 0) {
        *err = K230_ERR_VB_INIT;
        free(ctx);
        return nullptr;
    }

    ret = kd_mpi_vb_init();
    if (ret != 0) {
        *err = K230_ERR_VB_INIT;
        free(ctx);
        return nullptr;
    }

    // ========================================
    // 2. Configure VICAP Device
    // ========================================
    k_vicap_dev_attr dev_attr;
    memset(&dev_attr, 0, sizeof(dev_attr));

    dev_attr.acq_win.width = config->width;
    dev_attr.acq_win.height = config->height;
    dev_attr.mode = VICAP_WORK_ONLINE_MODE;
    dev_attr.pipe_ctrl.bits.af_enable = 0;
    dev_attr.pipe_ctrl.bits.ae_enable = 1;
    dev_attr.pipe_ctrl.bits.awb_enable = 1;
    dev_attr.sensor_type = get_sdk_sensor_type(config->sensor_type);
    dev_attr.dw_enable = K_FALSE;

    ret = kd_mpi_vicap_set_dev_attr(ctx->dev_num, dev_attr);
    if (ret != 0) {
        kd_mpi_vb_exit();
        *err = K230_ERR_VICAP_DEV;
        free(ctx);
        return nullptr;
    }

    // ========================================
    // 3. Configure VICAP Channel
    // ========================================
    k_vicap_chn_attr chn_attr;
    memset(&chn_attr, 0, sizeof(chn_attr));

    chn_attr.out_win.width = config->width;
    chn_attr.out_win.height = config->height;
    chn_attr.crop_win.width = config->width;
    chn_attr.crop_win.height = config->height;
    chn_attr.scale_win.width = config->width;
    chn_attr.scale_win.height = config->height;
    chn_attr.crop_enable = K_FALSE;
    chn_attr.scale_enable = K_FALSE;
    chn_attr.chn_enable = K_TRUE;
    chn_attr.pix_format = PIXEL_FORMAT_YVU_SEMIPLANAR_420;  // NV12
    chn_attr.buffer_num = 4;
    chn_attr.buffer_size = frame_size;

    ret = kd_mpi_vicap_set_chn_attr(ctx->dev_num, ctx->chn_num, chn_attr);
    if (ret != 0) {
        kd_mpi_vb_exit();
        *err = K230_ERR_VICAP_CHN;
        free(ctx);
        return nullptr;
    }

    // ========================================
    // 4. Initialize and Start VICAP
    // ========================================
    ret = kd_mpi_vicap_init(ctx->dev_num);
    if (ret != 0) {
        kd_mpi_vb_exit();
        *err = K230_ERR_VICAP_INIT;
        free(ctx);
        return nullptr;
    }

    ret = kd_mpi_vicap_start_stream(ctx->dev_num);
    if (ret != 0) {
        kd_mpi_vicap_deinit(ctx->dev_num);
        kd_mpi_vb_exit();
        *err = K230_ERR_VICAP_START;
        free(ctx);
        return nullptr;
    }

    ctx->is_streaming = K_TRUE;
    *err = K230_OK;
    return ctx;
}

K230Error k230_capture_grayscale(K230CaptureContext* ctx, uint8_t* buffer, size_t len) {
    if (!ctx || !buffer) {
        return K230_ERR_INVALID_ARG;
    }

    size_t expected_len = ctx->config.width * ctx->config.height;
    if (len < expected_len) {
        return K230_ERR_INVALID_ARG;
    }

    k_s32 ret;

    // ========================================
    // 1. Dump Frame from VICAP
    // ========================================
    memset(&ctx->frame_info, 0, sizeof(ctx->frame_info));

    // Timeout in milliseconds (-1 = blocking)
    ret = kd_mpi_vicap_dump_frame(ctx->dev_num, ctx->chn_num,
                                   VICAP_DUMP_YUV, &ctx->frame_info, 1000);
    if (ret != 0) {
        return K230_ERR_FRAME_DUMP;
    }

    // ========================================
    // 2. Map Frame Memory
    // ========================================
    k_u64 phys_addr = ctx->frame_info.v_frame.phys_addr[0];
    k_u32 y_size = ctx->config.width * ctx->config.height;

    ctx->mapped_addr = kd_mpi_sys_mmap_cached(phys_addr, y_size);
    if (!ctx->mapped_addr) {
        kd_mpi_vicap_dump_release(ctx->dev_num, ctx->chn_num, &ctx->frame_info);
        return K230_ERR_MMAP;
    }

    // ========================================
    // 3. Copy Y Plane (Grayscale)
    // ========================================
    // For NV12, Y plane is the first width*height bytes
    memcpy(buffer, ctx->mapped_addr, y_size);

    // ========================================
    // 4. Cleanup
    // ========================================
    kd_mpi_sys_munmap(ctx->mapped_addr, y_size);
    ctx->mapped_addr = nullptr;

    kd_mpi_vicap_dump_release(ctx->dev_num, ctx->chn_num, &ctx->frame_info);

    return K230_OK;
}

void k230_capture_deinit(K230CaptureContext* ctx) {
    if (!ctx) {
        return;
    }

    // Unmap if still mapped
    if (ctx->mapped_addr) {
        k_u32 y_size = ctx->config.width * ctx->config.height;
        kd_mpi_sys_munmap(ctx->mapped_addr, y_size);
        ctx->mapped_addr = nullptr;
    }

    // Stop stream
    if (ctx->is_streaming) {
        kd_mpi_vicap_stop_stream(ctx->dev_num);
        ctx->is_streaming = K_FALSE;
    }

    // Deinitialize VICAP
    kd_mpi_vicap_deinit(ctx->dev_num);

    // Exit VB pool
    kd_mpi_vb_exit();

    free(ctx);
}

const char* k230_error_string(K230Error err) {
    switch (err) {
        case K230_OK:
            return "Success";
        case K230_ERR_VB_INIT:
            return "Video buffer pool initialization failed";
        case K230_ERR_VICAP_DEV:
            return "VICAP device configuration failed";
        case K230_ERR_VICAP_CHN:
            return "VICAP channel configuration failed";
        case K230_ERR_VICAP_INIT:
            return "VICAP initialization failed";
        case K230_ERR_VICAP_START:
            return "VICAP stream start failed";
        case K230_ERR_FRAME_DUMP:
            return "Frame capture (dump) failed";
        case K230_ERR_MMAP:
            return "Memory mapping failed";
        case K230_ERR_INVALID_ARG:
            return "Invalid argument";
        case K230_ERR_ALLOC:
            return "Memory allocation failed";
        case K230_ERR_TIMEOUT:
            return "Capture timeout";
        default:
            return "Unknown error";
    }
}
