/**
 * K230 SDK VICAP API (Stub)
 *
 * Video Input Capture API for camera access.
 * Real headers should be extracted from K230 board or SDK.
 */

#ifndef K230_MPI_VICAP_H
#define K230_MPI_VICAP_H

#include "k230_mpi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* VICAP device/channel identifiers */
typedef k_u32 k_vicap_dev;
typedef k_u32 k_vicap_chn;

/* Sensor types (common sensors supported by K230) */
typedef enum {
    /* OV5647 variants */
    OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR = 0,
    OV5647_MIPI_CSI0_1280X720_30FPS_10BIT_LINEAR,
    OV5647_MIPI_CSI0_640X480_60FPS_10BIT_LINEAR,

    /* IMX335 variants */
    IMX335_MIPI_CSI0_2LANE_1920X1080_30FPS_12BIT_LINEAR = 100,
    IMX335_MIPI_CSI0_4LANE_2592X1944_30FPS_12BIT_LINEAR,

    /* GC2093 variants */
    GC2093_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR = 200,
    GC2093_MIPI_CSI0_1280X720_60FPS_10BIT_LINEAR,
} k_vicap_sensor_type;

/* VICAP work mode */
typedef enum {
    VICAP_WORK_ONLINE_MODE = 0,  /* Online mode (direct ISP output) */
    VICAP_WORK_OFFLINE_MODE,     /* Offline mode (from DDR) */
} k_vicap_work_mode;

/* Dump type */
typedef enum {
    VICAP_DUMP_YUV = 0,
    VICAP_DUMP_RGB,
} k_vicap_dump_type;

/* Pipe control bits */
typedef union {
    struct {
        k_u32 af_enable : 1;
        k_u32 ae_enable : 1;
        k_u32 awb_enable : 1;
        k_u32 dnr3_enable : 1;
        k_u32 reserved : 28;
    } bits;
    k_u32 data;
} k_vicap_pipe_ctrl;

/* Device attributes */
typedef struct {
    k_rect acq_win;              /* Acquisition window */
    k_vicap_work_mode mode;      /* Work mode */
    k_vicap_pipe_ctrl pipe_ctrl; /* ISP pipeline control */
    k_vicap_sensor_type sensor_type;
    k_bool dw_enable;            /* Dewarp enable */
    k_u32 cpature_frame;         /* Capture frame count (0 = continuous) */
} k_vicap_dev_attr;

/* Channel attributes */
typedef struct {
    k_rect out_win;              /* Output window */
    k_rect crop_win;             /* Crop window */
    k_rect scale_win;            /* Scale window */
    k_bool crop_enable;
    k_bool scale_enable;
    k_bool chn_enable;
    k_pixel_format pix_format;   /* Output pixel format */
    k_u32 buffer_num;            /* Number of buffers */
    k_u32 buffer_size;           /* Size of each buffer */
} k_vicap_chn_attr;

/**
 * Set VICAP device attributes.
 */
k_s32 kd_mpi_vicap_set_dev_attr(k_vicap_dev dev, k_vicap_dev_attr attr);

/**
 * Set VICAP channel attributes.
 */
k_s32 kd_mpi_vicap_set_chn_attr(k_vicap_dev dev, k_vicap_chn chn, k_vicap_chn_attr attr);

/**
 * Initialize VICAP device.
 */
k_s32 kd_mpi_vicap_init(k_vicap_dev dev);

/**
 * Deinitialize VICAP device.
 */
k_s32 kd_mpi_vicap_deinit(k_vicap_dev dev);

/**
 * Start VICAP stream.
 */
k_s32 kd_mpi_vicap_start_stream(k_vicap_dev dev);

/**
 * Stop VICAP stream.
 */
k_s32 kd_mpi_vicap_stop_stream(k_vicap_dev dev);

/**
 * Dump (capture) a frame from VICAP.
 *
 * @param dev       Device number
 * @param chn       Channel number
 * @param type      Dump type (YUV or RGB)
 * @param frame     Output frame info
 * @param timeout   Timeout in milliseconds (-1 = blocking)
 * @return          0 on success
 */
k_s32 kd_mpi_vicap_dump_frame(k_vicap_dev dev, k_vicap_chn chn,
                               k_vicap_dump_type type,
                               k_video_frame_info* frame,
                               k_s32 timeout);

/**
 * Release a dumped frame back to the pool.
 */
k_s32 kd_mpi_vicap_dump_release(k_vicap_dev dev, k_vicap_chn chn,
                                 const k_video_frame_info* frame);

#ifdef __cplusplus
}
#endif

#endif /* K230_MPI_VICAP_H */
