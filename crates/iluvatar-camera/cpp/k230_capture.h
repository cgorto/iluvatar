/**
 * K230 VICAP Camera Capture Shim
 *
 * Minimal C interface wrapping the K230 SDK's VICAP API for hardware-accelerated
 * camera capture. Designed for FFI with Rust.
 *
 * This bypasses V4L2's userspace copies for better performance on the K230.
 */

#ifndef K230_CAPTURE_H
#define K230_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque context handle */
typedef struct K230CaptureContext K230CaptureContext;

/** Error codes returned by capture functions */
typedef enum K230Error {
    K230_OK = 0,
    K230_ERR_VB_INIT = -1,        /** Video buffer pool initialization failed */
    K230_ERR_VICAP_DEV = -2,      /** VICAP device configuration failed */
    K230_ERR_VICAP_CHN = -3,      /** VICAP channel configuration failed */
    K230_ERR_VICAP_INIT = -4,     /** VICAP initialization failed */
    K230_ERR_VICAP_START = -5,    /** VICAP stream start failed */
    K230_ERR_FRAME_DUMP = -6,     /** Frame dump (capture) failed */
    K230_ERR_MMAP = -7,           /** Memory mapping failed */
    K230_ERR_INVALID_ARG = -8,    /** Invalid argument */
    K230_ERR_ALLOC = -9,          /** Memory allocation failed */
    K230_ERR_TIMEOUT = -10,       /** Capture timeout */
} K230Error;

/** Supported sensor types */
typedef enum K230SensorType {
    K230_SENSOR_OV5647 = 0,       /** OV5647 (CanMV-K230 default) */
    K230_SENSOR_IMX335 = 1,       /** IMX335 */
    K230_SENSOR_GC2093 = 2,       /** GC2093 */
} K230SensorType;

/** Configuration for capture initialization */
typedef struct K230CaptureConfig {
    uint32_t width;               /** Capture width in pixels */
    uint32_t height;              /** Capture height in pixels */
    uint32_t fps;                 /** Target frame rate */
    K230SensorType sensor_type;   /** Sensor type */
    uint32_t dev_num;             /** VICAP device number (usually 0) */
    uint32_t chn_num;             /** VICAP channel number (usually 0) */
} K230CaptureConfig;

/**
 * Initialize K230 VICAP capture.
 *
 * Sets up video buffer pools, configures VICAP device and channel,
 * and starts the capture stream.
 *
 * @param config  Capture configuration
 * @param err     Output error code (K230_OK on success)
 * @return        Context handle on success, NULL on failure
 */
K230CaptureContext* k230_capture_init(const K230CaptureConfig* config, K230Error* err);

/**
 * Capture a grayscale frame.
 *
 * Dumps a frame from VICAP, extracts the Y plane from NV12 data,
 * and copies it to the provided buffer.
 *
 * @param ctx     Context from k230_capture_init
 * @param buffer  Output buffer for grayscale data (must be width*height bytes)
 * @param len     Buffer length in bytes
 * @return        K230_OK on success, error code on failure
 */
K230Error k230_capture_grayscale(K230CaptureContext* ctx, uint8_t* buffer, size_t len);

/**
 * Deinitialize and free capture resources.
 *
 * Stops the stream, releases buffer pools, and frees the context.
 *
 * @param ctx  Context to deinitialize (may be NULL)
 */
void k230_capture_deinit(K230CaptureContext* ctx);

/**
 * Get a human-readable error string.
 *
 * @param err  Error code
 * @return     Static string describing the error
 */
const char* k230_error_string(K230Error err);

#ifdef __cplusplus
}
#endif

#endif /* K230_CAPTURE_H */
