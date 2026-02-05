/**
 * K230 SDK Common Types (Stub)
 *
 * Minimal type definitions for compilation.
 * Real headers should be extracted from K230 board or SDK.
 */

#ifndef K230_MPI_TYPES_H
#define K230_MPI_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Basic types */
typedef int32_t k_s32;
typedef uint32_t k_u32;
typedef uint64_t k_u64;
typedef uint8_t k_u8;
typedef int k_bool;

#define K_TRUE  1
#define K_FALSE 0

/* Pixel formats */
typedef enum {
    PIXEL_FORMAT_YVU_SEMIPLANAR_420 = 0,  /* NV12 */
    PIXEL_FORMAT_YVU_SEMIPLANAR_422,      /* NV16 */
    PIXEL_FORMAT_RGB_888,
    PIXEL_FORMAT_BGR_888,
} k_pixel_format;

/* Rectangle */
typedef struct {
    k_u32 x;
    k_u32 y;
    k_u32 width;
    k_u32 height;
} k_rect;

/* Video frame */
typedef struct {
    k_pixel_format pixel_format;
    k_u32 width;
    k_u32 height;
    k_u64 phys_addr[3];  /* Physical addresses for planes */
    k_u64 virt_addr[3];  /* Virtual addresses for planes */
    k_u32 stride[3];     /* Stride for each plane */
} k_video_frame;

typedef struct {
    k_video_frame v_frame;
    k_u32 pool_id;
} k_video_frame_info;

#ifdef __cplusplus
}
#endif

#endif /* K230_MPI_TYPES_H */
