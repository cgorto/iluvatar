/**
 * K230 SDK Video Buffer API (Stub)
 *
 * Video buffer pool management.
 * Real headers should be extracted from K230 board or SDK.
 */

#ifndef K230_MPI_VB_H
#define K230_MPI_VB_H

#include "k230_mpi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VB_MAX_COMM_POOLS 16

/* Buffer remap mode */
typedef enum {
    VB_REMAP_MODE_NONE = 0,
    VB_REMAP_MODE_NOCACHE,
    VB_REMAP_MODE_CACHED,
} k_vb_remap_mode;

/* Common pool configuration */
typedef struct {
    k_u64 blk_size;
    k_u32 blk_cnt;
    k_vb_remap_mode mode;
} k_vb_pool_config;

/* VB configuration */
typedef struct {
    k_u32 max_pool_cnt;
    k_vb_pool_config comm_pool[VB_MAX_COMM_POOLS];
} k_vb_config;

/**
 * Set video buffer configuration.
 * Must be called before kd_mpi_vb_init().
 */
k_s32 kd_mpi_vb_set_config(const k_vb_config* config);

/**
 * Initialize video buffer pools.
 */
k_s32 kd_mpi_vb_init(void);

/**
 * Deinitialize and release video buffer pools.
 */
k_s32 kd_mpi_vb_exit(void);

#ifdef __cplusplus
}
#endif

#endif /* K230_MPI_VB_H */
