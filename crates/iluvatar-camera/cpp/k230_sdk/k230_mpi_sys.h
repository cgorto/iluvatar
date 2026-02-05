/**
 * K230 SDK System API (Stub)
 *
 * Memory mapping and system utilities.
 * Real headers should be extracted from K230 board or SDK.
 */

#ifndef K230_MPI_SYS_H
#define K230_MPI_SYS_H

#include "k230_mpi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Map physical memory to virtual address (cached).
 *
 * @param phys_addr  Physical address to map
 * @param size       Size in bytes
 * @return           Virtual address, or NULL on failure
 */
void* kd_mpi_sys_mmap_cached(k_u64 phys_addr, k_u32 size);

/**
 * Map physical memory to virtual address (uncached).
 *
 * @param phys_addr  Physical address to map
 * @param size       Size in bytes
 * @return           Virtual address, or NULL on failure
 */
void* kd_mpi_sys_mmap(k_u64 phys_addr, k_u32 size);

/**
 * Unmap previously mapped memory.
 *
 * @param virt_addr  Virtual address from kd_mpi_sys_mmap*
 * @param size       Size that was mapped
 * @return           0 on success
 */
k_s32 kd_mpi_sys_munmap(void* virt_addr, k_u32 size);

#ifdef __cplusplus
}
#endif

#endif /* K230_MPI_SYS_H */
