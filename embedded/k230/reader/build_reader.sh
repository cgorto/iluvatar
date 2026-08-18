#!/usr/bin/env bash
set -euo pipefail

# Build the DATAFIFO-to-TCP forwarder for the K230 little Linux core.
#
# Required environment:
#   K230_MPP_LIB_DIR  Directory containing libdatafifo_linux.a.
#
# Set CROSS_GCC to override cross-compiler discovery.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="${SCRIPT_DIR}/reader"

: "${K230_MPP_LIB_DIR:?Set K230_MPP_LIB_DIR to the K230 MPP static-library directory}"

if [[ -z "${CROSS_GCC:-}" ]]; then
    if command -v riscv64-linux-gnu-gcc >/dev/null 2>&1; then
        CROSS_GCC="riscv64-linux-gnu-gcc"
    elif command -v riscv64-unknown-linux-gnu-gcc >/dev/null 2>&1; then
        CROSS_GCC="riscv64-unknown-linux-gnu-gcc"
    else
        echo "No RISC-V glibc cross-compiler found; set CROSS_GCC." >&2
        exit 1
    fi
fi

DATAFIFO_LIB="${K230_MPP_LIB_DIR}/libdatafifo_linux.a"
if [[ ! -f "${DATAFIFO_LIB}" ]]; then
    printf 'Missing build input: %s\n' "${DATAFIFO_LIB}" >&2
    exit 1
fi

"${CROSS_GCC}" -O2 -Wall -Wextra -Werror \
    -I"${SCRIPT_DIR}/vendor" \
    -c "${SCRIPT_DIR}/reader.c" \
    -o "${SCRIPT_DIR}/reader.o"

"${CROSS_GCC}" -static \
    -o "${OUTPUT}" \
    "${SCRIPT_DIR}/reader.o" \
    "${DATAFIFO_LIB}" \
    -lpthread

printf 'Built %s\n' "${OUTPUT}"
file "${OUTPUT}"
