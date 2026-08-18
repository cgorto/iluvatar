#!/usr/bin/env bash
set -euo pipefail

# Cross-compile the RT-Smart camera process for the K230 big core.
#
# Required environment:
#   K230_MUSL_TOOLCHAIN  Kendryte RT-Smart riscv64 musl toolchain root.
#   K230_MPP_LIB_DIR     Directory containing the RT-Smart MPP static libraries.
#
# The MPP binaries are supplied by the K230 SDK and are deliberately not
# redistributed in this repository.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
K230_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_DIR="${K230_DIR}/runtime"
BUILD_DIR="${SCRIPT_DIR}/build"
OUTPUT="${SCRIPT_DIR}/rtsmart-camera"

: "${K230_MUSL_TOOLCHAIN:?Set K230_MUSL_TOOLCHAIN to the RT-Smart musl toolchain root}"
: "${K230_MPP_LIB_DIR:?Set K230_MPP_LIB_DIR to the K230 MPP static-library directory}"

CROSS_GCC="${K230_MUSL_TOOLCHAIN}/bin/riscv64-unknown-linux-musl-gcc"
SYSROOT="${K230_MUSL_TOOLCHAIN}/riscv64-unknown-linux-musl/lib"

for path in "${CROSS_GCC}" "${SYSROOT}/crti.o" "${K230_MPP_LIB_DIR}/libvicap.a"; do
    if [[ ! -e "${path}" ]]; then
        printf 'Missing build input: %s\n' "${path}" >&2
        exit 1
    fi
done
if ! command -v odin >/dev/null 2>&1; then
    echo "Odin is required on PATH." >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}"

"${CROSS_GCC}" -c -mcmodel=medany \
    "${RUNTIME_DIR}/crt_rtsmart.S" -o "${BUILD_DIR}/crt_rtsmart.o"
"${CROSS_GCC}" -c -mcmodel=medany \
    "${RUNTIME_DIR}/start_c.c" -o "${BUILD_DIR}/start_c.o"

odin build "${SCRIPT_DIR}/src" \
    -target:linux_riscv64 \
    -build-mode:obj \
    -out:"${BUILD_DIR}/camera" \
    -o:speed \
    -target-features:"v,zvl128b"

"${CROSS_GCC}" -nostartfiles -static -mcmodel=medany \
    -T "${RUNTIME_DIR}/link.lds" \
    -o "${OUTPUT}" \
    "${SYSROOT}/crti.o" \
    "${BUILD_DIR}/crt_rtsmart.o" \
    "${BUILD_DIR}/start_c.o" \
    "${BUILD_DIR}"/camera*.o \
    -Wl,--start-group \
    -L"${K230_MPP_LIB_DIR}" \
    -ldatafifo \
    -lvicap -lsensor -lvb -lsys -lcommon -lvideo_in \
    -l3a -lauto_ctrol -lcam_caldb -lcam_device -lcam_engine \
    -lcameric_drv -lcameric_reg_drv -lisp_drv -lisi \
    -lhal -lebase -loslayer -lstart_engine \
    -lcmd_buffer -lbuffer_management -lvirtual_hal -lfpga \
    -lswitch -lbinder \
    -lt_common_c -lt_database_c -lt_json_c -lt_mxml_c \
    -lc -lgcc -lm -lpthread \
    -Wl,--end-group \
    "${SYSROOT}/crtn.o"

printf 'Built %s\n' "${OUTPUT}"
file "${OUTPUT}"
