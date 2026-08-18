#!/usr/bin/env python3
"""Replace the embedded RT-Smart fastboot ELF and rebuild a K230 RTT partition."""

import argparse
import gzip
import hashlib
import pathlib
import struct
import zlib

FIRMWARE_HEADER_SIZE = 528
UIMAGE_HEADER_SIZE = 64
K230_MAGIC = 0x3033324B
UIMAGE_MAGIC = 0x27051956


def elf_file_size(blob: bytes, start: int) -> int:
    fields = struct.unpack_from("<16sHHIQQQIHHHHHH", blob, start)
    (_, _, _, _, _, phoff, shoff, _, _, phentsz, phnum,
     shentsz, shnum, _) = fields
    end = max(phoff + phentsz * phnum, shoff + shentsz * shnum)
    for index in range(shnum):
        section = struct.unpack_from(
            "<IIQQQQIIQQ", blob, start + shoff + index * shentsz
        )
        section_type = section[1]
        section_offset = section[4]
        section_size = section[5]
        if section_type != 8:  # SHT_NOBITS has no file payload.
            end = max(end, section_offset + section_size)
    return end


def unpack_partition(partition: bytes):
    magic, firmware_length, crypto_type = struct.unpack_from("<III", partition, 0)
    if magic != K230_MAGIC or crypto_type != 0:
        raise ValueError("expected an unsigned K230 firmware image")

    firmware = partition[FIRMWARE_HEADER_SIZE:FIRMWARE_HEADER_SIZE + firmware_length]
    expected_hash = partition[12:44]
    if hashlib.sha256(firmware).digest() != expected_hash:
        raise ValueError("firmware SHA-256 mismatch")

    version = firmware[:4]
    header = bytearray(firmware[4:4 + UIMAGE_HEADER_SIZE])
    fields = struct.unpack(">7I4B32s", header)
    magic, _, _, image_size, _, _, data_crc, _, _, image_type, compression, _ = fields
    if magic != UIMAGE_MAGIC or image_type != 4 or compression != 1:
        raise ValueError("expected a gzip-compressed multi-file U-Boot image")

    payload = firmware[4 + UIMAGE_HEADER_SIZE:4 + UIMAGE_HEADER_SIZE + image_size]
    component_size = struct.unpack_from(">I", payload, 0)[0]
    if struct.unpack_from(">I", payload, 4)[0] != 0:
        raise ValueError("expected exactly one multi-image component")
    compressed = bytearray(payload[8:8 + component_size])
    if zlib.crc32(payload) & 0xFFFFFFFF != data_crc:
        raise ValueError("U-Boot payload CRC mismatch")
    if compressed[:3] != b"\x1f\x8b\x09":
        raise ValueError("expected K230 private-gzip marker")
    compressed[2] = 8
    image = gzip.decompress(compressed)
    return version, header, image


def rebuild_partition(original: bytes, version: bytes, uimage_header: bytearray,
                      image: bytes) -> bytes:
    compressed = bytearray(gzip.compress(image, compresslevel=9, mtime=0))
    compressed[2] = 9  # K230's private-gzip marker; payload remains DEFLATE.
    payload = struct.pack(">II", len(compressed), 0) + compressed

    fields = list(struct.unpack(">7I4B32s", uimage_header))
    fields[1] = 0  # Header CRC while calculating it.
    fields[3] = len(payload)
    fields[6] = zlib.crc32(payload) & 0xFFFFFFFF
    header = bytearray(struct.pack(">7I4B32s", *fields))
    fields[1] = zlib.crc32(header) & 0xFFFFFFFF
    header = struct.pack(">7I4B32s", *fields)

    firmware = version + header + payload
    firm_header = bytearray(original[:FIRMWARE_HEADER_SIZE])
    struct.pack_into("<I", firm_header, 4, len(firmware))
    firm_header[12:44] = hashlib.sha256(firmware).digest()

    if len(firm_header) + len(firmware) > len(original):
        raise ValueError("rebuilt firmware does not fit in the partition")
    output = bytearray(len(original))
    output[:FIRMWARE_HEADER_SIZE] = firm_header
    output[FIRMWARE_HEADER_SIZE:FIRMWARE_HEADER_SIZE + len(firmware)] = firmware
    return bytes(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("original_partition", type=pathlib.Path)
    parser.add_argument("replacement_elf", type=pathlib.Path)
    parser.add_argument("output_partition", type=pathlib.Path)
    args = parser.parse_args()

    original = args.original_partition.read_bytes()
    replacement = args.replacement_elf.read_bytes()
    version, uimage_header, image = unpack_partition(original)

    elf_offset = image.find(b"\x7fELF")
    if elf_offset < 0:
        raise ValueError("embedded fastboot ELF not found")
    original_elf_size = elf_file_size(image, elf_offset)
    if len(replacement) > original_elf_size:
        raise ValueError(
            f"replacement ELF ({len(replacement)}) exceeds embedded slot "
            f"({original_elf_size})"
        )

    patched = bytearray(image)
    patched[elf_offset:elf_offset + original_elf_size] = b"\0" * original_elf_size
    patched[elf_offset:elf_offset + len(replacement)] = replacement

    # This board image's /bin/init.sh redirects startup to /sharefs/init.
    # Point it at the embedded slot so startup does not depend on sharefs.
    init_offset = image.rfind(b"/sharefs/init", 0, elf_offset)
    if init_offset < 0:
        raise ValueError("embedded init script not found")
    init_newline = image.find(b"\n", init_offset, elf_offset)
    if init_newline < 0:
        raise ValueError("embedded init script terminator not found")
    init_file_size = init_newline - init_offset + 1
    init_command = b"/bin/fastboot_app.elf"
    if len(init_command) + 1 > init_file_size:
        raise ValueError("embedded init script slot is too small")
    patched_init = init_command.ljust(init_file_size - 1, b" ") + b"\n"
    patched[init_offset:init_offset + init_file_size] = patched_init

    output = rebuild_partition(original, version, uimage_header, patched)

    # Validate all nested checksums and ensure the replacement survived.
    _, _, verification = unpack_partition(output)
    if verification[elf_offset:elf_offset + len(replacement)] != replacement:
        raise ValueError("replacement verification failed")
    args.output_partition.write_bytes(output)
    print(f"embedded ELF offset: 0x{elf_offset:x}")
    print(f"embedded ELF slot:   {original_elf_size} bytes")
    print(f"replacement ELF:     {len(replacement)} bytes")
    print(f"partition image:     {len(output)} bytes")


if __name__ == "__main__":
    main()
