# Validated firmware snapshot

These are the exact ELF, HEX and BIN files archived during the successful 2026-11-11 hardware tests. Firmware architecture and source were not changed for publication.

- MCU: CH32V305GBU6, 16MHz HSE, 144MHz system clock.
- Linker layout: 128KB Flash, 32KB RAM, 2KB stack.
- Recommended embedded-Flash request clock: 2.5MHz.
- Compiler: MounRiver RISC-V Embedded GCC 8.2.0, `rv32imacxw`, `ilp32`, `-O2`.
- Independent clean-output rebuild: exit 0, zero errors, zero warnings; rebuilt BIN matches the validated BIN byte for byte.

Use HEX or ELF with a CH32V305-capable WCH-Link programmer; raw BIN starts at the application Flash origin specified by `Ld/Link.ld` (logical address 0). Configure the programmer's MCU-specific address mapping accordingly.

Check files against [SHA256SUMS.txt](SHA256SUMS.txt). [manifest.json](manifest.json) records file identities and publication build checks. [source-sha256.json](source-sha256.json) identifies the build inputs after Git's CRLF-to-LF text normalization; these source digests describe the published text representation, while the firmware digests describe unchanged binary files.

This snapshot is experimental. Third-party WCH notices apply to the included vendor code and resulting firmware; see [NOTICE.md](../NOTICE.md).
