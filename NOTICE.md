# Copyright and third-party notices

The project's original code and documentation are provided under the [MIT License](LICENSE), copyright 2026 Zhang Shuo (lishuo4096).

## WCH SDK files

This repository includes WCH SDK files in `Peripheral/`, `Core/`, `Startup/`, `Debug/` and portions of `User/`, including modified USBHS device code. Their original copyright and usage notices remain in the source files and continue to apply. The root MIT license does not replace third-party terms.

The WCH headers identify Nanjing Qinheng Microelectronics Co., Ltd. as the copyright holder and include this notice:

> Attention: This software (modified or not) and binary are used for microcontroller manufactured by Nanjing Qinheng Microelectronics.

The bundled firmware is built for the WCH CH32V305. Project-authored modifications to vendor files are provided under MIT subject to the retained vendor notices for the underlying material.

## References and acknowledgments

The development directory contains reference checkouts that are intentionally excluded from this published repository. Protocol and implementation references include:

- [tpunix/FT-LINK](https://github.com/tpunix/FT-LINK)
- [sipeed/RV-Debugger-BL702](https://github.com/sipeed/RV-Debugger-BL702)
- OpenOCD MPSSE driver and pyftdi opcode definitions, as noted in `User/mpsse.c`
- FTDI AN_108, Command Processor for MPSSE

These references retain their respective licenses. This repository does not distribute FTDI drivers, Gowin software, captured driver binaries, or the reference-project checkouts. FTDI and Gowin product names identify compatibility targets; this is an independent experimental project.
