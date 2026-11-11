# Flash test console logs

The eight `flash_2p5_01`, `native_*`, `confirm_*` and `restore_*` files are native Gowin CLI runs with hardware USB capture enabled, without Frida API observation or logic-start pauses. See [validation.md](../validation.md) for the complete result table.

Two separately named diagnostics are also retained:

- `flash_3MHz.console.log`: blocking observation before IR75; **excluded** from native pass/fail statistics because it pauses the target command.
- `diagnostic_3p333333MHz.console.log`: asynchronous Frida observation without an explicit target pause; **separate diagnostic conditions**, successful full Verify Success.

Logs retain the tool's carriage-return progress updates. USB captures, logic sample binaries, software payload traces and the FPGA example image remain in the local debugging archive, outside this repository.
