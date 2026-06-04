# T-SIM7080G-S3 UART Bring-Up Notes

## Goal

Receive framed GV2 UART output on the LilyGO T-SIM7080G-S3 and confirm that JPEG frames and state packets arrive correctly.

## Final Working Pin Mapping

Board reference image used:

- `doc/T-SIM7080G-S3_1.jpg`

Relevant labels from the board image:

- `GPIO17` = `U1TXD`
- `GPIO18` = `U1RXD`

Final working UART configuration:

- T-SIM `RX = GPIO18`
- T-SIM `TX = GPIO17`
- GV2 power enable = `GPIO43 HIGH`

Wiring:

- `GV2 TX -> T-SIM GPIO18`
- `GV2 RX -> T-SIM GPIO17`
- `GND -> GND`
- New PCB revision: GPIO43 controls the switched GV2 ground path; firmware drives it HIGH at boot to activate the GV2.

## Why The Earlier Attempt Failed

The earlier `RX=16 / TX=17` assumption was wrong for this board.

`GPIO16` is labeled `U0CTS` on the T-SIM7080G-S3 pinout image, not the UART1 RX pin used here.

## PlatformIO Settings

The working `t-sim7080g-s3` environment uses:

```ini
-DGV2_POWER_GPIO_CFG=43
-DUSB_SERIAL_BAUD_CFG=921600
-DVISION_UART_BAUD_CFG=921600
-DVISION_UART_RX_GPIO_CFG=18
-DVISION_UART_TX_GPIO_CFG=17
-DENABLE_UART_ONLY_CFG=1
```

## UART Logic Summary

The firmware now:

1. Boots in UART-only mode
2. Skips mode selection and motor-control flow
3. Initializes `VisionUART` on `GPIO18/17`
4. Prints startup diagnostics
5. Watches for framed GV2 packets
6. Detects:
   - `VSTS` state frames
   - `VSTJ` JPEG frames
7. Prints parsed metadata like:

```text
recv #1 len=3924 state=1 class=3 conf=0.859
```

8. For diagnostics, also prints:
   - a short raw hex preview of incoming UART bytes
   - periodic status if no bytes are seen

## Key Evidence From The Working Test

The successful log started with:

```text
VISION UART INIT
RX=18 TX=17 baud=921600
framed GV2 parser enabled
raw preview + RX status enabled
```

Then it showed:

```text
[VISION UART] raw preview: 56 53 54 53 56 53 54 53 56 53 54 53 56 53 54 4A
recv #1 len=3924 state=1 class=3 conf=0.859
```

Meaning:

- `56 53 54 53` = `VSTS`
- `56 53 54 4A` = `VSTJ`

So the T-SIM is now receiving and parsing GV2 UART correctly.

## Important Conclusion

The issue was not:

- baud rate
- parser logic
- PlatformIO monitor settings

The real issue was:

- incorrect T-SIM UART pin mapping

## Current Behavior

The firmware now suppresses raw JPEG forwarding to the USB serial monitor.

Current serial behavior:

1. print UART bring-up diagnostics
2. print parsed metadata such as:

```text
recv #1 len=7640 state=0 class=3 conf=0.412
```

3. do not dump the full JPEG binary to the terminal

The short raw hex preview used for diagnostics may still appear at startup when bytes are first received.

## Additional Checkpoint

An additional field test confirmed that `RX=16 / TX=17` also works in the current hardware setup:

```text
RX=16 TX=17 baud=921600
framed GV2 parser enabled
raw preview + RX status enabled

=== UART-only bring-up mode ===
Mode prompt and motor controls are disabled.
Vision UART RX=GPIO16 TX=GPIO17
[VISION UART] raw preview: 56 53 54 53 56 53 54 4A
recv #1 len=7640 state=0 class=3 conf=0.412
```

So at this stage we know:

- the GV2 framed UART protocol is received correctly on the T-SIM
- the parser works
- JPEG payload forwarding to the terminal has been disabled

The next practical step is:

1. write the raw JPEG directly to SD
2. keep metadata as text or JSON
3. serve the JPEG to a browser as `image/jpeg`

## Codex Prompt Record

This is the key working prompt/context that led to the fix:

> Check this image `C:\DEV\T-SIMMotorShield\doc\T-SIM7080G-S3_1.jpg`
>
> The image showed:
> - `GPIO17` = `U1TXD`
> - `GPIO18` = `U1RXD`
>
> Therefore adjust the firmware and wiring to:
> - `GV2 RX -> T-SIM GPIO17`
> - `GV2 TX -> T-SIM GPIO18`
> - firmware `RX=18`, `TX=17`

## Next Step

Use this verified UART configuration as the base for:

- SD image capture
- JPEG file persistence
- browser display from the ESP32-S3
