# QNX HIL infusion pump

This repository contains the project files for a QNX based hardware-in-the-loop infusion pump demo.

The setup uses a Raspberry Pi running QNX Neutrino as the controller and an STM32 Nucleo-F401RE as the hardware side of the pump. The two boards talk over UART. The QNX side uses native message passing between processes, with pulses used for alarm notification.

This is a classroom prototype. It is not certified medical software and should not be described as production ready.

## What is in this repo

- `codes/`
  - QNX-side code split into working slices.
  - Earlier slices keep the simpler versions around so the build-up can be followed.
- `STM/infusion_pump_hil/`
  - STM32CubeIDE project for the Nucleo-F401RE firmware.
  - Handles UART packets, motor state, LED output, and button-based fault injection.
- `FINAL_REPORT.pdf` and `FINAL_REPORT.docx`
  - Final project report.

## Hardware used

- Raspberry Pi 4B running QNX Neutrino 8.0
- STM32 Nucleo-F401RE
- UART wiring between Pi GPIO14/GPIO15 and STM32 USART1 PA9/PA10
- 115200 baud, 8N1
- STM32 onboard LED on PA5
- STM32 user button on PC13 for fault injection

The UART wiring used in the project was:

```text
STM32 PA9  TX  ->  Pi GPIO15 RX, pin 10
STM32 PA10 RX  ->  Pi GPIO14 TX, pin 8
STM32 GND      ->  Pi GND, pin 6
```

## Packet format

The QNX and STM32 sides use the same framed packet format:

```text
[0xAA][CMD][LEN][DATA...][CRC8]
```

CRC is calculated over `CMD + LEN + DATA`. The implementation uses CRC8-MAXIM/Dallas.

Main commands:

| ID | Command | Direction |
| --- | --- | --- |
| `0x01` | `MOTOR_START` | QNX to STM32 |
| `0x02` | `MOTOR_STOP` | QNX to STM32 |
| `0x03` | `STATUS_REQ` | QNX to STM32 |
| `0x04` | `HEARTBEAT` | STM32 to QNX |
| `0x05` | `ACK` | STM32 to QNX |
| `0x06` | `FAULT` | STM32 to QNX |
| `0x07` | `STATUS_RSP` | STM32 to QNX |

## QNX process layout

The final demo path is built around three small QNX programs:

- `demo_client`
  - Sends `start`, `stop`, and `status` commands.
- `motor_ctrl`
  - Owns `/dev/ser1`.
  - Converts QNX IPC requests into UART packets.
  - Parses replies from STM32.
  - Watches for faults and heartbeat loss in the later slices.
- `alarm_mgr`
  - Receives fault pulses from `motor_ctrl`.
  - Prints the alarm event seen by QNX.

Normal control flow:

```text
demo_client -> MsgSend -> motor_ctrl -> UART -> STM32
STM32 -> UART reply -> motor_ctrl -> MsgReply -> demo_client
```

Fault flow:

```text
STM32 fault or QNX timeout -> motor_ctrl -> pulse -> alarm_mgr
```

## Slice status

The `codes/` folders are snapshots of the project as it grew.

| Folder | Main point | Status from project notes |
| --- | --- | --- |
| `codes/slice1` | Framed UART packets and CRC | Hardware validated |
| `codes/slice2` | QNX `motor_ctrl` server and `demo_client` | Hardware validated |
| `codes/slice3` | `alarm_mgr` and pulse-based fault reporting | Hardware validated |
| `codes/slice4` | Heartbeat and disconnect detection | Hardware validated |

The slice notes record the final hardware run on 2026-04-14.

## Running the QNX side

The QNX image used in this project has a read-only root filesystem. Runtime files have to go under `/var`, and `/var` is wiped on reboot.

For the later slices, the intended startup order is:

```text
alarm_mgr
motor_ctrl
demo_client start
demo_client status
demo_client stop
```

For the heartbeat slice, start the motor, let heartbeats arrive, then disconnect/reset the STM32 side to show the heartbeat-miss alarm.

## STM32 behavior

The STM32 firmware acts as the hardware side of the pump demo.

It receives framed UART packets, checks CRC, updates a small state machine, and sends replies back to QNX.

LED behavior:

```text
IDLE     -> LED off
RUNNING  -> LED on
FAULT    -> LED blinking
```

The user button is used as a fault trigger in the demo.

## Notes

- USART1 on PA9/PA10 is used for the external UART link.
- USART2 was avoided because it is routed through ST-Link on the Nucleo board.
- QNX serial device used during development: `/dev/ser1`.
- `slay` is used on QNX instead of `killall`.
- The final report contains the fuller write-up. This README is only a map of the repository and demo.