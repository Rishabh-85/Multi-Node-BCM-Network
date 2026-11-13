# Multi-Node-BCM-Network
ESRTOS Project

# Multi-Node Body Control Network over CAN

Two STM32F446RE (NUCLEO-F446RE) ECUs exchanging CAN frames at 500 kbit/s through MCP2515 SPI CAN modules.

## 1. System Overview

**Node A (Body Control Module)** reads the ignition, headlight, and turn-indicator switches, drives the lights and warning buzzer, decides when to auto-lock, and broadcasts a control frame (`0x100`) every 100 ms.
**Node B (Door/Window Module)** receives that frame, drives the lock and window servos, reads the door switch, and replies with a status frame (`0x200`) every 100 ms. Each node declares the other lost after 500 ms without a frame: Node A warns on UART and blinks LD2, and Node B enters fail-safe (window disabled, lock holds).

## 2. Hardware & Pinout

MCP2515 wiring is identical on both nodes (SPI2, mode 0, 5.625 MHz):

| Component | STM32 Pin | MCP2515 Pin | Notes |
|---|---|---|---|
| SPI SCK | PB13 | SCK | SPI2 clock |
| SPI MISO | PB14 | SO | Master In Slave Out |
| SPI MOSI | PB15 | SI | Master Out Slave In |
| Chip Select | PB12 | CS | Active low, idles high |
| Interrupt | PB1 | INT | Active low on frame received (polled) |
| Power | 5 V (3.3 V if module is 3.3 V) | VCC | Check module rating |
| Ground | GND | GND | Common ground with the other node |
| CAN Bus | — | CANH / CANL | 120 Ω termination at both ends, twisted pair |

MCP2515 modules use an **8 MHz crystal**; bit timing is `CNF1=0x00, CNF2=0x90, CNF3=0x02` (500 kbit/s, 62.5% sample point).

Node I/O (all switches active low with internal pull-up, 30 ms debounce):

| Function | Node A | Node B |
|---|---|---|
| Ignition / Door switch | PC13 (blue button) | PC13 (blue button, held = door open) |
| Switch 2 | PA1 headlight | PA0 window UP |
| Switch 3 | PA4 left turn | PA4 window DOWN |
| Switch 4 | PB5 right turn | PB5 lock / unlock |
| Outputs | PA6 headlight LED, PA7 left LED, PB6 right LED, PC7 buzzer | PA6 window servo (TIM3 CH1), PA7 lock servo (TIM3 CH2) |
| Status LED | PA5 (LD2): fast blink = Node B lost | PA5 (LD2): solid = link OK, fast blink = Node A lost |
| Debug UART | PA2 / PA3 (USART2, 115200 8N1) | PA2 / PA3 (USART2, 115200 8N1) |

Servos run from an external 5 V supply with its ground tied to the Nucleo GND.

## 3. CAN Message Frame Matrix

Standard 11-bit IDs, data frames, 500 kbit/s. Lower ID wins arbitration.

| CAN ID (Hex) | Sender | Receiver | DLC | Payload Bytes | Function |
|---|---|---|---|---|---|
| `0x100` | Node A | Node B | 3 | D0: ignition (0/1), D1: lock command (0 = unlock, 1 = lock), D2: alive counter | BCM control command, every 100 ms |
| `0x200` | Node B | Node A | 4 | D0: door (0 = closed, 1 = open), D1: lock state (0/1), D2: window position (0–100%), D3: alive counter | Door/window status, every 100 ms |

D4–D7 are not used. Link timeout: **500 ms** with no frame from the peer.

## 4. How to Build & Flash

1. Open **STM32CubeIDE** and import the node project (**File → Import → Existing Projects into Workspace**). Open the `.ioc` file if you want to inspect or regenerate the CubeMX configuration.
2. Build with **Project → Build All** (Node A and Node B are separate projects).
3. Connect one NUCLEO-F446RE over USB and flash with **Run** (the on-board **ST-LINK** programs it). Flash Node A to one board and Node B to the other.
4. In Node B's `main.c`, make sure `LOOPBACK_TEST` is `0` for the real bus (`1` makes the MCP2515 talk to itself for single-board testing).
5. Open each board's ST-LINK virtual COM port at **115200, 8N1** and press the black reset button to see the start-up log.
