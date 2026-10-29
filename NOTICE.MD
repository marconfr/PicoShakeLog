# PicoShakeLog Firmware – Overview

PicoShakeLog is a dual-core firmware for the Raspberry Pi Pico, designed for real-time IMU data acquisition and streaming over WiFi/TCP. The system is modular, with clear separation between acquisition (Core 1) and network communication (Core 0).

## Main Objects & Methods

- **psl_state_handler_t**: Central structure holding all configuration, state, and error flags.
- **Core 0 (Network)**:  
  - `core0_init_wifi_tcp()`: Initializes WiFi SoftAP and TCP server.
  - `core0_main_loop()`: Main loop handling client connections, command parsing, and data streaming.
- **Core 1 (Acquisition)**:  
  - `core1_entry()`: Main acquisition loop, waits for acquisition flag, handles DMA IRQs.
  - `core1_start_dma_transfer()`: Starts DMA transfer on IMU interrupt.
- **FIFO Buffer**:  
  - `psl_fifo_t`: Thread-safe circular buffer for inter-core data exchange.
  - `fifo_init()`, `fifo_free()`: Buffer management.
- **Shared State**:  
  - `psl_shared_state_t`: Synchronizes acquisition and client status between cores.
  - `shared_state_set_acquisition()`, `shared_state_get_acquisition()`: Thread-safe flag access.
- **Hardware Config**:  
  - All SPI, DMA, and WiFi parameters are centralized in `psl_config.h`.

## Principle Diagrams

### System Architecture

```mermaid
flowchart TD
    subgraph Core0[Core 0: Network]
        A1[WiFi SoftAP Init]
        A2[TCP Server Init]
        A3[Wait for Client]
        A4[Receive Command]
        A5[Send Data from FIFO]
    end
    subgraph Core1[Core 1: Acquisition]
        B1[Wait for acquisition_active]
        B2[IMU INT triggers DMA]
        B3[DMA reads IMU data]
        B4[Push data to FIFO]
    end
    subgraph Shared[Shared State]
        S1[acquisition_active flag]
        S2[client_connected flag]
    end
    subgraph FIFO[Inter-core FIFO]
        F1[Thread-safe Buffer]
    end
    A1 --> A2 --> A3 --> A4
    A4 -- Start/Stop --> S1
    S1 -- Read --> B1
    B1 --> B2 --> B3 --> B4 --> F1
    F1 -- Data --> A5
    A5 -- TCP --> Client
    A3 -- Client Connected --> S2
    S2 -- Read --> B1
```

### Data Flow

```mermaid
sequenceDiagram
    participant Client
    participant Core0 as Core 0 (Network)
    participant Shared as Shared State
    participant Core1 as Core 1 (Acquisition)
    participant FIFO as FIFO Buffer
    Client->>Core0: Connect (TCP)
    Core0->>Shared: Set client_connected = true
    Client->>Core0: Send CMD_START
    Core0->>Shared: Set acquisition_active = true
    Core1->>Shared: Read acquisition_active
    Core1->>IMU: Wait for INT
    IMU-->>Core1: INT (data ready)
    Core1->>DMA: Start DMA transfer
    DMA-->>Core1: DMA complete
    Core1->>FIFO: Push IMU data
    Core0->>FIFO: Read data
    Core0->>Client: Send data
    Client->>Core0: Send CMD_STOP
    Core0->>Shared: Set acquisition_active = false
    Core1->>Shared: Read acquisition_active (stop)
```

---

This notice gives a concise overview. Doxygen comments in the code provide detailed documentation for each function and structure.

```
doxygen Doxyfile
```
