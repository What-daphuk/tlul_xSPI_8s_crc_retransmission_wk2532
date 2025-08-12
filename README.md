# tlul_xSPI_8s_crc_retransmission_wk2532
This Verilog project bridges a TileLink (TL-UL) SoC interface to an xSPI peripheral. It features a complete TL-UL master/slave and an xSPI controller/slave with CRC8 error checking and retransmission. The system translates TL-UL read/write requests into serial xSPI transactions, ideal for interfacing with flash memory or other peripherals.
# xSPI Controller with TileLink UL Interface

This project implements a custom, simplified SPI-like serial protocol called **xSPI**, along with a complete **TileLink Uncached-Lite (TL-UL)** master/slave system that uses the xSPI peripheral for communication.  
The design is written in Verilog and integrates both the xSPI protocol and TileLink UL for SoC-level simulation.

---

## 📜 Project Overview

The **xSPI** protocol is a serial communication interface for transferring commands, addresses, and data between a controller (master) and a slave device.  
It includes CRC checks for both the **command/address phase** and the **data phase** to ensure data integrity.

This project wraps the xSPI peripheral inside a TL-UL interface, enabling a TL-UL master to initiate read/write requests that are translated into xSPI transactions.  
The xSPI slave executes these transactions, accesses its memory, and sends responses back over the TL-UL bus.

---

## 📂 File Structure

All modules are contained within a single Verilog file: **`xSPI.v`**

### **xSPI Implementation**
- **`xspi_top`** – Top-level xSPI module connecting the controller and slave.
- **`xspi_sopi_controller`** – The xSPI master FSM.
- **`xspi_sopi_slave`** – The xSPI slave FSM with simple memory.
- **`crc8` / `crc8_slave`** – CRC-8 calculation modules.

### **TileLink to xSPI Bridge**
- **`tlul_spi_interconnect`** – Translates TL-UL transactions to xSPI control signals.
- **`top_tlul_spi`** – Wrapper connecting `tlul_spi_interconnect` to `xspi_top`.

### **TileLink System**
- **`tilelink_ul_master_top`** – TL-UL master for testing.
- **`tilelink_ul_slave_top`** – TL-UL slave wrapping the xSPI peripheral.
- **`tilelink_wrapper_top`** – Final top-level simulation module connecting master and slave.

---

## 🔹 Module Descriptions

### **xSPI Modules**
- **xspi_top**  
  Connects master and slave via a shared IO bus and manages bus direction.

- **xspi_sopi_controller**  
  Master FSM that sends command, address, and data with CRC checks. Supports retransmission (up to 3 times) on CRC error.

- **xspi_sopi_slave**  
  Slave FSM that verifies incoming CRC, stores data in an internal 64-bit memory register, and responds to read requests.

- **crc8 / crc8_slave**  
  Implements CRC-8 calculation with polynomial `0x07`. The slave version is clocked on the negative edge.

---

### **TileLink Modules**
- **tlul_spi_interconnect**  
  Converts TL-UL Get / PutFullData requests into xSPI transactions.

- **top_tlul_spi**  
  Connects TL-UL logic to the xSPI core.

- **tilelink_ul_slave_top**  
  TL-UL slave device with xSPI backend.

- **tilelink_ul_master_top**  
  TL-UL master for generating test transactions.

- **tilelink_wrapper_top**  
  Highest-level module connecting master and slave for simulation.

---

## 📡 Features

- **Master-Slave Architecture** (1:1 connection)
- **Command-Driven Protocol**:
  - `0xA5` → Write 64 bits
  - `0xFF` → Read 64 bits
- **CRC-8 Error Checking** for both address/command and data phases.
- **Error Recovery**: Retransmits up to 3 times on CRC failure.
- **TileLink UL Integration**: Fully TL-UL compatible peripheral.

---

## 🔄 xSPI Protocol

Each transaction contains:

1. **Command** (1 byte) – `0xA5` for write, `0xFF` for read.  
2. **Address** (6 bytes) – 48-bit address.  
3. **Command/Address CRC** (1 byte) – CRC over command + address.  
4. **Data Transfer** (8 bytes):  
   - Write → Controller sends data  
   - Read → Slave sends data after latency (6 cycles)  
5. **Data CRC** (1 byte) – CRC over data.  
6. **CS_n** – Chip select asserted low for transaction framing.

---

## 🔗 TileLink Integration

**TL-UL → xSPI Mapping**:
- `PutFullData (a_opcode = 0)` → xSPI Write (`0xA5`)
- `Get (a_opcode = 4)` → xSPI Read (`0xFF`)

The **tilelink_ul_slave_top** handles the TL-UL handshake.  
The **tlul_spi_interconnect** translates bus protocol signals to xSPI signals.

---

## 🧪 How to Simulate

1. **Instantiate** `tilelink_wrapper_top` in your testbench.
2. **Drive the Master**:
   - `a_valid_in` – Assert to start a request.
   - `a_opcode_in` – `0` for write, `4` for read.
   - `a_address_in` – Target address.
   - `a_data_in` – Data for write requests.
3. **Observe the Slave**:
   - `d_valid_tb` – Response ready signal.
   - `d_data_tb` – Read data for read transactions.
   - `d_opcode_tb` – Response type.

**Typical Test Flow**:
1. Perform a **write** to a memory location.
2. Perform a **read** from the same location.
3. Verify that the data matches.

---

## ⚙️ CRC Polynomial
