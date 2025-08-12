//! @file xSPI.v
//! @brief Top-level and submodules for a simple SPI-like protocol (xSPI) with controller and slave, and a TL-UL interface.

/**
 * @module xspi_top
 * @brief Top-level module connecting xSPI controller and slave via a shared IO bus.
 * @param clk                   System clock
 * @param rst_n                 Active-low reset
 * @param start                 Start transaction
 * @param command               8-bit command
 * @param address               48-bit address
 * @param wr_data               64-bit data to write
 * @param rd_data               64-bit data read
 * @param done                  Transaction done flag
 * @param ready                 Slave ready flag
 * @param crc_ca_match_slave    CRC match for command/address from slave
 * @param crc_ca_error_slave    CRC error for command/address from slave
 * @param crc_ca_match_master   CRC match for command/address from master
 * @param crc_ca_error_master   CRC error for command/address from master
 * @param crc_data_match_master CRC match for data from master
 * @param crc_data_error_master CRC error for data from master
 * @param crc_data_match_slave  CRC match for data from slave
 * @param crc_data_error_slave  CRC error for data from slave
 */
module xspi_top (
    input  wire        clk,                   //!< System clock
    input  wire        rst_n,                 //!< Active-low reset
    input  wire        start,                 //!< Start transaction
    input  wire [7:0]  command,               //!< 8-bit command
    input  wire [47:0] address,               //!< 48-bit address
    input  wire [63:0] wr_data,               //!< 64-bit data to write
    output wire [63:0] rd_data,               //!< 64-bit data read
    output wire        done,                  //!< Transaction done flag
    output wire        ready,                 //!< Slave ready flag
    output wire        crc_ca_match_slave,
    output wire        crc_ca_error_slave,
    output wire        crc_ca_match_master,
    output wire        crc_ca_error_master,
    output wire        crc_data_match_master,
    output wire        crc_data_error_master,
    output wire        crc_data_match_slave,
    output wire        crc_data_error_slave
);

    // Internal bus and control signals
    wire        cs_n;                  //!< Chip select (active low)
    wire        sck;                   //!< SPI clock
    wire [7:0]  master_io_out;         //!< Controller output to bus
    wire [7:0]  slave_io_out;          //!< Slave output to bus
    wire        master_io_oe;          //!< Controller output enable
    wire        slave_io_oe;           //!< Slave output enable
    wire [7:0]  io_bus;                //!< Shared IO bus
    wire        data_strobe;           //!< Data strobe for read operations

    /**
     * @brief Simplified IO bus multiplexing.
     * @details This assigns the output of the master or slave to the bus based on their output enable signals.
     */
    assign io_bus = master_io_oe ? master_io_out : 
                    slave_io_oe  ? slave_io_out  : 8'h00;

    /**
     * @brief xSPI controller (master) instance.
     */
    xspi_sopi_controller master (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(master_io_out),
        .io_in(io_bus),
        .io_oe(master_io_oe),
        .start(start),
        .command_in(command),
        .address_in(address),
        .wr_data_in(wr_data),
        .rd_data(rd_data),
        .done(done),
        .data_strobe(data_strobe),
        .crc_ca_match(crc_ca_match_master),
        .crc_data_match(crc_data_match_master),
        .crc_ca_error(crc_ca_error_master),
        .crc_data_error(crc_data_error_master),
        .crc_ca_error_slave(crc_ca_error_slave),
        .crc_data_error_slave(crc_data_error_slave)
    );

    /**
     * @brief xSPI slave instance.
     */
    xspi_sopi_slave slave (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(slave_io_out),
        .io_in(io_bus),
        .io_oe(slave_io_oe),
        .ready(ready),
        .data_strobe(data_strobe),
        .crc_ca_match(crc_ca_match_slave),
        .crc_data_match(crc_data_match_slave),
        .crc_ca_error(crc_ca_error_slave),
        .crc_data_error(crc_data_error_slave),
        .crc_ca_error_master(crc_ca_error_master),
        .crc_data_error_master(crc_data_error_master)
    );


endmodule

/**
 * @module xspi_sopi_controller
 * @brief xSPI controller (master) for command/address/data transfer with CRC and retransmission.
 * @param clk                 System clock
 * @param rst_n               Active-low reset
 * @param cs_n                Chip select (active low)
 * @param sck                 SPI clock
 * @param io_out              Output to IO bus
 * @param io_in               Input from IO bus
 * @param io_oe               Output enable for IO bus
 * @param start               Start transaction
 * @param command_in          8-bit command
 * @param address_in          48-bit address
 * @param wr_data_in          64-bit data to write
 * @param data_strobe         Data strobe for read operations
 * @param rd_data             64-bit data read
 * @param done                Transaction done flag
 * @param crc_ca_match        CRC match for command/address
 * @param crc_data_match      CRC match for data
 * @param crc_ca_error        CRC error for command/address
 * @param crc_data_error      CRC error for data
 * @param crc_ca_error_slave  CRC error from slave for command/address
 * @param crc_data_error_slave CRC error from slave for data
 */
module xspi_sopi_controller (
    input  wire        clk,                   //!< System clock
    input  wire        rst_n,                 //!< Active-low reset

    // SPI signals - separated input/output
    output reg         cs_n,                  //!< Chip select (active low)
    output reg         sck,                   //!< SPI clock
    output reg [7:0]   io_out,                //!< Output to IO bus
    input  wire [7:0]  io_in,                 //!< Input from IO bus
    output reg         io_oe,                 //!< Output enable for IO bus

    // Control signals
    input  wire        start,                 //!< Start transaction
    input  wire [7:0]  command_in,            //!< 8-bit command
    input  wire [47:0] address_in,            //!< 48-bit address
    input  wire [63:0] wr_data_in,            //!< 64-bit data to write
    input  wire        data_strobe,

    output reg [63:0]  rd_data,               //!< 64-bit data read
    output reg         done,                  //!< Transaction done flag
    output reg         crc_ca_match,
    output reg         crc_data_match,
    output reg         crc_ca_error,
    output reg         crc_data_error,
    input wire         crc_ca_error_slave,
    input wire         crc_data_error_slave
);
    reg [2:0] wait_cycles;      //!< Counter for read latency wait cycles
    reg [7:0] command;          //!< Internal register for command
    reg [47:0] address;         //!< Internal register for address
    reg [63:0] wr_data;         //!< Internal register for write data
    reg [3:0] retransmit_cnt;   //!< Retransmission counter

    // === FSM States ===
    reg [3:0] state;            //!< Current FSM state
    reg [3:0] next_state;       //!< Next FSM state
    localparam Latency           = 3'd6; //!< Latency cycles for read operations
    localparam STATE_IDLE        = 4'd0; //!< Idle state
    localparam STATE_CMD         = 4'd1; //!< Send command
    localparam STATE_ADDR        = 4'd2; //!< Send address
    localparam STATE_SEND_CRC_CA = 4'd3; //!< Send CRC for command/address
    localparam STATE_WR_DATA     = 4'd4; //!< Write data
    localparam STATE_SEND_CRC_DATA = 4'd5; //!< Send CRC for data
    localparam STATE_RD_DATA     = 4'd6; //!< Read data
    localparam STATE_RECV_CRC_DATA = 4'd7; //!< Receive CRC for data
    localparam STATE_RECV_CRC_CA = 4'd8; //!< Receive CRC for command/address
    localparam STATE_FINISH      = 4'd9; //!< Finish/cleanup state

    // === Counters and Buffers ===
    reg [3:0] byte_cnt;         //!< Byte counter for address/data
    reg [63:0] rdata_buf;       //!< Buffer for read data

    // === CRC Logic ===
    reg crc_ca_clear, crc_ca_enable;     //!< Control signals for command/address CRC
    wire [7:0] crc_ca_out;               //!< Calculated CRC for command/address
    reg [7:0] crc_ca_recv;               //!< Received CRC for command/address
    reg crc_data_clear, crc_data_enable; //!< Control signals for data CRC
    wire [7:0] crc_data_out;             //!< Calculated CRC for data
    reg [7:0] crc_data_recv;             //!< Received CRC for data
    reg [7:0] data_for_crc;              //!< Data input to CRC modules

    /**
     * @brief CRC8 instance for command and address.
     */
    crc8 crc_ca_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_ca_enable),
        .clear(crc_ca_clear),
        .data_in(data_for_crc),
        .crc_out(crc_ca_out)
    );

    /**
     * @brief CRC8 instance for data.
     */
    crc8 crc_data_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_data_enable),
        .clear(crc_data_clear),
        .data_in(data_for_crc),
        .crc_out(crc_data_out)
    );

    /**
     * @brief FSM sequential logic: state update.
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= STATE_IDLE;
        else
            state <= next_state;
    end

    /**
     * @brief FSM combinational logic: next state determination.
     */
    always @(*) begin
        next_state = state;
        case (state)
            STATE_IDLE:
                if (start)
                    next_state = STATE_CMD;
            STATE_CMD:
                next_state = STATE_ADDR;
            STATE_ADDR:
                if (byte_cnt == 6)
                    next_state = STATE_SEND_CRC_CA;
            STATE_SEND_CRC_CA:
                next_state = (command == 8'hFF) ? STATE_RECV_CRC_CA : (command == 8'hA5) ? STATE_WR_DATA : STATE_FINISH;
            STATE_RECV_CRC_CA:
                next_state = (command == 8'hFF) ? STATE_RD_DATA : STATE_FINISH;
            STATE_WR_DATA:
                if (byte_cnt == 14)
                    next_state = STATE_SEND_CRC_DATA;
            STATE_SEND_CRC_DATA:
                next_state = STATE_FINISH;
            STATE_RD_DATA:
                if (byte_cnt == 15)
                    next_state = STATE_RECV_CRC_DATA;
            STATE_RECV_CRC_DATA:
                next_state = STATE_FINISH;
            STATE_FINISH:
                // Check for retransmission conditions
                if ((crc_ca_error_slave == 1'b1 || crc_data_error_slave == 1'b1 || crc_data_error == 1'b1) && retransmit_cnt < 4'd3) begin
                    next_state = STATE_CMD;  // Retransmit from command
                end else begin
                    next_state = STATE_IDLE;
                end
            default:
                next_state = STATE_IDLE;
        endcase
    end

    /**
     * @brief Main operation: drive SPI signals and manage data transfer based on FSM state.
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n   <= 1'b1;
            sck    <= 1'b0;
            io_out <= 8'h00;
            io_oe  <= 1'b0;
            done   <= 1'b0;
            rd_data <= 64'h0;
            rdata_buf <= 64'h0;
            byte_cnt <= 4'd0;
            wait_cycles <= 3'd0;
            crc_ca_clear <= 1'b1;
            crc_ca_enable <= 1'b0;
            crc_data_clear <= 1'b1;
            crc_data_enable <= 1'b0;
            crc_ca_match <= 1'b0;
            crc_ca_error <= 1'b0;
            crc_data_match <= 1'b0;
            crc_data_error <= 1'b0;
            retransmit_cnt <= 0;

        end else begin
            // Default disables
            crc_ca_enable <= 1'b0;
            crc_data_enable <= 1'b0;
            case (state)
                STATE_IDLE: begin
                    done   <= 1'b0;
                    cs_n   <= 1'b1;
                    sck    <= 1'b0;
                    io_oe  <= 1'b0;
                    byte_cnt <= 4'd0;
                    wait_cycles <= 0;
                    crc_ca_clear <= 1'b1;
                    crc_data_clear <= 1'b1;
                    crc_ca_match <= 1'b0;
                    crc_ca_error <= 1'b0;
                    crc_data_match <= 1'b0;
                    crc_data_error <= 1'b0;
                    command<=command_in;
                    address<=address_in;
                    wr_data<=wr_data_in;
                end
                STATE_CMD: begin
                    cs_n   <= 1'b0;
                    io_oe  <= 1'b1;
                    io_out <= command;
                    byte_cnt <= 1;
                    // CRC for command
                    crc_ca_clear <= 1'b0;
                    crc_ca_enable <= 1'b1;
                    data_for_crc <= command;
                end
                STATE_ADDR: begin
                    io_oe  <= 1'b1;
                    case (byte_cnt)
                        1: begin io_out <= address[47:40]; data_for_crc <= address[47:40]; end
                        2: begin io_out <= address[39:32]; data_for_crc <= address[39:32]; end
                        3: begin io_out <= address[31:24]; data_for_crc <= address[31:24]; end
                        4: begin io_out <= address[23:16]; data_for_crc <= address[23:16]; end
                        5: begin io_out <= address[15:8];  data_for_crc <= address[15:8];  end
                        6: begin io_out <= address[7:0];   data_for_crc <= address[7:0];   end
                    endcase
                  if (byte_cnt<=5)
                    crc_ca_enable <= 1'b1;
                  else crc_ca_enable <= 1'b0;
                    crc_ca_clear <= 1'b0;
                    byte_cnt <= byte_cnt + 1;
                end
                STATE_SEND_CRC_CA: begin
                    io_oe <= 1'b1;
                    io_out <= crc_ca_out;
                    crc_ca_enable <= 1'b0;
                end
                STATE_RECV_CRC_CA: begin
                    io_oe <= 1'b0;
                    crc_ca_recv <= io_in;
                    if (io_in == crc_ca_out) begin
                        crc_ca_match <= 1'b1;
                        crc_ca_error <= 1'b0;
                    end else begin
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b1;
                    end
                end
                STATE_WR_DATA: begin
                    io_oe  <= 1'b1;
                    case (byte_cnt)
                        7: begin io_out <= wr_data[63:56]; data_for_crc <= wr_data[63:56]; end
                        8: begin io_out <= wr_data[55:48]; data_for_crc <= wr_data[55:48]; end
                        9: begin io_out <= wr_data[47:40]; data_for_crc <= wr_data[47:40]; end
                        10: begin io_out <= wr_data[39:32]; data_for_crc <= wr_data[39:32]; end
                        11: begin io_out <= wr_data[31:24]; data_for_crc <= wr_data[31:24]; end
                        12: begin io_out <= wr_data[23:16]; data_for_crc <= wr_data[23:16]; end
                        13: begin io_out <= wr_data[15:8];  data_for_crc <= wr_data[15:8];  end
                        14: begin io_out <= wr_data[7:0];   data_for_crc <= wr_data[7:0];   end
                    endcase
                  if (byte_cnt<=13)
                    crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                    crc_data_clear <= 0;
                    byte_cnt <= byte_cnt + 1;
                end
                STATE_SEND_CRC_DATA: begin
                    crc_data_clear<=0;
                    io_oe <= 1'b1;
                    io_out <= crc_data_out;
                    crc_data_enable <= 1'b0;
                end
                STATE_RD_DATA: begin
                    crc_ca_match <= 1'b0;
                    io_oe <= 1'b0;
                    case (byte_cnt)
                        8: begin rdata_buf[63:56] <= io_in; data_for_crc <= io_in; end
                        9: begin rdata_buf[55:48] <= io_in; data_for_crc <= io_in; end
                        10: begin rdata_buf[47:40] <= io_in; data_for_crc <= io_in; end
                        11: begin rdata_buf[39:32] <= io_in; data_for_crc <= io_in; end
                        12: begin rdata_buf[31:24] <= io_in; data_for_crc <= io_in; end
                        13: begin rdata_buf[23:16] <= io_in; data_for_crc <= io_in; end
                        14: begin rdata_buf[15:8]  <= io_in; data_for_crc <= io_in; end
                        15: begin rdata_buf[7:0]   <= io_in; data_for_crc <= io_in; end
                    endcase
                  if (byte_cnt>=8 && byte_cnt<=14)
                        crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                      crc_data_clear <= 0;
                  if (Latency-1 > wait_cycles) begin
                        wait_cycles <= wait_cycles +1;
                    end else begin
                        byte_cnt <= byte_cnt + 1;
                    end
                end
                STATE_RECV_CRC_DATA: begin
                    io_oe <= 1'b0;
                    crc_data_recv <= io_in;
                    if (io_in == crc_data_out) begin
                        crc_data_match <= 1'b1;
                        crc_data_error <= 1'b0;
                    end else begin
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b1;
                    end
                end
                STATE_FINISH: begin
                    if (crc_ca_error_slave == 1'b1 || crc_data_error_slave == 1'b1 || crc_data_error == 1'b1) begin
                        // Retransmission needed - reset counters and signals
                        cs_n  <= 1'b1;  // Deassert CS for retransmission
                        io_oe <= 1'b0;
                        done  <= 1'b0;
                        byte_cnt <= 4'd0;
                        wait_cycles <= 3'd0;
                        retransmit_cnt <= retransmit_cnt + 1;  // Increment retransmission counter
                        crc_ca_clear <= 1'b1;
                        crc_data_clear <= 1'b1;
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b0;
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b0;
                    end else begin
                        // Normal completion
                        crc_data_match <= 1'b0;
                        cs_n  <= 1'b1;
                        io_oe <= 1'b0;
                        done  <= 1'b1;
                        rd_data <= rdata_buf;
                    end
                end
            endcase
        end
    end
endmodule

/**
 * @module xspi_sopi_slave
 * @brief xSPI slave for command/address/data transfer and simple memory with CRC.
 * @param clk                 System clock
 * @param rst_n               Active-low reset
 * @param cs_n                Chip select (active low)
 * @param sck                 SPI clock
 * @param io_out              Output to IO bus
 * @param io_in               Input from IO bus
 * @param io_oe               Output enable for IO bus
 * @param ready               Ready flag (debug/status)
 * @param data_strobe         Data strobe for read operations
 * @param crc_ca_match        CRC match for command/address
 * @param crc_data_match      CRC match for data
 * @param crc_ca_error        CRC error for command/address
 * @param crc_data_error      CRC error for data
 * @param crc_ca_error_master CRC error from master for command/address
 * @param crc_data_error_master CRC error from master for data
 */
module xspi_sopi_slave (
    input  wire        clk,                   //!< System clock
    input  wire        rst_n,                 //!< Active-low reset

    input  wire        cs_n,                  //!< Chip select (active low)
    input  wire        sck,                   //!< SPI clock
    output reg [7:0]   io_out,                //!< Output to IO bus
    input  wire [7:0]  io_in,                 //!< Input from IO bus
    output reg         io_oe,                 //!< Output enable for IO bus

    output reg         ready,                 //!< Ready flag (debug/status)
    output wire        data_strobe,
    output reg         crc_ca_match,
    output reg         crc_data_match,
    output reg         crc_ca_error,
    output reg         crc_data_error,
    input wire         crc_ca_error_master,
    input wire         crc_data_error_master
);

    // === FSM States ===
    reg [3:0] state;            //!< Current FSM state
    localparam STATE_IDLE        = 4'd0; //!< Idle state
    localparam STATE_CMD         = 4'd1; //!< Receive command
    localparam STATE_ADDR        = 4'd2; //!< Receive address
    localparam STATE_RECV_CRC_CA = 4'd3; //!< Receive CRC for command/address
    localparam STATE_WAIT_LATENCY= 4'd4; //!< Wait latency for read
    localparam STATE_WR_DATA     = 4'd5; //!< Receive write data
    localparam STATE_RECV_CRC_DATA= 4'd6; //!< Receive CRC for data
    localparam STATE_RD_DATA     = 4'd7; //!< Send read data
    localparam STATE_SEND_CRC_DATA= 4'd8; //!< Send CRC for data
    localparam STATE_DONE        = 4'd9; //!< Done state

    // === Internal Registers and Counters ===
    reg [3:0] byte_cnt;         //!< Byte counter for transactions
    reg [2:0] latency_cnt;      //!< Counter for read latency
    reg [3:0] retransmit_cnt;   //!< Retransmission counter for slave
    reg [7:0]  command_reg;     //!< Register to store command
    reg [47:0] addr_reg;        //!< Register to store address
    reg [63:0] data_reg;        //!< Register to store data
    reg [63:0] mem;             //!< Simple 64-bit memory register

    // === CRC Logic ===
    reg crc_ca_clear, crc_ca_enable;     //!< Control signals for command/address CRC
    wire [7:0] crc_ca_out;               //!< Calculated CRC for command/address
    reg [7:0] crc_ca_recv;               //!< Received CRC for command/address
    reg crc_data_clear, crc_data_enable; //!< Control signals for data CRC
    wire [7:0] crc_data_out;             //!< Calculated CRC for data
    reg [7:0] crc_data_recv;             //!< Received CRC for data
    reg [7:0] data_for_crc;              //!< Data input to CRC modules

    assign data_strobe = (state == STATE_RD_DATA || state == STATE_SEND_CRC_DATA) ? clk : 1'b0;

    /**
     * @brief CRC8 instance for command and address.
     */
    crc8 crc_ca_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_ca_enable),
        .clear(crc_ca_clear),
        .data_in(data_for_crc),
        .crc_out(crc_ca_out)
    );
    
    /**
     * @brief CRC8 instance for data (slave version, clocked on negedge).
     */
    crc8_slave crc_data_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_data_enable),
        .clear(crc_data_clear),
        .data_in(data_for_crc),
        .crc_out(crc_data_out)
    );

    /**
     * @brief Main FSM and data handling logic for the slave.
     */
    always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= STATE_IDLE;
            io_out       <= 8'h00;
            io_oe        <= 1'b0;
            byte_cnt     <= 4'd0;
            latency_cnt  <= 3'd0;
            retransmit_cnt <= 4'd0;
            ready        <= 1'b0;
            command_reg  <= 8'h00;
            addr_reg     <= 48'h0;
            data_reg     <= 64'h0;
            crc_ca_clear <= 1'b1;
            crc_ca_enable <= 1'b0;
            crc_data_clear <= 1'b1;
            crc_data_enable <= 1'b0;
            crc_ca_match <= 1'b0;
            crc_ca_error <= 1'b0;
            crc_data_match <= 1'b0;
            crc_data_error <= 1'b0;
        end else begin
            crc_ca_enable <= 1'b0;
            crc_data_enable <= 1'b0;
            case (state)
                STATE_IDLE: begin
                    ready <= 0;
                    byte_cnt <= 0;
                    io_oe <= 1'b0;
                    crc_ca_clear <= 1'b1;
                    crc_data_clear <= 1'b1;
                    crc_ca_match <= 1'b0;
                    crc_ca_error <= 1'b0;
                    crc_data_match <= 1'b0;
                    crc_data_error <= 1'b0;
                    if (!cs_n) begin
                        state <= STATE_CMD;
                    end
                end
                STATE_CMD: begin
                    if (!cs_n) begin
                        command_reg <= io_in;
                        byte_cnt    <= 1;
                        state       <= STATE_ADDR;
                        crc_ca_clear <= 1'b0;
                        crc_ca_enable <= 1'b1;
                        data_for_crc <= io_in;
                    end
                end
                STATE_ADDR: begin
                    case (byte_cnt)
                        1: begin addr_reg[47:40] <= io_in; data_for_crc <= io_in; end
                        2: begin addr_reg[39:32] <= io_in; data_for_crc <= io_in; end
                        3: begin addr_reg[31:24] <= io_in; data_for_crc <= io_in; end
                        4: begin addr_reg[23:16] <= io_in; data_for_crc <= io_in; end
                        5: begin addr_reg[15:8]  <= io_in; data_for_crc <= io_in; end
                        6: begin addr_reg[7:0]   <= io_in; data_for_crc <= io_in; end
                    endcase
                  if (byte_cnt<=5)
                    crc_ca_enable <= 1'b1;
                  else crc_ca_enable <= 1'b0;
                    crc_ca_clear <= 1'b0;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt == 6) state <= STATE_RECV_CRC_CA;
                end
                STATE_RECV_CRC_CA: begin
                    crc_ca_recv <= io_in;
                    if (io_in == crc_ca_out) begin
                      if (retransmit_cnt < 3) begin //! * This induces error thrice even when not errornous
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b1;
                      end
                      else begin
                        crc_ca_match <= 1'b1;
                        crc_ca_error <= 1'b0;
                      end
                    end else begin
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b1;
                    end
                    if (command_reg == 8'hFF) begin
                        latency_cnt <= 0;
                        state <= STATE_WAIT_LATENCY;
                    end else if (command_reg == 8'hA5) begin
                        state <= STATE_WR_DATA;
                    end else begin
                        state <= STATE_DONE;
                    end
                end
                STATE_WAIT_LATENCY: begin
                  crc_ca_match<=0;
                    latency_cnt <= latency_cnt + 1;
                    if (latency_cnt == 3'd5) begin
                        data_reg <= mem;
                        byte_cnt <= 7;
                        io_oe    <= 1'b1;
                        state    <= STATE_RD_DATA;
                        crc_data_clear <= 1'b1;
                    end
                end
                STATE_WR_DATA: begin
                  crc_ca_match<=0;
                    case (byte_cnt)
                        7:  begin data_reg[63:56] <= io_in; data_for_crc <= io_in; end
                        8:  begin data_reg[55:48] <= io_in; data_for_crc <= io_in; end
                        9:  begin data_reg[47:40] <= io_in; data_for_crc <= io_in; end
                        10: begin data_reg[39:32] <= io_in; data_for_crc <= io_in; end
                        11: begin data_reg[31:24] <= io_in; data_for_crc <= io_in; end
                        12: begin data_reg[23:16] <= io_in; data_for_crc <= io_in; end
                        13: begin data_reg[15:8]  <= io_in; data_for_crc <= io_in; end
                        14: begin data_reg[7:0]   <= io_in; data_for_crc <= io_in; end
                    endcase
                   if (byte_cnt>=7 && byte_cnt<=13)
                    crc_data_enable <= 1'b1;
                   else crc_data_enable <= 1'b0;
                    crc_data_clear <= 0;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt == 14) state <= STATE_RECV_CRC_DATA;
                end
                STATE_RECV_CRC_DATA: begin
                    crc_data_clear<=0;
                    crc_data_recv <= io_in;
                    if (io_in == crc_data_out) begin
                        crc_data_match <= 1'b1;
                        crc_data_error <= 1'b0;
                    end else begin
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b1;
                    end
                    mem <= data_reg;
                    state <= STATE_DONE;
                end
                STATE_RD_DATA: begin
                    case (byte_cnt)
                        7:  begin io_out <= data_reg[63:56]; data_for_crc <= data_reg[63:56]; end
                        8:  begin io_out <= data_reg[55:48]; data_for_crc <= data_reg[55:48]; end
                        9:  begin io_out <= data_reg[47:40]; data_for_crc <= data_reg[47:40]; end
                        10: begin io_out <= data_reg[39:32]; data_for_crc <= data_reg[39:32]; end
                        11: begin io_out <= data_reg[31:24]; data_for_crc <= data_reg[31:24]; end
                        12: begin io_out <= data_reg[23:16]; data_for_crc <= data_reg[23:16]; end
                        13: begin io_out <= data_reg[15:8];  data_for_crc <= data_reg[15:8];  end
                        14: begin io_out <= data_reg[7:0];   data_for_crc <= data_reg[7:0];   end
                    endcase
                  if (byte_cnt>=7 && byte_cnt<=13)
                    crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                    crc_data_clear <= 0;
                    byte_cnt <= byte_cnt + 1;
                  if (byte_cnt == 14) state <= STATE_SEND_CRC_DATA;
                end
                STATE_SEND_CRC_DATA: begin
                    io_out <= crc_data_out;
                    io_oe <= 1'b1;
                    crc_data_enable <= 1'b0;
                    state <= STATE_DONE;
                end
                STATE_DONE: begin
                    crc_data_match <= 1'b0;
                    io_oe <= 1'b0;
                    
                    // Check for retransmission conditions
                    if ((crc_ca_error_master == 1'b1 || crc_data_error_master == 1'b1 || crc_ca_error == 1'b1 || crc_data_error == 1'b1) && retransmit_cnt < 4'd3) begin
                        // Reset for retransmission
                        byte_cnt <= 4'd0;
                        latency_cnt <= 3'd0;
                        retransmit_cnt <= retransmit_cnt + 1;
                        crc_ca_clear <= 1'b1;
                        crc_data_clear <= 1'b1;
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b0;
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b0;
                        state <= STATE_CMD;
                        ready <= 1'b0;
                    end else begin
                        state <= STATE_IDLE;
                        ready <= 1'b1;
                    end
                end
                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule

/**
 * @module crc8
 * @brief CRC8 calculation module (master version).
 * @details Calculates an 8-bit CRC with a polynomial of 0x07. This version is sensitive to the positive edge of the clock.
 * @param clk     System clock
 * @param rst     Asynchronous reset
 * @param enable  Enable CRC calculation
 * @param clear   Clear CRC output
 * @param data_in 8-bit input data
 * @param crc_out 8-bit CRC output
 */
module crc8 (
    input  wire       clk,     ///< System clock
    input  wire       rst,     ///< Asynchronous reset
    input  wire       enable,  ///< Enable CRC calculation
    input  wire       clear,   ///< Clear CRC output
    input  wire [7:0] data_in, ///< 8-bit input data
    output reg  [7:0] crc_out  ///< 8-bit CRC output
);
    parameter POLY = 8'h07; ///< CRC polynomial
    wire [7:0] crc_in = crc_out ^ data_in;
    wire [7:0] stage0 = (crc_in[7]) ? (crc_in << 1) ^ POLY : (crc_in << 1);
    wire [7:0] stage1 = (stage0[7]) ? (stage0 << 1) ^ POLY : (stage0 << 1);
    wire [7:0] stage2 = (stage1[7]) ? (stage1 << 1) ^ POLY : (stage1 << 1);
    wire [7:0] stage3 = (stage2[7]) ? (stage2 << 1) ^ POLY : (stage2 << 1);
    wire [7:0] stage4 = (stage3[7]) ? (stage3 << 1) ^ POLY : (stage3 << 1);
    wire [7:0] stage5 = (stage4[7]) ? (stage4 << 1) ^ POLY : (stage4 << 1);
    wire [7:0] stage6 = (stage5[7]) ? (stage5 << 1) ^ POLY : (stage5 << 1);
    wire [7:0] crc_next = (stage6[7]) ? (stage6 << 1) ^ POLY : (stage6 << 1);
    
    always @(posedge clk or posedge rst) begin
        if (rst)
            crc_out <= 8'h00;
        else if (clear)
            crc_out <= 8'h00;
        else if (enable)
            crc_out <= crc_next;
    end
endmodule 

/**
 * @module crc8_slave
 * @brief CRC8 calculation module (slave version).
 * @details Calculates an 8-bit CRC with a polynomial of 0x07. This version is sensitive to the negative edge of the clock.
 * @param clk     System clock
 * @param rst     Asynchronous reset
 * @param enable  Enable CRC calculation
 * @param clear   Clear CRC output
 * @param data_in 8-bit input data
 * @param crc_out 8-bit CRC output
 */
module crc8_slave (
    input  wire       clk,     ///< System clock
    input  wire       rst,     ///< Asynchronous reset
    input  wire       enable,  ///< Enable CRC calculation
    input  wire       clear,   ///< Clear CRC output
    input  wire [7:0] data_in, ///< 8-bit input data
    output reg  [7:0] crc_out  ///< 8-bit CRC output
);
    parameter POLY = 8'h07; ///< CRC polynomial
    wire [7:0] crc_in = crc_out ^ data_in;
    wire [7:0] stage0 = (crc_in[7]) ? (crc_in << 1) ^ POLY : (crc_in << 1);
    wire [7:0] stage1 = (stage0[7]) ? (stage0 << 1) ^ POLY : (stage0 << 1);
    wire [7:0] stage2 = (stage1[7]) ? (stage1 << 1) ^ POLY : (stage1 << 1);
    wire [7:0] stage3 = (stage2[7]) ? (stage2 << 1) ^ POLY : (stage2 << 1);
    wire [7:0] stage4 = (stage3[7]) ? (stage3 << 1) ^ POLY : (stage3 << 1);
    wire [7:0] stage5 = (stage4[7]) ? (stage4 << 1) ^ POLY : (stage4 << 1);
    wire [7:0] stage6 = (stage5[7]) ? (stage5 << 1) ^ POLY : (stage5 << 1);
    wire [7:0] crc_next = (stage6[7]) ? (stage6 << 1) ^ POLY : (stage6 << 1);

    always @(negedge clk or posedge rst) begin
        if (rst)
            crc_out <= 8'h00;
        else if (clear)
            crc_out <= 8'h00;
        else if (enable)
            crc_out <= crc_next;
    end
endmodule 

//======================================================================
// MODULE: tlul_spi_interconnect
//======================================================================
/**
 * @module tlul_spi_interconnect
 * @brief Interconnect logic to bridge a TileLink-UL interface to the xSPI controller.
 * @param TL_ADDR_WIDTH Width of the TileLink address bus.
 * @param TL_DATA_WIDTH Width of the TileLink data bus.
 * @param clk       System clock.
 * @param rst       Active-high reset.
 * @param enable    Enable signal from the TL-UL slave to start a transaction.
 * @param a_ready   TL-UL A-channel ready signal.
 * @param a_opcode  TL-UL A-channel opcode (read/write).
 * @param a_address TL-UL A-channel address.
 * @param a_data    TL-UL A-channel write data.
 * @param valid     TL-UL D-channel valid signal.
 * @param d_data    TL-UL D-channel read data.
 * @param spi_start Starts the SPI transaction.
 * @param spi_cmd   Command sent to the SPI controller.
 * @param spi_addr  Address sent to the SPI controller.
 * @param spi_data  Write data sent to the SPI controller.
 * @param spi_done  Signal from SPI controller indicating transaction completion.
 * @param data_end  Signal from SPI controller indicating read data is ready.
 * @param spi_resp  Response data from the SPI controller.
 */
module tlul_spi_interconnect #(
    parameter TL_ADDR_WIDTH = 64,
    parameter TL_DATA_WIDTH = 64
)(
    input  wire                         clk,
    input  wire                         rst,

    // TL-UL A Channel
    input  wire                         enable,
    output reg                          a_ready,
    input  wire [2:0]                   a_opcode,
    input  wire [TL_ADDR_WIDTH-1:0]     a_address,
    input  wire [TL_DATA_WIDTH-1:0]     a_data,

    // TL-UL D Channel
    output wire                         valid,
    output wire [63:0]                  d_data,

    // SPI interface
    output reg                          spi_start,
    output wire [7:0]                   spi_cmd,
    output wire [47:0]                  spi_addr,
    output wire [63:0]                  spi_data,
    input  wire                         spi_done,
    input  wire                         data_end,
    input  wire [63:0]                  spi_resp
);
    reg [7:0]  cmd_reg;                  //!< Register for SPI command.
    reg [63:0] addr_reg;                 //!< Register for SPI address.
    reg [63:0] data_reg;                 //!< Register for SPI write data.
    reg        transaction_in_progress;  //!< Flag indicating an active transaction.
    reg        response_ready;           //!< Flag indicating response data is ready.

    assign spi_cmd  = cmd_reg;
    assign spi_addr = addr_reg[47:0];
    assign spi_data = data_reg[63:0];

    /**
     * @brief Main logic for handling TL-UL to SPI conversion.
     */
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cmd_reg <= 8'h00;
            addr_reg <= 64'h0;
            data_reg <= 64'h0;
            spi_start <= 1'b0;
            transaction_in_progress <= 1'b0;
            response_ready <= 1'b0;
            a_ready <= 1'b0;
        end else begin
            spi_start <= 1'b0; // Default to 0, set to 1 only when a new transaction starts
            a_ready <= !transaction_in_progress; // Ready if no transaction is in progress

            // Initiate a new transaction if enabled and no transaction is ongoing
            if (enable && !transaction_in_progress) begin
                if (a_opcode == 3'd0) begin // TL-UL Write (opcode 0)
                    cmd_reg <= 8'hA5; // SPI Write Command
                    addr_reg <= a_address;
                    data_reg <= a_data;
                    spi_start <= 1'b1; // Assert spi_start to begin SPI transaction
                    transaction_in_progress <= 1'b1; // Mark transaction as in progress
                end else if (a_opcode == 3'd4) begin // TL-UL Read (opcode 4)
                    cmd_reg <= 8'hFF; // SPI Read Command
                    addr_reg <= a_address;
                    data_reg <= 64'd0; // Data is not used for read command
                    spi_start <= 1'b1; // Assert spi_start to begin SPI transaction
                    transaction_in_progress <= 1'b1; // Mark transaction as in progress
                end
            end

            // Handle SPI transaction completion
            // For write operations, spi_done signals completion
            if (spi_done && a_opcode == 3'd0) begin
                transaction_in_progress <= 1'b0; // Clear transaction in progress for write
            end
            // For read operations, data_end signals completion of data transfer
            if (data_end && a_opcode == 3'd4) begin
                transaction_in_progress <= 1'b0; // Clear transaction in progress for read
                response_ready <= 1'b1; // Indicate that response data is ready
            end

            // Clear response_ready after one cycle if it was set
            if (response_ready) begin
                response_ready <= 1'b0;
            end
        end
    end

    /**
     * @brief D-channel response logic.
     */
    assign valid = spi_done;
    assign d_data = (spi_done) ? spi_resp : 64'b0; 
endmodule

//======================================================================
// MODULE: top_tlul_spi 
//======================================================================
/**
 * @module top_tlul_spi
 * @brief Top-level wrapper connecting the TL-UL interconnect to the xSPI core.
 * @param clk       System clock.
 * @param rst       Active-high reset.
 * @param enable    Enable signal for a transaction.
 * @param a_opcode  TL-UL A-channel opcode.
 * @param a_address TL-UL A-channel address.
 * @param a_data    TL-UL A-channel write data.
 * @param a_ready   TL-UL A-channel ready signal.
 * @param valid     TL-UL D-channel valid signal.
 * @param d_data    TL-UL D-channel read data.
 */
module top_tlul_spi (
    input  wire [63:0] a_data,
    input  wire [63:0] a_address,
    input  wire [2:0]  a_opcode,
    input  wire        enable,
    input  wire        rst,
    input  wire        clk,
    output wire [63:0] d_data,
    output wire        valid,
    output wire        a_ready
);

    wire        spi_start;  //!< Internal wire for starting SPI transaction.
    wire [7:0]  spi_cmd;    //!< Internal wire for SPI command.
    wire [47:0] spi_addr;   //!< Internal wire for SPI address.
    wire [63:0] spi_data;   //!< Internal wire for SPI write data.
    wire        spi_done;   //!< Internal wire for SPI done signal.
    wire        data_end;   //!< Internal wire for SPI data end signal.
    wire [63:0] spi_resp;   //!< Internal wire for SPI response data.

    /**
     * @brief Instance of the TL-UL to SPI interconnect.
     */
    tlul_spi_interconnect interconnect_inst (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .a_ready(a_ready),
        .a_opcode(a_opcode),
        .a_address(a_address),
        .a_data(a_data),
        .valid(valid),
        .d_data(d_data),
        .spi_start(spi_start),
        .spi_cmd(spi_cmd),
        .spi_addr(spi_addr),
        .spi_data(spi_data),
        .spi_done(spi_done),
        .data_end(data_end),
        .spi_resp(spi_resp)
    );

    /**
     * @brief Instance of the xSPI top module.
     */
    xspi_top xspi_inst (
        .clk(clk),
        .rst_n(!rst),
        .start(spi_start),
        .command(spi_cmd),
        .address(spi_addr),
        .wr_data(spi_data),
        .rd_data(spi_resp),
        .done(spi_done),
        .ready(data_end)
    );
endmodule

/**
 * @module tilelink_ul_slave_top
 * @brief Top-level module for a TileLink-UL slave that interfaces with peripherals like GPIO and Flash via the xSPI bridge.
 * @param TL_ADDR_WIDTH Width of the TileLink address bus.
 * @param TL_DATA_WIDTH Width of the TileLink data bus.
 * @param TL_STRB_WIDTH Width of the TileLink strobe bus.
 * @param TL_SOURCE_WIDTH Width of the TileLink source ID.
 * @param TL_SINK_WIDTH Width of the TileLink sink ID.
 * @param TL_OPCODE_WIDTH Width of the TileLink opcode.
 * @param TL_PARAM_WIDTH Width of the TileLink parameter field.
 * @param TL_SIZE_WIDTH Width of the TileLink size field.
 * @param MEM_BASE_ADDR Base address for the memory-mapped peripheral.
 * @param DEPTH Depth of the memory.
 */
module tilelink_ul_slave_top #( 
    // Core interface widths
    parameter TL_ADDR_WIDTH      = 64,                    //!< Address width
    parameter TL_DATA_WIDTH      = 64,                    //!< Data width
    parameter TL_STRB_WIDTH      = TL_DATA_WIDTH / 8,     //!< Byte mask width/Byte strobe.

    // TileLink metadata widths
    parameter TL_SOURCE_WIDTH    = 3,                     //!< Tags each request with a unique ID.
    parameter TL_SINK_WIDTH      = 3,                     //!< Tags each response with an ID that matches the request.
    parameter TL_OPCODE_WIDTH    = 3,                     //!< Opcode width for instructions
    parameter TL_PARAM_WIDTH     = 3,                     //!< Reserved for future performance hints (must be 0)
    parameter TL_SIZE_WIDTH      = 8,                     //!< Width of size field (2^size bytes)
    
    // A Channel Opcodes
    parameter PUT_FULL_DATA_A    = 3'd0,
    parameter PUT_PARTIAL_DATA_A = 3'd1,
    parameter ARITHMETIC_DATA_A  = 3'd2,
    parameter LOGICAL_DATA_A     = 3'd3,
    parameter GET_A              = 3'd4,
    parameter INTENT_A           = 3'd5,
    parameter ACQUIRE_BLOCK_A    = 3'd6,
    parameter ACQUIRE_PERM_A     = 3'd7,

    // D Channel Opcodes
    parameter ACCESS_ACK_D       = 3'd0,
    parameter ACCESS_ACK_DATA_D  = 3'd1,
    parameter HINT_ACK_D         = 3'd2,
    parameter GRANT_D            = 3'd4,
    parameter GRANT_DATA_D       = 3'd5,
    parameter RELEASE_ACK_D      = 3'd6,
    
    // Slave FSM States
    parameter REQUEST            = 2'd1,
    parameter RESPONSE           = 2'd2,

    // Memory parameters
    parameter MEM_BASE_ADDR      = 64'h0000_0000_0000_0000, //!< Base address for memory
    parameter DEPTH              = 512                      //!< Memory depth (number of entries)
)(
    input  wire                         clk,
    input  wire                         rst,

    // A Channel: Received from MASTER
    output wire                         a_ready,        //!< Slave is ready to accept a request.
    input  wire                         a_valid,        //!< Master has a valid request.
    input  wire [TL_OPCODE_WIDTH-1:0]   a_opcode,
    input  wire [TL_PARAM_WIDTH-1:0]    a_param,
    input  wire [TL_ADDR_WIDTH-1:0]     a_address,
    input  wire [TL_SIZE_WIDTH-1:0]     a_size,
    input  wire [TL_STRB_WIDTH-1:0]     a_mask,
    input  wire [TL_DATA_WIDTH-1:0]     a_data,
    input  wire [TL_SOURCE_WIDTH-1:0]   a_source,

    // D Channel: Sent to MASTER
    output reg                          d_valid,
    input  wire                         d_ready,        //!< Master is ready to accept a response.
    output reg [TL_OPCODE_WIDTH-1:0]   d_opcode,
    output reg [TL_PARAM_WIDTH-1:0]    d_param,
    output reg [TL_SIZE_WIDTH-1:0]     d_size,
    output reg [TL_SINK_WIDTH-1:0]     d_sink,
    output reg [TL_SOURCE_WIDTH-1:0]   d_source,
    output reg [TL_DATA_WIDTH-1:0]     d_data,
    output reg                          d_error
);

    reg [1:0] slave_state;          //!< State variable for slave FSM

    // State flags
    wire in_request;                //!< High when in REQUEST state.
    wire in_response;               //!< High when in RESPONSE state.
    
    // Memory Interface Signals
    reg  [TL_ADDR_WIDTH-1:0] waddr; //!< Write address for memory.
    reg                      wen,ren; //!< Write and read enable for memory.
    reg  [TL_DATA_WIDTH-1:0] wdata; //!< Write data for memory.
    reg  [TL_ADDR_WIDTH-1:0] raddr; //!< Read address for memory.
    wire [TL_DATA_WIDTH-1:0] rdata; //!< Read data from memory.
    
    integer i,j;                    //!< Loop variables for partial write.
    
    // Registers for A Channel (latching inputs)
    reg [TL_OPCODE_WIDTH-1:0]   a_opcode_reg;
    reg [TL_PARAM_WIDTH-1:0]    a_param_reg;
    reg [TL_ADDR_WIDTH-1:0]     a_address_reg;
    reg [TL_SIZE_WIDTH-1:0]     a_size_reg;
    reg [TL_STRB_WIDTH-1:0]     a_mask_reg;
    reg [TL_DATA_WIDTH-1:0]     a_data_reg;
    reg [TL_SOURCE_WIDTH-1:0]   a_source_reg;
    
    reg response_pending;           //!< Flag to indicate a response is pending to be sent.
    reg mem_enable;                 //!< Flag to enable memory access for one transaction.

    wire mem_acc_done;              //!< Memory access (read) done signal from the bridge.
    wire mem_write_done;            //!< Memory write done signal from the bridge.

    wire tlul_spi_bridge_enable;    //!< Enable signal for the TL-UL to SPI bridge.
    wire bridge_output_valid;       //!< Valid signal from the bridge indicating completion.

    assign tlul_spi_bridge_enable = ren | wen;
    assign mem_acc_done = bridge_output_valid; 
    assign mem_write_done = bridge_output_valid;

    // FSM State flags
    assign in_request  = (slave_state == REQUEST);
    assign in_response = (slave_state == RESPONSE);
    
    /**
     * @brief Slave FSM state transition logic.
     */
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            slave_state <= REQUEST;
        end
        else begin
            case (slave_state)
                REQUEST: begin
                    if (a_valid)
                        slave_state <= RESPONSE;
                    else
                        slave_state <= REQUEST;
                end
                RESPONSE: begin
                    if (d_ready & response_pending) begin
                        slave_state <= REQUEST; 
                    end else begin
                        slave_state <= RESPONSE;
                    end
                end
                default: slave_state <= REQUEST;
            endcase
        end
    end 
    
    assign a_ready = in_request;

    /**
     * @brief Slave response and memory control logic.
     */
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wen  <= 1'b0;
            waddr <= {TL_ADDR_WIDTH{1'b0}};
            wdata <= {TL_DATA_WIDTH{1'b0}};
            response_pending <= 0;
            mem_enable <= 0;
            ren  <= 1'b0;
        end else if (in_response) begin
            case (a_opcode_reg)
                PUT_FULL_DATA_A: begin
                    if (!mem_enable & !wen) begin
                        wen   <= 1'b1;
                        waddr <= a_address_reg;
                        wdata <= a_data_reg;
                        mem_enable <= 1'b1;
                        response_pending <= 1'b0;
                    end else begin
                        wen   <= 1'b0;
                        waddr <= {TL_ADDR_WIDTH{1'b0}};
                        wdata <= {TL_DATA_WIDTH{1'b0}};
                        mem_enable <= mem_enable;
                    end
            
                    if (mem_write_done && !response_pending) begin
                        response_pending <= 1'b1;
                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_D; 
                        d_param   <= {TL_PARAM_WIDTH{1'b0}};
                        d_size    <= a_size_reg;
                        d_sink    <= {TL_SINK_WIDTH{1'b0}};
                        d_source  <= a_source_reg;
                        d_data    <= 0; 
                        d_error   <= 1'b0;
                    end else if (response_pending) begin
                        if(!d_ready) begin
                            d_valid   <= 1'b1;
                            // Keep D-channel values asserted
                        end else begin
                            response_pending <= 1'b0;
                            mem_enable <= 0;
                            d_valid   <= 1'b0;
                        end                     
                    end else begin
                        d_valid   <= 1'b0;
                    end
                    ren <= 1'b0;
                end

                PUT_PARTIAL_DATA_A: begin
                    if (!mem_enable & !wen) begin
                        wen   <= 1'b1;
                        waddr <= a_address_reg;
                        for (i = 0; i < TL_STRB_WIDTH; i=i+1)
                            for (j = 0; j < 8; j=j+1) 
                                wdata[j + i*8] <= a_mask_reg[i] ? a_data_reg[j + i*8] : rdata[j + i*8]; // A simplified implementation
                        mem_enable <= 1'b1;
                        response_pending <= 1'b0;
                    end else begin
                        wen   <= 1'b0;
                    end
            
                    if (mem_write_done && !response_pending) begin
                        response_pending <= 1'b1;
                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_D; 
                    end else if (response_pending && d_ready) begin
                        response_pending <= 1'b0;
                        mem_enable <= 0;
                        d_valid   <= 1'b0;
                    end
                    ren <= 1'b0;
                end

                GET_A: begin
                    if (!mem_enable & !ren) begin
                        ren  <= 1'b1;
                        raddr <= a_address_reg;
                        mem_enable <= 1'b1;
                        response_pending <= 1'b0;
                    end else begin
                        ren  <= 1'b0;
                    end
            
                    if (mem_acc_done && !response_pending) begin
                        response_pending <= 1'b1;
                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_DATA_D; 
                        d_data    <= rdata; 
                    end else if (response_pending && d_ready) begin
                        response_pending <= 1'b0;
                        mem_enable <= 0;
                        d_valid   <= 1'b0;
                    end
                    wen <= 1'b0;
                end

                default: begin
                    wen <= 1'b0;
                    ren <= 1'b0;
                    mem_enable <= 0;
                    d_valid   <= 1'b0;
                end
            endcase
        end else begin
            wen <= 1'b0;
            ren <= 1'b0;
            mem_enable <= 0;
            d_valid   <= 1'b0;
        end
    end

    /**
     * @brief Logic for latching A-channel inputs.
     */
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            a_opcode_reg  <= {TL_OPCODE_WIDTH{1'b0}};
            a_param_reg   <= {TL_PARAM_WIDTH{1'b0}};
            a_address_reg <= {TL_ADDR_WIDTH{1'b0}};
            a_size_reg    <= {TL_SIZE_WIDTH{1'b0}};
            a_mask_reg    <= {TL_STRB_WIDTH{1'b0}};
            a_data_reg    <= {TL_DATA_WIDTH{1'b0}};
            a_source_reg  <= {TL_SOURCE_WIDTH{1'b0}};
        end else if (a_valid) begin
            a_opcode_reg  <= a_opcode;
            a_param_reg   <= a_param;
            a_address_reg <= a_address;
            a_size_reg    <= a_size;
            a_mask_reg    <= a_mask;
            a_data_reg    <= a_data;
            a_source_reg  <= a_source;
        end
    end

    /**
     * @brief Instantiation of the TL-UL to SPI bridge.
     */
    top_tlul_spi u_top_tlul_spi (
        .clk      (clk),
        .rst      (rst),
        .enable   (tlul_spi_bridge_enable),
        .a_opcode (a_opcode_reg),
        .a_address(a_address_reg),
        .a_data   (wdata),
        .a_ready  (),
        .valid    (bridge_output_valid),
        .d_data   (rdata)
    );      
endmodule


/**
 * @module tilelink_ul_master_top
 * @brief Top-level module for a TileLink-UL master.
 * @details This module generates TL-UL requests based on inputs (presumably from a testbench or CPU) and handles responses from the slave.
 */
module tilelink_ul_master_top #( 
    // Core interface widths
    parameter TL_ADDR_WIDTH      = 64,                    //!< Address width
    parameter TL_DATA_WIDTH      = 64,                    //!< Data width
    parameter TL_STRB_WIDTH      = TL_DATA_WIDTH / 8,     //!< Byte mask width/Byte strobe

    // TileLink metadata widths
    parameter TL_SOURCE_WIDTH    = 3,                     //!< Request ID
    parameter TL_SINK_WIDTH      = 3,                     //!< Response ID
    parameter TL_OPCODE_WIDTH    = 3,                     //!< Opcode width
    parameter TL_PARAM_WIDTH     = 3,                     //!< Reserved (0)
    parameter TL_SIZE_WIDTH      = 8,                     //!< log2(size in bytes)
    
    // A Channel Opcodes
    parameter PUT_FULL_DATA_A    = 3'd0,
    parameter PUT_PARTIAL_DATA_A = 3'd1,
    parameter ARITHMETIC_DATA_A  = 3'd2,
    parameter LOGICAL_DATA_A     = 3'd3,
    parameter GET_A              = 3'd4,
    parameter INTENT_A           = 3'd5,
    parameter ACQUIRE_BLOCK_A    = 3'd6,
    parameter ACQUIRE_PERM_A     = 3'd7,

    // D Channel Opcodes
    parameter ACCESS_ACK_D       = 3'd0,
    parameter ACCESS_ACK_DATA_D  = 3'd1,
    parameter HINT_ACK_D         = 3'd2,
    parameter GRANT_D            = 3'd4,
    parameter GRANT_DATA_D       = 3'd5,
    parameter RELEASE_ACK_D      = 3'd6,

    // Master FSM States
    parameter REQUEST            = 2'd1,
    parameter RESPONSE           = 2'd2,
    parameter CLEANUP            = 2'd3,
    parameter IDLE               = 2'd0 
)(
    input  wire                         clk,
    input  wire                         rst,

    // Inputs for commands from testbench
    input  wire                         a_valid_in,
    input  wire [TL_OPCODE_WIDTH-1:0]   a_opcode_in,
    input  wire [TL_PARAM_WIDTH-1:0]    a_param_in,
    input  wire [TL_ADDR_WIDTH-1:0]     a_address_in,
    input  wire [TL_SIZE_WIDTH-1:0]     a_size_in,
    input  wire [TL_STRB_WIDTH-1:0]     a_mask_in,
    input  wire [TL_DATA_WIDTH-1:0]     a_data_in,
    input  wire [TL_SOURCE_WIDTH-1:0]   a_source_in,

    // A Channel: Sent TO SLAVE
    input  wire                         a_ready,        //!< Slave is ready to accept data
    output reg                          a_valid,        //!< Master asserts to send valid request
    output reg  [TL_OPCODE_WIDTH-1:0]   a_opcode,
    output reg  [TL_PARAM_WIDTH-1:0]    a_param,
    output reg  [TL_ADDR_WIDTH-1:0]     a_address,
    output reg  [TL_SIZE_WIDTH-1:0]     a_size,
    output reg  [TL_STRB_WIDTH-1:0]     a_mask,
    output reg  [TL_DATA_WIDTH-1:0]     a_data,
    output reg  [TL_SOURCE_WIDTH-1:0]   a_source,

    // D Channel: Received FROM SLAVE
    input  wire                         d_valid,        //!< Slave has a valid response
    output wire                         d_ready,        //!< Master is ready to accept response
    input  wire [TL_OPCODE_WIDTH-1:0]   d_opcode,
    input  wire [TL_PARAM_WIDTH-1:0]    d_param,
    input  wire [TL_SIZE_WIDTH-1:0]     d_size,
    input  wire [TL_SINK_WIDTH-1:0]     d_sink,
    input  wire [TL_SOURCE_WIDTH-1:0]   d_source,
    input  wire [TL_DATA_WIDTH-1:0]     d_data,
    input  wire                         d_error
);

    reg [1:0] master_state, next_state; //!< FSM state registers

    // State flags
    wire is_request;                    //!< High when in REQUEST state
    wire is_response;                   //!< High when in RESPONSE state
    
    // Registers for holding A-channel request if slave is not ready
    reg         r_a_valid;
    reg [TL_OPCODE_WIDTH-1:0] r_a_opcode;
    reg [TL_PARAM_WIDTH-1:0]  r_a_param;
    reg [TL_ADDR_WIDTH-1:0]   r_a_address;
    reg [TL_SIZE_WIDTH-1:0]   r_a_size;
    reg [TL_STRB_WIDTH-1:0]   r_a_mask;
    reg [TL_DATA_WIDTH-1:0]   r_a_data;
    reg [TL_SOURCE_WIDTH-1:0] r_a_source;

    assign is_request  = (master_state == REQUEST);
    assign is_response = (master_state == RESPONSE);
    
    /**
     * @brief Master FSM state transition logic.
     */
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            master_state <= REQUEST;
        end else begin
            master_state <= next_state;
        end
    end

    /**
     * @brief Master FSM next state logic.
     */
    always @(*) begin
        case (master_state)
            REQUEST: begin
                if (a_valid_in & a_ready) begin
                    next_state = RESPONSE;
                end else begin
                    next_state = REQUEST;
                end
            end
            RESPONSE: begin
                if (d_valid) begin
                    next_state = REQUEST;
                end else begin
                    next_state = RESPONSE;
                end
            end
            default: next_state = REQUEST;
        endcase
    end

    assign d_ready = is_response;
    
    /**
     * @brief Logic to register the A-channel request.
     */
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_a_valid <= 1'b0;
            r_a_opcode <= 0;
            r_a_param  <= 0;
            r_a_address <= 0;
            r_a_size   <= 0;
            r_a_mask   <= 0;
            r_a_data   <= 0;
            r_a_source <= 0; 
        end 
        else if (a_valid_in) begin
            r_a_valid <= a_valid_in;
            r_a_opcode <= a_opcode_in;
            r_a_param  <= a_param_in;
            r_a_address <= a_address_in;
            r_a_size   <= a_size_in;
            r_a_mask   <= a_mask_in;
            r_a_data   <= a_data_in;
            r_a_source <= a_source_in; 
        end
        else if (is_response) begin
            r_a_valid <= 1'b0;
        end
    end

    /**
     * @brief Logic to drive the A-channel outputs.
     */
    always @(*) begin
        if (rst) begin
            a_valid   = 1'b0;
            a_opcode  = 0;
            a_param   = 0;
            a_address = 0;
            a_size    = 0;
            a_mask    = 0;
            a_data    = 0;
            a_source  = 0;
        end else if (a_valid_in) begin
            a_valid   = a_valid_in;
            a_opcode  = a_opcode_in;
            a_param   = a_param_in;
            a_address = a_address_in;
            a_size    = a_size_in;
            a_mask    = a_mask_in;
            a_data    = a_data_in;
            a_source  = a_source_in;
        end else if (is_request) begin
            a_valid   = r_a_valid;
            a_opcode  = r_a_opcode;
            a_param   = r_a_param;
            a_address = r_a_address;
            a_size    = r_a_size;
            a_mask    = r_a_mask;
            a_data    = r_a_data;
            a_source  = r_a_source;
        end else begin
            a_valid   = 1'b0;
        end
    end

endmodule

/**
 * @module tilelink_wrapper_top
 * @brief Top-level wrapper connecting a TileLink master and slave for simulation or integration.
 */
module tilelink_wrapper_top #( 
    parameter TL_ADDR_WIDTH      = 64,
    parameter TL_DATA_WIDTH      = 64,
    parameter TL_STRB_WIDTH      = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH    = 3,
    parameter TL_SINK_WIDTH      = 3,
    parameter TL_OPCODE_WIDTH    = 3,
    parameter TL_PARAM_WIDTH     = 3,
    parameter TL_SIZE_WIDTH      = 8,
    parameter MEM_BASE_ADDR      = 64'h0000_0000_0000_0000,
    parameter DEPTH              = 512
)(
    input  wire                         clk,
    input  wire                         rst,

    // Inputs to drive the master from testbench
    input  wire                         a_valid_in,
    input  wire [TL_OPCODE_WIDTH-1:0]   a_opcode_in,
    input  wire [TL_PARAM_WIDTH-1:0]    a_param_in,
    input  wire [TL_ADDR_WIDTH-1:0]     a_address_in,
    input  wire [TL_SIZE_WIDTH-1:0]     a_size_in,
    input  wire [TL_STRB_WIDTH-1:0]     a_mask_in,
    input  wire [TL_DATA_WIDTH-1:0]     a_data_in,
    input  wire [TL_SOURCE_WIDTH-1:0]   a_source_in,

    // Outputs for testbench visibility
    output wire                         a_valid_tb,
    output wire [TL_OPCODE_WIDTH-1:0]   a_opcode_tb,
    output wire [TL_PARAM_WIDTH-1:0]    a_param_tb,
    output wire [TL_ADDR_WIDTH-1:0]     a_address_tb,
    output wire [TL_SIZE_WIDTH-1:0]     a_size_tb,
    output wire [TL_STRB_WIDTH-1:0]     a_mask_tb,
    output wire [TL_DATA_WIDTH-1:0]     a_data_tb,
    output wire [TL_SOURCE_WIDTH-1:0]   a_source_tb,
    output wire                         a_ready_tb,

    output wire                         d_valid_tb,
    output wire                         d_ready_tb,
    output wire [TL_OPCODE_WIDTH-1:0]   d_opcode_tb,
    output wire [TL_PARAM_WIDTH-1:0]    d_param_tb,
    output wire [TL_SIZE_WIDTH-1:0]     d_size_tb,
    output wire [TL_SINK_WIDTH-1:0]     d_sink_tb,
    output wire [TL_SOURCE_WIDTH-1:0]   d_source_tb,
    output wire [TL_DATA_WIDTH-1:0]     d_data_tb,
    output wire                         d_error_tb
);

    // Internal wires for A and D channels
    wire a_ready;
    wire a_valid;
    wire [TL_OPCODE_WIDTH-1:0] a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  a_param;
    wire [TL_ADDR_WIDTH-1:0]   a_address;
    wire [TL_SIZE_WIDTH-1:0]   a_size;
    wire [TL_STRB_WIDTH-1:0]   a_mask;
    wire [TL_DATA_WIDTH-1:0]   a_data;
    wire [TL_SOURCE_WIDTH-1:0] a_source;

    wire d_valid;
    wire d_ready;
    wire [TL_OPCODE_WIDTH-1:0] d_opcode;
    wire [TL_PARAM_WIDTH-1:0]  d_param;
    wire [TL_SIZE_WIDTH-1:0]   d_size;
    wire [TL_SINK_WIDTH-1:0]   d_sink;
    wire [TL_SOURCE_WIDTH-1:0] d_source;
    wire [TL_DATA_WIDTH-1:0]   d_data;
    wire                       d_error;

    // Assign outputs for testbench
    assign a_valid_tb   = a_valid;
    assign a_opcode_tb  = a_opcode;
    assign a_param_tb   = a_param;
    assign a_address_tb = a_address;
    assign a_size_tb    = a_size;
    assign a_mask_tb    = a_mask;
    assign a_data_tb    = a_data;
    assign a_source_tb  = a_source;
    assign a_ready_tb   = a_ready;

    assign d_valid_tb   = d_valid;
    assign d_ready_tb   = d_ready;
    assign d_opcode_tb  = d_opcode;
    assign d_param_tb   = d_param;
    assign d_size_tb    = d_size;
    assign d_sink_tb    = d_sink;
    assign d_source_tb  = d_source;
    assign d_data_tb    = d_data;
    assign d_error_tb   = d_error;

    /**
     * @brief Instantiation of the TileLink master.
     */
    tilelink_ul_master_top #(
        .TL_ADDR_WIDTH(TL_ADDR_WIDTH),
        .TL_DATA_WIDTH(TL_DATA_WIDTH)
    ) master_inst (
        .clk(clk),
        .rst(rst),
        .a_valid_in(a_valid_in),
        .a_opcode_in(a_opcode_in),
        .a_param_in(a_param_in),
        .a_address_in(a_address_in),
        .a_size_in(a_size_in),
        .a_mask_in(a_mask_in),
        .a_data_in(a_data_in),
        .a_source_in(a_source_in),
        .a_ready(a_ready),
        .a_valid(a_valid),
        .a_opcode(a_opcode),
        .a_param(a_param),
        .a_address(a_address),
        .a_size(a_size),
        .a_mask(a_mask),
        .a_data(a_data),
        .a_source(a_source),
        .d_valid(d_valid),
        .d_ready(d_ready),
        .d_opcode(d_opcode),
        .d_param(d_param),
        .d_size(d_size),
        .d_sink(d_sink),
        .d_source(d_source),
        .d_data(d_data),
        .d_error(d_error)
    );

    /**
     * @brief Instantiation of the TileLink slave.
     */
    tilelink_ul_slave_top #(
        .TL_ADDR_WIDTH(TL_ADDR_WIDTH),
        .TL_DATA_WIDTH(TL_DATA_WIDTH),
        .MEM_BASE_ADDR(MEM_BASE_ADDR),
        .DEPTH(DEPTH)
    ) slave_inst (
        .clk(clk),
        .rst(rst),
        .a_ready(a_ready),
        .a_valid(a_valid),
        .a_opcode(a_opcode),
        .a_param(a_param),
        .a_address(a_address),
        .a_size(a_size),
        .a_mask(a_mask),
        .a_data(a_data),
        .a_source(a_source),
        .d_valid(d_valid),
        .d_ready(d_ready),
        .d_opcode(d_opcode),
        .d_param(d_param),
        .d_size(d_size),
        .d_sink(d_sink),
        .d_source(d_source),
        .d_data(d_data),
        .d_error(d_error)
    );
    
endmodule

