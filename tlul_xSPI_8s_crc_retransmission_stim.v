module tilelink_ul_bridge_spi_tb #(  // Parameters should go in parameter list
    parameter TL_ADDR_WIDTH   = 64,
    parameter TL_DATA_WIDTH   = 64,
    parameter TL_STRB_WIDTH   = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH = 3,
    parameter TL_SINK_WIDTH   = 3,
    parameter TL_OPCODE_WIDTH = 3,
    parameter TL_PARAM_WIDTH  = 3,
    parameter TL_SIZE_WIDTH   = 8,
    parameter MEM_BASE_ADDR   = 64'd000, // Base address for memory
    parameter DEPTH           = 512,     // Memory depth (number of entries)    
    parameter NUM_SLAVES      = 3
)(
    // Clock and reset
    output reg clk,
    output reg rst,

    // Input stimulus
    output reg                      a_valid_in,
    output reg [TL_OPCODE_WIDTH-1:0] a_opcode_in,
    output reg [TL_PARAM_WIDTH-1:0]  a_param_in,
    output reg [TL_ADDR_WIDTH-1:0]   a_address_in,
    output reg [TL_SIZE_WIDTH-1:0]   a_size_in,
    output reg [TL_STRB_WIDTH-1:0]   a_mask_in,
    output reg [TL_DATA_WIDTH-1:0]   a_data_in,
    output reg [TL_SOURCE_WIDTH-1:0] a_source_in
);

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

    // Stimulus
    always begin
        

        // Initialize
        rst = 1;
        a_valid_in   = 0;
        a_opcode_in  = 0;
        a_param_in   = 0;
        a_address_in = 0;
        a_size_in    = 0;
        a_mask_in    = 0;
        a_data_in    = 0;
        a_source_in  = 0;
    
        // Reset
        #20;
        rst = 0;
        #20;
    
        //////////////////////////
        // PUT_FULL_DATA to slave 0
        //////////////////////////
        @(posedge clk);
        a_valid_in   <= 1;
        a_opcode_in  <= 3'd0; // PUT_FULL_DATA_A
        a_param_in   <= 3'd0;
        a_address_in <= 64'hDEADBEEF_CAFEBABE;
        a_size_in    <= 3'd3; // 8 bytes
        a_mask_in    <= 8'hFF;
        a_data_in    <= 64'hDEADBEEF_CAFEBABE;
        a_source_in  <= 3'd0;
    
        @(posedge clk);
        a_valid_in <= 0;
    

repeat (200) begin        
	@(posedge clk); 
end
    

        @(posedge clk);
    
        //////////////////////////
        // GET from slave 0
        //////////////////////////
        @(posedge clk);
        a_valid_in   <= 1;
        a_opcode_in  <= 3'd4; // GET_A
        a_param_in   <= 3'd0;
        a_address_in <= 64'd0;
        a_size_in    <= 3'd3;
        a_mask_in    <= 8'hFF;
        a_data_in    <= 64'h0; // ignored
        a_source_in  <= 3'd0;
    
        @(posedge clk);
        a_valid_in <= 0;
    
        // Wait for response
repeat (200) begin        
	@(posedge clk); 
end
        @(posedge clk);
    
    
        $finish;
    end


endmodule

