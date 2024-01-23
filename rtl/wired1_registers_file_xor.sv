`include "wired0_defines"

// MULTIPORT REGISTER FILE
// XOR-WPORT

module wired_registers_file_xor #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter int unsigned R_PORT_COUNT = 2,
    parameter int unsigned W_PORT_COUNT = 2,
    parameter REGISTERS_FILE_TYPE = "ff", // optional: ff, latch
    parameter bit NEED_RESET = 0,
    parameter logic[DEPTH-1:0][DATA_WIDTH-1:0] RESET_VAL = '0
    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,
    input    [R_PORT_COUNT-1:0][ADDR_DEPTH-1:0] raddr_i,
    output T [R_PORT_COUNT-1:0]                 rdata_o,

    input    [W_PORT_COUNT-1:0][ADDR_DEPTH-1:0] waddr_i,
    input    [W_PORT_COUNT-1:0]                 we_i,
    input  T [W_PORT_COUNT-1:0]                 wdata_i
);

    T [W_PORT_COUNT-1:0][W_PORT_COUNT-2:0] wdata_old;
    T [W_PORT_COUNT-1:0][R_PORT_COUNT-1:0] rdata_raw;
    T [W_PORT_COUNT-1:0] wdata_xor;
    for(genvar wp = 0 ; wp < W_PORT_COUNT ; wp+=1) begin
        logic [W_PORT_COUNT-2:0][ADDR_DEPTH-1:0] waddr_old;
        for(genvar w = 0 ; w < W_PORT_COUNT ; w+=1) begin
            if(w > wp) begin
                assign waddr_old[w-1] = waddr_i[w];
            end else if(w < wp) begin
                assign waddr_old[w] = waddr_i[w];
            end
        end
        wired_register_file#(
            .DATA_WIDTH,
            .DEPTH(DEPTH),
            .R_PORT_COUNT(R_PORT_COUNT + W_PORT_COUNT - 1),
            .REGISTERS_FILE_TYPE,
            .NEED_RESET,
            .RESET_VAL
        ) slice_regfiles (
            `_WIRED_GENERAL_CONN,
            .raddr_i({raddr_i,       waddr_old}),
            .raddr_o({rdata_raw[wp], wdata_old[wp]}),
            .waddr_i(waddr_i[wp]),
            .we_i(we_i[wp])
            .wdata_i(wdata_xor[wp])
        );
        // WDATA XOR
        always_comb begin
            wdata_xor[wp] = wdata_i[wp];
            for(integer i = 0 ; i < W_PORT_COUNT ; i+=1) begin
                if(i < wp) wdata_xor[wp] ^= wdata_old[i][wp];
                else if(i > wp) wdata_xor[wp] ^= wdata_old[i][wp-1];
            end
        end
    end

    // RDATA XOR
    for(genvar rp = 0 ; rp < R_PORT_COUNT ; rp+=1) begin
        always_comb begin
            rdata_o[rp] = '0;
            for(integer i = 0 ; i < W_PORT_COUNT ; i+=1) begin
                rdata_o[rp] ^= rdata_raw[i][rp];
            end
        end
    end
    

endmodule
