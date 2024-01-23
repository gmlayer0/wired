`include "wired0_defines"

// BANKED REGISTER FILE
// BANKED-WPORT

module wired_registers_file_banked #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter int unsigned R_PORT_COUNT = 2,
    parameter int unsigned W_PORT_COUNT = 2,
    parameter REGISTERS_FILE_TYPE = "ff", // optional: ff, latch
    parameter bit NEED_RESET = 0,
    parameter logic[DEPTH-1:0][DATA_WIDTH-1:0] RESET_VAL = '0
    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH    = (DEPTH > 1) ? $clog2(DEPTH) : 1
    parameter int unsigned BADDR_DEPTH   = $clog2(W_PORT_COUNT)
)(
    `_WIRED_GENERAL_DEFINE,
    input    [R_PORT_COUNT-1:0][ADDR_DEPTH-1:0] raddr_i,
    output T [R_PORT_COUNT-1:0]                 rdata_o,

    input    [W_PORT_COUNT-1:0][ADDR_DEPTH-1:0] waddr_i,
    input    [W_PORT_COUNT-1:0]                 we_i,
    input  T [W_PORT_COUNT-1:0]                 wdata_i
);

    // RPORT
    wire [R_PORT_COUNT-1:0][ADDR_DEPTH-1:BADDR_DEPTH] raddr;
    wire [W_PORT_COUNT-1:0][R_PORT_COUNT-1:0][DATA_WIDTH-1:0] rdata;
    for(genvar r = 0 ; r < R_PORT_COUNT ; r += 1) begin
        assign raddr[r] = raddr_i[r][ADDR_DEPTH-1:BADDR_DEPTH];
        assign rdata_o[r] = rdata[raddr_i[BADDR_DEPTH-1:0]][r];
    end

    for(genvar b = 0 ; b < W_PORT_COUNT ; b += 1) begin
        logic [ADDR_DEPTH-1:BADDR_DEPTH] waddr;
        logic we;
        logic [DATA_WIDTH-1:0] wdata;
        always_comb begin
            waddr = '0;
            we = '0;
            wdata = '0;
            for(genvar p = 0 ; p < W_PORT_COUNT ; p += 1) begin
                if(waddr_i[p][BADDR_DEPTH-1:0] == b[BADDR_DEPTH-1:0] && we_i[p]) begin
                    waddr |= waddr_i[p][ADDR_DEPTH-1:BADDR_DEPTH];
                    we    |= '1;
                    wdata |= wdata_i[p];
                end
            end
        end
        wired_register_file#(
            .DATA_WIDTH,
            .DEPTH(DEPTH/W_PORT_COUNT),
            .R_PORT_COUNT,
            .REGISTERS_FILE_TYPE,
            .NEED_RESET,
            .RESET_VAL
        ) slice_regfiles (
            `_WIRED_GENERAL_CONN,
            .raddr_i(raddr),
            .raddr_o(rdata[b]),

            .waddr_i(waddr),
            .we_i(we)
            .wdata_i(wdata)
        );
    end

endmodule
