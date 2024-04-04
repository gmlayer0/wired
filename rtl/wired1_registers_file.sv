`include "wired0_defines.svh"

module wired_registers_file #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter int unsigned R_PORT_COUNT = 2,
    parameter REGISTERS_FILE_TYPE = "ff", // optional: ff, latch
    parameter bit NEED_RESET = 0,
    parameter bit NEED_FORWARD = 0,
    parameter logic[DEPTH-1:0][DATA_WIDTH-1:0] RESET_VAL = '0,
    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,
    input    [R_PORT_COUNT-1:0][ADDR_DEPTH-1:0] raddr_i,
    output T [R_PORT_COUNT-1:0]                 rdata_o,

    input    [ADDR_DEPTH-1:0] waddr_i,
    input                        we_i,
    input  T                  wdata_i
);

    wire [DEPTH-1:0][DATA_WIDTH-1:0] regfiles;
    if(REGISTERS_FILE_TYPE == "ff") begin
        wired_registers_file_ff #(
            .DATA_WIDTH,
            .DEPTH,
            .NEED_RESET,
            .RESET_VAL
        ) regcore (
            `_WIRED_GENERAL_CONN,
            .waddr_i,
            .we_i,
            .wdata_i,
            // outport
            .regfiles_o(regfiles)
        );
    end else if(REGISTERS_FILE_TYPE == "latch") begin
        wired_registers_file_latch #(
            .DATA_WIDTH,
            .DEPTH,
            .NEED_RESET,
            .RESET_VAL
        ) regcore (
            `_WIRED_GENERAL_CONN,
            .waddr_i,
            .we_i,
            .wdata_i,
            // outport
            .regfiles_o(regfiles)
        );
    end

    // Read port generation
    for(genvar i = 0 ; i < R_PORT_COUNT ; i++) begin
        assign rdata_o[i] = (NEED_FORWARD && we_i && raddr_i[i] == waddr_i) ? wdata_i : regfiles[raddr_i[i]];
    end

endmodule
