`include "wired0_defines.svh"

module wired_registers_file #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter int unsigned R_PORT_COUNT = 2,
    parameter int REGISTERS_FILE_TYPE = 0, // optional: 0:ff, 1:latch, 2:fpga
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

    if(REGISTERS_FILE_TYPE == 2 && DEPTH == 64 && !NEED_RESET && !NEED_FORWARD && (R_PORT_COUNT % 7) == 0) begin
        // 使用 7r1w 的寄存器堆构建
        for(genvar i = 0 ; i < R_PORT_COUNT/7 ; i += 1) begin : gen_7r1w_64d
            fpga_ram_7r1w_64d # (
                .WIDTH(DATA_WIDTH)
            )
            fpga_ram_7r1w_64d_inst (
                .clk(clk),
                .addr0(raddr_i[i * 7 + 0]),
                .addr1(raddr_i[i * 7 + 1]),
                .addr2(raddr_i[i * 7 + 2]),
                .addr3(raddr_i[i * 7 + 3]),
                .addr4(raddr_i[i * 7 + 4]),
                .addr5(raddr_i[i * 7 + 5]),
                .addr6(raddr_i[i * 7 + 6]),
                .dout0(rdata_o[i * 7 + 0]),
                .dout1(rdata_o[i * 7 + 1]),
                .dout2(rdata_o[i * 7 + 2]),
                .dout3(rdata_o[i * 7 + 3]),
                .dout4(rdata_o[i * 7 + 4]),
                .dout5(rdata_o[i * 7 + 5]),
                .dout6(rdata_o[i * 7 + 6]),
                .addrw(waddr_i),
                .wea(we_i),
                .din(wdata_i)
            );
        end
    end else if(REGISTERS_FILE_TYPE == 2 && DEPTH == 64 && !NEED_RESET && !NEED_FORWARD && (R_PORT_COUNT % 3) == 0) begin
        // 使用 3r1w 的寄存器堆构建
        for(genvar i = 0 ; i < R_PORT_COUNT/3 ; i += 1) begin : gen_3r1w_64d
            fpga_ram_3r1w_64d # (
                .WIDTH(DATA_WIDTH)
            )
            fpga_ram_3r1w_64d_inst (
                .clk(clk),
                .addr0(raddr_i[i * 3 + 0]),
                .addr1(raddr_i[i * 3 + 1]),
                .addr2(raddr_i[i * 3 + 2]),
                .dout0(rdata_o[i * 3 + 0]),
                .dout1(rdata_o[i * 3 + 1]),
                .dout2(rdata_o[i * 3 + 2]),
                .addrw(waddr_i),
                .wea(we_i),
                .din(wdata_i)
            );
        end
    end else if(REGISTERS_FILE_TYPE == 2 && DEPTH == 32 && !NEED_RESET && !NEED_FORWARD && (R_PORT_COUNT % 7) == 0) begin
        // 使用 7r1w 的寄存器堆构建
        for(genvar i = 0 ; i < R_PORT_COUNT/7 ; i += 1) begin : gen_7r1w_32d
            fpga_ram_7r1w_32d # (
                .WIDTH(DATA_WIDTH)
            )
            fpga_ram_7r1w_32d_inst (
                .clk(clk),
                .addr0(raddr_i[i * 7 + 0]),
                .addr1(raddr_i[i * 7 + 1]),
                .addr2(raddr_i[i * 7 + 2]),
                .addr3(raddr_i[i * 7 + 3]),
                .addr4(raddr_i[i * 7 + 4]),
                .addr5(raddr_i[i * 7 + 5]),
                .addr6(raddr_i[i * 7 + 6]),
                .dout0(rdata_o[i * 7 + 0]),
                .dout1(rdata_o[i * 7 + 1]),
                .dout2(rdata_o[i * 7 + 2]),
                .dout3(rdata_o[i * 7 + 3]),
                .dout4(rdata_o[i * 7 + 4]),
                .dout5(rdata_o[i * 7 + 5]),
                .dout6(rdata_o[i * 7 + 6]),
                .addrw(waddr_i),
                .wea(we_i),
                .din(wdata_i)
            );
        end
    end else if(REGISTERS_FILE_TYPE == 2 && DEPTH == 32 && !NEED_RESET && !NEED_FORWARD && (R_PORT_COUNT % 3) == 0) begin
        // 使用 3r1w 的寄存器堆构建
        for(genvar i = 0 ; i < R_PORT_COUNT/3 ; i += 1) begin : gen_3r1w_32d
            fpga_ram_3r1w_32d # (
                .WIDTH(DATA_WIDTH)
            )
            fpga_ram_3r1w_32d_inst (
                .clk(clk),
                .addr0(raddr_i[i * 3 + 0]),
                .addr1(raddr_i[i * 3 + 1]),
                .addr2(raddr_i[i * 3 + 2]),
                .dout0(rdata_o[i * 3 + 0]),
                .dout1(rdata_o[i * 3 + 1]),
                .dout2(rdata_o[i * 3 + 2]),
                .addrw(waddr_i),
                .wea(we_i),
                .din(wdata_i)
            );
        end
    end else begin
        // 使用 flipflop 构建
        wire [DEPTH-1:0][DATA_WIDTH-1:0] regfiles;
        if(REGISTERS_FILE_TYPE == 0 || REGISTERS_FILE_TYPE == 2) begin
            wired_registers_file_ff #(
                .DATA_WIDTH(DATA_WIDTH),
                .DEPTH(DEPTH),
                .NEED_RESET(NEED_RESET),
                .RESET_VAL(RESET_VAL)
            ) regcore_ff (
                `_WIRED_GENERAL_CONN,
                .waddr_i,
                .we_i,
                .wdata_i,
                // outport
                .regfiles_o(regfiles)
            );
        end else if(REGISTERS_FILE_TYPE == 1) begin
            wired_registers_file_latch #(
                .DATA_WIDTH(DATA_WIDTH),
                .DEPTH(DEPTH),
                .NEED_RESET(NEED_RESET),
                .RESET_VAL(RESET_VAL)
            ) regcore_latch (
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
    end
endmodule
