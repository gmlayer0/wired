`include "wired0_defines"

// Fuction module for Wired project
// This module take two inst register infomation as input(combinational)
module wired_backend #(
    parameter CPU_ID = 1'd0
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到前端（中断由前端输入，打在指令包中）

    // 连接到内存总线（TILELINK-C）

);

    /* 解码级 / WAW 冲突解除 D */
    
    // --- 来自前端的输入及握手 ---
    logic [1:0] fd_valid, fd_ready, dr_valid, dr_ready;
    // --- 插入特殊的 FIFO，阻断前后端握手长延迟 ---
    logic [1:0][31:0] d_inst_q, d_inst;
    // --- 输出的 payload 定义 ---
    logic [1:0] d_decode_err;
    d_t [1:0] d_decode_info;
    for(genvar i = 0 ; i < 2 ; i++) begin : gen_decoder
        wired_decoder decoder(
            .inst_i(d_inst[i]),
            .decode_err_o(d_decode_err[i]),
            .is_o(d_decode_info[i])
        );
    end

    /* 重命名级 R */

    // --- ARF ---
    // 连接到提交级的信号
    logic [1:0]       r_we;
    arch_rid_t [1:0]  r_waddr;
    logic [1:0][31:0] r_wdata;
    wired_registers_file_banked # (
        .DATA_WIDTH(32),
        .DEPTH(32),
        .R_PORT_COUNT(4),
        .W_PORT_COUNT(2),
    )
    arf (
        `_WIRED_GENERAL_CONN,
        .raddr_i(),
        .rdata_o(),
        .waddr_i(r_waddr),
        .we_i(r_we),
        .wdata_i(r_wdata)
    );

    // ROB (分发 / 提交级别)

    // Reserve Station（分发 / ROB 写回）

    // 执行单元

endmodule
