`include "wired0_defines.svh"

// Fuction module for Wired project
module wired_backend #(
    parameter int unsigned SOURCE_WIDTH  = 1,
    parameter int unsigned SINK_WIDTH    = 1,
    parameter int CPU_ID = 0
)(
    `_WIRED_GENERAL_DEFINE,

    input [8:0] interrupt_i, // 输入中断

    // 来自前端（中断由前端输入，打在指令包中）
    input  logic                 pkg_valid_i,
    output logic                 pkg_ready_o,
    input  logic [1:0]            pkg_mask_i,
    input  pipeline_ctrl_pack_t [1:0]  pkg_i,

    // 后端反馈
    output csr_t                       csr_o,
    output tlb_update_t         tlb_update_o,
    output bpu_correct_t       bpu_correct_o,

    // 连接到内存总线（TILELINK-C）
    `TL_DECLARE_HOST_PORT(128, 32, SOURCE_WIDTH, SINK_WIDTH, tl)
);
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
