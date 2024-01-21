`include "wired0_defines"

// Fuction module for Wired project
// This module take two inst register infomation as input(combinational)
module wired_rename #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,

    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到 RENAME 级别的端口
    input arch_rid_t[3:0] r_rarid_i,
    output rob_rid_t[3:0] r_rrrid_o,
    output    logic [3:0] r_prf_valid_o, // PRF 中存储的值是有效的

    input     logic [1:0] r_issue_i,
    input arch_rid_t[1:0] r_warid_i,
    output rob_rid_t[1:0] r_wrrid_o,
    output    logic [1:0] r_tier_id_o    // 提交时使用的 tier id

    // 连接到 COMMIT 级别的端口
    input     logic [1:0] c_commit_i,
    input arch_rid_t[1:0] c_warid_i,
    input  rob_rid_t[1:0] c_wrrid_i,
    input     logic [1:0] c_tier_id_i

    // 注意，在后端需要撤销时，由 commit 端口对 ROB 中的指令全部执行一次提交，以恢复重命名表的状态

);

    // Rename 模块内部有两张表，一张是 Rename 级的重命名映射表，一张是提交级的重命名映射表
    // 两张表构建一个 arch_rid -> {tier_id, rob_id}  的映射，当两张表映射结果一致时，说明此时 PRF 中的数据是最新的有效数据

    // Rename 级别表
    wired_registers_file_banked # (
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(DEPTH),
        .R_PORT_COUNT(R_PORT_COUNT),
        .W_PORT_COUNT(W_PORT_COUNT),
        .REGISTERS_FILE_TYPE(REGISTERS_FILE_TYPE),
        .NEED_RESET(NEED_RESET),
        .DEPTH(DEPTH),
        .ADDR_DEPTH(ADDR_DEPTH)
    )
    r_rename_table (
        ._WIRED_GENERAL_DEFINE(_WIRED_GENERAL_DEFINE),
        .raddr_i(raddr_i),
        .rdata_o(rdata_o),
        .waddr_i(waddr_i),
        .we_i(we_i),
        .wdata_i(wdata_i)
    );

    // Commit 级别表
    wired_registers_file_banked # (
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(DEPTH),
        .R_PORT_COUNT(R_PORT_COUNT),
        .W_PORT_COUNT(W_PORT_COUNT),
        .REGISTERS_FILE_TYPE(REGISTERS_FILE_TYPE),
        .NEED_RESET(NEED_RESET),
        .DEPTH(DEPTH),
        .ADDR_DEPTH(ADDR_DEPTH)
    )
    w_rename_table (
        ._WIRED_GENERAL_DEFINE(_WIRED_GENERAL_DEFINE),
        .raddr_i(raddr_i),
        .rdata_o(rdata_o),
        .waddr_i(waddr_i),
        .we_i(we_i),
        .wdata_i(wdata_i)
    );

endmodule
