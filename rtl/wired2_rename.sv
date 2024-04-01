`include "wired0_defines.svh"

// Fuction module for Wired project
// This module take two inst register infomation as input(combinational)
// And generate all register id after rename.
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
    output    logic       r_ready_o,     // 发射就绪，注意，发射条件为 指令到达 P 级时，保证依然有两个 ROB 表项可用
    input arch_rid_t[1:0] r_warid_i,
    output rob_rid_t[1:0] r_wrrid_o,
    output    logic [1:0] r_tier_id_o,   // 提交时使用的 tier id

    // 连接到 COMMIT 级别的端口
    input     logic [1:0] c_retire_i,
    input arch_rid_t[1:0] c_warid_i,
    input  rob_rid_t[1:0] c_wrrid_i,
    input     logic [1:0] c_tier_id_i,

    // 注意，在后端需要撤销时，由 commit 端口对 ROB 中的指令全部执行一次提交，以恢复重命名表的状态
    // 即 RENAME 模块对后端撤销情况并不知情
    input     logic       c_flush_i,     // 正在清空管线中的指令并恢复 rename_table 状态
    output    logic       empty_o        // ROB 清空信号
);
    logic[1:0] r_issue; // 指令实际发射
    assign r_issue = {r_ready_o, r_ready_o} & r_issue_i;

    // ROB 计数逻辑
    // 注意 ROB 共计 64 项目，SKIDBUF 中可能存储两条指令
    // ROB 中占用表项小于等于 62 为发射的前提条件
    // 打一拍，占用表项小于等于 60 为下一拍可以发射的前提条件
    logic[`_WIRED_PARAM_ROB_LEN:0] rob_count_q, rob_count;
    logic[`_WIRED_PARAM_ROB_LEN:0] rob_diff;
    always_comb begin
        rob_diff = r_issue[0] + r_issue[1] - c_retire_i[0] - c_retire_i[1];
    end
    always_comb begin
        rob_count = rob_diff + rob_count_q;
    end
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            rob_count_q <= '0;
        end else begin
            rob_count_q <= rob_count;
        end
    end

    // r_ready_o 生成逻辑
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            r_ready_o <= '1;
        end else begin
            r_ready_o <= rob_count_q <= 7'd60 && (!c_flush_i);
        end
    end

    // empty_o 生成逻辑
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            empty_o <= '1;
        end else begin
            if(empty_o) begin
                if(r_issue) begin
                    empty_o <= '0;
                end
            end else begin
                if(rob_count == '0) begin
                    empty_o <= '1;
                end
            end
        end
    end

    typedef struct packed {
        logic tier_id;
        logic[`_WIRED_PARAM_ROB_LEN-1:0] rob_id;
    } rename_entry_t;
    // Rename 级别表
    rename_entry_t [3:0] r_rename_result;
    rename_entry_t [1:0] r_rename_new;
    // Commit 级别表
    rename_entry_t [3:0] cr_rename_result;
    rename_entry_t [1:0] cw_rename_result;
    rename_entry_t [1:0] c_rename_new;
    // 读端口处理
    for(genvar p = 0 ; p < 4 ; p += 1) begin
        assign r_rrrid_o[p] = r_rename_result[p].rob_id;
        assign r_prf_valid_o[p] = r_rename_result[p] == cr_rename_result[p];
    end
    // 写端口处理（rob号分配）
    logic[`_WIRED_PARAM_ROB_LEN-1:0] free_rob_q, free_rob_p1_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            free_rob_q <= '0;
            free_rob_p1_q <= `_WIRED_PARAM_ROB_LEN'd1;
        end else begin
            free_rob_q <= free_rob_q + r_issue[0] + r_issue[1];
            free_rob_p1_q <= free_rob_p1_q + r_issue[0] + r_issue[1];
        end
    end
    assign r_rename_new[0].rob_id = free_rob_q;
    assign r_rename_new[1].rob_id = free_rob_p1_q;
    for(genvar p = 0 ; p < 2 ; p += 1) begin
        assign r_rename_new[p].tier_id = ~cw_rename_result[p].tier_id;
        assign r_wrrid_o[p] = r_rename_new[p].rob_id;
        assign r_tier_id_o[p] = r_rename_new[p].tier_id;

        assign c_rename_new[p].tier_id = c_tier_id_i[p];
        assign c_rename_new[p].rob_id = c_wrrid_i[p];
    end

    // Rename 模块内部有两张表，一张是 Rename 级的重命名映射表，一张是提交级的重命名映射表
    // 两张表构建一个 arch_rid -> {tier_id, rob_id}  的映射，当两张表映射结果一致时，说明此时 PRF 中的数据是最新的有效数据

    // Rename 级别表
    wired_registers_file_banked # (
        .DATA_WIDTH(`_WIRED_PARAM_ROB_LEN + 1),
        .DEPTH(1 << `_WIRED_PARAM_PRF_LEN),
        .R_PORT_COUNT(4),
        .W_PORT_COUNT(2),
        .REGISTERS_FILE_TYPE("ff"),
        .NEED_RESET(1)
    )
    r_rename_table (
        `_WIRED_GENERAL_CONN,
        .raddr_i(r_rarid_i),
        .rdata_o(r_rename_result),
        .waddr_i(r_warid_i),
        .we_i(r_issue),
        .wdata_i(r_rename_new)
    );
    // Commit 级别表
    wired_registers_file_banked # (
        .DATA_WIDTH(`_WIRED_PARAM_ROB_LEN + 1),
        .DEPTH(1 << `_WIRED_PARAM_PRF_LEN),
        .R_PORT_COUNT(6),
        .W_PORT_COUNT(2),
        .REGISTERS_FILE_TYPE("ff"),
        .NEED_RESET(1)
    )
    c_rename_table (
        `_WIRED_GENERAL_CONN,
        .raddr_i({r_rarid_i,r_warid_i}),
        .rdata_o({cr_rename_result, cw_rename_result}),
        .waddr_i(c_warid_i),
        .we_i(c_retire_i),
        .wdata_i(c_rename_new)
    );

endmodule
