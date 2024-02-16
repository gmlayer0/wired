`include "wired0_defines"

// Fuction module for Wired project
module wired_rob #(
    parameter int INST_PAYLOAD_SIZE = 128
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到 DISPATCH(P) 级别的端口

    // Part 1: ROB 读端口
    // ROB 中读取到的数据
    input  rob_rid_t        [3:0] p_rrrid_i,
    output logic            [3:0] p_rob_valid_o, // PRF 中存储的值是有效的
    output rob_entry_data_t [3:0] p_rrdata_o,

    // Part 2: ROB 写端口（P级）
    input logic              [1:0] p_valid_i,
    input rob_rid_t          [1:0] p_wrrid_i,
    input rob_entry_static_t [1:0] p_winfo_i,

    // Part 3: ROB 写端口（CDB）
    input pipeline_cdb_t [1:0] cdb_i,

    // Part 4: ROB 读端口（C级）
    

    // 注意，在后端需要撤销时，由 commit 端口对 ROB 中的指令全部执行一次提交，以恢复重命名表的状态
    // 即 RENAME 模块对后端撤销情况并不知情
);

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
            free_rob_q <= free_rob_q + r_issue_i[0] + r_issue_i[1];
            free_rob_p1_q <= free_rob_p1_q + r_issue_i[0] + r_issue_i[1];
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
        .we_i(r_issue_i),
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
        .we_i(c_commit_i),
        .wdata_i(c_rename_new)
    );

endmodule
