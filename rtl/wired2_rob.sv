`include "wired0_defines"

// Fuction module for Wired project
module wired_rob (
    `_WIRED_GENERAL_DEFINE,

    // 连接到 DISPATCH(P) 级别的端口

    // Part 1: ROB 读端口
    // ROB 中读取到的数据（P 级）
    input  rob_rid_t        [3:0] p_rrrid_i,
    output logic            [3:0] p_rob_valid_o, // PRF 中存储的值是有效的
    output rob_entry_data_t [3:0] p_rrdata_o,

    // Part 2: ROB 写端口（P级）
    input logic              [1:0] p_valid_i, // 即发射信号
    input rob_rid_t          [1:0] p_wrrid_i,
    input rob_entry_static_t [1:0] p_winfo_i,

    // Part 3: ROB 写端口（CDB）
    input pipeline_cdb_t     [1:0] cdb_i,

    // Part 4: ROB 读端口（C级）
    input  rob_rid_t         [1:0] c_rrrid_i,
    output logic             [1:0] c_rob_valid_o,
    output rob_entry_t       [1:0] c_rob_entry_o,

    // 注意，在后端需要撤销时，由 commit 端口对 ROB 中的指令全部执行一次提交，以恢复重命名表的状态
    // 即 RENAME 模块对后端撤销情况并不知情
    input  logic             [1:0] c_retire_i
);

    // 定义四张表
    // static 表，双写双读
    rob_entry_static_t [1:0] c_rob_entry_static;
    wired_registers_file_banked # (
        .DATA_WIDTH($bits(rob_entry_static_t)),
        .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
        .R_PORT_COUNT(2),
        .W_PORT_COUNT(2),
        .NEED_RESET(0)
    )
    rob_static (
        `_WIRED_GENERAL_CONN,
        .raddr_i(c_rrrid_i),
        .rdata_o(c_rob_entry_static),
        .waddr_i(p_wrrid_i),
        .we_i(p_valid_i),
        .wdata_i(p_winfo_i)
    );

    // Data 表
    rob_entry_data_t [3:0] p_rob_entry_data;
    rob_entry_data_t [1:0] c_rob_entry_data;
    wired_registers_file_banked # (
        .DATA_WIDTH($bits(rob_entry_data_t)),
        .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
        .R_PORT_COUNT(6),
        .W_PORT_COUNT(2),
        .NEED_RESET(0)
    )
    rob_data (
        `_WIRED_GENERAL_CONN,
        .raddr_i({c_rrrid_i, p_rrrid_i}),
        .rdata_o({c_rob_entry_data, p_rob_entry_data}),
        .waddr_i({cdb_i[1].wid, cdb_i[0].wid}),
        .we_i({cdb_i[1].valid, cdb_i[0].valid}),
        .wdata_i({cdb_i[1].wdata, cdb_i[0].wdata})
    );

    // Dynamic 表
    rob_entry_dynamic_t [1:0] c_rob_entry_dynamic;
    rob_entry_dynamic_t [1:0] cdb_entry_dynamic;
    for(genvar i = 0 ; i < 2 ; i++) begin
        assign cdb_entry_dynamic[i].excp = cdb[i].excp;
        assign cdb_entry_dynamic[i].need_jump = cdb[i].need_jump;
        assign cdb_entry_dynamic[i].jump_target = cdb[i].jump_target;
        assign cdb_entry_dynamic[i].uncached = cdb[i].uncached;          // 对于 Uncached 的指令，一定会触发流水线冲刷，重新执行，结果直接写入 ARF，不经过 ROB。
        assign cdb_entry_dynamic[i].store_buffer = cdb[i].store_buffer;      // 提交一条 Store_buffer 中的写请求
        assign cdb_entry_dynamic[i].store_conditional = cdb[i].store_conditional; // 条件写，若未命中，则直接失败并冲刷流水线
    end
    wired_registers_file_banked # (
        .DATA_WIDTH($bits(rob_entry_dynamic_t)),
        .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
        .R_PORT_COUNT(2),
        .W_PORT_COUNT(2),
        .NEED_RESET(0)
    )
    rob_dynamic (
        `_WIRED_GENERAL_CONN,
        .raddr_i(c_rrrid_i),
        .rdata_o(c_rob_entry_dynamic),
        .waddr_i({cdb_i[1].wid, cdb_i[0].wid}),
        .we_i({cdb_i[1].valid, cdb_i[0].valid}),
        .wdata_i(cdb_entry_dynamic)
    );

    // Valid 表，分两部分
    logic [1:0] c_rob_valid_q;
    logic [3:0] p_rob_valid_pt, p_rob_valid_ct;
    logic [1:0] c_rob_valid_pt, c_rob_valid_ct;
    assign p_rob_valid_o = p_rob_valid_pt ^ p_rob_valid_ct;
    assign c_rob_valid_o = c_rob_valid_q & (c_rob_valid_pt ^ c_rob_valid_ct);
    logic [1:0] p_rob_last_valid, c_rob_last_valid;
    wired_registers_file_banked # (
        .DATA_WIDTH(1),
        .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
        .R_PORT_COUNT(8),
        .W_PORT_COUNT(2),
        .NEED_RESET(0)
    )
    rob_valid_p (
        `_WIRED_GENERAL_CONN,
        .raddr_i({c_rrrid_i, p_rrrid_i, cdb_i[1].wid, cdb_i[0].wid}),
        .rdata_o({c_rob_valid_pt, p_rob_valid_pt, p_rob_last_valid}),
        .waddr_i(p_wrrid_i),
        .we_i(p_valid_i),
        .wdata_i(c_rob_last_valid)
    );
    wired_registers_file_banked # (
        .DATA_WIDTH(1),
        .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
        .R_PORT_COUNT(8),
        .W_PORT_COUNT(2),
        .NEED_RESET(0)
    )
    rob_valid_p (
        `_WIRED_GENERAL_CONN,
        .raddr_i({c_rrrid_i, p_rrrid_i, p_wrrid_i}),
        .rdata_o({c_rob_valid_ct, p_rob_valid_ct, c_rob_last_valid}),
        .waddr_i({cdb_i[1].wid, cdb_i[0].wid}),
        .we_i({cdb_i[1].valid, cdb_i[0].valid}),
        .wdata_i(~p_rob_last_valid)
    );

    logic[`_WIRED_PARAM_ROB_LEN:0] valid_rob_entry_q;
    logic[`_WIRED_PARAM_ROB_LEN:0] valid_rob_entry_diff;
    assign valid_rob_entry_diff = p_valid_i[1] + p_valid_i[0] - c_retire_i[1] - c_retire_i[0];
    
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            valid_rob_entry_q <= '0;
            c_rob_valid_q <= '0;
        end else begin
            valid_rob_entry_q <= valid_rob_entry_q + valid_rob_entry_diff;
            c_rob_valid_q[0] <= (valid_rob_entry_q + valid_rob_entry_diff) >= 1;
            c_rob_valid_q[1] <= (valid_rob_entry_q + valid_rob_entry_diff) >= 2;
        end
    end

endmodule
