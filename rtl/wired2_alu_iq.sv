`include "wired0_defines"

// Fuction module for Wired project
// alu issue queue + alu
module wired_alu_iq #(
    parameter int IQ_SIZE = `_WIRED_PARAM_INT_IQ_DEPTH
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到 DISPATCH(P) 级别的端口
    input pipeline_ctrl_p_t [1:0] p_ctrl_i,  // 来自 P 级的所有指令信息，全部提供给 ISSUE QUEUE，由 ISSUE QUEUE 进一步处理细分
    input pipeline_data_t   [1:0] p_data_i,  // 注意：这里已经读取过 ROB ，且对来自 CDB 的数据做了转发
    input  logic            [1:0] p_valid_i,
    output logic                  p_ready_o, // 提示 alu_iq 非满，可以接受此两条指令

    // 连接到 CDB ARBITER 的端口，做仲裁(调度为固定优先级别 ALU > LSU > MDU)
    // 因此来自 ALU 的两条指令几乎永远可以同时提交到 CDB
    // 但需要考虑 ROB 的 BANK CONFLICT 问题。
    output pipeline_cdb_t [1:0] cdb_payload_o,
    output logic          [1:0] cdb_valid_o,
    input  logic          [1:0] cdb_ready_i, // 这里可以接 FIFO

    // CDB 嗅探端口
    input pipeline_cdb_data_t [1:0] cdb_i,

    // FLUSH 端口
    input logic flush_i // 后端正在清洗管线，发射所有指令而不等待就绪
);
    logic [IQ_SIZE-1:0] empty_q; // 标识 IQ ENTRY 可被占用
    logic [IQ_SIZE-1:0] issue_rdy_q;  // 标识 IQ ENTRY 可发射
    // Todo: AGE-MAP BASED OPTIMIZATION

    // IQ 有一个比较奇特的设计，整体 IQ 为 8 项，但其中仅有4项是两个 ALU 均可以发射的，其余4项则是独占的
    // 对于填入，优先级倒置

    // 如示例：
    // IQ:   0 1 2 3 4 5 6 7
    // ALU0: 0 1 2 3 4 5
    // ALU1:     5 4 3 2 1 0
    // UPD0: 3 2 1 0 7 6 5 4
    // UPD1: 4 5 6 7 0 1 2 3

    // 也就是每个 ALU 可以执行的直接数据源是 6 项目
    // 仅需要 2 x LUT6 + MUX2 即可完成一个数据源的选择，逻辑深度为3

    // UPD 逻辑
    // IQ:   0 1 2 3 4 5 6 7
    // UPD0: 3 2 1 0 7 6 5 4
    // UPD1: 4 5 6 7 0 1 2 3
    logic [1:0][IQ_SIZE-1:0] upd_sel_oh;
    parameter integer UPDPIO [7:0] = {4,5,6,7,0,1,2,3};
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[0]) : (7-UPDPIO[0])] = empty_q[i == 0 ? (UPDPIO[0]) : (7-UPDPIO[0])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[1]) : (7-UPDPIO[1])] = empty_q[i == 0 ? (UPDPIO[1]) : (7-UPDPIO[1])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[0]) : (7-UPDPIO[0])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[2]) : (7-UPDPIO[2])] = empty_q[i == 0 ? (UPDPIO[2]) : (7-UPDPIO[2])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[1]) : (7-UPDPIO[1])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[3]) : (7-UPDPIO[3])] = empty_q[i == 0 ? (UPDPIO[3]) : (7-UPDPIO[3])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[2]) : (7-UPDPIO[2])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[4]) : (7-UPDPIO[4])] = empty_q[i == 0 ? (UPDPIO[4]) : (7-UPDPIO[4])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[3]) : (7-UPDPIO[3])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[5]) : (7-UPDPIO[5])] = empty_q[i == 0 ? (UPDPIO[5]) : (7-UPDPIO[5])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[4]) : (7-UPDPIO[4])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[6]) : (7-UPDPIO[6])] = empty_q[i == 0 ? (UPDPIO[6]) : (7-UPDPIO[6])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[5]) : (7-UPDPIO[5])];
        assign upd_sel_oh[i][i == 0 ? (UPDPIO[7]) : (7-UPDPIO[7])] = empty_q[i == 0 ? (UPDPIO[7]) : (7-UPDPIO[7])] & ~upd_sel_oh[i][i == 0 ? (UPDPIO[6]) : (7-UPDPIO[6])];
    end

    // CDB 接口上的 FIFO 队列， 不满的时候才可以以发射指令到 FU 执行，一旦有一个 FIFO 满，就阻止指令发射。
    // 这样保证在 ALU 中的两条指令起步走，转发的两个源头也是齐步走的
    logic free_cnt_q;
    // fixme: finish free_cnt_q logic

    // Reserve station static entry 定义
    typedef struct packed {
        alu_grand_op_t gop;
        alu_op_t op;
        logic[31:0] pc;
    } iq_static_t;
    // 输入给 IQ 的 static 信息
    iq_static_t [1:0] p_static;

    // IQ 中存储的信息
    word_t      [IQ_SIZE-1:0][1:0] iq_data;
    iq_static_t [IQ_SIZE-1:0] iq_static;
    logic [IQ_SIZE-1:0][1:0][1:0] b2b_src; // IQ_INDEX REG_INDEX SRC_INDEX

    // 解包信息
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        always_comb begin
            p_static[i].gop = p_ctrl_i[i].decode_info.alu_grand_op;
            p_static[i].op  = p_ctrl_i[i].decode_info.alu_op;
            p_static[i].pc  = p_ctrl_i[i].pc;
        end
    end

    // 例化 Reserve station entry
    for(genvar i = 0 ; i < IQ_SIZE ; i += 1) begin
        wire [1:0] update_by;
        for(genvar j = 0 ; j < IQ_SIZE ; j += 1) begin
            assign update_by[j] = upd_sel_oh[j][i] & p_valid_i[j];
        end
        wired_iq_entry # (
            .CDB_COUNT(2),
            .PAYLOAD_SIZE($bits(iq_static_t)),
            .FORWARD_COUNT(2)
        )
        wired_iq_entry_inst (
            `_WIRED_GENERAL_CONN,
            .sel_i(ready_sel[0][i] | ready_sel[1][i] | flush_i),
            .updata_i(|update_by),
            .data_i(update_by[1] ? p_data_i[1] : p_data_i[0]),
            .payload_i(update_by[0] ? p_static[0] : p_static[1]),
            .b2b_valid_i(),
            .b2b_rid_i(),
            .cdb_i(cdb_i),
            .empty_o(empty_q[i]),
            .ready_o(issue_rdy_q[i]),
            .b2b_sel_o(b2b_src[i]),
            .data_o(iq_data[i]),
            .payload_o(iq_static[i])
        );
    end

    iq_static_t [1:0] sel_static_q, sel_static;
    logic [1:0][1:0] sel_forward_q, sel_forward;
    rob_rid_t [1:0] sel_rid_q, sel_rob;
    word_t [1:0] sel_data_q, sel_data, alu_data;

    // 选择两个用于 ALU 输入的 data 和 static

    // 握手控制，主要与 CDB 有关
    logic [1:0] excute_valid; // 标记 Excute 级的两个执行槽是否有效
    logic [1:0] excute_ready; // 当此信号为高时候，才可以向 Excute 级别写入新的指令

    // 例化两个 ALU 和 jump 模块 用于处理所有计算指令以及分支指令
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        wired_alu  wired_alu_inst (
            .r0_i(),
            .r1_i(),
            .pc_i(),
            .grand_op_i(),
            .op_i(),
            .res_o()
        );
        wired_jump  wired_jump_inst (
            .r0_i(),
            .r1_i(),
            .pc_i(),
            .addr_imm_i(),
            .target_type_i(),
            .cmp_type_i(),
            .jump_o(),
            .jump_target_o()
        );
    end

    // 连接到 CDB 的两个 skid buf

endmodule
