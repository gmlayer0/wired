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
    output                        p_ready_o, // 提示 alu_iq 非满，可以接受此两条指令

    // 连接到 CDB ARBITER 的端口，做仲裁(调度为固定优先级别 LSU > MDU > ALU)
    output pipeline_cdb_t [1:0] cdb_payload_o,
    output logic          [1:0] cdb_valid_o,
    input  logic          [1:0] cdb_ready_i, // 这里可以接 SKID BUF

    // CDB 嗅探端口
    input pipeline_cdb_t [1:0] cdb_i,

    // FLUSH 端口
    input logic flush_i // 后端正在清洗管线，发射所有指令而不等待就绪
);
    logic [IQ_SIZE-1:0] empty_q; // 标识 IQ ENTRY 被占用
    logic [IQ_SIZE-1:0] ready_q;  // 标识 IQ ENTRY 可以发射

    // IQ 分配逻辑，固定优先级调度，从队首 / 队尾两个方向分别寻找 leading zero
    logic [1:0][IQ_SIZE-1:0] empty_sel;
    `_WIRED_GET_ONEHOT(empty_q, empty_sel[0], IQ_SIZE)
    `_WIRED_GET_ONEHOT_REVERSE(empty_q, empty_sel[1], IQ_SIZE)

    // IQ 选择逻辑，固定优先级调度，从队首 / 队尾两个方向分别寻找 leading one
    // 若两个寻找到的结果一致，则单发射，否之双发射成功
    logic [1:0][IQ_SIZE-1:0] ready_sel;
    `_WIRED_GET_ONEHOT(ready_q, ready_sel[0], IQ_SIZE)
    `_WIRED_GET_ONEHOT_REVERSE(ready_q, ready_sel[1], IQ_SIZE)

    // 连接到 ALU 的接口
    logic [1:0] fu_valid;
    assign fu_valid[0] = |ready_q;
    assign fu_valid[1] = ready_sel[0] != ready_sel[1];
    

endmodule
