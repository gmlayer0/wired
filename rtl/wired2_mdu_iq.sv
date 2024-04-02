`include "wired0_defines.svh"

// Fuction module for Wired project
// mdu issue queue + mdu
module wired_mdu_iq #(
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
    output pipeline_cdb_t cdb_payload_o,
    output logic          cdb_valid_o,
    input  logic          cdb_ready_i, // 这里可以接 FIFO

    // CDB 嗅探端口
    input pipeline_cdb_t [1:0] cdb_i,

    // FLUSH 端口
    input logic flush_i // 后端正在清洗管线，发射所有指令而不等待就绪
);
    logic [IQ_SIZE-1:0] empty_q; // 标识 IQ ENTRY 可被占用
    logic [IQ_SIZE-1:0] fire_rdy_q;  // 标识 IQ ENTRY 可发射

    // MDU 的 IQ 采用与 ALU 的 IQ 基本相同的乱序发射策略，不过奇怪的是，ALU 之外的 IQ 居然不支持背靠背旁路！是不是很神奇 OvO？

    // 乘法器为完全流水化的，除法器则为多周期（可能可以部分流水）的，采用 valid-ready 握手输入， valid-ready 握手输出。

endmodule
