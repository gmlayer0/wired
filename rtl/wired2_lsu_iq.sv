`include "wired0_defines.svh"

// Fuction module for Wired project
// lsu issue queue + lsu
module wired_lsu_iq #(
    parameter int IQ_SIZE = `_WIRED_PARAM_LSU_IQ_DEPTH
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

    // 注意，LSU 的 iq 是完全顺序发射的
    // 实际上被实现为一个 FIFO，有队顶指针，队尾指针，分配策略与 ALU 与 MDU 的 IQ 存在本质不同。
    localparam integer PTR_LEN = $clog2(IQ_SIZE);
    logic [PTR_LEN-1:0] iq_head_q, iq_head_p1_q, iq_tail_q;
    logic [PTR_LEN:0] iq_cnt_q;
    logic [PTR_LEN:0] iq_cnt;
    logic [PTR_LEN-1:0] iq_head, iq_head_p1, iq_tail;
    wire top_valid, top_ready, top_fire;
    assign top_fire = top_valid & top_ready;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            iq_head_q    <= '0;
            iq_head_p1_q <= 1;
            iq_tail_q <= '0;
            iq_cnt_q  <= '0;
            p_ready_o <= '1;
        end else begin
            iq_head_q    <= iq_head;
            iq_head_p1_q <= iq_head_p1;
            iq_tail_q <= iq_tail;
            iq_cnt_q  <= iq_cnt;
            p_ready_o <= iq_cnt <= (IQ_SIZE-2);
        end
    end
    always_comb begin
        iq_head = iq_head_q;
        iq_head_p1 = iq_head_p1_q;
        if(p_ready_o) begin
            iq_head += p_valid_i[0] + p_valid_i[1];
            iq_head_p1 += p_valid_i[0] + p_valid_i[1];
        end
    end
    always_comb begin
        iq_tail = iq_tail_q;
        if(top_fire) begin
            iq_tail += 1;
        end
    end
    always_comb begin
        iq_cnt = iq_cnt_q;
        if(p_ready_o) begin
            iq_cnt += p_valid_i[0] + p_valid_i[1];
        end
        if(top_fire) begin
            iq_cnt -= 1;
        end
    end
    // Reserve station static entry 定义
    typedef struct packed {
        decode_info_alu_t di;
        logic[31:0] pc;
    } iq_static_t;
    // 输入给 IQ 的 static 信息
    iq_static_t [1:0] p_static;
    // IQ 中存储的信息
    word_t      [IQ_SIZE-1:0][1:0] iq_data;
    iq_static_t [IQ_SIZE-1:0] iq_static;
    // 解包信息
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        always_comb begin
            p_static[i].di  = get_alu_from_p(p_ctrl_i[i].di);
            p_static[i].pc  = p_ctrl_i[i].pc;
        end
    end


    // 例化 Reserve station entry
    for(genvar i = 0 ; i < IQ_SIZE ; i += 1) begin
        wire [1:0] update_by;
            assign update_by[0] = iq_head_q == i[PTR_LEN-1:0];
            assign update_by[1] = iq_head_p1_q == i[PTR_LEN-1:0];
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
            .ready_o(fire_rdy_q[i]),
            .b2b_sel_o(b2b_src[i]),
            .data_o(iq_data[i]),
            .payload_o(iq_static[i])
        );
    end

endmodule
