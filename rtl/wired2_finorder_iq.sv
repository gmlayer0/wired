`include "wired0_defines.svh"

// Fuction module for Wired project
// FPU inorder issue queue
module wired_finorder_iq #(
    parameter int IQ_SIZE = 4, // 不用很大，4 项即可
    parameter int WAKEUP_SRC_CNT = 1
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到 DISPATCH(P) 级别的端口
    input pipeline_ctrl_p_t       p_ctrl_i,  // 来自 P 级的所有指令信息，全部提供给 ISSUE QUEUE，由 ISSUE QUEUE 进一步处理细分
    input pipeline_data_t         p_data_i,  // 注意：这里已经读取过 ROB ，且对来自 CDB 的数据做了转发
    input  logic                  p_valid_i,
    output logic                  p_ready_o, // 提示 alu_iq 非满，可以接受此两条指令

    // 连接到 CDB ARBITER 的端口，做仲裁(调度为固定优先级别 ALU > LSU > MDU > FCC > FPU)
    // 因此来自 ALU 的两条指令几乎永远可以同时提交到 CDB
    // 但需要考虑 ROB 的 BANK CONFLICT 问题。
    output pipeline_cdb_t cdb_o,
    input  logic          cdb_ready_i, // 这里可以接 FIFO

    // CDB 嗅探端口
    input pipeline_cdb_t [1:0] cdb_i,

    // FLUSH 端口
    input logic flush_i, // 后端正在清洗管线，发射所有指令而不等待就绪
    input logic fcc_i,   // 后端记录的正确 fcc

    // 执行单元端口
    output logic         ex_valid_o,
    input  logic         ex_ready_i,
    output iq_fcc_req_t  ex_req_o,

    input  logic         ex_valid_i,
    output logic         ex_ready_o,
    input  iq_fcc_resp_t ex_resp_i

    // 以下为可选端口
    ,input  logic     [WAKEUP_SRC_CNT-1:0] wkup_valid_i
    ,input  rob_rid_t [WAKEUP_SRC_CNT-1:0] wkup_rid_i
    ,input  logic     [WAKEUP_SRC_CNT-1:0][31:0] wkup_data_i
);
    logic [IQ_SIZE-1:0] empty_q; // 标识 IQ ENTRY 可被占用
    logic [IQ_SIZE-1:0] fire_rdy_q;  // 标识 IQ ENTRY 可发射

    // 注意，此 iq 是完全顺序发射的
    // 实际上被实现为一个 FIFO，有队顶指针，队尾指针，分配策略与 ALU 与 MDU 的 IQ 存在本质不同。
    localparam integer PTR_LEN = $clog2(IQ_SIZE);
    logic [PTR_LEN-1:0] iq_head_q, iq_tail_q;
    logic [PTR_LEN:0] iq_cnt_q;
    logic [PTR_LEN:0] iq_cnt;
    logic [PTR_LEN-1:0] iq_head, iq_tail;
    wire top_valid, top_ready, top_fire; // TODO: CONNECT TOP_READY TO LSU
    assign top_fire = top_valid & top_ready;
    assign top_valid = fire_rdy_q[iq_tail_q];
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            iq_head_q    <= '0;
            iq_tail_q <= '0;
            iq_cnt_q  <= '0;
            p_ready_o <= '1;
        end else begin
            iq_head_q    <= iq_head;
            iq_tail_q <= iq_tail;
            iq_cnt_q  <= iq_cnt;
            p_ready_o <= iq_cnt <= (IQ_SIZE-1);
        end
    end
    always_comb begin
        iq_head = iq_head_q;
        if(p_ready_o) begin
            iq_head += p_valid_i;
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
            iq_cnt += p_valid_i;
        end
        if(top_fire) begin
            iq_cnt -= 1;
        end
    end
    // Reserve station static entry 定义
    typedef struct packed {
        decode_info_fcc_t di;
        logic [31:0] pc;
        logic[27:0] addr_imm;
        rob_rid_t wreg;
    } iq_static_t;
    // 输入给 IQ 的 static 信息
    iq_static_t p_static;
    // IQ 中存储的信息
    word_t      [IQ_SIZE-1:0][1:0] iq_data;
    iq_static_t [IQ_SIZE-1:0] iq_static;
    word_t [1:0] s_data;
    iq_static_t  s_static;
        logic [IQ_SIZE-1:0][1:0][WAKEUP_SRC_CNT-1:0] wkup_src; // IQ_INDEX REG_INDEX STACK_IDX SRC_INDEX
        logic [1:0][WAKEUP_SRC_CNT-1:0] s_wkup_src;
        assign s_wkup_src = wkup_src[iq_tail_q];
    assign s_data = iq_data[iq_tail_q];
    assign s_static = iq_static[iq_tail_q];
    // 解包信息
    always_comb begin
        p_static.di       = get_fcc_from_p(p_ctrl_i.di);
        p_static.pc       = p_ctrl_i.pc;
        p_static.wreg     = p_ctrl_i.wreg.rob_id;
    end

    // 例化 Reserve station entry
    for(genvar i = 0 ; i < IQ_SIZE ; i += 1) begin
        wire update_by;
        wire is_top;
        assign is_top = iq_head_q == i[PTR_LEN-1:0];
            assign update_by = is_top && p_valid_i;
        wired_iq_entry # (
            .CDB_COUNT(2),
            .PAYLOAD_SIZE($bits(iq_static_t))
        )
        wired_iq_entry_inst (
            `_WIRED_GENERAL_CONN,
            .sel_i((iq_tail_q == i[PTR_LEN-1:0] && fire_rdy_q[i] && top_ready) | flush_i),
            .updata_i(update_by),
            .data_i(p_data_i),
            .payload_i(p_static),
            .wkup_valid_i('0),
            .wkup_rid_i('0),
            .wkup_sel_o(),
            .cdb_i(cdb_i),
            .empty_o(empty_q[i]),
            .ready_mask_i('0),
            .ready_o(fire_rdy_q[i]),
            .data_o(iq_data[i]),
            .payload_o(iq_static[i])
        );
    end

    // 连接 FU
    word_t [1:0] real_data;
    iq_static_t s_iq_q;
    logic s_top_valid_q;
    logic s_top_ready;
    assign top_ready = s_top_ready | !s_top_valid_q;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            s_top_valid_q <= '0;
        end else if(top_ready) begin
            s_top_valid_q <= top_valid;
        end
    end
    // rob_rid_t lsu_resp_rid;
    // wired_fifo #(
    //     .DATA_WIDTH($bits(rob_rid_t)), // rid, wdata, jumppc, jump
    //     .DEPTH(8) // FIXME: FIND MIN-VALUE
    // ) wired_rid_fifo (
    //     .clk(clk),
    //     .rst_n(rst_n && !flush_i),
    //     .inport_valid_i(lsu_req_valid_o & lsu_req_ready_i),
    //     .inport_ready_o(),
    //     .inport_payload_i(s_iq_q.wreg),
    //     .outport_valid_o(),
    //     .outport_ready_i(lsu_resp_valid_i & lsu_resp_ready_o),
    //     .outport_payload_o(lsu_resp_rid)
    // );
    always_ff @(posedge clk) begin
        if(top_ready) begin
            // real_data <= s_data;
            s_iq_q <= s_static;
        end
    end
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        wired_wkupdreg # (
            .WAKEUP_SRC_CNT(WAKEUP_SRC_CNT)
            )
            wired_wkupdreg_inst (
                `_WIRED_GENERAL_CONN,
                .ready_i(top_ready),
                .wkup_src_i(s_wkup_src[i]),
                .data_i(s_data[i]),
                .wkup_data_i(wkup_data_i),
                .data_o(real_data[i])
            );
    end
    assign ex_valid_o        = s_top_valid_q;
    assign s_top_ready       = ex_ready_i;
    assign ex_req_o.cond     = s_iq_q.addr_imm[11:7];
    assign ex_req_o.pc       = s_iq_q.pc;
    assign ex_req_o.addr_imm = s_iq_q.addr_imm;
    assign ex_req_o.upd_fcc  = s_iq_q.di.upd_fcc;
    assign ex_req_o.fcmp     = s_iq_q.di.fcmp;
    assign ex_req_o.fsel     = s_iq_q.di.fsel;
    assign ex_req_o.fclass   = s_iq_q.di.fclass;
    assign ex_req_o.beqz     = s_iq_q.di.bceqz;
    assign ex_req_o.bnez     = s_iq_q.di.bcnez;
    assign ex_req_o.r0       = real_data[0];
    assign ex_req_o.r1       = real_data[1];
    assign ex_req_o.wid      = s_iq_q.wreg;

    // 连接 CDB
    pipeline_cdb_t fifo_cdb;
    assign fifo_cdb.excp              = '0;
    assign fifo_cdb.need_jump         = ex_resp_i.need_jump;
    assign fifo_cdb.target_addr       = ex_resp_i.target_addr;
    assign fifo_cdb.uncached          = '0;
    assign fifo_cdb.wrong_forward     = '0;
    assign fifo_cdb.wdata             = ex_resp_i.result;
    assign fifo_cdb.fp_excp           = ex_resp_i.fp_excp;
    assign fifo_cdb.fcc               = ex_resp_i.fcc;
    assign fifo_cdb.wid               = ex_resp_i.wid;
    assign fifo_cdb.valid             = '0;
    logic cdb_raw_valid;
    pipeline_cdb_t cdb_raw;
    wired_fifo #(
        .DATA_WIDTH($bits(pipeline_cdb_t)), // rid, wdata, jumppc, jump
        .DEPTH(16)
    ) wired_commit_fifo (
        .clk(clk),
        .rst_n(rst_n && !flush_i),
        .inport_valid_i(ex_valid_i),
        .inport_ready_o(ex_ready_o),
        .inport_payload_i(fifo_cdb),
        .outport_valid_o(cdb_raw_valid),
        .outport_ready_i(cdb_ready_i),
        .outport_payload_o(cdb_raw)
    );
    always_comb begin
        cdb_o = cdb_raw;
        cdb_o.valid = cdb_raw_valid;
    end

endmodule
