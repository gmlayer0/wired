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
    output pipeline_cdb_t cdb_o,
    input  logic          cdb_ready_i, // 这里可以接 FIFO

    // CDB 嗅探端口
    input pipeline_cdb_t [1:0] cdb_i,

    // FLUSH 端口
    input logic flush_i, // 后端正在清洗管线，发射所有指令而不等待就绪

    output logic         lsu_req_valid_o,
    input  logic         lsu_req_ready_i,
    output iq_lsu_req_t  lsu_req_o,

    input  logic         lsu_resp_valid_i,
    output logic         lsu_resp_ready_o,
    input  iq_lsu_resp_t lsu_resp_i
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
    wire top_valid, top_ready, top_fire; // TODO: CONNECT TOP_READY TO LSU
    assign top_fire = top_valid & top_ready;
    assign top_valid = fire_rdy_q[iq_tail_q];
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
        decode_info_lsu_t di;
        logic[31:0] pc;
        logic[27:0] addr_imm;
        rob_rid_t   wreg;
        logic[4:0]  op_code;
    } iq_static_t;
    // 输入给 IQ 的 static 信息
    iq_static_t [1:0] p_static;
    // IQ 中存储的信息
    word_t      [IQ_SIZE-1:0][1:0] iq_data;
    iq_static_t [IQ_SIZE-1:0] iq_static;
    word_t [1:0] s_data;
    iq_static_t  s_static;
    assign s_data = iq_data[iq_tail_q];
    assign s_static = iq_static[iq_tail_q];
    // 解包信息
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        always_comb begin
            p_static[i].di       = get_lsu_from_p(p_ctrl_i[i].di);
            p_static[i].pc       = p_ctrl_i[i].pc;
            p_static[i].addr_imm = p_ctrl_i[i].addr_imm;
            p_static[i].wreg     = p_ctrl_i[i].wreg.rob_id;
            p_static[i].op_code  = p_ctrl_i[i].op_code;
        end
    end


    // 例化 Reserve station entry
    for(genvar i = 0 ; i < IQ_SIZE ; i += 1) begin
        wire [1:0] update_by;
        wire [1:0] is_top;
        assign is_top[0] = iq_head_q == i[PTR_LEN-1:0];
        assign is_top[1] = iq_head_p1_q == i[PTR_LEN-1:0];
            assign update_by[0] = is_top[0] && p_valid_i[0];
            assign update_by[1] = p_valid_i[1] &&
                                ((is_top[0] && !p_valid_i[0]) || (is_top[1] && p_valid_i[0]));
        wired_iq_entry # (
            .CDB_COUNT(2),
            .PAYLOAD_SIZE($bits(iq_static_t)),
            .FORWARD_COUNT(2)
        )
        wired_iq_entry_inst (
            `_WIRED_GENERAL_CONN,
            .sel_i((iq_tail_q == i[PTR_LEN-1:0] && fire_rdy_q[i] && top_ready) | flush_i),
            .updata_i(|update_by),
            .data_i(update_by[1] ? p_data_i[1] : p_data_i[0]),
            .payload_i(update_by[0] ? p_static[0] : p_static[1]),
            .b2b_valid_i(),
            .b2b_rid_i(),
            .cdb_i(cdb_i),
            .empty_o(empty_q[i]),
            .ready_o(fire_rdy_q[i]),
            .b2b_sel_o(),
            .data_o(iq_data[i]),
            .payload_o(iq_static[i])
        );
    end

    // 连接 lsu
    word_t [1:0] s_data_q;
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
            s_data_q <= s_data;
            s_iq_q <= s_static;
        end
    end
    iq_lsu_req_t s_lsu_req;
    always_comb begin
        s_lsu_req = '0;
        s_lsu_req.wid = s_iq_q.wreg;
        s_lsu_req.vaddr = {{4{s_iq_q.addr_imm[27]}},
                              s_iq_q.addr_imm} + s_data_q[1];
        s_lsu_req.msize = '0;
        case (s_iq_q.di.mem_type[1:0])
        2'd1: s_lsu_req.msize = 2'd2; // Word
        2'd2: s_lsu_req.msize = 2'd1; // Half
        default/*2'd3*/: s_lsu_req.msize = 2'd0; // Byte
        endcase
        s_lsu_req.msigned = !s_iq_q.di.mem_type[2];
        s_lsu_req.strb  = '0;
        if(s_iq_q.di.mem_write) begin
            case (s_iq_q.di.mem_type[1:0])
            2'd1: s_lsu_req.strb = 4'b1111; // Word
            2'd2: s_lsu_req.strb = 4'b0011 << {s_lsu_req.vaddr[1],1'd0}; // Half
            default/*2'd3*/: s_lsu_req.strb = 4'b0001 << s_lsu_req.vaddr[1:0]; // Byte
            endcase
        end
        
        s_lsu_req.cacop = s_iq_q.di.mem_write ? WR_ALLOC :
                          s_iq_q.di.dbarrier ? NOT_VALID_INV_PARM : RD_ALLOC;
        if(s_iq_q.di.mem_cacop) begin
            if(s_iq_q.op_code[2:0] == 3'd1) begin// dcacheop
                case(s_iq_q.op_code[4:3])
                2'd0: begin
                    s_lsu_req.cacop = IDX_INIT;
                end
                2'd1: begin
                    s_lsu_req.cacop = IDX_INV;
                end
                default /*2'd2*/: begin
                    s_lsu_req.cacop = HIT_INV;
                end
                endcase
            end else begin
                s_lsu_req.cacop = '0;
            end
        end
        s_lsu_req.dbar  = s_iq_q.di.dbarrier;
        s_lsu_req.llsc  = s_iq_q.di.llsc_inst;
        case (s_iq_q.di.mem_type[1:0])
        default/*2'd1*/: s_lsu_req.wdata = s_data_q[0];
        2'd2: s_lsu_req.wdata = (s_data_q[0]) << {s_lsu_req.vaddr[1], 4'd0};   // Half
        2'd3: s_lsu_req.wdata = (s_data_q[0]) << {s_lsu_req.vaddr[1:0], 3'd0}; // Byte
        endcase
    end
    assign lsu_req_valid_o = s_top_valid_q;
    assign s_top_ready = lsu_req_ready_i;
    assign lsu_req_o = s_lsu_req;

    // 连接 CDB
    pipeline_cdb_t fifo_cdb;
    assign fifo_cdb.excp              = lsu_resp_i.excp;
    assign fifo_cdb.need_jump         = '0;
    assign fifo_cdb.target_addr       = lsu_resp_i.vaddr;
    assign fifo_cdb.uncached          = lsu_resp_i.uncached;
    assign fifo_cdb.wdata             = lsu_resp_i.rdata;
    assign fifo_cdb.wid               = lsu_resp_i.wid;
    assign fifo_cdb.valid             = '0;
    logic cdb_raw_valid;
    pipeline_cdb_t cdb_raw;
    wired_fifo #(
        .DATA_WIDTH($bits(pipeline_cdb_t)), // rid, wdata, jumppc, jump
        .DEPTH(2)
    ) wired_commit_fifo (
        .clk(clk),
        .rst_n(rst_n && !flush_i),
        .inport_valid_i(lsu_resp_valid_i),
        .inport_ready_o(lsu_resp_ready_o),
        .inport_payload_i(fifo_cdb),
        .outport_valid_o(cdb_raw_valid),
        .outport_ready_i(cdb_ready_i),
        .outport_payload_o(cdb_raw)
    );
    always_comb begin
        cdb_o = cdb_raw;
        assign cdb_o.valid = cdb_raw_valid;
    end

endmodule
