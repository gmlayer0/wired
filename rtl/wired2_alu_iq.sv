`include "wired0_defines.svh"

// Fuction module for Wired project
// alu issue queue + alu
module wired_alu_iq #(
    parameter int IQ_SIZE = `_WIRED_PARAM_INT_IQ_DEPTH,
    parameter int B2B_STACK_SIZE = `_WIRED_PARAM_INT_IQ_B2B_STACK_SIZE
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
    output pipeline_cdb_t [1:0] cdb_o,
    input  logic          [1:0] cdb_ready_i, // 这里可以接 FIFO

    // CDB 嗅探端口
    input pipeline_cdb_t [1:0] cdb_i,

    // CDB 加速端口
    // input pipeline_cdb_t cdb_acc_i,

    // FLUSH 端口
    input logic flush_i // 后端正在清洗管线，发射所有指令而不等待就绪
);
    logic [IQ_SIZE-1:0] empty_q; // 标识 IQ ENTRY 可被占用
    logic [IQ_SIZE-1:0] fire_rdy_q;  // 标识 IQ ENTRY 可发射
    // Todo: AGE-MAP BASED OPTIMIZATION

    // IQ 有一个比较奇特的设计，整体 IQ 为 8 项，但其中仅有4项是两个 ALU 均可以发射的，其余4项则是独占的
    // 对于填入，优先级倒置

    // 如示例：
    // IQ:   0 1 2 3 4 5 6 7
    // ALU0: 0 1 2 3 4 5
    // ALU1:     5 4 3 2 1 0
    // UPD0: 3 2 1 0 7 6 5 4
    // UPD1: 4 5 6 7 0 1 2 3
    logic [1:0][IQ_SIZE-1:0] fire_sel_oh;
    localparam integer FIREPIO [IQ_SIZE-1:0] = {7,6,5,4,3,2,1,0};
    localparam integer FIRERANGE = 3 * IQ_SIZE / 4; // [IQ_SIZE/2, IQ_SIZE]
    for(genvar i = 0 ; i < 2 ; i += 1) begin : GENFIRE_PER_ALU
        always_comb begin
            fire_sel_oh[i] = '0;
            for(integer x = FIRERANGE-1 ; x >= 0 ; x -= 1) begin
                if(fire_rdy_q[(IQ_SIZE-1-2*FIREPIO[x])*i + FIREPIO[x]]) begin 
                    fire_sel_oh[i] = '0;
                    fire_sel_oh[i][(IQ_SIZE-1-2*FIREPIO[x])*i + FIREPIO[x]] = '1;
                end
            end
        end
    end

    // 也就是每个 ALU 可以执行的直接数据源是 6 项目
    // 仅需要 2 x LUT6 + MUX2 即可完成一个数据源的选择，逻辑深度为3

    // UPD 逻辑
    // IQ:   0 1 2 3 4 5 6 7
    // UPD0: 3 2 1 0 7 6 5 4
    // UPD1: 4 5 6 7 0 1 2 3
    logic [1:0][IQ_SIZE-1:0] upd_sel_oh;
    localparam integer UPDPIO [IQ_SIZE-1:0] = {4,5,6,7,0,1,2,3};
    for(genvar i = 0 ; i < 2 ; i += 1) begin : GENUPD_PER_ALU
        always_comb begin
            upd_sel_oh[i] = '0;
            for(integer x = IQ_SIZE-1 ; x >= 0 ; x -= 1) begin
                if(empty_q[(IQ_SIZE-1-2*UPDPIO[x])*i + UPDPIO[x]]) begin 
                    upd_sel_oh[i] = '0;
                    upd_sel_oh[i][(IQ_SIZE-1-2*UPDPIO[x])*i + UPDPIO[x]] = '1;
                end
            end
        end
    end

    // CDB 接口上的 FIFO 队列， 不满的时候才可以以发射指令到 FU 执行，一旦有一个 FIFO 满，就阻止指令发射。
    // 这样保证在 ALU 中的两条指令起步走，转发的两个源头也是齐步走的
    logic [1:0] excute_valid; // 标记 Excute 级的两个执行槽是否有效
    logic [3:0] free_cnt_q;
    wire  [3:0] free_cnt = free_cnt_q - p_valid_i[0] - p_valid_i[1] +
    (excute_ready&excute_valid[0]) + (excute_ready&excute_valid[1]);
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            free_cnt_q <= IQ_SIZE;
        end else begin
            free_cnt_q <= free_cnt;
        end
    end
    always_ff @(posedge clk) begin
        p_ready_o <= free_cnt >= 2;
    end


    // Reserve station static entry 定义
    typedef struct packed {
        decode_info_alu_t di;
        logic[31:0] pc;
        logic[27:0] addr_imm;
        rob_rid_t wreg;
    } iq_static_t;
    // 输入给 IQ 的 static 信息
    iq_static_t [1:0] p_static;

    // IQ 中存储的信息
    word_t      [IQ_SIZE-1:0][1:0] iq_data;
    iq_static_t [IQ_SIZE-1:0] iq_static;
    logic [IQ_SIZE-1:0][1:0][B2B_STACK_SIZE-1:0][1:0] b2b_src; // IQ_INDEX REG_INDEX STACK_IDX SRC_INDEX
    logic       excute_ready; // 当此信号为高时候，才可以向 Excute 级别写入新的指令，齐步走信号
    logic     [B2B_STACK_SIZE-1:0][1:0] b2b_valid;                // OK
    rob_rid_t [B2B_STACK_SIZE-1:0][1:0] b2b_rid;                  // OK
    rob_rid_t [B2B_STACK_SIZE-1:0][1:0] b2b_rid_d, b2b_rid_q;     // OK
    logic     [B2B_STACK_SIZE-1:0][1:0] b2b_valid_d, b2b_valid_q; // OK
    assign b2b_valid = excute_ready ? b2b_valid_d : b2b_valid_q;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            b2b_valid_q <= '0;
        end else begin
            if(excute_ready) b2b_valid_q <= b2b_valid_d;
        end
    end
    assign b2b_rid = excute_ready ? b2b_rid_d : b2b_rid_q;
    always_ff @(posedge clk) begin
        if(excute_ready) b2b_rid_q <= b2b_rid_d;
    end

    // 解包信息
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        always_comb begin
            p_static[i].di       = get_alu_from_p(p_ctrl_i[i].di);
            p_static[i].pc       = p_ctrl_i[i].pc;
            p_static[i].wreg     = p_ctrl_i[i].wreg.rob_id;
            p_static[i].addr_imm = p_ctrl_i[i].addr_imm;
        end
    end

    // 例化 Reserve station entry
    // 与 Excute Unit 的握手信号
    for(genvar i = 0 ; i < IQ_SIZE ; i += 1) begin
        wire [1:0] update_by;
        for(genvar j = 0 ; j < 2 ; j += 1) begin
            assign update_by[j] = upd_sel_oh[j][i] & p_valid_i[j];
        end
        wired_iq_entry # (
            .CDB_COUNT(2),
            .PAYLOAD_SIZE($bits(iq_static_t)),
            .FORWARD_COUNT(2 * B2B_STACK_SIZE)
        )
        wired_iq_entry_inst (
            `_WIRED_GENERAL_CONN,
            .sel_i(((fire_sel_oh[0][i] | fire_sel_oh[1][i]) & excute_ready) | flush_i),
            .updata_i(|update_by),
            .data_i(update_by[1] ? p_data_i[1] : p_data_i[0]),
            .payload_i(update_by[0] ? p_static[0] : p_static[1]),
            .b2b_valid_i(b2b_valid),
            .b2b_rid_i(b2b_rid),
            .cdb_i(cdb_i),
            .empty_o(empty_q[i]),
            .ready_o(fire_rdy_q[i]),
            .b2b_sel_o(b2b_src[i]),
            .data_o(iq_data[i]),
            .payload_o(iq_static[i])
        );
    end
    iq_static_t [1:0] sel_static_q,  sel_static;
    word_t [1:0][1:0] sel_data_q,    sel_data;
    logic [1:0][1:0][B2B_STACK_SIZE-1:0][1:0] sel_forward_q, sel_forward; // ok
    word_t [B2B_STACK_SIZE-1:0][1:0] fwd_data_q, fwd_data; // TODO: FINISHME
    // 选择两个用于 ALU 输入的 data 和 static
    for(genvar s = 0 ; s < 2 ; s += 1) begin
        always_comb begin
            sel_static[s] = '0;
            sel_forward[s] = '0;
            sel_data[s] = '0;
            b2b_valid_d[0][s] = '0;
            b2b_rid_d[0][s] = '0;
            for(integer i = 0 ; i < IQ_SIZE ; i += 1) begin
                if(fire_sel_oh[s][i]) begin
                    sel_static[s]  |= iq_static[i];
                    sel_forward[s] |= b2b_src[i];
                    sel_data[s]    |= iq_data[i];
                    b2b_valid_d[0][s] |= '1;
                    b2b_rid_d[0][s]   |= iq_static[i].wreg;
                end
            end
            for(integer i = 1 ; i < B2B_STACK_SIZE ; i += 1) begin
                b2b_valid_d[i][s] = b2b_valid_q[i-1][s];
                b2b_rid_d[i][s]   = b2b_rid_q[i-1][s];
            end
        end
        assign excute_valid[0] = |fire_rdy_q[5:0];
        assign excute_valid[1] = (|fire_rdy_q[7:2]) && (fire_sel_oh[0] != fire_sel_oh[1]);
    end
    always_ff @(posedge clk) begin
        if(excute_ready) begin
            sel_static_q  <= sel_static;
            sel_forward_q <= sel_forward;
            sel_data_q    <= sel_data;
            fwd_data_q    <= fwd_data;
        end
    end
    rob_rid_t[1:0]   ex_rid;
    logic [1:0] excute_valid_q;
    logic [1:0] l_excute_ready;
    wire  [1:0] fifo_ready;
    for(genvar p = 0 ; p < 2 ; p += 1) begin
        assign ex_rid[p] = sel_static_q[p].wreg;
        always_ff @(posedge clk) begin
            if(!rst_n || flush_i) begin
                excute_valid_q[p] <= '0;
            end else begin
                if(excute_ready) begin
                    excute_valid_q[p] <= excute_valid[p];
                end else begin
                    if(excute_valid_q[p] && fifo_ready[p]) begin
                        excute_valid_q[p] <= '0;
                    end
                end
            end
        end
        assign l_excute_ready[p] = (!excute_valid_q[p]) || fifo_ready[p];
    end
    assign excute_ready = &l_excute_ready;
    // 例化两个 ALU 和 jump 模块 用于处理所有计算指令以及分支指令
    logic[1:0]       ex_jump;
    logic[1:0][31:0] ex_jump_target;
    logic[1:0][31:0] ex_jump_target_ext; // 为 csr 及其它特殊指令准备
    logic[1:0][31:0] ex_wdata;
    logic[1:0][31:0] ex_wdata_ext; // 为 csr 指令准备
    assign fwd_data[0] = ex_wdata;
    if(B2B_STACK_SIZE > 1) assign fwd_data[B2B_STACK_SIZE-1:1] = fwd_data_q[B2B_STACK_SIZE-2:0];
    word_t [1:0][1:0] real_data; // 转发后的数据
    for(genvar p = 0 ; p < 2 ; p += 1) begin
        wire [4:0] attach_rd = sel_static_q[p].addr_imm[22:18];
        wire [4:0] attach_rj = sel_static_q[p].addr_imm[27:23];
        always_comb begin
            for(integer i = 0 ; i < 2 ; i += 1) begin
                real_data[p][i] = (|sel_forward_q[p][i]) ? '0 : sel_data_q[p][i];
                for(integer s = 0 ; s < B2B_STACK_SIZE ; s += 1) begin 
                    for(integer f = 0 ; f < 2 ; f += 1) begin
                        real_data[p][i] |= sel_forward_q[p][i][s][f] ? fwd_data_q[s][f] : '0;
                    end
                end
            end
        end
        wired_alu  wired_alu_inst (
            .r0_i(real_data[p][0]),
            .r1_i(real_data[p][1]),
            .pc_i(sel_static_q[p].pc),
            .grand_op_i(sel_static_q[p].di.alu_grand_op),
            .op_i(sel_static_q[p].di.alu_op),
            .res_o(ex_wdata[p])
        );
        logic [31:0] csrxchg_wdata; // 实际上是写 mask
        assign csrxchg_wdata = real_data[p][0];
        assign ex_wdata_ext[p] = sel_static_q[p].di.csr_op_en ? csrxchg_wdata : ex_wdata[p];
        wired_jump  wired_jump_inst (
            .r0_i(real_data[p][0]),
            .r1_i(real_data[p][1]),
            .pc_i(sel_static_q[p].pc),
            .addr_imm_i(sel_static_q[p].addr_imm),
            .target_type_i(sel_static_q[p].di.target_type),
            .cmp_type_i(sel_static_q[p].di.cmp_type),
            .jump_o(ex_jump[p]),
            .jump_target_o(ex_jump_target[p])
        );
        logic [31:0] csrxchg_jump_target; // 实际上是写 mask
        logic [31:0] invtlb_target; // 实际上是写 mask
        assign invtlb_target = {real_data[p][0][31:13], 3'd0, real_data[p][1][9:0]};
        assign csrxchg_jump_target = attach_rj == 5'd0 ? '0 : (attach_rj == 5'd1 ? '1 : real_data[p][1]);
        assign ex_jump_target_ext[p] = sel_static_q[p].di.csr_op_en ? csrxchg_jump_target : 
                                       sel_static_q[p].di.invtlb_en ? invtlb_target       :ex_jump_target[p];
    end

    // 连接到 CDB 的两个 FIFO
    for(genvar p = 0 ; p < 2 ; p += 1) begin
        logic c_jump;
        rob_rid_t c_rid;
        logic[31:0] c_jump_target;
        logic[31:0] c_wdata;
        wired_fifo #(
            .DATA_WIDTH($bits(rob_rid_t) + 32 + 32 + 1), // rid, wdata, jumppc, jump
            .DEPTH(2)
        )
        wired_commit_fifo(
            .clk(clk),
            .rst_n(rst_n && !flush_i),
            .inport_valid_i(excute_valid_q[p]),
            .inport_ready_o(fifo_ready[p]),
            // 对于 csr 指令， ex_wdata == r0, target_addr = rj == 0 ? '0 : rj == 1 ? '1 : r1;
            .inport_payload_i({ex_rid[p], ex_wdata_ext[p], ex_jump_target_ext[p], ex_jump[p]}),
            .outport_valid_o(cdb_o[p].valid),
            .outport_ready_i(cdb_ready_i[p]),
            .outport_payload_o({c_rid, c_wdata, c_jump_target, c_jump})
        );
        assign cdb_o[p].excp              = '0;
        assign cdb_o[p].need_jump         = c_jump;
        assign cdb_o[p].target_addr       = c_jump_target;
        assign cdb_o[p].uncached          = '0;
        // assign cdb_o[p].store_buffer      = '0;
        // assign cdb_o[p].store_conditional = '0;
        assign cdb_o[p].wdata             = c_wdata;
        assign cdb_o[p].wid               = c_rid;
    end

endmodule
