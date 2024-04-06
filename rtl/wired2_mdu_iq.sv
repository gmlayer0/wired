`include "wired0_defines.svh"

// Fuction module for Wired project
// mdu issue queue
// 此模块支持乱序执行

module wired_mdu_iq #(
    parameter int IQ_SIZE = `_WIRED_PARAM_MDU_IQ_DEPTH // 不用很大，4 项即可
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

    // 执行单元端口
    output logic         ex_valid_o,
    input  logic         ex_ready_i,
    output iq_mdu_req_t  ex_req_o,

    input  logic         ex_valid_i,
    output logic         ex_ready_o,
    input  iq_mdu_resp_t ex_resp_i,

    // FLUSH 端口
    input logic flush_i // 后端正在清洗管线，发射所有指令而不等待就绪
);
    logic [IQ_SIZE-1:0] empty_q; // 标识 IQ ENTRY 可被占用
    logic [IQ_SIZE-1:0] fire_rdy_q;  // 标识 IQ ENTRY 可发射
    // Todo: AGE-MAP BASED OPTIMIZATION

    // IQ 有一个比较奇特的设计，整体 IQ 为 8 项，但其中仅有4项是两个 ALU 均可以发射的，其余4项则是独占的
    // 对于填入，优先级倒置

    // 如示例：
    // IQ:   0 1 2 3
    // ALU0: 0 1 2 3
    // UPD0: 0 1 2 3
    // UPD1: 3 2 1 0
    logic [IQ_SIZE-1:0] fire_sel_oh;
    localparam integer FIREPIO [IQ_SIZE-1:0] = {3,2,1,0};
    always_comb begin
        fire_sel_oh = '0;
        for(integer x = IQ_SIZE-1 ; x >= 0 ; x -= 1) begin
            if(fire_rdy_q[FIREPIO[x]]) begin 
                fire_sel_oh = '0;
                fire_sel_oh[FIREPIO[x]] = '1;
            end
        end
    end
    // 也就是每个 MDU 可以执行的直接数据源是 4 项目

    // UPD 逻辑
    // IQ:   0 1 2 3
    // UPD0: 0 1 2 3
    // UPD1: 3 2 1 0
    logic [1:0][IQ_SIZE-1:0] upd_sel_oh;
    localparam integer UPDPIO [IQ_SIZE-1:0] = {3,2,1,0};
    for(genvar i = 0 ; i < 2 ; i += 1) begin : GENUPD_PER_MDU
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
    wire excute_valid = |fire_sel_oh; // 标记 Excute 级的两个执行槽是否有效
    logic [2:0] free_cnt_q;
    wire  [2:0] free_cnt = free_cnt_q - p_valid_i[0] - p_valid_i[1] + (excute_ready&excute_valid);
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
        decode_info_mdu_t di;
        logic[31:0] pc; // 仅供调试用
        rob_rid_t wreg;
    } iq_static_t;
    // 输入给 IQ 的 static 信息
    iq_static_t [1:0] p_static;

    // IQ 中存储的信息
    word_t      [IQ_SIZE-1:0][1:0] iq_data;
    iq_static_t [IQ_SIZE-1:0] iq_static;
    logic       excute_ready; // 当此信号为高时候，才可以向 Excute 级别写入新的指令

    // 解包信息
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        always_comb begin
            p_static[i].di       = get_mdu_from_p(p_ctrl_i[i].di);
            p_static[i].pc       = p_ctrl_i[i].pc;
            p_static[i].wreg     = p_ctrl_i[i].wreg.rob_id;
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
            .FORWARD_COUNT(1)
        )
        wired_iq_entry_inst (
            `_WIRED_GENERAL_CONN,
            .sel_i((fire_sel_oh[i] & excute_ready) | flush_i),
            .updata_i(|update_by),
            .data_i(update_by[1] ? p_data_i[1] : p_data_i[0]),
            .payload_i(update_by[0] ? p_static[0] : p_static[1]),
            .b2b_valid_i('0),
            .b2b_rid_i('0),
            .cdb_i(cdb_i),
            .empty_o(empty_q[i]),
            .ready_o(fire_rdy_q[i]),
            .b2b_sel_o(),
            .data_o(iq_data[i]),
            .payload_o(iq_static[i])
        );
    end
    iq_static_t sel_static_q,  sel_static;
    word_t [1:0] sel_data_q,    sel_data;
    // 选择两个用于 MDU 输入的 data 和 static
    always_comb begin
        sel_static = '0;
        sel_data = '0;
        for(integer i = 0 ; i < IQ_SIZE ; i += 1) begin
            if(fire_sel_oh[i]) begin
                sel_static  |= iq_static[i];
                sel_data    |= iq_data[i];
            end
        end
    end
    always_ff @(posedge clk) begin
        if(excute_ready) begin
            sel_static_q  <= sel_static;
            sel_data_q    <= sel_data;
        end
    end
    logic excute_valid_q;
    assign excute_ready = !excute_valid_q || ex_ready_i;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            excute_valid_q <= '0;
        end else if(excute_ready) begin
            excute_valid_q <= excute_valid;
        end
    end
    // 连接到 MDU
    assign ex_valid_o   = excute_valid_q;
    assign ex_req_o.op  = sel_static_q.di.alu_op[1:0];
    assign ex_req_o.r0  = sel_data_q[0];
    assign ex_req_o.r1  = sel_data_q[1];
    assign ex_req_o.wid = sel_static_q.wreg;

    // 连接到 CDB 的 FIFO
    rob_rid_t c_rid;
    logic[31:0] c_wdata;
    wired_fifo #(
        .DATA_WIDTH($bits(rob_rid_t) + 32), // rid, wdata, jumppc, jump
        .DEPTH(2)
    )
    wired_commit_fifo(
        .clk(clk),
        .rst_n(rst_n && !flush_i),
        .inport_valid_i(ex_valid_i),
        .inport_ready_o(ex_ready_o),
        .inport_payload_i({ex_resp_i.wid, ex_resp_i.result}),
        .outport_valid_o(cdb_o.valid),
        .outport_ready_i(cdb_ready_i),
        .outport_payload_o({c_rid, c_wdata})
    );
    assign cdb_o.excp              = '0;
    assign cdb_o.need_jump         = '0;
    assign cdb_o.target_addr       = '0;
    assign cdb_o.uncached          = '0;
    // assign cdb_o[p].store_buffer      = '0;
    // assign cdb_o[p].store_conditional = '0;
    assign cdb_o.wdata             = c_wdata;
    assign cdb_o.wid               = c_rid;

endmodule
