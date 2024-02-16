`include "wired0_defines"

// Fuction module for Wired project
// Single issue queue entry CDB_FORWARDINGtion

module wired_iq_entry #(
    parameter int CDB_COUNT = 2,
    parameter int FORWARD_COUNT = 2,
    parameter bit CDB_FORWARDING = 0
)(
    `_WIRED_GENERAL_DEFINE,

    input logic sel_i,     // 指令被发射标记
    input logic updata_i,  // 新的指令加入标记
    input pipeline_data_t data_i,

    // 背靠背唤醒
    input logic     [FORWARD_COUNT-1:0] forward_valid_i,
    input rob_rid_t [FORWARD_COUNT-1:0] forward_rid_i,
    
    // CDB 数据前递
    input pipeline_cdb_data_t [CDB_COUNT-1:0] cdb_i,

    output logic  ready_o, // 指令数据就绪，可以发射

    // 背靠背唤醒数据源
    output logic  [1:0][FORWARD_COUNT-1:0] forward_o, // Onehot Encoding
    // output logic  [1:0][CDB_COUNT-1:0] cdb_forward_o,
    output word_t [1:0] data_o
);

    // 标记 IQ Entry 中存储的是一条有效的指令
    logic valid_inst;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            valid_inst <= '0;
        end else begin
            if(updata_i) begin
                valid_inst <= '1;
            end else if(sel_i) begin
                valid_inst <= '0;
            end
        end
    end
    
    // 标记 IQ Entry 中存储的数据源有效
    logic [1:0] valid_q;
    // 实际组合逻辑有效信号
    logic [1:0] valid;
    // 记录 IQ Entry 中的有效数据
    word_t [1:0] data_q;
    // 记录 IQ Entry 中的数据源，便于捕获 CDB 中的写回数据
    rob_rid_t [1:0] rid_q;

    for(genvar i = 0 ; i < 2 ; i+=1) begin : each_reg
        // 前递逻辑
        logic [CDB_COUNT-1:0] cdb_hit;
        for(genvar j = 0 ; j < CDB_COUNT ; j++) begin
            assign cdb_hit[j] = cdb_i[j].valid && cdb_i[j].wid == rid_q[i];
        end
        wire cdb_forward = |cdb_hit;
        word_t cdb_result;
        always_comb begin
            cdb_result = '0;
            for(integer j = 0 ; j < CDB_COUNT ; j+=1) begin
                cdb_result |= cdb_hit[j] ? cdb_i[j].wdata : '0;
            end
        end

        logic [FORWARD_COUNT-1:0] forward_hit;
        for(genvar j = 0 ; j < FORWARD_COUNT ; j++) begin
            assign forward_hit[j] = forward_valid_i[j] && forward_rid_i[j] == rid_q[i];
        end

        // 更新逻辑
        always_ff @(posedge clk) begin
            if(updata_i) begin
                data_q[i] <= data_i.rdata[i];
                valid_q[i] <= data_i.valid[i];
            end
            else if(cdb_forward) begin
                data_q[i] <= cdb_result;
                valid_q[i] <= '1;
            end
        end
        always_ff @(posedge clk) begin
            if(updata_i) begin
                rid_q[i] <= data_i.rreg[i];
            end
        end

        // 输出逻辑
        // assign cdb_forward_o[i] = cdb_hit;
        assign forward_o[i] = forward_hit;

        always_comb begin
            valid[i] = valid_q[i];
            if((cdb_forward && CDB_FORWARDING) || (|forward_hit)) valid[i] |= '1;
        end
        // 数据输出，可配置的 CDB 前递使能
        if(CDB_FORWARDING) begin
            assign data_o[i] = cdb_forward ? cdb_result : data_q[i];
        end else begin
            assign data_o[i] = data_q[i];
        end
    end

    assign ready_o = &valid && valid_inst;

endmodule
