`include "wired0_defines.svh"

// Fuction module for Wired project
// Single issue queue entry

module wired_iq_entry #(
    parameter int CDB_COUNT = 2,
    parameter int PAYLOAD_SIZE = 32,
    parameter int FORWARD_COUNT = 2
)(
    `_WIRED_GENERAL_DEFINE,

    input logic sel_i,     // 指令被发射标记
    input logic updata_i,  // 新的指令加入标记
    input pipeline_data_t data_i,            // 新指令的输入数据
    input logic [PAYLOAD_SIZE-1:0] payload_i, // 新指令的控制数据

    // 背靠背唤醒
    input logic     [FORWARD_COUNT-1:0] b2b_valid_i,
    input rob_rid_t [FORWARD_COUNT-1:0] b2b_rid_i,
    
    // CDB 数据前递
    input pipeline_cdb_data_t [CDB_COUNT-1:0] cdb_i,

    output logic  empty_o, // IQ 项目有效
    output logic  ready_o, // 指令数据就绪，可以发射

    // 背靠背唤醒数据源
    output logic  [1:0][FORWARD_COUNT-1:0] b2b_sel_o, // Onehot Encoding
    // output logic  [1:0][CDB_COUNT-1:0] cdb_forward_o,
    output word_t [1:0] data_o,
    output logic [PAYLOAD_SIZE-1:0] payload_o
);

    // 标记 IQ Entry 中存储的是一条有效的指令
    logic valid_inst_q, empty_inst_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            valid_inst_q <= '0;
            empty_inst_q <= '1;
        end else begin
            if(updata_i) begin
                valid_inst_q <= '1;
                empty_inst_q <= '0;
            end else if(sel_i) begin
                valid_inst_q <= '0;
                empty_inst_q <= '1;
            end
        end
    end
    assign empty_o = empty_inst_q; // OK

    // 记录两个数据的状态
    logic[1:0] value_ready; // 下一周期数据就绪的组合逻辑信号
    logic issue_ready_q;
    always_ff @(posedge clk) begin
        issue_ready_q <= &value_ready; // 至多 5-6 级, 120mhz 问题不大，130mhz 有困难。
    end
    assign ready_o = issue_ready_q; // 0 级逻辑输出 OK

    // 记录前递源状态，在 genblock 中生成
    logic [1:0][FORWARD_COUNT-1:0] b2b_sel_q;// 前周期周期唤醒数据情况
    assign b2b_sel_o = b2b_sel_q; // OK

    logic [PAYLOAD_SIZE-1:0] payload_q;
    always_ff @(posedge clk) begin
        if(updata_i) begin
            payload_q <= payload_i;
        end
    end
    assign payload_o = payload_q;

    for(genvar i = 0 ; i < 2 ; i+=1) begin : each_reg
        // 每个数据源独有的储存结构
        word_t data_q;
        logic data_rdy_q;
        rob_rid_t rid_q;
        assign data_o[i] = data_q; // OK

        // 前递逻辑
        logic [CDB_COUNT-1:0] cdb_hit;
        for(genvar j = 0 ; j < CDB_COUNT ; j++) begin
            assign cdb_hit[j] = cdb_i[j].valid && cdb_i[j].wid == rid_q; // 6-6 比较，需要两个 lut6 + lut2，与valid合并为 lut6 + lut3，两级
        end
        wire cdb_forward = |cdb_hit; // 当 cdb 仅有两个时，两个 lut3 合为一个 lut5，恰好两级
        word_t cdb_result;
        always_comb begin
            cdb_result = '0;
            for(integer j = 0 ; j < CDB_COUNT ; j+=1) begin
                cdb_result |= cdb_hit[j] ? cdb_i[j].wdata : '0;
            end
        end

        // 更新逻辑
        always_ff @(posedge clk) begin
            if(updata_i) begin
                data_q <= data_i.rdata[i];
                data_rdy_q <= data_i.valid[i];
                rid_q <= data_i.rreg[i];
            end
            else if(cdb_forward) begin
                data_q <= cdb_result;
                data_rdy_q <= '1;
            end
        end

        // 背靠背唤醒机制
        logic [FORWARD_COUNT-1:0] b2b_hit;
        for(genvar j = 0 ; j < FORWARD_COUNT ; j++) begin
            assign b2b_hit[j] = b2b_valid_i[j] && forward_rid_i[j] == rid_q;
        end
        always_ff @(posedge clk) begin
            b2b_sel_q[i] <= b2b_hit;
        end
        wire b2b_forward = |b2b_hit; // 同 CDB 分析，从 forward_rid_i 到此处为 2 级。

        // 组合逻辑生成下一周期数据有效信息
        // 有意思的是，这个部分恰好是一个 LUT6 哦，结合之前的，到此处为 3 级。
        // 考虑 forward_rid_i 来自本周期，还有额外两级逻辑，此信号最长 5 级。
        always_comb begin
            if(valid_inst_q) begin 
                value_ready[i] = data_rdy_q;
                if(updata_i) begin
                    value_ready[i] = data_i.valid[i];
                end else if(sel_i) begin
                    value_ready[i] = '0;
                end else begin
                    if(cdb_forward) begin
                        value_ready[i] = '1;
                    end else if(b2b_forward) begin
                        value_ready[i] = '1;
                    end
                end
            end else begin
                if(updata_i) begin
                    value_ready[i] = data_i.valid[i];
                end
            end
        end
        
    end

    assign ready_o = &valid && valid_inst_q;
    assign payload_o = payload_q;

endmodule
