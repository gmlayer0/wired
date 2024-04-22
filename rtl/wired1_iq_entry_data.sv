`include "wired0_defines.svh"

// Fuction module for Wired project
// Single issue queue entry

module wired_iq_entry_data #(
    parameter int CDB_COUNT = 2,
    parameter int WAKEUP_SRC_CNT = 2
  )(
    `_WIRED_GENERAL_DEFINE,

    input logic sel_i,     // 指令被发射标记
    input logic updata_i,  // 新的指令加入标记
    input logic valid_inst_i,
    // input pipeline_data_t data_i,            // 新指令的输入数据
    input logic        data_valid_i,
    input rob_rid_t    data_rid_i,
    input logic [31:0] data_i,

    // 背靠背唤醒
    input logic     [WAKEUP_SRC_CNT-1:0] wkup_valid_i,
    input rob_rid_t [WAKEUP_SRC_CNT-1:0] wkup_rid_i,
    input logic     [WAKEUP_SRC_CNT-1:0][31:0] wkup_data_i,

    // CDB 数据前递
    input pipeline_cdb_t [CDB_COUNT-1:0] cdb_i,

    output logic  value_ready_o, // 指令数据已获得或可被转发（纯组合逻辑输出）

    // 唤醒数据源
    output logic  [WAKEUP_SRC_CNT-1:0] wkup_sel_o, // Onehot Encoding
    output word_t data_o
  );
  // 每个数据源独有的储存结构（寄存器）
  word_t data_q;
  logic data_rdy_q;
  rob_rid_t rid_q;
  logic[WAKEUP_SRC_CNT-1:0] wkup_sel_q;

  // 组合逻辑信号
  logic [CDB_COUNT-1:0] cdb_hit;
  logic cdb_forward;
  word_t cdb_result;
  logic [WAKEUP_SRC_CNT-1:0] wkup_hit;
  logic wkup_forward;
  logic [31:0] wkup_sel_result;
  assign data_o = (|wkup_sel_q) ? wkup_sel_result : data_q; // OK
  always_comb begin
    wkup_sel_result = '0;
    for(integer i = 0 ; i < WAKEUP_SRC_CNT ; i += 1) begin
      wkup_sel_result |= wkup_sel_q[i] ? wkup_data_i[i] : '0;
    end
  end


  for(genvar j = 0 ; j < 2 ; j++) begin
    assign cdb_hit[j] = !data_rdy_q &&
           j[0] == rid_q[0] &&
           cdb_i[j].valid &&
           cdb_i[j].wid[`_WIRED_PARAM_ROB_LEN-1:1] == rid_q[`_WIRED_PARAM_ROB_LEN-1:1]; // 6-6 比较，需要两个 lut6 + lut2，与valid合并为 lut6 + lut3，两级
  end
  for(genvar j = 2 ; j < CDB_COUNT ; j++) begin
    assign cdb_hit[j] = !data_rdy_q &&
           cdb_i[j].valid &&
           cdb_i[j].wid == rid_q; // 6-6 比较，需要两个 lut6 + lut2，与valid合并为 lut6 + lut3，两级
  end
  assign cdb_forward = |cdb_hit; // 当 cdb 仅有两个时，两个 lut3 合为一个 lut5，恰好两级
  always_comb begin
    if(CDB_COUNT == 2)
      cdb_result = cdb_i[rid_q[0]].wdata;
    else begin
      cdb_result = '0;
      for(integer j = 0 ; j < CDB_COUNT ; j+=1) begin
        cdb_result |= cdb_hit[j] ? cdb_i[j].wdata : '0;
      end
    end
  end

  // 更新逻辑
  always_ff @(posedge clk) begin
    if(sel_i) wkup_sel_q <= '0;
    else wkup_sel_q <= wkup_sel_o;
  end
  always_ff @(posedge clk) begin
    if(updata_i) begin
      data_rdy_q <= data_valid_i;
      rid_q <= data_rid_i;
    end else if(cdb_forward | wkup_forward) begin
      data_rdy_q <= '1;
    end
  end
  always_ff @(posedge clk) begin
    if(updata_i) begin
      data_q <= data_i;
    end else if(cdb_forward) begin
      data_q <= cdb_result;
    end else if(|wkup_sel_q) begin
      data_q <= wkup_sel_result;
    end
  end

  // 背靠背唤醒机制
  for(genvar j = 0 ; j < WAKEUP_SRC_CNT ; j++) begin
    assign wkup_hit[j] = valid_inst_i && (!data_rdy_q) && wkup_valid_i[j] && wkup_rid_i[j] == rid_q;
  end
  always_ff @(posedge clk) begin
    if(sel_i) wkup_sel_o <= '0;
    else wkup_sel_o <= wkup_hit;
  end
  assign wkup_forward = |wkup_hit; // 同 CDB 分析，从 wkup_rid_i 到此处为 2 级。

  // 组合逻辑生成下一周期数据有效信息
  // 有意思的是，这个部分恰好是一个 LUT6 哦，结合之前的，到此处为 3 级。
  // 考虑 wkup_rid_i 来自本周期，还有额外两级逻辑，此信号最长 5 级。
  always_comb begin
    value_ready_o = '0;
    if(valid_inst_i) begin
      value_ready_o = data_rdy_q;
      if(updata_i) begin
        value_ready_o = data_valid_i;
      end else if(sel_i) begin
        value_ready_o = '0;
      end else begin
        if(cdb_forward | wkup_forward) begin
          value_ready_o = '1;
        end
      end
    end else begin
      if(updata_i) begin
        value_ready_o = data_valid_i;
      end
    end
  end

endmodule
