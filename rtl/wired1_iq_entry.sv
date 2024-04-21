`include "wired0_defines.svh"

// Fuction module for Wired project
// Single issue queue entry

module wired_iq_entry #(
    parameter int CDB_COUNT = 2,
    parameter int PAYLOAD_SIZE = 32,
    parameter int WAKEUP_SRC_CNT = 2,
    parameter int RREG_CNT = 2
)(
    `_WIRED_GENERAL_DEFINE,

    input logic sel_i,     // 指令被发射标记
    input logic updata_i,  // 新的指令加入标记
    input pipeline_data_t data_i,            // 新指令的输入数据
    input logic [PAYLOAD_SIZE-1:0] payload_i, // 新指令的控制数据

    // 背靠背唤醒
    input logic     [RREG_CNT-1:0][WAKEUP_SRC_CNT-1:0] wkup_valid_i,
    input rob_rid_t [RREG_CNT-1:0][WAKEUP_SRC_CNT-1:0] wkup_rid_i,
    
    // CDB 数据前递
    input pipeline_cdb_t [CDB_COUNT-1:0] cdb_i,

    output logic  empty_o, // IQ 项目有效
    input  logic  [RREG_CNT-1:0] ready_mask_i, // 不等待 masked 的部分就绪
    output logic  ready_o, // 指令数据就绪，可以发射
    output logic  [RREG_CNT-1:0] data_ready_o,

    // 唤醒数据源
    output logic  [RREG_CNT-1:0][WAKEUP_SRC_CNT-1:0] wkup_sel_o, // Onehot Encoding
    output word_t [RREG_CNT-1:0] data_o,
    output logic  [PAYLOAD_SIZE-1:0] payload_o
);

  // 生成静态部分
  wired_iq_entry_static # (
    .PAYLOAD_SIZE(PAYLOAD_SIZE)
  )
  wired_iq_entry_static_inst (
    `_WIRED_GENERAL_CONN,
    .sel_i(sel_i),
    .updata_i(updata_i),
    .payload_i(payload_i),
    .payload_o(payload_o),
    .empty_o(empty_o)
  );

  // 生成动态捕获部分
  logic [RREG_CNT-1:0] value_ready;
  for(genvar i = 0 ; i < RREG_CNT ; i += 1) begin
    wired_iq_entry_data # (
      .CDB_COUNT(CDB_COUNT),
      .WAKEUP_SRC_CNT(WAKEUP_SRC_CNT)
    )
    wired_iq_entry_data_inst (
      `_WIRED_GENERAL_CONN,
      .sel_i(sel_i),
      .updata_i(updata_i),
      .data_valid_i(data_i.valid[i]),
      .data_rid_i(data_i.rreg[i]),
      .data_i(data_i.rdata[i]),
      .wkup_valid_i(wkup_valid_i[i]),
      .wkup_rid_i(wkup_rid_i[i]),
      .cdb_i(cdb_i),
      .value_ready_o(value_ready[i]),
      .wkup_sel_o(wkup_sel_o[i]),
      .data_o(data_o[i])
    );
  end
  always_ff @(posedge clk) begin
    ready_o <= &(value_ready | ready_mask_i);
    data_ready_o <= value_ready;
  end

endmodule
