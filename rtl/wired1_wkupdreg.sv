`include "wired0_defines.svh"

// 这个模块用于统一在 IQ 发射时做数据前递的工作
module wired_wkupdreg #(
  parameter int unsigned WAKEUP_SRC_CNT
)(
  `_WIRED_GENERAL_DEFINE,

  input  logic                            ready_i,      // FU1-FU2 握手

  input  logic [WAKEUP_SRC_CNT-1:0]       wkup_src_i,   // SEL 周期信号
  input  logic                     [31:0] data_i,       // SEL 周期信号
  
  input  logic [WAKEUP_SRC_CNT-1:0][31:0] wkup_data_i,  // FU1 周期信号
  output logic                     [31:0] data_o        // FU1 周期输出信号
);


logic [31:0] data_q;
logic [WAKEUP_SRC_CNT-1:0] wkup_src_q;
always_ff @(posedge clk) begin
  if(ready_i) begin
    wkup_src_q <= wkup_src_i;
  end else begin
    wkup_src_q <= '0; // 仅维持一个周期
  end
end
always_ff @(posedge clk) begin
  if(ready_i) begin
    data_q <= data_i;
  end else begin
    data_q <= data_o;
  end
end

always_comb begin
  data_o = (|wkup_src_q) ? '0 : data_q;
  for(integer i = 0 ; i < WAKEUP_SRC_CNT ; i += 1) begin
    data_o |= wkup_src_q[i] ? wkup_data_i[i] : '0;
  end
end

endmodule
