`include "wired0_defines.svh"

module wired_registers_file_latch #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter bit NEED_RESET = 0,
    parameter logic[DATA_WIDTH-1:0] RESET_VAL = '0,

    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,
    input    [ADDR_DEPTH-1:0] waddr_i,
    input                        we_i,
    input  T                  wdata_i,

    output T [DEPTH-1:0] regfiles_o
);

  // 全局时钟使能
  wire clk_int;
  wired_clock_gate gclock_gate (
    `_WIRED_GENERAL_CONN,
    .en_i(we_i || (NEED_RESET && !rst_n)),
    .clk_o(clk_int)
  );

  // 对输入数据加一级 FF
  reg [DATA_WIDTH-1:0] wdata_q;
  always_ff @(posedge clk_int) begin
    wdata_q <= rst_n ? wdata_i : RESET_VAL;
  end

  // 写地址解码
  wire [DEPTH - 1 : 0] addr_decode;
  for(genvar i = 0 ; i < DEPTH ; i++) begin
    assign addr_decode[i[$clog2(DEPTH)-1:0]] = (we_i && (waddr_i == i[$clog2(DEPTH)-1:0])) || (NEED_RESET && !rst_n);
  end

  // latch 时钟生成
  wire [DEPTH - 1 : 0] mem_clock;
  for(genvar i = 0 ; i < DEPTH ; i++) begin
    wired_clock_gate iclock_gate (
      `_WIRED_GENERAL_CONN,
      .en_i(addr_decode[i[$clog2(DEPTH)-1:0]]),
      .clk_o(mem_clock[i[$clog2(DEPTH)-1:0]])
    );
  end

  // latch 堆
  reg[DEPTH - 1 : 0][DATA_WIDTH - 1 : 0] regfiles_l;
  for(genvar i = 0 ; i < DEPTH ; i++) begin
    always_latch begin
      if(mem_clock[i[$clog2(DEPTH)-1:0]]) begin
        regfiles_l[i[$clog2(DEPTH)-1:0]] = wdata_q;
      end
    end
  end
  assign regfiles_o = regfiles_l;

endmodule
