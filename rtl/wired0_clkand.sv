`include "wired0_defines"

module wired0_clock_and (
    input clk_i,
    input en_i,
    output clk_o
  );

  // Modify by your design
  assign clk_o = en_i & clk_i;

endmodule
