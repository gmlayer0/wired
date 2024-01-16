`include "wired0_defines"

module wired1_clock_gate (
    `_WIRED_GENERAL_DEFINE
    input  en_i,
    output clk_o
  );

  reg en_latched;

  always_latch
  begin
    if (!clk_i)
    begin
      en_latched = en_i;
    end
  end

  wired0_clkand clkand(
    .clk_i,
    .en_i(en_latched),
    .clk_o
  );

endmodule
