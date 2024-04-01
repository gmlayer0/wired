`include "tl_util.svh"

module wired_mp import muntjac_pkg::*; #(
  parameter NumCores = 2,
  parameter DmaSourceWidth = 2,
  parameter DeviceSourceWidth = 5,
  parameter SinkWidth = 1,
  parameter AddrWidth = 56,
  parameter bit EnableHpm = 1'b0
) (
  input clk,
  input rst_n,
);

endmodule