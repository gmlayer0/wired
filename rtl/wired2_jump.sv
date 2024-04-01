`include "wired0_defines.svh"

// Fuction module for Wired project
module wired_jump (
    input   logic [31:0] r0_i,
    input   logic [31:0] r1_i,
    input   logic [31:0] pc_i,
    input   logic [27:0] addr_imm_i,
    input   logic [1:0]  target_type_i, // 0 for no branch, 1 for call, 2 for return, 3 for immediate
    input   logic [3:0]  cmp_type_i,

    output  logic        jump_o,
    output  logic [31:0] jump_target_o
  );

  // jump_o 逻辑
  logic[32:0] r0,r1;
  assign r0 = {(~r0_i[31]) & cmp_type_i[0],r0_i};
  assign r1 = {(~r1_i[31]) & cmp_type_i[0],r1_i};
  logic[3:1] cmp_result;
  assign cmp_result[3] = r1 < r0;
  assign cmp_result[2] = r1 == r0;
  assign cmp_result[1] = r1 > r0;
  assign jump_o = |(cmp_result & cmp_type_i[3:1]);

  // 目标地址计算逻辑
  assign jump_target_o = {{4{addr_imm_i[27]}}, addr_imm_i} + (target_type == `_TARGET_ABS ? r1 : pc_i);

endmodule
