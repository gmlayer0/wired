`include "wired0_defines.svh"

// Fuction module for Wired project
module wired_alu (
    input   logic [31:0] r0_i,
    input   logic [31:0] r1_i,
    input   logic [31:0] pc_i,
    input   logic [2:0] grand_op_i,
    input   logic [2:0] op_i,

    output  logic [31:0] res_o
  );
  logic [31:0] bw_result, li_result, int_result, sft_result, count_result;

  always_comb
  begin
    case(grand_op_i)
      default: /*_ALU_GTYPE_BW*/
      begin
        res_o = bw_result;
      end
      `_ALU_GTYPE_LI :
      begin
        res_o = li_result;
      end
      `_ALU_GTYPE_INT :
      begin
        res_o = int_result;
      end
      `_ALU_GTYPE_SFT :
      begin
        res_o = sft_result;
      end
      `_ALU_GTYPE_COUNT :
      begin
        res_o = count_result;
      end
    endcase
  end

  always_comb
  begin
    case (op_i)
      default: /*_ALU_STYPE_NOR*/
      begin
        bw_result = ~(r1_i | r0_i);
      end
      `_ALU_STYPE_AND :
      begin
        bw_result = r1_i & r0_i;
      end
      `_ALU_STYPE_OR :
      begin
        bw_result = r1_i | r0_i;
      end
      `_ALU_STYPE_XOR :
      begin
        bw_result = r1_i ^ r0_i;
      end
      `_ALU_STYPE_ANDN :
      begin
        bw_result = r1_i & (~r0_i);
      end
      `_ALU_STYPE_ORN :
      begin
        bw_result = r1_i | (~r0_i);
      end
    endcase
  end

  always_comb
  begin
    case (op_i)
      default:
      begin  // `_ALU_STYPE_LUI
        li_result = {r0_i[19:0], 12'd0};
      end
      `_ALU_STYPE_PCPLUS4:
      begin
        li_result = 32'd4 + pc_i;
      end
      `_ALU_STYPE_PCADDU12I:
      begin
        li_result = {r0_i[19:0], 12'd0} + pc_i;
      end
      `_ALU_STYPE_PCADDI:
      begin
        li_result = {{10{r0_i[19]}}, r0_i[19:0], 2'd0} + pc_i;
      end
      `_ALU_STYPE_PCALAU12I:
      begin
        li_result = {{r0_i[19:0], 12'd0} + pc_i} & 32'hfffff000;
      end
    endcase
  end

  logic[32:0] sub_r,r1,r0;
  logic ext;
  assign ext = ~op_i[0]; // JUDGE SLT, SHARE BETWEEN SUB AND CMP
  assign r1 = {{~r1_i[31] & ext},r1_i};
  assign r0 = {{~r0_i[31] & ext},r0_i};
  assign sub_r = r1 - r0;

  always_comb begin
  case (op_i)
    default: /*_ALU_STYPE_ADD*/
    begin
      int_result = r1_i + r0_i;
    end
    `_ALU_STYPE_SUB:
    begin
      int_result = sub_r[31:0];
    end
    `_ALU_STYPE_SLT:
    begin
      int_result = {31'd0, sub_r[32]};
    end
    `_ALU_STYPE_SLTU:
    begin
      int_result = {31'd0, sub_r[32]};
    end
  endcase
end

always_comb
begin
  case (op_i)
    default/* `_ALU_STYPE_SRA */:
    begin
      sft_result = $signed($signed(r1_i) >>> $signed(r0_i[4:0]));
    end
    `_ALU_STYPE_SRL:
    begin
      sft_result = r1_i >> r0_i[4:0];
    end
    `_ALU_STYPE_SLL:
    begin
      sft_result = r1_i << r0_i[4:0];
    end
    `_ALU_STYPE_ROTR:
    begin
      sft_result = (r1_i >> r0_i[4:0]) | (r1_i << (32-r0_i[4:0]));
    end
  endcase
end

// Count area
wire [5:0] lc, tc; 
cv_lzc #(
  .WIDTH(32),.MODE(1)
) leading_count (
  .in_i(op_i[0] ? r0_i : ~r0_i),
  .cnt_o(lc[4:0]),
  .empty_o(lc[5])
);

cv_lzc #(
  .WIDTH(32),.MODE(0)
) trailing_count (
  .in_i(op_i[0] ? r0_i : ~r0_i),
  .cnt_o(tc[4:0]),
  .empty_o(tc[5])
);

assign count_result = {26'd0, op_i[1] ? {tc[5], {5{~tc[5]}} & tc[4:0]} : {lc[5], {5{~lc[5]}} & lc[4:0]}};



endmodule
