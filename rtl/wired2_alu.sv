`include "wired0_defines.svh"

// Fuction module for Wired project
module wired_alu (
    input   logic [31:0] r0_i,
    input   logic [31:0] r1_i,
    input   logic [31:0] pc_i,
    input   logic [1:0] grand_op_i,
    input   logic [1:0] op_i,

    output  logic [31:0] res_o
  );
  logic [31:0] bw_result, li_result, int_result, sft_result;

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
    endcase
  end

  always_comb
  begin
    case (op_i)
      default:
      begin  // 2'b01 == `_ALU_STYPE_LUI
        li_result = {r0_i[19:0], 12'd0};
      end
      `_ALU_STYPE_PCPLUS4:
      begin  // 2'b10
        li_result = 32'd4 + pc_i;
      end
      `_ALU_STYPE_PCADDUI:
      begin  // 2'b11
        li_result = {r0_i[19:0], 12'd0} + pc_i;
      end
    endcase
  end

  logic[32:0] sub_r,r1,r0;
  logic ext;
  assign ext = ~op_i[0]; // JUDGE SLT, SHARE BETWEEN SUB AND CMP
  assign r1 = {{~r1_i[31] & ext},r1_i};
  assign r0 = {{~r0_i[31] & ext},r0_i};
  assign sub_r = r1 - r0;

  always_comb begi
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
      g2_result = $signed($signed(r1_i) >>> $signed(r0_i[4:0]));
    end
    `_ALU_STYPE_SRL:
    begin
      g2_result = r1_i >> r0_i[4:0];
    end
    `_ALU_STYPE_SLL:
    begin
      g2_result = r1_i << r0_i[4:0];
    end
  endcase
end

endmodule
