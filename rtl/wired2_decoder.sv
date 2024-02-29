`include "wired0_decoder.svh"

module wired_decoder(
    input  logic [31:0] inst_i,
    output logic decode_err_o,
    output is_t is_o
);

    always_comb begin
        decode_err_o = 1'b1;
        is_o.inst = inst_i;
        is_o.alu_inst = 1'd0;
        is_o.mdu_inst = 1'd0;
        is_o.lsu_inst = 1'd0;
        is_o.reg_type_r0 = `_REG_R0_NONE;
        is_o.reg_type_r1 = `_REG_R1_NONE;
        is_o.reg_type_w = `_REG_W_NONE;
        is_o.imm_type = `_IMM_U5;
        is_o.addr_imm_type = `_ADDR_IMM_S26;
        is_o.slot0 = 1'd0;
        is_o.alu_grand_op = 2'd0;
        is_o.alu_op = 2'd0;
        is_o.target_type = 1'd0;
        is_o.cmp_type = 4'd0;
        is_o.jump_inst = 1'd0;
        unique casez(inst_i)
            32'b010011??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_NONE;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCPLUS4;
                is_o.target_type = `_TARGET_ABS;
                is_o.cmp_type = `_CMP_NOCONDITION;
                is_o.jump_inst = 1'd1;
            end
            32'b010100??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_NONE;
                is_o.reg_type_r1 = `_REG_R1_NONE;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NOCONDITION;
                is_o.jump_inst = 1'd1;
            end
            32'b010101??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_NONE;
                is_o.reg_type_r1 = `_REG_R1_NONE;
                is_o.reg_type_w = `_REG_W_BL1;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCPLUS4;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NOCONDITION;
                is_o.jump_inst = 1'd1;
            end
            32'b010110??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RD;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_E;
                is_o.jump_inst = 1'd1;
            end
            32'b010111??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RD;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NE;
                is_o.jump_inst = 1'd1;
            end
            32'b011000??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RD;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_LT;
                is_o.jump_inst = 1'd1;
            end
            32'b011001??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RD;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_GE;
                is_o.jump_inst = 1'd1;
            end
            32'b011010??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RD;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_LTU;
                is_o.jump_inst = 1'd1;
            end
            32'b011011??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RD;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_GEU;
                is_o.jump_inst = 1'd1;
            end
            32'b0001010?????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_NONE;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S20;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_LUI;
            end
            32'b0001110?????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_NONE;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S20;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCADDUI;
            end
            32'b0000001000??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S12;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLT;
            end
            32'b0000001001??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S12;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLTU;
            end
            32'b0000001010??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S12;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_ADD;
            end
            32'b0000001101??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U12;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_AND;
            end
            32'b0000001110??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U12;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b0000001111??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U12;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_XOR;
            end
            32'b00000000000100000???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_ADD;
            end
            32'b00000000000100010???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SUB;
            end
            32'b00000000000100100???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLT;
            end
            32'b00000000000100101???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLTU;
            end
            32'b00000000000101000???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_NOR;
            end
            32'b00000000000101001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_AND;
            end
            32'b00000000000101010???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b00000000000101011???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_XOR;
            end
            32'b00000000000101110???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SLL;
            end
            32'b00000000000101111???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRL;
            end
            32'b00000000000110000???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRA;
            end
            32'b00000000000111000???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MUL;
                is_o.alu_op = `_MUL_TYPE_MULL;
            end
            32'b00000000000111001???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MUL;
                is_o.alu_op = `_MUL_TYPE_MULH;
            end
            32'b00000000000111010???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MUL;
                is_o.alu_op = `_MUL_TYPE_MULHU;
            end
            32'b00000000001000000???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_DIV_TYPE_DIV;
            end
            32'b00000000001000001???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_DIV_TYPE_MOD;
            end
            32'b00000000001000010???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_DIV_TYPE_DIVU;
            end
            32'b00000000001000011???????????????: begin
                decode_err_o = 1'b0;
                is_o.mdu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_RK;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_DIV_TYPE_MODU;
            end
            32'b00000000010000001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SLL;
            end
            32'b00000000010001001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRL;
            end
            32'b00000000010010001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_R0_IMM;
                is_o.reg_type_r1 = `_REG_R1_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRA;
            end
        endcase
    end

endmodule
