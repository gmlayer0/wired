`include "wired0_decoder.svh"

module wired_decoder(
    input  logic [31:0] inst_i,
    output logic decode_err_o,
    output decode_info_d_t is_o
);

    always_comb begin
        decode_err_o = 1'b1;
        is_o.ertn_inst = 1'd0;
        is_o.priv_inst = 1'd0;
        is_o.wait_inst = 1'd0;
        is_o.syscall_inst = 1'd0;
        is_o.break_inst = 1'd0;
        is_o.csr_op_en = 1'd0;
        is_o.csr_rdcnt = 2'd0;
        is_o.tlbsrch_en = 1'd0;
        is_o.tlbrd_en = 1'd0;
        is_o.tlbwr_en = 1'd0;
        is_o.tlbfill_en = 1'd0;
        is_o.invtlb_en = 1'd0;
        is_o.fpu_op = 4'd0;
        is_o.fpu_mode = 1'd0;
        is_o.rnd_mode = 4'd0;
        is_o.fpd_inst = 1'd0;
        is_o.fcsr_upd = 1'd0;
        is_o.fcmp = 1'd0;
        is_o.fcsr2gr = 1'd0;
        is_o.gr2fcsr = 1'd0;
        is_o.upd_fcc = 1'd0;
        is_o.fsel = 1'd0;
        is_o.fclass = 1'd0;
        is_o.bceqz = 1'd0;
        is_o.bcnez = 1'd0;
        is_o.inst = inst_i;
        is_o.alu_inst = 1'd0;
        is_o.mul_inst = 1'd0;
        is_o.div_inst = 1'd0;
        is_o.lsu_inst = 1'd0;
        is_o.fpu_inst = 1'd0;
        is_o.fbranch_inst = 1'd0;
        is_o.reg_type_r0 = `_REG_ZERO;
        is_o.reg_type_r1 = `_REG_ZERO;
        is_o.reg_type_w = `_REG_W_NONE;
        is_o.imm_type = `_IMM_U5;
        is_o.addr_imm_type = `_ADDR_IMM_S26;
        is_o.slot0 = 1'd0;
        is_o.refetch = 1'd0;
        is_o.need_fa = 1'd0;
        is_o.fr0 = 1'd0;
        is_o.fr1 = 1'd0;
        is_o.fr2 = 1'd0;
        is_o.fw = 1'd0;
        is_o.alu_grand_op = 3'd0;
        is_o.alu_op = 3'd0;
        is_o.target_type = 1'd0;
        is_o.cmp_type = 4'd0;
        is_o.jump_inst = 1'd0;
        is_o.mem_type = 3'd0;
        is_o.mem_write = 1'd0;
        is_o.mem_read = 1'd0;
        is_o.mem_cacop = 1'd0;
        is_o.llsc_inst = 1'd0;
        is_o.dbarrier = 1'd0;
        unique casez(inst_i)
            32'b010000??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S21;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_E;
                is_o.jump_inst = 1'd1;
            end
            32'b010001??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S21;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NE;
                is_o.jump_inst = 1'd1;
            end
            32'b010011??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCPLUS4;
                is_o.target_type = `_TARGET_ABS;
                is_o.cmp_type = `_CMP_NOCONDITION;
                is_o.jump_inst = 1'd1;
            end
            32'b010100??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NOCONDITION;
                is_o.jump_inst = 1'd1;
            end
            32'b010101??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_BL1;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.slot0 = 1'd1;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCPLUS4;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NOCONDITION;
                is_o.jump_inst = 1'd1;
            end
            32'b010110??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_E;
                is_o.jump_inst = 1'd1;
            end
            32'b010111??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_NE;
                is_o.jump_inst = 1'd1;
            end
            32'b011000??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_LT;
                is_o.jump_inst = 1'd1;
            end
            32'b011001??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_GE;
                is_o.jump_inst = 1'd1;
            end
            32'b011010??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_LTU;
                is_o.jump_inst = 1'd1;
            end
            32'b011011??????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S16;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_GEU;
                is_o.jump_inst = 1'd1;
            end
            32'b0001010?????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S20;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_LUI;
            end
            32'b0001100?????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S20;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCADDI;
            end
            32'b0001101?????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S20;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCALAU12I;
            end
            32'b0001110?????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S20;
                is_o.alu_grand_op = `_ALU_GTYPE_LI;
                is_o.alu_op = `_ALU_STYPE_PCADDU12I;
            end
            32'b00000100????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.csr_op_en = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b00100000????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S14;
                is_o.slot0 = 1'd1;
                is_o.mem_type = `_MEM_TYPE_WORD;
                is_o.mem_read = 1'd1;
                is_o.llsc_inst = 1'd1;
            end
            32'b00100001????????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S14;
                is_o.slot0 = 1'd1;
                is_o.mem_type = `_MEM_TYPE_WORD;
                is_o.mem_write = 1'd1;
                is_o.llsc_inst = 1'd1;
            end
            32'b0000001000??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S12;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLT;
            end
            32'b0000001001??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S12;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLTU;
            end
            32'b0000001010??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_S12;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_ADD;
            end
            32'b0000001101??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U12;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_AND;
            end
            32'b0000001110??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U12;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b0000001111??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U12;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_XOR;
            end
            32'b0000011000??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
                is_o.mem_type = `_MEM_TYPE_BYTE;
                is_o.mem_cacop = 1'd1;
            end
            32'b0010100000??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.mem_type = `_MEM_TYPE_BYTE;
                is_o.mem_read = 1'd1;
            end
            32'b0010100001??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.mem_type = `_MEM_TYPE_HALF;
                is_o.mem_read = 1'd1;
            end
            32'b0010100010??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.mem_type = `_MEM_TYPE_WORD;
                is_o.mem_read = 1'd1;
            end
            32'b0010100100??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.slot0 = 1'd1;
                is_o.mem_type = `_MEM_TYPE_BYTE;
                is_o.mem_write = 1'd1;
            end
            32'b0010100101??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.slot0 = 1'd1;
                is_o.mem_type = `_MEM_TYPE_HALF;
                is_o.mem_write = 1'd1;
            end
            32'b0010100110??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.slot0 = 1'd1;
                is_o.mem_type = `_MEM_TYPE_WORD;
                is_o.mem_write = 1'd1;
            end
            32'b0010101000??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.mem_type = `_MEM_TYPE_UBYTE;
                is_o.mem_read = 1'd1;
            end
            32'b0010101001??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.mem_type = `_MEM_TYPE_UHALF;
                is_o.mem_read = 1'd1;
            end
            32'b0010101011??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
            end
            32'b0010101100??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_ZERO;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.fw = 1'd1;
                is_o.mem_type = `_MEM_TYPE_WORD;
                is_o.mem_read = 1'd1;
            end
            32'b0010101101??????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.lsu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_NONE;
                is_o.addr_imm_type = `_ADDR_IMM_S12;
                is_o.slot0 = 1'd1;
                is_o.fr0 = 1'd1;
                is_o.mem_type = `_MEM_TYPE_WORD;
                is_o.mem_write = 1'd1;
            end
            32'b000010000001????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::FMADD;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fr2 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b000010000101????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::FMADD;
                is_o.fpu_mode = 1'd1;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fr2 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b000010001001????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::FNMSUB;
                is_o.fpu_mode = 1'd1;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fr2 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b000010001101????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::FNMSUB;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fr2 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b000011000001????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.fcmp = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
            end
            32'b000011010000????????????????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.fsel = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000000000010??????????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_BYTEPICK;
            end
            32'b00000000000100000???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_ADD;
            end
            32'b00000000000100010???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SUB;
            end
            32'b00000000000100100???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLT;
            end
            32'b00000000000100101???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_INT;
                is_o.alu_op = `_ALU_STYPE_SLTU;
            end
            32'b00000000000100110???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_MASKEQZ;
            end
            32'b00000000000100111???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_MASKNEZ;
            end
            32'b00000000000101000???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_NOR;
            end
            32'b00000000000101001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_AND;
            end
            32'b00000000000101010???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b00000000000101011???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_XOR;
            end
            32'b00000000000101100???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_ORN;
            end
            32'b00000000000101101???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_ANDN;
            end
            32'b00000000000101110???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SLL;
            end
            32'b00000000000101111???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRL;
            end
            32'b00000000000110000???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRA;
            end
            32'b00000000000110110???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_ROTR;
            end
            32'b00000000000111000???????????????: begin
                decode_err_o = 1'b0;
                is_o.mul_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_MULL;
            end
            32'b00000000000111001???????????????: begin
                decode_err_o = 1'b0;
                is_o.mul_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_MULH;
            end
            32'b00000000000111010???????????????: begin
                decode_err_o = 1'b0;
                is_o.mul_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_MULHU;
            end
            32'b00000000001000000???????????????: begin
                decode_err_o = 1'b0;
                is_o.div_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_DIV;
            end
            32'b00000000001000001???????????????: begin
                decode_err_o = 1'b0;
                is_o.div_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_MOD;
            end
            32'b00000000001000010???????????????: begin
                decode_err_o = 1'b0;
                is_o.div_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_DIVU;
            end
            32'b00000000001000011???????????????: begin
                decode_err_o = 1'b0;
                is_o.div_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_op = `_MDU_TYPE_MODU;
            end
            32'b00000000001010100???????????????: begin
                decode_err_o = 1'b0;
                is_o.break_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
            end
            32'b00000000001010110???????????????: begin
                decode_err_o = 1'b0;
                is_o.syscall_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
            end
            32'b00000000010000001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SLL;
            end
            32'b00000000010001001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRL;
            end
            32'b00000000010010001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_SRA;
            end
            32'b00000000010011001???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_U5;
                is_o.alu_grand_op = `_ALU_GTYPE_SFT;
                is_o.alu_op = `_ALU_STYPE_ROTR;
            end
            32'b00000000011?????0???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RD;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_BSTRINS;
            end
            32'b00000000011?????1???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.addr_imm_type = `_ADDR_IMM_S26;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_BSTRPICK;
            end
            32'b00000001000000001???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::ADD;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000001000000101???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::ADD;
                is_o.fpu_mode = 1'd1;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000001000001001???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::MUL;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000001000001101???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::DIV;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000001000010001???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::MINMAX;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RTZ};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000001000010101???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::MINMAX;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RNE};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000001000100101???????????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::SGNJ;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RNE};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RK;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b00000110010010001???????????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.wait_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b00000110010010011???????????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.invtlb_en = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RK;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b00111000011100100???????????????: begin
                decode_err_o = 1'b0;
                is_o.lsu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
                is_o.dbarrier = 1'd1;
            end
            32'b00111000011100101???????????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000000000000000000100??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_COUNT;
                is_o.alu_op = `_ALU_STYPE_CLO;
            end
            32'b0000000000000000000101??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_COUNT;
                is_o.alu_op = `_ALU_STYPE_CLZ;
            end
            32'b0000000000000000000110??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_COUNT;
                is_o.alu_op = `_ALU_STYPE_CTO;
            end
            32'b0000000000000000000111??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_COUNT;
                is_o.alu_op = `_ALU_STYPE_CTZ;
            end
            32'b0000000000000000001100??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_REV;
                is_o.alu_op = `_ALU_STYPE_REV;
            end
            32'b0000000000000000010010??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_REV;
                is_o.alu_op = `_ALU_STYPE_BITREV4B;
            end
            32'b0000000000000000010100??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_REV;
                is_o.alu_op = `_ALU_STYPE_BITREVW;
            end
            32'b0000000000000000010110??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_EXTH;
            end
            32'b0000000000000000010111??????????: begin
                decode_err_o = 1'b0;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_ZERO;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.alu_grand_op = `_ALU_GTYPE_MISC;
                is_o.alu_op = `_ALU_STYPE_EXTB;
            end
            32'b0000000000000000011000??????????: begin
                decode_err_o = 1'b0;
                is_o.csr_rdcnt = `_RDCNT_ID_VLOW;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_w = `_REG_W_RJD;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000000000000000011001??????????: begin
                decode_err_o = 1'b0;
                is_o.csr_rdcnt = `_RDCNT_VHIGH;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_w = `_REG_W_RJD;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000000100010100000001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::SGNJ;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RDN};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100010100000101??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::SGNJ;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RTZ};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100010100001101??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.fclass = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100010100010001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::SQRT;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100010100010101??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::DIV;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_IMM;
                is_o.reg_type_r1 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.imm_type = `_IMM_F1;
                is_o.fr1 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100010100100101??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b0000000100010100101001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fw = 1'd1;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b0000000100010100101101??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b0000000100010100110000??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.gr2fcsr = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
                is_o.alu_grand_op = `_ALU_GTYPE_BW;
                is_o.alu_op = `_ALU_STYPE_OR;
            end
            32'b0000000100010100110010??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.fcsr2gr = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000000100010100110100??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.upd_fcc = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.fr0 = 1'd1;
            end
            32'b0000000100010100110101??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fw = 1'd1;
            end
            32'b0000000100010100110110??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.upd_fcc = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
            end
            32'b0000000100010100110111??????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.reg_type_w = `_REG_W_RD;
            end
            32'b0000000100011010000001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::F2I;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RDN};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100011010010001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::F2I;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RUP};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100011010100001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::F2I;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RTZ};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100011010110001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::F2I;
                is_o.fpu_mode = 1'd0;
                is_o.rnd_mode = {1'd1,fpnew_pkg::RNE};
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100011011000001??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::F2I;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000000100011101000100??????????: begin
                decode_err_o = 1'b0;
                is_o.fpu_op = fpnew_pkg::I2F;
                is_o.fpu_mode = 1'd0;
                is_o.fpd_inst = 1'd1;
                is_o.fpu_inst = 1'd1;
                is_o.reg_type_r0 = `_REG_RJ;
                is_o.reg_type_w = `_REG_W_RD;
                is_o.fr0 = 1'd1;
                is_o.fw = 1'd1;
            end
            32'b0000011001001000001010??????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.tlbsrch_en = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000011001001000001011??????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.tlbrd_en = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000011001001000001100??????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.tlbwr_en = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000011001001000001101??????????: begin
                decode_err_o = 1'b0;
                is_o.priv_inst = 1'd1;
                is_o.tlbfill_en = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b0000011001001000001110??????????: begin
                decode_err_o = 1'b0;
                is_o.ertn_inst = 1'd1;
                is_o.priv_inst = 1'd1;
                is_o.alu_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.refetch = 1'd1;
            end
            32'b010010????????????????00????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.bceqz = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_E;
                is_o.jump_inst = 1'd1;
            end
            32'b010010????????????????01????????: begin
                decode_err_o = 1'b0;
                is_o.fpd_inst = 1'd1;
                is_o.bcnez = 1'd1;
                is_o.fbranch_inst = 1'd1;
                is_o.slot0 = 1'd1;
                is_o.target_type = `_TARGET_REL;
                is_o.cmp_type = `_CMP_E;
                is_o.jump_inst = 1'd1;
            end
        endcase
    end

endmodule
