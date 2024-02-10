`ifndef _DECODE_HEADER
`define _DECODE_HEADER

`define _REG_R0_IMM (2'b11)
`define _REG_R0_RD (2'b10)
`define _REG_R0_RK (2'b01)
`define _REG_R0_NONE (2'b00)
`define _REG_R1_RJ (1'b1)
`define _REG_R1_NONE (1'b0)
`define _REG_W_BL1 (2'b11)
`define _REG_W_RJD (2'b10)
`define _REG_W_RD (2'b01)
`define _REG_W_NONE (2'b00)
`define _IMM_U12 (3'd0)
`define _IMM_U5 (3'd0)
`define _IMM_S12 (3'd1)
`define _IMM_S20 (3'd2)
`define _IMM_S16 (3'd3)
`define _IMM_S21 (3'd4)
`define _ADDR_IMM_S26 (2'd0)
`define _ADDR_IMM_S12 (2'd1)
`define _ADDR_IMM_S14 (2'd2)
`define _ADDR_IMM_S16 (2'd3)
`define _ALU_GTYPE_BW (2'd0)
`define _ALU_GTYPE_LI (2'd1)
`define _ALU_GTYPE_INT (2'd2)
`define _ALU_GTYPE_SFT (2'd3)
`define _ALU_STYPE_NOR (2'b00)
`define _ALU_STYPE_AND (2'b01)
`define _ALU_STYPE_OR (2'b10)
`define _ALU_STYPE_XOR (2'b11)
`define _ALU_STYPE_PCPLUS4 (2'b10)
`define _ALU_STYPE_PCADDUI (2'b11)
`define _ALU_STYPE_LUI (2'b01)
`define _ALU_STYPE_ADD (2'b00)
`define _ALU_STYPE_SUB (2'b01)
`define _ALU_STYPE_SLT (2'b10)
`define _ALU_STYPE_SLTU (2'b11)
`define _ALU_STYPE_SRA (2'b00)
`define _ALU_STYPE_SLL (2'b10)
`define _ALU_STYPE_SRL (2'b11)
`define _MDU_TYPE_MULL (3'b000)
`define _MDU_TYPE_MULH (3'b001)
`define _MDU_TYPE_MULHU (3'b010)
`define _MDU_TYPE_DIV (3'b100)
`define _MDU_TYPE_DIVU (3'b101)
`define _MDU_TYPE_MOD (3'b110)
`define _MDU_TYPE_MODU (3'b111)

typedef logic [31 : 0] inst_t;
typedef logic [0 : 0] alu_inst_t;
typedef logic [0 : 0] mdu_inst_t;
typedef logic [0 : 0] lsu_inst_t;
typedef logic [1 : 0] reg_type_r0_t;
typedef logic [0 : 0] reg_type_r1_t;
typedef logic [1 : 0] reg_type_w_t;
typedef logic [2 : 0] imm_type_t;
typedef logic [1 : 0] addr_imm_type_t;
typedef logic [1 : 0] alu_grand_op_t;
typedef logic [1 : 0] alu_op_t;

typedef struct packed {
    inst_t inst;
} c_t;

typedef struct packed {
    inst_t inst;
} mdu_t;

typedef struct packed {
    inst_t inst;
} lsu_t;

typedef struct packed {
    alu_grand_op_t alu_grand_op;
    alu_op_t alu_op;
    inst_t inst;
} alu_t;

typedef struct packed {
    alu_inst_t alu_inst;
    mdu_inst_t mdu_inst;
    lsu_inst_t lsu_inst;
    inst_t inst;
    inst_t inst;
    alu_grand_op_t alu_grand_op;
    alu_op_t alu_op;
    inst_t inst;
} p_t;

typedef struct packed {
    alu_inst_t alu_inst;
    mdu_inst_t mdu_inst;
    lsu_inst_t lsu_inst;
    inst_t inst;
    inst_t inst;
    alu_grand_op_t alu_grand_op;
    alu_op_t alu_op;
    inst_t inst;
} r_t;

typedef struct packed {
    reg_type_r0_t reg_type_r0;
    reg_type_r1_t reg_type_r1;
    reg_type_w_t reg_type_w;
    imm_type_t imm_type;
    addr_imm_type_t addr_imm_type;
    alu_inst_t alu_inst;
    mdu_inst_t mdu_inst;
    lsu_inst_t lsu_inst;
    inst_t inst;
    inst_t inst;
    alu_grand_op_t alu_grand_op;
    alu_op_t alu_op;
    inst_t inst;
} d_t;

function automatic c_t get_c_from_mdu(input mdu_t mdu);
    c_t ret;
    ret.inst = mdu.inst;
    return ret;
endfunction

function automatic c_t get_c_from_lsu(input lsu_t lsu);
    c_t ret;
    ret.inst = lsu.inst;
    return ret;
endfunction

function automatic c_t get_c_from_alu(input alu_t alu);
    c_t ret;
    ret.inst = alu.inst;
    return ret;
endfunction

function automatic mdu_t get_mdu_from_p(input p_t p);
    mdu_t ret;
    ret.inst = p.inst;
    return ret;
endfunction

function automatic lsu_t get_lsu_from_p(input p_t p);
    lsu_t ret;
    ret.inst = p.inst;
    return ret;
endfunction

function automatic alu_t get_alu_from_p(input p_t p);
    alu_t ret;
    ret.alu_grand_op = p.alu_grand_op;
    ret.alu_op = p.alu_op;
    ret.inst = p.inst;
    return ret;
endfunction

function automatic p_t get_p_from_r(input r_t r);
    p_t ret;
    ret.alu_inst = r.alu_inst;
    ret.mdu_inst = r.mdu_inst;
    ret.lsu_inst = r.lsu_inst;
    ret.inst = r.inst;
    ret.inst = r.inst;
    ret.alu_grand_op = r.alu_grand_op;
    ret.alu_op = r.alu_op;
    ret.inst = r.inst;
    return ret;
endfunction

function automatic r_t get_r_from_d(input d_t d);
    r_t ret;
    ret.alu_inst = d.alu_inst;
    ret.mdu_inst = d.mdu_inst;
    ret.lsu_inst = d.lsu_inst;
    ret.inst = d.inst;
    ret.inst = d.inst;
    ret.alu_grand_op = d.alu_grand_op;
    ret.alu_op = d.alu_op;
    ret.inst = d.inst;
    return ret;
endfunction

function automatic d_t get_d_from_Entry(input Entry_t Entry);
    d_t ret;
    ret.reg_type_r0 = Entry.reg_type_r0;
    ret.reg_type_r1 = Entry.reg_type_r1;
    ret.reg_type_w = Entry.reg_type_w;
    ret.imm_type = Entry.imm_type;
    ret.addr_imm_type = Entry.addr_imm_type;
    ret.alu_inst = Entry.alu_inst;
    ret.mdu_inst = Entry.mdu_inst;
    ret.lsu_inst = Entry.lsu_inst;
    ret.inst = Entry.inst;
    ret.inst = Entry.inst;
    ret.alu_grand_op = Entry.alu_grand_op;
    ret.alu_op = Entry.alu_op;
    ret.inst = Entry.inst;
    return ret;
endfunction

`endif
