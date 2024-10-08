`ifndef _DECODE_HEADER
`define _DECODE_HEADER

`define _INV_TLB_ALL (4'b1111)
`define _INV_TLB_MASK_G (4'b1000)
`define _INV_TLB_MASK_NG (4'b0100)
`define _INV_TLB_MASK_ASID (4'b0010)
`define _INV_TLB_MASK_VA (4'b0001)
`define _RDCNT_NONE (2'd0)
`define _RDCNT_ID_VLOW (2'd1)
`define _RDCNT_VHIGH (2'd2)
`define _RDCNT_VLOW (2'd3)
`define _REG_ZERO (3'b000)
`define _REG_RD (3'b001)
`define _REG_RJ (3'b010)
`define _REG_RK (3'b011)
`define _REG_IMM (3'b100)
`define _REG_W_NONE (2'b00)
`define _REG_W_RD (2'b01)
`define _REG_W_RJD (2'b10)
`define _REG_W_BL1 (2'b11)
`define _IMM_U12 (3'd0)
`define _IMM_U5 (3'd0)
`define _IMM_S12 (3'd1)
`define _IMM_S20 (3'd2)
`define _IMM_S16 (3'd3)
`define _IMM_F1 (3'd4)
`define _IMM_S21 (3'd5)
`define _ADDR_IMM_S26 (3'd0)
`define _ADDR_IMM_S12 (3'd1)
`define _ADDR_IMM_S14 (3'd2)
`define _ADDR_IMM_S16 (3'd3)
`define _ADDR_IMM_S21 (3'd4)
`define _ALU_GTYPE_BW (3'd0)
`define _ALU_GTYPE_LI (3'd1)
`define _ALU_GTYPE_INT (3'd2)
`define _ALU_GTYPE_SFT (3'd3)
`define _ALU_GTYPE_COUNT (3'd4)
`define _ALU_GTYPE_MISC (3'd5)
`define _ALU_GTYPE_REV (3'd6)
`define _ALU_STYPE_NOR (3'b000)
`define _ALU_STYPE_AND (3'b001)
`define _ALU_STYPE_OR (3'b010)
`define _ALU_STYPE_XOR (3'b011)
`define _ALU_STYPE_ANDN (3'b100)
`define _ALU_STYPE_ORN (3'b101)
`define _ALU_STYPE_LUI (3'b00)
`define _ALU_STYPE_PCPLUS4 (3'b01)
`define _ALU_STYPE_PCADDU12I (3'b10)
`define _ALU_STYPE_PCADDI (3'b11)
`define _ALU_STYPE_PCALAU12I (3'b100)
`define _ALU_STYPE_ADD (3'b00)
`define _ALU_STYPE_SUB (3'b01)
`define _ALU_STYPE_SLT (3'b10)
`define _ALU_STYPE_SLTU (3'b11)
`define _ALU_STYPE_SRL (3'b000)
`define _ALU_STYPE_SLL (3'b001)
`define _ALU_STYPE_SRA (3'b010)
`define _ALU_STYPE_ROTR (3'b100)
`define _ALU_STYPE_CLO (3'b100)
`define _ALU_STYPE_CLZ (3'b101)
`define _ALU_STYPE_CTO (3'b110)
`define _ALU_STYPE_CTZ (3'b111)
`define _ALU_STYPE_EXTB (3'b000)
`define _ALU_STYPE_EXTH (3'b001)
`define _ALU_STYPE_BYTEPICK (3'b010)
`define _ALU_STYPE_ALSL (3'B011)
`define _ALU_STYPE_MASKNEZ (3'b100)
`define _ALU_STYPE_MASKEQZ (3'b101)
`define _ALU_STYPE_BSTRPICK (3'b110)
`define _ALU_STYPE_BSTRINS (3'b111)
`define _ALU_STYPE_REV (3'b000)
`define _ALU_STYPE_BITREV4B (3'b001)
`define _ALU_STYPE_BITREVW (3'b010)
`define _MDU_TYPE_MULL (3'b00)
`define _MDU_TYPE_MULH (3'b01)
`define _MDU_TYPE_MULHU (3'b11)
`define _MDU_TYPE_DIV (3'b00)
`define _MDU_TYPE_DIVU (3'b01)
`define _MDU_TYPE_MOD (3'b10)
`define _MDU_TYPE_MODU (3'b11)
`define _TARGET_REL (1'b0)
`define _TARGET_ABS (1'b1)
`define _CMP_NOCONDITION (4'b1110)
`define _CMP_E (4'b0100)
`define _CMP_NE (4'b1010)
`define _CMP_LE (4'b1101)
`define _CMP_GT (4'b0011)
`define _CMP_LT (4'b1001)
`define _CMP_GE (4'b0111)
`define _CMP_LTU (4'b1000)
`define _CMP_GEU (4'b0110)
`define _MEM_TYPE_NONE (3'd0)
`define _MEM_TYPE_WORD (3'd1)
`define _MEM_TYPE_HALF (3'd2)
`define _MEM_TYPE_BYTE (3'd3)
`define _MEM_TYPE_UWORD (3'd5)
`define _MEM_TYPE_UHALF (3'd6)
`define _MEM_TYPE_UBYTE (3'd7)

typedef logic [0 : 0] ertn_inst_t;
typedef logic [0 : 0] priv_inst_t;
typedef logic [0 : 0] wait_inst_t;
typedef logic [0 : 0] syscall_inst_t;
typedef logic [0 : 0] break_inst_t;
typedef logic [0 : 0] csr_op_en_t;
typedef logic [1 : 0] csr_rdcnt_t;
typedef logic [0 : 0] tlbsrch_en_t;
typedef logic [0 : 0] tlbrd_en_t;
typedef logic [0 : 0] tlbwr_en_t;
typedef logic [0 : 0] tlbfill_en_t;
typedef logic [0 : 0] invtlb_en_t;
typedef logic [3 : 0] fpu_op_t;
typedef logic [0 : 0] fpu_mode_t;
typedef logic [3 : 0] rnd_mode_t;
typedef logic [0 : 0] fpd_inst_t;
typedef logic [0 : 0] fcsr_upd_t;
typedef logic [0 : 0] fcmp_t;
typedef logic [0 : 0] fcsr2gr_t;
typedef logic [0 : 0] gr2fcsr_t;
typedef logic [0 : 0] upd_fcc_t;
typedef logic [0 : 0] fsel_t;
typedef logic [0 : 0] fclass_t;
typedef logic [0 : 0] bceqz_t;
typedef logic [0 : 0] bcnez_t;
typedef logic [31 : 0] inst_t;
typedef logic [0 : 0] alu_inst_t;
typedef logic [0 : 0] mul_inst_t;
typedef logic [0 : 0] div_inst_t;
typedef logic [0 : 0] lsu_inst_t;
typedef logic [0 : 0] fpu_inst_t;
typedef logic [0 : 0] fbranch_inst_t;
typedef logic [2 : 0] reg_type_r0_t;
typedef logic [2 : 0] reg_type_r1_t;
typedef logic [1 : 0] reg_type_w_t;
typedef logic [2 : 0] imm_type_t;
typedef logic [2 : 0] addr_imm_type_t;
typedef logic [0 : 0] slot0_t;
typedef logic [0 : 0] refetch_t;
typedef logic [0 : 0] need_fa_t;
typedef logic [0 : 0] fr0_t;
typedef logic [0 : 0] fr1_t;
typedef logic [0 : 0] fr2_t;
typedef logic [0 : 0] fw_t;
typedef logic [2 : 0] alu_grand_op_t;
typedef logic [2 : 0] alu_op_t;
typedef logic [0 : 0] target_type_t;
typedef logic [3 : 0] cmp_type_t;
typedef logic [0 : 0] jump_inst_t;
typedef logic [2 : 0] mem_type_t;
typedef logic [0 : 0] mem_write_t;
typedef logic [0 : 0] mem_read_t;
typedef logic [0 : 0] mem_cacop_t;
typedef logic [0 : 0] llsc_inst_t;
typedef logic [0 : 0] dbarrier_t;

typedef struct packed {
    fcmp_t fcmp;
    upd_fcc_t upd_fcc;
} decode_info_c_fcc_common_t;

typedef struct packed {
    cmp_type_t cmp_type;
    csr_op_en_t csr_op_en;
    invtlb_en_t invtlb_en;
    target_type_t target_type;
} decode_info_c_alu_common_t;

typedef struct packed {
    alu_op_t alu_op;
} decode_info_alu_mdu_common_t;

typedef struct packed {
    dbarrier_t dbarrier;
    llsc_inst_t llsc_inst;
    mem_cacop_t mem_cacop;
    mem_read_t mem_read;
    mem_write_t mem_write;
} decode_info_c_lsu_common_t;

typedef struct packed {
    cmp_type_t cmp_type;
    csr_op_en_t csr_op_en;
    dbarrier_t dbarrier;
    fcmp_t fcmp;
    inst_t inst;
    invtlb_en_t invtlb_en;
    jump_inst_t jump_inst;
    llsc_inst_t llsc_inst;
    lsu_inst_t lsu_inst;
    mem_cacop_t mem_cacop;
    mem_read_t mem_read;
    mem_write_t mem_write;
    refetch_t refetch;
    target_type_t target_type;
    upd_fcc_t upd_fcc;
    wait_inst_t wait_inst;
} decode_info_c_t;

typedef struct packed {
    bceqz_t bceqz;
    bcnez_t bcnez;
    fclass_t fclass;
    fcmp_t fcmp;
    fsel_t fsel;
    upd_fcc_t upd_fcc;
} decode_info_fcc_t;

typedef struct packed {
    fpu_mode_t fpu_mode;
    fpu_op_t fpu_op;
    rnd_mode_t rnd_mode;
} decode_info_fpu_t;

typedef struct packed {
    alu_op_t alu_op;
} decode_info_mdu_t;

typedef struct packed {
    dbarrier_t dbarrier;
    llsc_inst_t llsc_inst;
    mem_cacop_t mem_cacop;
    mem_read_t mem_read;
    mem_type_t mem_type;
    mem_write_t mem_write;
} decode_info_lsu_t;

typedef struct packed {
    alu_grand_op_t alu_grand_op;
    alu_op_t alu_op;
    cmp_type_t cmp_type;
    csr_op_en_t csr_op_en;
    invtlb_en_t invtlb_en;
    target_type_t target_type;
} decode_info_alu_t;

typedef struct packed {
    cmp_type_t cmp_type;
    csr_op_en_t csr_op_en;
    csr_rdcnt_t csr_rdcnt;
    dbarrier_t dbarrier;
    ertn_inst_t ertn_inst;
    fcmp_t fcmp;
    fcsr2gr_t fcsr2gr;
    fcsr_upd_t fcsr_upd;
    gr2fcsr_t gr2fcsr;
    inst_t inst;
    invtlb_en_t invtlb_en;
    jump_inst_t jump_inst;
    llsc_inst_t llsc_inst;
    lsu_inst_t lsu_inst;
    mem_cacop_t mem_cacop;
    mem_read_t mem_read;
    mem_write_t mem_write;
    priv_inst_t priv_inst;
    refetch_t refetch;
    slot0_t slot0;
    target_type_t target_type;
    tlbfill_en_t tlbfill_en;
    tlbrd_en_t tlbrd_en;
    tlbsrch_en_t tlbsrch_en;
    tlbwr_en_t tlbwr_en;
    upd_fcc_t upd_fcc;
    wait_inst_t wait_inst;
} decode_info_rob_t;

typedef struct packed {
    alu_grand_op_t alu_grand_op;
    alu_inst_t alu_inst;
    alu_op_t alu_op;
    bceqz_t bceqz;
    bcnez_t bcnez;
    break_inst_t break_inst;
    cmp_type_t cmp_type;
    csr_op_en_t csr_op_en;
    csr_rdcnt_t csr_rdcnt;
    dbarrier_t dbarrier;
    div_inst_t div_inst;
    ertn_inst_t ertn_inst;
    fbranch_inst_t fbranch_inst;
    fclass_t fclass;
    fcmp_t fcmp;
    fcsr2gr_t fcsr2gr;
    fcsr_upd_t fcsr_upd;
    fpd_inst_t fpd_inst;
    fpu_inst_t fpu_inst;
    fpu_mode_t fpu_mode;
    fpu_op_t fpu_op;
    fsel_t fsel;
    gr2fcsr_t gr2fcsr;
    inst_t inst;
    invtlb_en_t invtlb_en;
    jump_inst_t jump_inst;
    llsc_inst_t llsc_inst;
    lsu_inst_t lsu_inst;
    mem_cacop_t mem_cacop;
    mem_read_t mem_read;
    mem_type_t mem_type;
    mem_write_t mem_write;
    mul_inst_t mul_inst;
    need_fa_t need_fa;
    priv_inst_t priv_inst;
    refetch_t refetch;
    rnd_mode_t rnd_mode;
    slot0_t slot0;
    syscall_inst_t syscall_inst;
    target_type_t target_type;
    tlbfill_en_t tlbfill_en;
    tlbrd_en_t tlbrd_en;
    tlbsrch_en_t tlbsrch_en;
    tlbwr_en_t tlbwr_en;
    upd_fcc_t upd_fcc;
    wait_inst_t wait_inst;
} decode_info_p_t;

typedef struct packed {
    addr_imm_type_t addr_imm_type;
    alu_grand_op_t alu_grand_op;
    alu_inst_t alu_inst;
    alu_op_t alu_op;
    bceqz_t bceqz;
    bcnez_t bcnez;
    break_inst_t break_inst;
    cmp_type_t cmp_type;
    csr_op_en_t csr_op_en;
    csr_rdcnt_t csr_rdcnt;
    dbarrier_t dbarrier;
    div_inst_t div_inst;
    ertn_inst_t ertn_inst;
    fbranch_inst_t fbranch_inst;
    fclass_t fclass;
    fcmp_t fcmp;
    fcsr2gr_t fcsr2gr;
    fcsr_upd_t fcsr_upd;
    fpd_inst_t fpd_inst;
    fpu_inst_t fpu_inst;
    fpu_mode_t fpu_mode;
    fpu_op_t fpu_op;
    fr0_t fr0;
    fr1_t fr1;
    fr2_t fr2;
    fsel_t fsel;
    fw_t fw;
    gr2fcsr_t gr2fcsr;
    imm_type_t imm_type;
    inst_t inst;
    invtlb_en_t invtlb_en;
    jump_inst_t jump_inst;
    llsc_inst_t llsc_inst;
    lsu_inst_t lsu_inst;
    mem_cacop_t mem_cacop;
    mem_read_t mem_read;
    mem_type_t mem_type;
    mem_write_t mem_write;
    mul_inst_t mul_inst;
    need_fa_t need_fa;
    priv_inst_t priv_inst;
    refetch_t refetch;
    reg_type_r0_t reg_type_r0;
    reg_type_r1_t reg_type_r1;
    reg_type_w_t reg_type_w;
    rnd_mode_t rnd_mode;
    slot0_t slot0;
    syscall_inst_t syscall_inst;
    target_type_t target_type;
    tlbfill_en_t tlbfill_en;
    tlbrd_en_t tlbrd_en;
    tlbsrch_en_t tlbsrch_en;
    tlbwr_en_t tlbwr_en;
    upd_fcc_t upd_fcc;
    wait_inst_t wait_inst;
} decode_info_d_t;

function automatic decode_info_c_fcc_common_t get_c_fcc_common_from_fcc(input decode_info_fcc_t fcc);
    decode_info_c_fcc_common_t ret;
    ret.fcmp = fcc.fcmp;
    ret.upd_fcc = fcc.upd_fcc;
    return ret;
endfunction

function automatic decode_info_c_fcc_common_t get_c_fcc_common_from_c(input decode_info_c_t c);
    decode_info_c_fcc_common_t ret;
    ret.fcmp = c.fcmp;
    ret.upd_fcc = c.upd_fcc;
    return ret;
endfunction

function automatic decode_info_c_alu_common_t get_c_alu_common_from_alu(input decode_info_alu_t alu);
    decode_info_c_alu_common_t ret;
    ret.cmp_type = alu.cmp_type;
    ret.csr_op_en = alu.csr_op_en;
    ret.invtlb_en = alu.invtlb_en;
    ret.target_type = alu.target_type;
    return ret;
endfunction

function automatic decode_info_c_alu_common_t get_c_alu_common_from_c(input decode_info_c_t c);
    decode_info_c_alu_common_t ret;
    ret.cmp_type = c.cmp_type;
    ret.csr_op_en = c.csr_op_en;
    ret.invtlb_en = c.invtlb_en;
    ret.target_type = c.target_type;
    return ret;
endfunction

function automatic decode_info_alu_mdu_common_t get_alu_mdu_common_from_mdu(input decode_info_mdu_t mdu);
    decode_info_alu_mdu_common_t ret;
    ret.alu_op = mdu.alu_op;
    return ret;
endfunction

function automatic decode_info_alu_mdu_common_t get_alu_mdu_common_from_alu(input decode_info_alu_t alu);
    decode_info_alu_mdu_common_t ret;
    ret.alu_op = alu.alu_op;
    return ret;
endfunction

function automatic decode_info_c_lsu_common_t get_c_lsu_common_from_lsu(input decode_info_lsu_t lsu);
    decode_info_c_lsu_common_t ret;
    ret.dbarrier = lsu.dbarrier;
    ret.llsc_inst = lsu.llsc_inst;
    ret.mem_cacop = lsu.mem_cacop;
    ret.mem_read = lsu.mem_read;
    ret.mem_write = lsu.mem_write;
    return ret;
endfunction

function automatic decode_info_c_lsu_common_t get_c_lsu_common_from_c(input decode_info_c_t c);
    decode_info_c_lsu_common_t ret;
    ret.dbarrier = c.dbarrier;
    ret.llsc_inst = c.llsc_inst;
    ret.mem_cacop = c.mem_cacop;
    ret.mem_read = c.mem_read;
    ret.mem_write = c.mem_write;
    return ret;
endfunction

function automatic decode_info_c_t get_c_from_rob(input decode_info_rob_t rob);
    decode_info_c_t ret;
    ret.cmp_type = rob.cmp_type;
    ret.csr_op_en = rob.csr_op_en;
    ret.dbarrier = rob.dbarrier;
    ret.fcmp = rob.fcmp;
    ret.inst = rob.inst;
    ret.invtlb_en = rob.invtlb_en;
    ret.jump_inst = rob.jump_inst;
    ret.llsc_inst = rob.llsc_inst;
    ret.lsu_inst = rob.lsu_inst;
    ret.mem_cacop = rob.mem_cacop;
    ret.mem_read = rob.mem_read;
    ret.mem_write = rob.mem_write;
    ret.refetch = rob.refetch;
    ret.target_type = rob.target_type;
    ret.upd_fcc = rob.upd_fcc;
    ret.wait_inst = rob.wait_inst;
    return ret;
endfunction

function automatic decode_info_fcc_t get_fcc_from_p(input decode_info_p_t p);
    decode_info_fcc_t ret;
    ret.bceqz = p.bceqz;
    ret.bcnez = p.bcnez;
    ret.fclass = p.fclass;
    ret.fcmp = p.fcmp;
    ret.fsel = p.fsel;
    ret.upd_fcc = p.upd_fcc;
    return ret;
endfunction

function automatic decode_info_fpu_t get_fpu_from_p(input decode_info_p_t p);
    decode_info_fpu_t ret;
    ret.fpu_mode = p.fpu_mode;
    ret.fpu_op = p.fpu_op;
    ret.rnd_mode = p.rnd_mode;
    return ret;
endfunction

function automatic decode_info_mdu_t get_mdu_from_p(input decode_info_p_t p);
    decode_info_mdu_t ret;
    ret.alu_op = p.alu_op;
    return ret;
endfunction

function automatic decode_info_lsu_t get_lsu_from_p(input decode_info_p_t p);
    decode_info_lsu_t ret;
    ret.dbarrier = p.dbarrier;
    ret.llsc_inst = p.llsc_inst;
    ret.mem_cacop = p.mem_cacop;
    ret.mem_read = p.mem_read;
    ret.mem_type = p.mem_type;
    ret.mem_write = p.mem_write;
    return ret;
endfunction

function automatic decode_info_alu_t get_alu_from_p(input decode_info_p_t p);
    decode_info_alu_t ret;
    ret.alu_grand_op = p.alu_grand_op;
    ret.alu_op = p.alu_op;
    ret.cmp_type = p.cmp_type;
    ret.csr_op_en = p.csr_op_en;
    ret.invtlb_en = p.invtlb_en;
    ret.target_type = p.target_type;
    return ret;
endfunction

function automatic decode_info_rob_t get_rob_from_p(input decode_info_p_t p);
    decode_info_rob_t ret;
    ret.cmp_type = p.cmp_type;
    ret.csr_op_en = p.csr_op_en;
    ret.csr_rdcnt = p.csr_rdcnt;
    ret.dbarrier = p.dbarrier;
    ret.ertn_inst = p.ertn_inst;
    ret.fcmp = p.fcmp;
    ret.fcsr2gr = p.fcsr2gr;
    ret.fcsr_upd = p.fcsr_upd;
    ret.gr2fcsr = p.gr2fcsr;
    ret.inst = p.inst;
    ret.invtlb_en = p.invtlb_en;
    ret.jump_inst = p.jump_inst;
    ret.llsc_inst = p.llsc_inst;
    ret.lsu_inst = p.lsu_inst;
    ret.mem_cacop = p.mem_cacop;
    ret.mem_read = p.mem_read;
    ret.mem_write = p.mem_write;
    ret.priv_inst = p.priv_inst;
    ret.refetch = p.refetch;
    ret.slot0 = p.slot0;
    ret.target_type = p.target_type;
    ret.tlbfill_en = p.tlbfill_en;
    ret.tlbrd_en = p.tlbrd_en;
    ret.tlbsrch_en = p.tlbsrch_en;
    ret.tlbwr_en = p.tlbwr_en;
    ret.upd_fcc = p.upd_fcc;
    ret.wait_inst = p.wait_inst;
    return ret;
endfunction

function automatic decode_info_p_t get_p_from_d(input decode_info_d_t d);
    decode_info_p_t ret;
    ret.alu_grand_op = d.alu_grand_op;
    ret.alu_inst = d.alu_inst;
    ret.alu_op = d.alu_op;
    ret.bceqz = d.bceqz;
    ret.bcnez = d.bcnez;
    ret.break_inst = d.break_inst;
    ret.cmp_type = d.cmp_type;
    ret.csr_op_en = d.csr_op_en;
    ret.csr_rdcnt = d.csr_rdcnt;
    ret.dbarrier = d.dbarrier;
    ret.div_inst = d.div_inst;
    ret.ertn_inst = d.ertn_inst;
    ret.fbranch_inst = d.fbranch_inst;
    ret.fclass = d.fclass;
    ret.fcmp = d.fcmp;
    ret.fcsr2gr = d.fcsr2gr;
    ret.fcsr_upd = d.fcsr_upd;
    ret.fpd_inst = d.fpd_inst;
    ret.fpu_inst = d.fpu_inst;
    ret.fpu_mode = d.fpu_mode;
    ret.fpu_op = d.fpu_op;
    ret.fsel = d.fsel;
    ret.gr2fcsr = d.gr2fcsr;
    ret.inst = d.inst;
    ret.invtlb_en = d.invtlb_en;
    ret.jump_inst = d.jump_inst;
    ret.llsc_inst = d.llsc_inst;
    ret.lsu_inst = d.lsu_inst;
    ret.mem_cacop = d.mem_cacop;
    ret.mem_read = d.mem_read;
    ret.mem_type = d.mem_type;
    ret.mem_write = d.mem_write;
    ret.mul_inst = d.mul_inst;
    ret.need_fa = d.need_fa;
    ret.priv_inst = d.priv_inst;
    ret.refetch = d.refetch;
    ret.rnd_mode = d.rnd_mode;
    ret.slot0 = d.slot0;
    ret.syscall_inst = d.syscall_inst;
    ret.target_type = d.target_type;
    ret.tlbfill_en = d.tlbfill_en;
    ret.tlbrd_en = d.tlbrd_en;
    ret.tlbsrch_en = d.tlbsrch_en;
    ret.tlbwr_en = d.tlbwr_en;
    ret.upd_fcc = d.upd_fcc;
    ret.wait_inst = d.wait_inst;
    return ret;
endfunction

function automatic string wired_disassembler(input logic [31:0] inst_i);
    string ret;
    logic[25:0] I26 = {inst_i[9:0], inst_i[25:10]};
    logic[20:0] I21 = {inst_i[4:0], inst_i[25:10]};
    logic[20:0] I20 = inst_i[24:5];
    logic[15:0] I16 = inst_i[25:10];
    logic[13:0] I14 = inst_i[23:10];
    logic[11:0] I12 = inst_i[21:10];
    logic[7:0] I8  = inst_i[17:10];
    logic[4:0] ra  = inst_i[19:15];
    logic[4:0] rk  = inst_i[14:10];
    logic[4:0] rj  = inst_i[ 9: 5];
    logic[4:0] rd  = inst_i[ 4: 0];
    casez(inst_i)
        32'b010000??????????????????????????: ret = {"beqz ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b010001??????????????????????????: ret = {"bnez ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b010011??????????????????????????: ret = {"jirl ", $sformatf("$r%02x, $r%02x, %05x=%d",rd, rj, I16<<2, $signed(I16))};
        32'b010100??????????????????????????: ret = {"b ", $sformatf("%07x",I26<<2)};
        32'b010101??????????????????????????: ret = {"bl ", $sformatf("%07x",I26<<2)};
        32'b010110??????????????????????????: ret = {"beq ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b010111??????????????????????????: ret = {"bne ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b011000??????????????????????????: ret = {"blt ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b011001??????????????????????????: ret = {"bge ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b011010??????????????????????????: ret = {"bltu ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b011011??????????????????????????: ret = {"bgeu ", $sformatf("$r%02x, $r%02x, %05x=%d",rj, rd, I16<<2, $signed(I16))};
        32'b0001010?????????????????????????: ret = {"lu12i.w ", $sformatf("$r%02x, %08x=%d",rd, I20 << 12, $signed(I20 << 12))};
        32'b0001100?????????????????????????: ret = {"pcaddi ", $sformatf("$r%02x, %08x=%d",rd, I20 << 2, $signed(I20 << 2))};
        32'b0001101?????????????????????????: ret = {"pcalau12i ", $sformatf("$r%02x, %08x=%d",rd, I20 << 2, $signed(I20 << 2))};
        32'b0001110?????????????????????????: ret = {"pcaddu12i ", $sformatf("$r%02x, %08x=%d",rd, I20 << 12, $signed(I20 << 12))};
        32'b00000100????????????????????????: ret = {"csrwrxchg ", $sformatf(" ")};
        32'b00100000????????????????????????: ret = {"ll.w ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I14<<2, $signed(I14<<2))};
        32'b00100001????????????????????????: ret = {"sc.w ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I14<<2, $signed(I14<<2))};
        32'b0000001000??????????????????????: ret = {"slti ", $sformatf("$r%02x, $r%02x, %03x=%d",rd, rj, I12, $signed(I12))};
        32'b0000001001??????????????????????: ret = {"sltui ", $sformatf("$r%02x, $r%02x, %03x=%d",rd, rj, I12, $signed(I12))};
        32'b0000001010??????????????????????: ret = {"addi.w ", $sformatf("$r%02x, $r%02x, %03x=%d",rd, rj, I12, $signed(I12))};
        32'b0000001101??????????????????????: ret = {"andi ", $sformatf("$r%02x, $r%02x, %03x",rd, rj, I12)};
        32'b0000001110??????????????????????: ret = {"ori ", $sformatf("$r%02x, $r%02x, %03x",rd, rj, I12)};
        32'b0000001111??????????????????????: ret = {"xori ", $sformatf("$r%02x, $r%02x, %03x",rd, rj, I12)};
        32'b0000011000??????????????????????: ret = {"cacop ", $sformatf("0x%02x, $r%02x, %03x=%d", rd, rj, I14<<2, $signed(I14<<2))};
        32'b0010100000??????????????????????: ret = {"ld.b ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010100001??????????????????????: ret = {"ld.h ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010100010??????????????????????: ret = {"ld.w ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010100100??????????????????????: ret = {"st.b ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010100101??????????????????????: ret = {"st.h ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010100110??????????????????????: ret = {"st.w ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010101000??????????????????????: ret = {"ld.bu ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010101001??????????????????????: ret = {"ld.hu ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010101011??????????????????????: ret = {"preld_nop ", $sformatf("0")};
        32'b0010101100??????????????????????: ret = {"fld.s ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b0010101101??????????????????????: ret = {"fst.s ", $sformatf("$r%02x, $r%02x, %03x=%d", rd, rj, I12, $signed(I12))};
        32'b000010000001????????????????????: ret = {"fmadd.s ", $sformatf(" ")};
        32'b000010000101????????????????????: ret = {"fmsub.s ", $sformatf(" ")};
        32'b000010001001????????????????????: ret = {"fnmadd.s ", $sformatf(" ")};
        32'b000010001101????????????????????: ret = {"fnmsub.s ", $sformatf(" ")};
        32'b000011000001????????????????????: ret = {"fcmp.cond.s ", $sformatf(" ")};
        32'b000011010000????????????????????: ret = {"fsel ", $sformatf(" ")};
        32'b00000000000001??????????????????: ret = {"alsl.w ", $sformatf("$r%02x, $r%02x, %03x",rd, rj, I12)};
        32'b00000000000010??????????????????: ret = {"bytepick.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100000???????????????: ret = {"add.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100010???????????????: ret = {"sub.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100100???????????????: ret = {"slt ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100101???????????????: ret = {"sltu ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100110???????????????: ret = {"MASKEQZ ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100111???????????????: ret = {"MASKNEZ ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101000???????????????: ret = {"nor ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101001???????????????: ret = {"and ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101010???????????????: ret = {"or ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101011???????????????: ret = {"xor ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101100???????????????: ret = {"orn ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101101???????????????: ret = {"andn ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101110???????????????: ret = {"sll.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101111???????????????: ret = {"srl.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000110000???????????????: ret = {"sra.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000110110???????????????: ret = {"rotr.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000111000???????????????: ret = {"mul.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000111001???????????????: ret = {"mulh.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000111010???????????????: ret = {"mulh.wu ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000001000000???????????????: ret = {"div.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000001000001???????????????: ret = {"mod.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000001000010???????????????: ret = {"div.wu ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000001000011???????????????: ret = {"mod.wu ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000001010100???????????????: ret = {"break ", $sformatf(" ")};
        32'b00000000001010110???????????????: ret = {"syscall ", $sformatf(" ")};
        32'b00000000010000001???????????????: ret = {"slli.w ", $sformatf("$r%02x, $r%02x, %02x",rd, rj, rk)};
        32'b00000000010001001???????????????: ret = {"srli.w ", $sformatf("$r%02x, $r%02x, %02x",rd, rj, rk)};
        32'b00000000010010001???????????????: ret = {"srai.w ", $sformatf("$r%02x, $r%02x, %02x",rd, rj, rk)};
        32'b00000000010011001???????????????: ret = {"rotri.w ", $sformatf("$r%02x, $r%02x, %02x",rd, rj, rk)};
        32'b00000000011?????0???????????????: ret = {"bstrins.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000011?????1???????????????: ret = {"bstrpick.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000001000000001???????????????: ret = {"fadd.s ", $sformatf(" ")};
        32'b00000001000000101???????????????: ret = {"fsub.s ", $sformatf(" ")};
        32'b00000001000001001???????????????: ret = {"fmul.s ", $sformatf(" ")};
        32'b00000001000001101???????????????: ret = {"fdiv.s ", $sformatf(" ")};
        32'b00000001000010001???????????????: ret = {"fmax.s ", $sformatf(" ")};
        32'b00000001000010101???????????????: ret = {"fmin.s ", $sformatf(" ")};
        32'b00000001000100101???????????????: ret = {"fcopysign.s ", $sformatf(" ")};
        32'b00000110010010001???????????????: ret = {"idle ", $sformatf(" ")};
        32'b00000110010010011???????????????: ret = {"invtlb ", $sformatf(" ")};
        32'b00111000011100100???????????????: ret = {"dbar ", $sformatf("0")};
        32'b00111000011100101???????????????: ret = {"ibar ", $sformatf("0")};
        32'b0000000000000000000100??????????: ret = {"clo.w ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000000101??????????: ret = {"clz.w ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000000110??????????: ret = {"cto.w ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000000111??????????: ret = {"ctz.w ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000001100??????????: ret = {"revb.2h ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000010010??????????: ret = {"bitrev.4b ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000010100??????????: ret = {"bitrev.w ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000010110??????????: ret = {"ext.w.h ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000010111??????????: ret = {"ext.w.b ", $sformatf("$r%02x, $r%02x",rd, rj)};
        32'b0000000000000000011000??????????: ret = {"rdcnt.w ", $sformatf(" ")};
        32'b0000000000000000011001??????????: ret = {"rdcnth.w ", $sformatf(" ")};
        32'b0000000100010100000001??????????: ret = {"fabs.s ", $sformatf(" ")};
        32'b0000000100010100000101??????????: ret = {"fneg.s ", $sformatf(" ")};
        32'b0000000100010100001101??????????: ret = {"fclass.s ", $sformatf(" ")};
        32'b0000000100010100010001??????????: ret = {"fsqrt.s ", $sformatf(" ")};
        32'b0000000100010100010101??????????: ret = {"frecip.s ", $sformatf(" ")};
        32'b0000000100010100100101??????????: ret = {"fmov.s ", $sformatf(" ")};
        32'b0000000100010100101001??????????: ret = {"movgr2fr.w ", $sformatf(" ")};
        32'b0000000100010100101101??????????: ret = {"movfr2gr.s ", $sformatf(" ")};
        32'b0000000100010100110000??????????: ret = {"movgr2fcsr ", $sformatf(" ")};
        32'b0000000100010100110010??????????: ret = {"movfcsr2gr ", $sformatf(" ")};
        32'b0000000100010100110100??????????: ret = {"movfr2cf ", $sformatf(" ")};
        32'b0000000100010100110101??????????: ret = {"movcf2fr ", $sformatf(" ")};
        32'b0000000100010100110110??????????: ret = {"movgr2cf ", $sformatf(" ")};
        32'b0000000100010100110111??????????: ret = {"movcf2gr ", $sformatf(" ")};
        32'b0000000100011010000001??????????: ret = {"ftintrm.w.s ", $sformatf(" ")};
        32'b0000000100011010010001??????????: ret = {"ftintrp.w.s ", $sformatf(" ")};
        32'b0000000100011010100001??????????: ret = {"ftintrz.w.s ", $sformatf(" ")};
        32'b0000000100011010110001??????????: ret = {"ftintrne.w.s ", $sformatf(" ")};
        32'b0000000100011011000001??????????: ret = {"ftint.w.s ", $sformatf(" ")};
        32'b0000000100011101000100??????????: ret = {"ffint.s.w ", $sformatf(" ")};
        32'b0000011001001000001010??????????: ret = {"tlbsrch ", $sformatf(" ")};
        32'b0000011001001000001011??????????: ret = {"tlbrd ", $sformatf(" ")};
        32'b0000011001001000001100??????????: ret = {"tlbwr ", $sformatf(" ")};
        32'b0000011001001000001101??????????: ret = {"tlbfill ", $sformatf(" ")};
        32'b0000011001001000001110??????????: ret = {"ertn ", $sformatf(" ")};
        32'b010010????????????????00????????: ret = {"bceqz ", $sformatf("%07x",I26<<2)};
        32'b010010????????????????01????????: ret = {"bcnez ", $sformatf("%07x",I26<<2)};
    endcase
    return ret;
endfunction
`endif
