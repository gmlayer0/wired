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
`define _ALU_STYPE_SRL (2'b00)
`define _ALU_STYPE_SLL (2'b01)
`define _ALU_STYPE_SRA (2'b10)
`define _MDU_TYPE_MULL (3'b000)
`define _MDU_TYPE_MULH (3'b001)
`define _MDU_TYPE_MULHU (3'b010)
`define _MDU_TYPE_DIV (3'b100)
`define _MDU_TYPE_DIVU (3'b101)
`define _MDU_TYPE_MOD (3'b110)
`define _MDU_TYPE_MODU (3'b111)
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
typedef logic [31 : 0] inst_t;
typedef logic [0 : 0] alu_inst_t;
typedef logic [0 : 0] mdu_inst_t;
typedef logic [0 : 0] lsu_inst_t;
typedef logic [1 : 0] reg_type_r0_t;
typedef logic [0 : 0] reg_type_r1_t;
typedef logic [1 : 0] reg_type_w_t;
typedef logic [2 : 0] imm_type_t;
typedef logic [1 : 0] addr_imm_type_t;
typedef logic [0 : 0] slot0_t;
typedef logic [0 : 0] refetch_t;
typedef logic [1 : 0] alu_grand_op_t;
typedef logic [1 : 0] alu_op_t;
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
    mem_write_t mem_write;
    mem_read_t mem_read;
    dbarrier_t dbarrier;
    llsc_inst_t llsc_inst;
} decode_info_common_t;

typedef struct packed {
    mem_read_t mem_read;
    dbarrier_t dbarrier;
    inst_t inst;
    refetch_t refetch;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
} decode_info_c_t;

typedef struct packed {
    mem_write_t mem_write;
    mem_read_t mem_read;
    dbarrier_t dbarrier;
    llsc_inst_t llsc_inst;
} decode_info_mdu_t;

typedef struct packed {
    mem_read_t mem_read;
    dbarrier_t dbarrier;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
    mem_type_t mem_type;
    mem_cacop_t mem_cacop;
} decode_info_lsu_t;

typedef struct packed {
    mem_read_t mem_read;
    dbarrier_t dbarrier;
    target_type_t target_type;
    alu_op_t alu_op;
    alu_grand_op_t alu_grand_op;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
    cmp_type_t cmp_type;
} decode_info_alu_t;

typedef struct packed {
    ertn_inst_t ertn_inst;
    mem_read_t mem_read;
    dbarrier_t dbarrier;
    tlbsrch_en_t tlbsrch_en;
    inst_t inst;
    csr_op_en_t csr_op_en;
    tlbfill_en_t tlbfill_en;
    tlbwr_en_t tlbwr_en;
    slot0_t slot0;
    jump_inst_t jump_inst;
    refetch_t refetch;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
    invtlb_en_t invtlb_en;
    wait_inst_t wait_inst;
    tlbrd_en_t tlbrd_en;
    priv_inst_t priv_inst;
    csr_rdcnt_t csr_rdcnt;
} decode_info_rob_t;

typedef struct packed {
    mem_read_t mem_read;
    break_inst_t break_inst;
    lsu_inst_t lsu_inst;
    csr_rdcnt_t csr_rdcnt;
    alu_inst_t alu_inst;
    inst_t inst;
    csr_op_en_t csr_op_en;
    tlbwr_en_t tlbwr_en;
    jump_inst_t jump_inst;
    mem_type_t mem_type;
    invtlb_en_t invtlb_en;
    mem_cacop_t mem_cacop;
    priv_inst_t priv_inst;
    dbarrier_t dbarrier;
    target_type_t target_type;
    alu_op_t alu_op;
    tlbfill_en_t tlbfill_en;
    slot0_t slot0;
    alu_grand_op_t alu_grand_op;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
    cmp_type_t cmp_type;
    ertn_inst_t ertn_inst;
    tlbsrch_en_t tlbsrch_en;
    mdu_inst_t mdu_inst;
    refetch_t refetch;
    syscall_inst_t syscall_inst;
    tlbrd_en_t tlbrd_en;
    wait_inst_t wait_inst;
} decode_info_p_t;

typedef struct packed {
    mem_read_t mem_read;
    break_inst_t break_inst;
    lsu_inst_t lsu_inst;
    csr_rdcnt_t csr_rdcnt;
    alu_inst_t alu_inst;
    inst_t inst;
    csr_op_en_t csr_op_en;
    addr_imm_type_t addr_imm_type;
    tlbwr_en_t tlbwr_en;
    jump_inst_t jump_inst;
    mem_type_t mem_type;
    invtlb_en_t invtlb_en;
    mem_cacop_t mem_cacop;
    dbarrier_t dbarrier;
    target_type_t target_type;
    alu_op_t alu_op;
    tlbfill_en_t tlbfill_en;
    slot0_t slot0;
    alu_grand_op_t alu_grand_op;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
    reg_type_r0_t reg_type_r0;
    reg_type_r1_t reg_type_r1;
    cmp_type_t cmp_type;
    ertn_inst_t ertn_inst;
    imm_type_t imm_type;
    tlbsrch_en_t tlbsrch_en;
    mdu_inst_t mdu_inst;
    refetch_t refetch;
    syscall_inst_t syscall_inst;
    wait_inst_t wait_inst;
    tlbrd_en_t tlbrd_en;
    priv_inst_t priv_inst;
} decode_info_r_t;

typedef struct packed {
    mem_read_t mem_read;
    break_inst_t break_inst;
    lsu_inst_t lsu_inst;
    csr_rdcnt_t csr_rdcnt;
    alu_inst_t alu_inst;
    inst_t inst;
    csr_op_en_t csr_op_en;
    addr_imm_type_t addr_imm_type;
    tlbwr_en_t tlbwr_en;
    jump_inst_t jump_inst;
    mem_type_t mem_type;
    invtlb_en_t invtlb_en;
    mem_cacop_t mem_cacop;
    dbarrier_t dbarrier;
    target_type_t target_type;
    alu_op_t alu_op;
    tlbfill_en_t tlbfill_en;
    slot0_t slot0;
    alu_grand_op_t alu_grand_op;
    llsc_inst_t llsc_inst;
    mem_write_t mem_write;
    reg_type_w_t reg_type_w;
    reg_type_r0_t reg_type_r0;
    reg_type_r1_t reg_type_r1;
    cmp_type_t cmp_type;
    ertn_inst_t ertn_inst;
    imm_type_t imm_type;
    tlbsrch_en_t tlbsrch_en;
    mdu_inst_t mdu_inst;
    refetch_t refetch;
    syscall_inst_t syscall_inst;
    wait_inst_t wait_inst;
    tlbrd_en_t tlbrd_en;
    priv_inst_t priv_inst;
} decode_info_d_t;

function automatic decode_info_common_t get_common_from_mdu(input decode_info_mdu_t mdu);
    decode_info_common_t ret;
    ret.mem_write = mdu.mem_write;
    ret.mem_read = mdu.mem_read;
    ret.dbarrier = mdu.dbarrier;
    ret.llsc_inst = mdu.llsc_inst;
    return ret;
endfunction

function automatic decode_info_common_t get_common_from_lsu(input decode_info_lsu_t lsu);
    decode_info_common_t ret;
    ret.mem_write = lsu.mem_write;
    ret.mem_read = lsu.mem_read;
    ret.dbarrier = lsu.dbarrier;
    ret.llsc_inst = lsu.llsc_inst;
    return ret;
endfunction

function automatic decode_info_common_t get_common_from_alu(input decode_info_alu_t alu);
    decode_info_common_t ret;
    ret.mem_write = alu.mem_write;
    ret.mem_read = alu.mem_read;
    ret.dbarrier = alu.dbarrier;
    ret.llsc_inst = alu.llsc_inst;
    return ret;
endfunction

function automatic decode_info_common_t get_common_from_c(input decode_info_c_t c);
    decode_info_common_t ret;
    ret.mem_write = c.mem_write;
    ret.mem_read = c.mem_read;
    ret.dbarrier = c.dbarrier;
    ret.llsc_inst = c.llsc_inst;
    return ret;
endfunction

function automatic decode_info_c_t get_c_from_rob(input decode_info_rob_t rob);
    decode_info_c_t ret;
    ret.mem_read = rob.mem_read;
    ret.dbarrier = rob.dbarrier;
    ret.inst = rob.inst;
    ret.refetch = rob.refetch;
    ret.llsc_inst = rob.llsc_inst;
    ret.mem_write = rob.mem_write;
    return ret;
endfunction

function automatic decode_info_mdu_t get_mdu_from_p(input decode_info_p_t p);
    decode_info_mdu_t ret;
    ret.mem_write = p.mem_write;
    ret.mem_read = p.mem_read;
    ret.dbarrier = p.dbarrier;
    ret.llsc_inst = p.llsc_inst;
    return ret;
endfunction

function automatic decode_info_lsu_t get_lsu_from_p(input decode_info_p_t p);
    decode_info_lsu_t ret;
    ret.mem_read = p.mem_read;
    ret.dbarrier = p.dbarrier;
    ret.llsc_inst = p.llsc_inst;
    ret.mem_write = p.mem_write;
    ret.mem_type = p.mem_type;
    ret.mem_cacop = p.mem_cacop;
    return ret;
endfunction

function automatic decode_info_alu_t get_alu_from_p(input decode_info_p_t p);
    decode_info_alu_t ret;
    ret.mem_read = p.mem_read;
    ret.dbarrier = p.dbarrier;
    ret.target_type = p.target_type;
    ret.alu_op = p.alu_op;
    ret.alu_grand_op = p.alu_grand_op;
    ret.llsc_inst = p.llsc_inst;
    ret.mem_write = p.mem_write;
    ret.cmp_type = p.cmp_type;
    return ret;
endfunction

function automatic decode_info_rob_t get_rob_from_p(input decode_info_p_t p);
    decode_info_rob_t ret;
    ret.ertn_inst = p.ertn_inst;
    ret.mem_read = p.mem_read;
    ret.dbarrier = p.dbarrier;
    ret.tlbsrch_en = p.tlbsrch_en;
    ret.inst = p.inst;
    ret.csr_op_en = p.csr_op_en;
    ret.tlbfill_en = p.tlbfill_en;
    ret.tlbwr_en = p.tlbwr_en;
    ret.slot0 = p.slot0;
    ret.jump_inst = p.jump_inst;
    ret.refetch = p.refetch;
    ret.llsc_inst = p.llsc_inst;
    ret.mem_write = p.mem_write;
    ret.invtlb_en = p.invtlb_en;
    ret.wait_inst = p.wait_inst;
    ret.tlbrd_en = p.tlbrd_en;
    ret.priv_inst = p.priv_inst;
    ret.csr_rdcnt = p.csr_rdcnt;
    return ret;
endfunction

function automatic decode_info_p_t get_p_from_r(input decode_info_r_t r);
    decode_info_p_t ret;
    ret.mem_read = r.mem_read;
    ret.break_inst = r.break_inst;
    ret.lsu_inst = r.lsu_inst;
    ret.csr_rdcnt = r.csr_rdcnt;
    ret.alu_inst = r.alu_inst;
    ret.inst = r.inst;
    ret.csr_op_en = r.csr_op_en;
    ret.tlbwr_en = r.tlbwr_en;
    ret.jump_inst = r.jump_inst;
    ret.mem_type = r.mem_type;
    ret.invtlb_en = r.invtlb_en;
    ret.mem_cacop = r.mem_cacop;
    ret.priv_inst = r.priv_inst;
    ret.dbarrier = r.dbarrier;
    ret.target_type = r.target_type;
    ret.alu_op = r.alu_op;
    ret.tlbfill_en = r.tlbfill_en;
    ret.slot0 = r.slot0;
    ret.alu_grand_op = r.alu_grand_op;
    ret.llsc_inst = r.llsc_inst;
    ret.mem_write = r.mem_write;
    ret.cmp_type = r.cmp_type;
    ret.ertn_inst = r.ertn_inst;
    ret.tlbsrch_en = r.tlbsrch_en;
    ret.mdu_inst = r.mdu_inst;
    ret.refetch = r.refetch;
    ret.syscall_inst = r.syscall_inst;
    ret.tlbrd_en = r.tlbrd_en;
    ret.wait_inst = r.wait_inst;
    return ret;
endfunction

function automatic decode_info_r_t get_r_from_d(input decode_info_d_t d);
    decode_info_r_t ret;
    ret.mem_read = d.mem_read;
    ret.break_inst = d.break_inst;
    ret.lsu_inst = d.lsu_inst;
    ret.csr_rdcnt = d.csr_rdcnt;
    ret.alu_inst = d.alu_inst;
    ret.inst = d.inst;
    ret.csr_op_en = d.csr_op_en;
    ret.addr_imm_type = d.addr_imm_type;
    ret.tlbwr_en = d.tlbwr_en;
    ret.jump_inst = d.jump_inst;
    ret.mem_type = d.mem_type;
    ret.invtlb_en = d.invtlb_en;
    ret.mem_cacop = d.mem_cacop;
    ret.dbarrier = d.dbarrier;
    ret.target_type = d.target_type;
    ret.alu_op = d.alu_op;
    ret.tlbfill_en = d.tlbfill_en;
    ret.slot0 = d.slot0;
    ret.alu_grand_op = d.alu_grand_op;
    ret.llsc_inst = d.llsc_inst;
    ret.mem_write = d.mem_write;
    ret.reg_type_r0 = d.reg_type_r0;
    ret.reg_type_r1 = d.reg_type_r1;
    ret.cmp_type = d.cmp_type;
    ret.ertn_inst = d.ertn_inst;
    ret.imm_type = d.imm_type;
    ret.tlbsrch_en = d.tlbsrch_en;
    ret.mdu_inst = d.mdu_inst;
    ret.refetch = d.refetch;
    ret.syscall_inst = d.syscall_inst;
    ret.wait_inst = d.wait_inst;
    ret.tlbrd_en = d.tlbrd_en;
    ret.priv_inst = d.priv_inst;
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
    unique casez(inst_i)
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
        32'b00000000000100000???????????????: ret = {"add.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100010???????????????: ret = {"sub.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100100???????????????: ret = {"slt ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000100101???????????????: ret = {"sltu ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101000???????????????: ret = {"nor ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101001???????????????: ret = {"and ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101010???????????????: ret = {"or ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101011???????????????: ret = {"xor ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101110???????????????: ret = {"sll.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000101111???????????????: ret = {"srl.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
        32'b00000000000110000???????????????: ret = {"sra.w ", $sformatf("$r%02x, $r%02x, $r%02x",rd, rj, rk)};
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
        32'b00000110010010001???????????????: ret = {"idle ", $sformatf(" ")};
        32'b00000110010010011???????????????: ret = {"invtlb ", $sformatf(" ")};
        32'b00111000011100100???????????????: ret = {"dbar ", $sformatf("0")};
        32'b00111000011100101???????????????: ret = {"ibar ", $sformatf("0")};
        32'b0000000000000000011000??????????: ret = {"rdcnt.w ", $sformatf(" ")};
        32'b0000000000000000011001??????????: ret = {"rdcnth.w ", $sformatf(" ")};
        32'b0000011001001000001010??????????: ret = {"tlbsrch ", $sformatf(" ")};
        32'b0000011001001000001011??????????: ret = {"tlbrd ", $sformatf(" ")};
        32'b0000011001001000001100??????????: ret = {"tlbwr ", $sformatf(" ")};
        32'b0000011001001000001101??????????: ret = {"tlbfill ", $sformatf(" ")};
        32'b0000011001001000001110??????????: ret = {"ertn ", $sformatf(" ")};
    endcase
    return ret;
endfunction
`endif
