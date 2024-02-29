`ifndef _WIRED_CSR_HEADER
`define _WIRED_CSR_HEADER

`define _TLB_ENTRY_NUM (32)

// 此文件中声明了 WIRED 工程将要使用到的全部 CSR TLB 定义
typedef struct packed {
  logic [31:0] crmd     ;
  logic [31:0] prmd     ;
  logic [31:0] euen     ;
  logic [31:0] ectl     ;
  logic [31:0] estat    ;
  logic [31:0] era      ;
  logic [31:0] badv     ;
  logic [31:0] eentry   ;
  logic [31:0] tlbidx   ;
  logic [31:0] tlbehi   ;
  logic [31:0] tlbelo0  ;
  logic [31:0] tlbelo1  ;
  logic [31:0] asid     ;
  logic [31:0] pgdl     ;
  logic [31:0] pgdh     ;
  logic [31:0] cpuid    ;
  logic [31:0] save0    ;
  logic [31:0] save1    ;
  logic [31:0] save2    ;
  logic [31:0] save3    ;
  logic [31:0] tid      ;
  logic [31:0] tcfg     ;
  logic [31:0] tval     ;
  logic [31:0] ticlr    ;
  logic [31:2] llbctl   ;
  logic        llbit    ;
  logic [31:0] tlbrentry;
  logic [31:0] dmw0     ;
  logic [31:0] dmw1     ;
} csr_t;

typedef struct packed {
  logic [18:0] vppn;
  logic [ 5:0] ps  ;
  logic        g   ;
  logic [ 9:0] asid;
  logic        e   ;
} tlb_key_t;

typedef struct packed{
  logic [19:0] ppn;
  logic [ 1:0] plv;
  logic [ 1:0] mat;
  logic        d  ;
  logic        v  ;
} tlb_value_t;

typedef struct packed {
  logic                               dmw  ;
  logic                               found;
  logic [$clog2(`_TLB_ENTRY_NUM)-1:0] index;
  logic [                        5:0] ps   ;
  tlb_value_t                         value;
} tlb_s_resp_t;

typedef struct packed {
  tlb_key_t         key  ;
  tlb_value_t [1:0] value;
} tlb_entry_t;

typedef struct packed {
  logic[`_TLB_ENTRY_NUM - 1 : 0] tlb_we;
  tlb_entry_t tlb_w_entry;
} tlb_update_req_t;

//INSTR
`define _INSTR_RJ       9:5
`define _INSTR_CSR_NUM  23:10
//CRMD
`define _CRMD_PLV       1:0
`define _CRMD_IE        2
`define _CRMD_DA        3
`define _CRMD_PG        4
`define _CRMD_DATF      6:5
`define _CRMD_DATM      8:7
//PRMD
`define _PRMD_PPLV      1:0
`define _PRMD_PIE       2
//EUEN
`define _EUEN_FPE       0
//ECTL
`define _ECTL_LIE       12:0
`define _ECTL_LIE1      9:0
`define _ECTL_LIE2      12:11
//ESTAT
`define _ESTAT_IS        12:0
`define _ESTAT_ECODE     21:16
`define _ESTAT_ESUBCODE  30:22
//EENTRY
`define _EENTRY_VA       31:6
//TLBIDX
`define _TLBIDX_INDEX     $clog2(`_TLB_ENTRY_NUM)-1:0
`define _TLBIDX_PS        29:24
`define _TLBIDX_NE        31
//TLBEHI
`define _TLBEHI_VPPN      31:13
//TLBELO
`define _TLBELO_TLB_V      0
`define _TLBELO_TLB_D      1
`define _TLBELO_TLB_PLV    3:2
`define _TLBELO_TLB_MAT    5:4
`define _TLBELO_TLB_G      6
`define _TLBELO_TLB_PPN    27:8
`define _TLBELO_TLB_PPN_EN 27:8   //todo
//ASID
`define _ASID  9:0
//CPUID
`define _COREID    8:0
//LLBCTL
`define _LLBCT_ROLLB     0
`define _LLBCT_WCLLB     1
`define _LLBCT_KLO       2
//TCFG
`define _TCFG_EN        0
`define _TCFG_PERIODIC  1
`define _TCFG_INITVAL   31:2
//TICLR
`define _TICLR_CLR       0
//TLBRENTRY
`define _TLBRENTRY_PA 31:6
//DMW
`define _DMW_PLV0      0
`define _DMW_PLV3      3 
`define _DMW_MAT       5:4
`define _DMW_PSEG      27:25
`define _DMW_VSEG      31:29
//PGDL PGDH PGD
`define _PGD_BASE      31:12

`define _ECODE_INT  6'h0
`define _ECODE_PIL  6'h1
`define _ECODE_PIS  6'h2
`define _ECODE_PIF  6'h3
`define _ECODE_PME  6'h4
`define _ECODE_PPI  6'h7
`define _ECODE_ADEF 6'h8
`define _ECODE_ADEM 6'h8
`define _ECODE_ALE  6'h9
`define _ECODE_SYS  6'hb
`define _ECODE_BRK  6'hc
`define _ECODE_INE  6'hd
`define _ECODE_IPE  6'he
`define _ECODE_FPD  6'hf
`define _ECODE_TLBR 6'h3f

`define _ESUBCODE_ADEF  9'h0
`define _ESUBCODE_ADEM  9'h1
`endif
