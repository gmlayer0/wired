`ifndef _WIRED_STRUCTURE_HEADER
`define _WIRED_STRUCTURE_HEADER

// 此文件中声明了 WIRED 工程将要使用到的全部 structure

`include "wired0_decoder.svh"

typedef logic[31:0] word_t;

// Frontend Begin
// BPU
typedef enum logic[1:0] {
  BPU_TARGET_NPC,
  BPU_TARGET_CALL,
  BPU_TARGET_RETURN,
  BPU_TARGET_IMM
} bpu_target_type_e;
typedef struct packed {
    logic taken;
    logic tid;  // 跳转生命周期 ID，后端仅接受有效的 Tier ID。每次重定向控制流的时候，也会修改前后端的 tier id。
                // 注意，对于重命名模块，在 flush 状态下 Tier ID 不一致的会被直接接受并丢弃，对于 Tier ID 一致的结果，会拉低 ready 等待。
    // logic pc_off;
    logic [                31:0]           predict_pc ;
    logic [                 1:0]           lphr       ;
    logic [`_WIRED_PARAM_BHT_DATA_LEN-1:0] history    ;
    bpu_target_type_e                      target_type;
    logic                                  dir_type   ;
    logic [`_WIRED_PARAM_RAS_ADDR_LEN-1:0] ras_ptr;
} bpu_predict_t;
typedef struct packed {
    logic                                  redirect   ; // 实际跳转信号
    logic                                  tid        ; // 重定向后的 Tier ID，复位一致默认为 0
    logic                                  true_taken ;
    logic                                  miss       ;
    logic [31:0]                           pc         ;
    logic [31:0]                           true_target;
    logic [31:0]                            btb_target;
    logic [1:0]                            lphr       ;
    logic [`_WIRED_PARAM_BHT_DATA_LEN-1:0] history    ;
    bpu_target_type_e                 true_target_type;
    logic                         true_conditional_jmp;
    logic [`_WIRED_PARAM_RAS_ADDR_LEN-1:0]     ras_ptr;
    logic need_update;
    logic ras_miss_type;
} bpu_correct_t;
typedef struct packed {
  // logic interrupt[9:0]; // ALL Interrupt including software Interruption
  // logic fetch_int;      // None Masked Interruption founded, if founded, this instruction is forced to become an nop.
  logic adef;           // Address translation failure will force this instruction to become an nop.
  logic tlbr;
  logic pif ;
  logic ppi ;
} fetch_excp_t;
typedef struct packed {
  // FRONTEND
  // logic interrupt[9:0]; // ALL Interrupt including software Interruption
  // logic fetch_int;      // None Masked Interruption founded, if founded, this instruction is forced to issue in ALU slot
  logic adef ;
  logic tlbr;
  logic pif  ;
  logic ppi ;

  // DECODE
  logic ine  ;
  logic ipe  ;
  logic sys  ;
  logic brk  ;
} static_excp_t;

typedef struct packed {
  logic pil  ;
  logic pis  ;
  logic pme  ;
  logic ppi  ;
  logic ale  ;
  logic tlbr ;
} lsu_excp_t;

typedef struct packed {
  // logic fetch_int;      // None Masked Interruption founded, if founded, this instruction is forced to issue in ALU slot
  logic adef ;
  logic itlbr;
  logic pif  ;
  logic ippi ;
  // DECODE
  logic ine  ;
  logic ipe  ;
  logic sys  ;
  logic brk  ;
  // LSU
  logic pil  ;
  logic pis  ;
  logic pme  ;
  logic ppi  ;
  logic ale  ;
  logic tlbr ;
} excp_t;


typedef logic[`_WIRED_PARAM_PRF_LEN-1:0] arch_rid_t; // 架构寄存器号
typedef logic[`_WIRED_PARAM_ROB_LEN-1:0] rob_rid_t;  // 重命名后寄存器号 == {}

typedef struct packed{
    arch_rid_t [1:0] r_reg; // 0 for rk, 1 for rj
    arch_rid_t       w_reg;
} reg_info_t;

// 输入到 Pack 级的指令流
typedef struct packed {
  decode_info_d_t        di;
  logic                 ine; // 解码产生
  reg_info_t             ri;
  logic[31:0]            pc;
  bpu_predict_t bpu_predict;
  fetch_excp_t   fetch_excp;
} pipeline_ctrl_pack_t;

// Backend Begin
// 解码出来的寄存器信息

typedef struct packed {
    arch_rid_t arch_id;
    rob_rid_t  rob_id;
} reg_ctrl_t; // Rename 级产生

// 输入到 disPatch(P) 级的指令流

function automatic logic[27:0] mkimm_addr(input logic[1:0] addr_imm_type, input logic[25:0] raw_imm);
case (addr_imm_type)
  default : /*`_ADDR_IMM_S12:*/
    begin
      mkimm_addr = {{16{raw_imm[21]}},raw_imm[21:10]};
    end
  `_ADDR_IMM_S14 : begin
    mkimm_addr = {{12{raw_imm[23]}},raw_imm[23:10],2'b00};
  end
  `_ADDR_IMM_S16 : begin
    mkimm_addr = {{10{raw_imm[25]}},raw_imm[25:10],2'b00};
  end
  `_ADDR_IMM_S26 : begin
    mkimm_addr = {raw_imm[9:0],raw_imm[25:10],2'b00};
  end
endcase
endfunction

function logic[31:0] mkimm_data(input logic[2:0] data_imm_type, input logic[25:0] raw_imm);
  // !!! CAUTIOUS !!! : DOESN'T SUPPORT IMM U16 | IMM S21 FOR NOW
  case(data_imm_type[1:0])
    // default/*IMM U5*/: begin
    //   mkimm_data = {27'd0, raw_imm[15:10]};
    // end NO NEED ANY MORE
    default : begin
      mkimm_data = {20'd0, raw_imm[21:10]};
    end
    `_IMM_S12 : begin
      mkimm_data = {{20{raw_imm[21]}}, raw_imm[21:10]};
    end
    `_IMM_S20 : begin
      mkimm_data = {{12{raw_imm[24]}}, raw_imm[24:5]};
    end
  endcase
endfunction

typedef struct packed{
  decode_info_p_t di; // 指令控制信息
  logic[1:0]      scyc_raw;    // 同周期内出现 RAW 冲突需要解决
  reg_ctrl_t      wreg;
  logic           wtier;       // 写寄存器 tier id
  logic[27:0]     addr_imm;    // 传入 LSU，用于计算 vaddr 或者计算 csr_id（给 ALU）
  logic[4:0]      op_code;     // 用于 Commit 级的 TLB 操作或者返回地址操作
  logic[31:0]     pc;
  bpu_predict_t   bpu_predict;
  static_excp_t   excp;
} pipeline_ctrl_p_t;

// 从 CDB 写入 ROB 的指令数据信息
typedef struct packed {
  // 控制流相关
  lsu_excp_t  excp;
  logic       need_jump;
  logic[31:0] target_addr;       // FOR LSU: VADDR, FOR ALU: TARGET_ADDR
                                 // FOR INVTLB: VPN-[31:12] ASID-[9:0]
  // 访存流相关
  logic       uncached;          // 对于 Uncached 的指令，一定会触发流水线冲刷，重新执行，结果直接写入 ARF，不经过 ROB。
  word_t      wdata;
  
  // 有效性
  rob_rid_t   wid;
  logic       valid;
} pipeline_cdb_t;

// typedef struct packed {
//   logic     valid;
//   rob_rid_t wid;
//   word_t    wdata;
// } pipeline_cdb_data_t;

typedef struct packed{
  logic [1:0] valid;
  reg_ctrl_t [1:0] rreg;
  word_t [1:0] rdata;
} pipeline_p_data_t; // Rename 级产生（读取 ARF），在读取 ROB 之前需要注意转发，在读取 ROB 后只需要监视 CDB

typedef struct packed{
  logic [1:0] valid;
  rob_rid_t [1:0] rreg;
  word_t [1:0] rdata;
} pipeline_data_t; // Rename 级产生（读取 ARF），在读取 ROB 之前需要注意转发，在读取 ROB 后只需要监视 CDB

// 提交级流水
// 提交级控制实际 ARF 写回，控制 Rename 表项回收，保证 ROB 永远不会出现 Overflow 的情况

typedef struct packed {
  logic valid;

  decode_info_rob_t di; // 指令控制信息
  arch_rid_t        wreg;
  logic             wtier;       // 写寄存器 tier id
  logic[4:0]        op_code;     // CSR 控制信息
  logic[13:0]       csr_id;
  logic[31:0]       pc;
  bpu_predict_t     bpu_predict; // 在 ALU 中仅检查是否跳转，跳转执行由提交级负责
  static_excp_t     static_excp;

  // 控制流相关
  lsu_excp_t  lsu_excp;
  logic       excp_found;
  logic       need_jump;
  logic[31:0] target_addr; // 这里做了复用，对于跳转指令为跳转目标，对于访存指令为访存虚地址，对于 CSR 指令，为待写入的数据 gpr[rd]。

  // 访存流相关
  logic uncached;          // 对于 Uncached 的指令，一定会触发流水线冲刷，重新执行，结果直接写入 ARF，不经过 ROB。
                           // Uncached 指令占用 Request_buffer (仅有一个表项，占用时不再进行后续指令执行)

  // 写回 ARF 的数据
  word_t wdata;
} rob_entry_t; // 聚合

function automatic excp_t gather_excp(static_excp_t static_i, lsu_excp_t lsu_i);
  excp_t ret;
  // ret.fetch_int = static_i.fetch_int;      // None Masked Interruption founded, if founded, this instruction is forced to issue in ALU slot
  ret.adef = static_i.adef;
  ret.itlbr = static_i.tlbr;
  ret.pif = static_i.pif ;
  ret.ippi = static_i.ppi;
  ret.ine = static_i.ine ;
  ret.ipe = static_i.ipe ;
  ret.sys = static_i.sys ;
  ret.brk = static_i.brk ;
  ret.pil = lsu_i.pil;
  ret.pis = lsu_i.pis;
  ret.pme = lsu_i.pme;
  ret.ppi = lsu_i.ppi;
  ret.ale = lsu_i.ale;
  ret.tlbr = lsu_i.tlbr;
  return ret;
endfunction

function automatic logic[31:0] gen_mask_word(logic [31:0] o, logic [31:0] n, logic [3:0] s);
  logic [31:0] r;
  r[7:0]   = s[0] ? n[7:0] : o[7:0];
  r[15:8]  = s[1] ? n[15:8] : o[15:8];
  r[23:16] = s[2] ? n[23:16] : o[23:16];
  r[31:24] = s[3] ? n[31:24] : o[31:24];
  return r;
endfunction

function automatic logic[31:0] mkrsft(input logic[31:0] raw, input logic[31:0] va, input logic[1:0] siz, input logic sign);
  // M1 RDATA 电路
  logic ext_sign;
  case(siz[0])
    default : begin
      ext_sign = raw[15] & sign;
    end
    1'b1 : begin
      // HALF
      ext_sign = va[1] ? (raw[31] & sign) :
        (raw[15] & sign);
    end
    1'b0 : begin
      // BYTE
      ext_sign = va[1] ? (va[0] ? (raw[31] & sign) : (raw[23] & sign)) :
        (va[0] ? (raw[15] & sign) : (raw[7] & sign));
    end
  endcase
  if(!siz[1]) begin // 2 byte or 1 byte
    mkrsft = {{16{ext_sign}}, va[1] ? raw[31:16] : raw[15:0]};
    if(!siz[0]) begin // 1 byte
      mkrsft[7:0]=va[0]?mkrsft[15:8] : mkrsft[7:0];
      mkrsft[15:8] = {8{ext_sign}};
    end
  end
  else begin
    mkrsft = raw; // 4 byte
  end
endfunction

typedef enum logic[2:0] {
  NOT_VALID_INV_PARM,
  HIT_INV,        // HIT AND INVALIDATE
  PRB_HIT_ADDR_N, // for prb inv toN
  PRB_HIT_ADDR_B, // for prb inv toB

  IDX_INIT,       // INDEXED INIT
  IDX_INV,        // INDEX INVALID WRITE BACK
  RD_ALLOC,       // for read inv, 找到一个合适的行释放并返回
  WR_ALLOC        // for write miss, check whether read hit, if hit, just return.
                  // otherwise, invalidate random way and return.
} inv_parm_e;

// IQ 到 Muler / Divider 的请求及反馈
typedef struct packed {
  logic[1:0]  op;
  logic[31:0] r0;
  logic[31:0] r1;
  rob_rid_t   wid;     // 写回地址
} iq_mdu_req_t;
typedef struct packed {
  logic[31:0] result;
  rob_rid_t   wid;     // 写回地址
} iq_mdu_resp_t;

// LSU IQ 到 LSU 的请求
typedef struct packed {
  logic  [3:0] strb;
  inv_parm_e   cacop;
  logic  dbar;          // 显式 dbar
  logic  llsc;          // LL 指令，需要写权限
  rob_rid_t    wid;     // 写回地址
  logic      msigned;   // 有符号拓展
  logic  [1:0] msize;   // 访存大小-1
  logic [31:0] vaddr;   // 虚拟地址
  logic [31:0] wdata;   // 写地址
} iq_lsu_req_t;

// LSU 到 LSU IQ 的响应
typedef struct packed {
  lsu_excp_t   excp;
  fetch_excp_t f_excp;
  logic       uncached;
  rob_rid_t   wid;     // 写回地址
  logic[31:0] vaddr;
  logic[63:0] rdata;   // 高位是 ICACHE 专用
} iq_lsu_resp_t;

// LSU 到 Manager 的请求
typedef struct packed {
  logic valid;

  logic uncached_load_req;
  logic uncached_store_req;
  inv_parm_e inv_req; // Cache Line 状态请求

  logic [1:0]  size;         // UNC: 0-1bytes, 1-2bytes, 2-4bytes, 3-8bytes
  logic [31:0] target_paddr; 
  
  logic sram_wb_req;
  logic [1:0]  way;     // 仅供写入 sram 时使用
  logic [3:0]  wstrobe; // 仅供写入 sram 时使用
  logic [31:0] wdata;
  logic [31:0] sram_addr;

} lsu_bus_req_t;

typedef struct packed {
  logic ready;             // 响应完成
  logic [63:0] rdata;      // 返回给请求方 LSU
                           // 注意：对于 size<=2 的请求仅使用低 32 位，否之使用全部 64 位
} lsu_bus_resp_t;

// TAG 内容
typedef struct packed {
  logic rp; // read permission
  logic wp; // write permission
  logic[19:0] p; // ppn
} cache_tag_t;

typedef struct packed {
  logic [11:0] daddr;
  logic  [1:0]  dway;
  logic [3:0][3:0]  dstrb;
  logic [3:0][31:0] d;

  logic [11:0] taddr;
  logic [3:0]       twe;
  cache_tag_t       t; // can only write one tag once
} dsram_snoop_t;

// CPU 提交级到 LSU 的请求
typedef struct packed {
  logic valid;

  logic uncached_load_req;
  logic uncached_store_req; // 对于 Uncached 的请求，暂存在 LSU 中，
                            // 并阻塞后续所有请求开始以保持 TSO
  logic refill_store_req;   // Store miss, 申请权限

  // 以上均需要 valid 及 ready 系统握手
  logic dbarrier_unlock;    // 流水线化，不需要握手，解除 dbarrier 约束
  logic storebuf_commit;    // 流水线化，不需要握手
} commit_lsu_req_t;

typedef struct packed {
  logic ready;
  logic storebuf_hit;       // storebuf 顶元素命中

  logic [31:0] uncached_load_resp;
} commit_lsu_resp_t;

typedef struct packed {
  logic [31:0] paddr;
  logic [3:0] hit; // cache hit
  logic [3:0] strb;
  logic [31:0] wdata;
`ifdef _VERILATOR
  logic [31:0] vaddr;
`endif
} sb_meta_t; // store_buffer 元数据

`endif
