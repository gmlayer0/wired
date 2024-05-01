// dcache cpu side
`include "wired0_defines.svh"


// 2024.4.6 这个模块是前后端通用的 cache 模块
module wired_cache #(
    parameter bit ICACHE = 0,     // 配置是否为 ICache
    parameter bit OUTPUT_BUF = 1, // 状态机输出到 lsu_resp 再打一拍
    parameter int SRAM_WIDTH = 32,
    parameter int PKG_SIZE = 1,
    parameter int SB_SIZE = 4,
    parameter int WKUPBUF_LEN = 16
  )(
    `_WIRED_GENERAL_DEFINE,

    // CPU LSU IQ 中的请求接口
    input  logic         lsu_req_valid_i,
    output logic         lsu_req_ready_o,
    input  iq_lsu_req_t  lsu_req_i,
    input  logic [PKG_SIZE-1:0] lsu_pkg_i,

    output logic         lsu_resp_valid_o,
    input  logic         lsu_resp_ready_i,
    output iq_lsu_resp_t lsu_resp_o,
    output logic [PKG_SIZE-1:0] lsu_pkg_o,

    // CPU COMMIT 中的交互端口
    input  commit_lsu_req_t  c_lsu_req_i,
    output commit_lsu_resp_t c_lsu_resp_o,

    // 到总线侧的请求接口
    output lsu_bus_req_t  bus_req_o,
    input  lsu_bus_resp_t bus_resp_i,

    // 用于 SNOOP 的总线更新接口
    input  dsram_snoop_t  snoop_i,

    // 用于地址翻译更新接口
    input  csr_t            csr_i,
    input  tlb_update_req_t tlb_update_i,

    // SRAM 读端口
    output logic [11:0]       p_addr_o,
    input  logic [3:0][SRAM_WIDTH-1:0]  p_rdata_i,
    input  cache_tag_t [3:0]  p_tag_i,

    // 无效化端口
    input  logic              flush_i

    // 以下为可配置项
  // `ifdef _WIRED_WAKEUP_SRC_CACHE_ENABLE
    // 推测唤醒端口，在 M1 级做指令唤醒用
    ,output logic             wkup_valid_o
    ,output rob_rid_t         wkup_rid_o
    ,output logic[31:0]       wkup_data_o
  // `endif
    // CDB 嗅探端口， store 指令数据未就绪时即可发送到 storebuffer，在 storebuffer 中监听 cdb。
    ,input pipeline_cdb_t [1:0] cdb_i
  );
  // 参数解析
  localparam bit ENABLE_STORE = !ICACHE;
  localparam bit ENABLE_SC_UNCACHE = !ICACHE; // unCache 指令在提交时才执行
  localparam bit ENABLE_64 = SRAM_WIDTH == 64;

  // 全局流控信号
  logic sb_ready, m2_stall;
  wire g_stall = !sb_ready | m2_stall;
  logic g_stall_q;
  always_ff @(posedge clk) g_stall_q <= g_stall;
  assign lsu_req_ready_o = !g_stall;

  // WKUP Slice，用于进行推测唤醒
  logic[WKUPBUF_LEN-1:0][31:4] wkup_regslice_q; // 最低位是有效位
  logic[WKUPBUF_LEN-1:0][31:4] wkup_regslice;
  // WKUP CNT，循环计数
  logic[$clog2(WKUPBUF_LEN)-1:0] wkup_upd_q;
  logic[$clog2(WKUPBUF_LEN)-1:0] wkup_upd;
  always_ff @(posedge clk) {wkup_regslice_q, wkup_upd_q} <= {wkup_regslice, wkup_upd};


  // REQ 级检查 WKUP Slice，提前检查命中情况
  logic[WKUPBUF_LEN+1:0] req_regslice_hit;

  // 连接 SRAM
  assign p_addr_o = lsu_req_i.vaddr;

  // M1 主要结构体
typedef struct packed {
  logic      [31:0] vaddr;   // 请求虚拟地址
  logic      [31:0] paddr;   // 请求物理地址
  logic       [1:0] msize;   // 访存大小(0-byte,1-half,2-word,3-dword)
  logic             msigned; // 有符号扩展
  rob_rid_t         wid;     // 写回地址
  logic             wreq;    // 写请求
  logic       [3:0] strb;    // 写掩码
  logic      [31:0] wdata;   // 写数据
  logic             uncache; // Uncached 请求
  logic       [2:0] cacop;   // cache 请求
  logic             dbar;    // 产生 dbar 效果
  logic             llsc;    // llsc 指令，读时需要申请写权限
  cache_tag_t [3:0] tag;     // sram tag
  logic [3:0][SRAM_WIDTH-1:0] data;    // sram data
  logic inv; logic pme; logic ppi; logic ale; logic tlbr; // TLB EXCP

  logic [PKG_SIZE-1:0] pkg;
} m1_pack_t;

// IQ -> M1 流水线
logic m1_valid_q;
always_ff @(posedge clk) begin
  if(!rst_n || flush_i) begin
    m1_valid_q <= 1'b0;
  end else if(!g_stall) begin
    m1_valid_q <= lsu_req_valid_i;
  end
end

iq_lsu_req_t m1_req_q;
logic[WKUPBUF_LEN+1:0] m1_wkup_hit_q;

logic [PKG_SIZE-1:0] req_pkg_q;
tlb_s_resp_t m1_tlb_resp;
always_ff @(posedge clk) if(!g_stall) begin
  req_pkg_q <= lsu_pkg_i;
  m1_req_q <= lsu_req_i;
  m1_wkup_hit_q <= req_regslice_hit;
end
wired_addr_trans # (
    .FETCH_ADDR(ICACHE)
)
wired_addr_trans_inst (
    `_WIRED_GENERAL_CONN,
    .clken_i('1),
    .vaddr_i(lsu_req_i.vaddr),
    .csr_i(csr_i),
    .tlb_update_req_i(tlb_update_i),
    .trans_result_o(m1_tlb_resp)
);
// M1 级别：进行地址比较，产生 hit miss，等待流水至 M2 处理
m1_pack_t m1_raw, m1; // 组合逻辑
m1_pack_t m1_q;
always_ff @(posedge clk) m1_q <= m1;
// m1_raw 生成
logic m1_tlb_no_excp; // TODO:对于 (sc && llbit == '0) || (cacheop && no_addr_trans)不触发异常

// 直接寻址的 cacop 不触发异常, barrier 不触发异常, LLBIT 为 0 的 SC 指令不触发异常
assign m1_tlb_no_excp = m1_req_q.cacop inside {IDX_INIT, IDX_INV} || m1_req_q.dbar || (m1_req_q.llsc && m1_req_q.strb[0] && !csr_i.llbit);
always_comb begin
  m1_raw.vaddr = m1_req_q.vaddr; m1_raw.paddr = {m1_tlb_resp.value.ppn, m1_req_q.vaddr[11:0]};
  m1_raw.msize = m1_req_q.msize; m1_raw.msigned = m1_req_q.msigned;
  m1_raw.wid  = m1_req_q.wid;    m1_raw.wreq = |m1_req_q.strb;
  m1_raw.strb = m1_req_q.strb;   m1_raw.wdata = m1_req_q.wdata; // 已对齐
  m1_raw.uncache = !m1_tlb_resp.value.mat[0];
  m1_raw.cacop = m1_req_q.cacop; m1_raw.dbar = m1_req_q.dbar;
  m1_raw.llsc = m1_req_q.llsc;   m1_raw.tag = p_tag_i;
  m1_raw.data = p_rdata_i;
  m1_raw.ale = (m1_req_q.msize == 2'd0) ? '0 :
               ((m1_req_q.msize == 2'd1) ?  m1_req_q.vaddr[0] :
                                          (|m1_req_q.vaddr[1:0]));
  m1_raw.tlbr = !m1_raw.ale && !m1_tlb_resp.found && !m1_tlb_no_excp;
  m1_raw.inv = !m1_raw.tlbr && !m1_tlb_resp.value.v && !m1_tlb_no_excp;
  m1_raw.ppi = !m1_raw.inv && (m1_tlb_resp.value.plv == 2'b00) && (csr_i.crmd[`_CRMD_PLV] == 2'd3) && !m1_tlb_no_excp;
  m1_raw.pme = !m1_raw.ppi && !m1_tlb_resp.value.d && (|m1_req_q.strb) && !m1_tlb_no_excp;

  m1_raw.pkg = req_pkg_q;
end
// M1 生成
always_comb begin
  m1 = g_stall_q ? m1_q : m1_raw;
  // 转发 snoop 逻辑
  if(snoop_i.daddr[11:4] == m1.paddr[11:4]) begin // 是同一个 Cache line
    if(ENABLE_64 && snoop_i.dstrb[{m1.paddr[3],1'b0}][0]) begin // 写一部分，就是全写了
      m1.data[snoop_i.dway] = {snoop_i.d[{m1.paddr[3],1'b1}], snoop_i.d[{m1.paddr[3],1'b0}]}; // 不存在部分写存在，全部转发即可
    end else if(!ENABLE_64) begin
      m1.data[snoop_i.dway] = gen_mask_word(m1.data[snoop_i.dway], snoop_i.d[m1.paddr[3:2]], snoop_i.dstrb[m1.paddr[3:2]]);
    end
  end
  if(snoop_i.taddr[11:4] == m1.paddr[11:4]) begin // 是同一个 Cache line
    for(integer w = 0 ; w < 4 ; w += 1) begin // 逐路检查
      if(snoop_i.twe[w]) begin
        m1.tag[w] = snoop_i.t;
      end
    end
  end
end
// M1 store buffer 例化
sb_meta_t       sb_w;
logic [SB_SIZE-1:0] sb_valid;     // M2 检查
sb_meta_t [SB_SIZE-1:0] sb_entry; // 输出表项
logic [$clog2(SB_SIZE)-1:0] sb_top_ptr;
logic sb_inv;             // M2 控制， TODO:CONN
sb_meta_t sb_top;
assign sb_top = sb_entry[sb_top_ptr]; // M2 使用，顶层表项
if(ENABLE_STORE) begin : StoreBuf
  wired_lsu_sb #(.SB_SIZE(SB_SIZE)) wired_lsu_sb_inst (
    `_WIRED_GENERAL_CONN,
  .flush_i(flush_i),
  .ready_o(sb_ready),
  .valid_i(!g_stall && m1_valid_q && m1.wreq),  /* 当且仅当 m1-m2 握手成功时，更新 storebuffer */
  .meta_i(sb_w),
  .valid_o(sb_valid),
  .meta_o(sb_entry),
  .top_o(sb_top_ptr),
  .invalid_i(sb_inv),
  .snoop_i(snoop_i)
  );
end else begin
  assign sb_ready = 1'b1;
  assign sb_valid = '0;
  assign sb_entry = '0;
  assign sb_top_ptr = '0;
end

// M1 地址比较逻辑
// SRAM Hit 生成
logic [3:0] m1_hit, m1_rhit, m1_whit;
for(genvar i = 0 ; i < 4 ; i += 1) begin
  assign m1_hit[i]  = m1.tag[i].p == m1.paddr[31:12]; // 只处理读命中。
  assign m1_rhit[i] = m1_hit[i] & m1.tag[i].rp;
  assign m1_whit[i] = m1_hit[i] & m1.tag[i].wp;
end
// sb Hit 生成
logic [SB_SIZE-1:0] m1_sb_hit;
for(genvar i = 0 ; i < SB_SIZE ; i += 1) begin
    assign m1_sb_hit[i] = sb_entry[i].paddr[31:2] == m1.paddr[31:2] && sb_valid[i];
end
logic [31:0] m1_sb_rdata;
logic [3:0] m1_sb_strb;
// assign m1_sb_strb = ({4{m1_sb_hit[0]}} & sb_entry[0].strb) |
//                     ({4{m1_sb_hit[1]}} & sb_entry[1].strb) |
//                     ({4{m1_sb_hit[2]}} & sb_entry[2].strb) |
//                     ({4{m1_sb_hit[3]}} & sb_entry[3].strb);
// assign m1_sb_rdata = ({32{m1_sb_hit[0]}} & sb_entry[0].wdata) |
//                      ({32{m1_sb_hit[1]}} & sb_entry[1].wdata) |
//                      ({32{m1_sb_hit[2]}} & sb_entry[2].wdata) |
//                      ({32{m1_sb_hit[3]}} & sb_entry[3].wdata);

always_comb begin
  m1_sb_strb = '0;
  m1_sb_rdata = '0;
  for(integer i = 0 ; i < SB_SIZE ; i += 1) begin
    m1_sb_strb       |= m1_sb_hit[i] ? sb_entry[i].fwd_strb  : '0;
    m1_sb_rdata[7:0]   |= (m1_sb_hit[i] & sb_entry[i].fwd_strb[0]) ? sb_entry[i].wdata[7:0]   : '0;
    m1_sb_rdata[15:8]  |= (m1_sb_hit[i] & sb_entry[i].fwd_strb[1]) ? sb_entry[i].wdata[15:8]  : '0;
    m1_sb_rdata[23:16] |= (m1_sb_hit[i] & sb_entry[i].fwd_strb[2]) ? sb_entry[i].wdata[23:16] : '0;
    m1_sb_rdata[31:24] |= (m1_sb_hit[i] & sb_entry[i].fwd_strb[3]) ? sb_entry[i].wdata[31:24] : '0;
  end
end

// sb_w 项目生成
always_comb begin
  sb_w = '0;
  sb_w.paddr = m1.paddr;
  sb_w.hit  = m1_whit;
  sb_w.fwd_strb = m1.strb; // 去除 multihit 影响后用于前递的 strb
  sb_w.strb = m1.strb;
  sb_w.uncached = m1.uncache;
  sb_w.wdata = m1.wdata;
`ifdef _VERILATOR
  sb_w.vaddr = m1.vaddr;
`endif
end
// 用于追踪是否需要更新

typedef struct packed {
  logic      [31:0] vaddr;    // 请求物理地址
  logic      [31:0] paddr;    // 请求物理地址
  logic       [1:0] msize;    // 访存大小(0-byte,1-half,2-word,3-dword)
  logic             msigned;  // 有符号扩展
  rob_rid_t         wid;    // 写回地址
  logic             wreq;     // 写请求
  logic       [3:0] strb;     // 写掩码
  logic      [31:0] wdata;    // 写数据
  logic             uncache;  // Uncached 请求
  logic       [2:0] cacop;    // cache 请求
  logic             dbar;     // 产生 dbar 效果
  logic             llsc;     // llsc 指令，读时需要申请写权限
  logic       [3:0] hit;      // 这里不用更新，到达此处的命中指令被认为已经完成访存
  logic             any_rhit;
  logic             any_whit; // 专供 ll 指令使用
  logic [SB_SIZE-1:0] sb_hit;   // store buffer 在 m2 暂停的时候，不会被更新，因此 sb_hit 可以一直使用。
  logic            any_sbhit;
  logic       [3:0]  sb_strb;
  logic      [31:0] sb_rdata;
  logic [3:0][SRAM_WIDTH-1:0] data;    // sram data

  // For wakeup
  logic any_wkup_hit; // 这条指令已存在 wkup 表项，不需要更新 wkup 表
  logic do_wkup;      // 这条指令发出了 wkup 请求

  logic found_excp;
  lsu_excp_t   d_excp;
  fetch_excp_t f_excp;

  logic [PKG_SIZE-1:0] pkg;
} m2_pack_t;

// M1 -> M2 流水线
logic m2_valid_q;
always_ff @(posedge clk) begin
  if(!rst_n || flush_i) begin
    m2_valid_q <= '0;
  end else if(!m2_stall) begin
    m2_valid_q <= m1_valid_q && sb_ready;
  end
end

m2_pack_t m1_gat; // 收集(gather) M1 侧的数据，生成 m2 数据
(*mark_debug="true"*)m2_pack_t m2_q;
always_ff @(posedge clk) if(!m2_stall) m2_q <= m1_gat;
always_comb begin
  m1_gat = '0;
  m1_gat.vaddr     = m1.vaddr;   m1_gat.paddr     = m1.paddr;
  m1_gat.msize     = m1.msize;   m1_gat.msigned   = m1.msigned;
  m1_gat.wid       = m1.wid;     m1_gat.wreq      = m1.wreq;
  m1_gat.strb      = m1.strb;    m1_gat.wdata     = m1.wdata;
  m1_gat.uncache   = m1.uncache; m1_gat.cacop     = m1.cacop;
  m1_gat.dbar      = m1.dbar || (m1.uncache && ENABLE_SC_UNCACHE);
  m1_gat.llsc      = m1.llsc;    m1_gat.hit       = m1_rhit;
  m1_gat.any_rhit  = |m1_rhit;   m1_gat.any_whit  = |m1_whit;
  m1_gat.sb_hit    = m1_sb_hit;  m1_gat.any_sbhit = |m1_sb_hit;
  m1_gat.sb_strb   = m1_sb_strb; m1_gat.sb_rdata  = m1_sb_rdata;
  m1_gat.data = m1.data;
  m1_gat.d_excp.pil  = m1.inv && !m1.wreq; m1_gat.d_excp.pis  = m1.inv && m1.wreq;
  m1_gat.d_excp.pme  = m1.pme;             m1_gat.d_excp.ppi  = m1.ppi;
  m1_gat.d_excp.ale  = m1.ale;             m1_gat.d_excp.tlbr = m1.tlbr;
  m1_gat.f_excp.adef = m1.ale;             m1_gat.f_excp.tlbr = m1.tlbr;
  m1_gat.f_excp.pif  = m1.inv;             m1_gat.f_excp.ppi  = m1.ppi;
  m1_gat.found_excp = m1.inv | m1.pme | m1.ppi | m1.ale | m1.tlbr;

  m1_gat.pkg = m1.pkg;
end

// M2 核心状态机
typedef enum logic[3:0] {
  S_NORMAL,
  S_MREFILL, // Read miss(LL include) REFILL
  S_MCACOP,  // Cache operation
  S_MUCLOAD, // Uncached load(Weakly ordered)
  S_CUCLOAD, // Uncached load
  S_CUCSTRD, // Uncached store
  S_CREFILL  // Store miss(SC exclude) REFILL
} fsm_e;
(*mark_debug="true"*) fsm_e fsm_q;
fsm_e fsm;
always_ff @(posedge clk) begin
  if(!rst_n) fsm_q <= S_NORMAL;
  else fsm_q <= fsm;
end
typedef enum logic[1:0] {
  M_NORMAL,
  M_HANDLED,
  M_DBAR     // 阻塞 LSU 后续请求，但响应 CPU 请求
//M_WAITSB    // Storebuffer Multihit
} mod_e;
(*mark_debug="true"*) mod_e mod_q;
mod_e mod;
always_ff @(posedge clk) begin
  if(!rst_n || flush_i) mod_q <= M_NORMAL;
  else mod_q <= mod;
end

logic m2_wkup_valid_q, m2_undergo_stall_q;

// 输出接口
logic resp_valid, resp_ready;
iq_lsu_resp_t resp;
logic [PKG_SIZE-1:0] resp_pkg;
// 状态机局部数据存储
typedef struct packed {
  logic      unc_msigned;
  logic [1:0]  unc_msize;
  logic [31:0] unc_paddr;
  logic [SRAM_WIDTH-1:0] fsm_rdata;
} m2_var_t;
m2_var_t m2_var;
m2_var_t m2_var_q;
// logic [3:0]  sb_mhit_mask;
// logic [3:0]  sb_fwd_mask;
// logic [31:0] sb_fwd_data;
always_ff @(posedge clk) m2_var_q <= m2_var;
always_comb begin
  // wkup slice
  wkup_regslice = wkup_regslice_q;
  wkup_upd = wkup_upd_q;

  // 主要输出及 sb
  sb_inv = '0;     // sb 控制信号
  resp_valid = '0; // 主要输出握手
  resp = '0;       // 主要输出数据
  resp.excp = m2_q.d_excp;
  // 这里主要看 m1 是否发出 wkup 信号
  // 若发出，则检查在 m2 是否经历过重填
  // 注意：暂停无关紧要，数据一定按时发出
  // resp.wrong_forward = m2_wkup_valid_q && m2_q.cacop == RD_ALLOC;
  resp.f_excp = m2_q.f_excp;
  resp.uncached = m2_q.uncache;
  resp.vaddr = m2_q.vaddr;
  for(integer i = 0 ; i < 4 ; i += 1) begin
    resp.rdata[SRAM_WIDTH-1:0] |= m2_q.hit[i] ? m2_q.data[i] : '0;
  end
  resp.wid = m2_q.wid;
  resp_pkg = m2_q.pkg;
  // sb_mhit_mask = '0; // 检查 multihit 使用
  // sb_fwd_mask = '0;
  // sb_fwd_data = '0;
  if(mod_q == M_HANDLED) begin
    resp.rdata[SRAM_WIDTH-1:0] = m2_var_q.fsm_rdata; // 使用 fsm 缓存好的数据即可
    // 在多核下，存在缓存中的表项被其它写者无效化的可能性
    resp.wrong_forward = m2_wkup_valid_q;
  end
  if(ENABLE_STORE) begin
    // for(integer i = 0 ; i < SB_SIZE ; i += 1) begin
    //   // sb_mhit_mask |= (m2_q.sb_hit[i] & sb_valid[i]) ? sb_entry[i].fwd_strb : '0;
    //   sb_fwd_mask  |= m2_q.sb_hit[i] ? sb_entry[i].fwd_strb : '0;
    //   sb_fwd_data[7:0]    |= (m2_q.sb_hit[i] & sb_entry[i].fwd_strb[0]) ? sb_entry[i].wdata[31:24] : '0;
    //   sb_fwd_data[15:8]   |= (m2_q.sb_hit[i] & sb_entry[i].fwd_strb[0]) ? sb_entry[i].wdata[31:24] : '0;
    //   sb_fwd_data[23:16]  |= (m2_q.sb_hit[i] & sb_entry[i].fwd_strb[0]) ? sb_entry[i].wdata[31:24] : '0;
    //   sb_fwd_data[31:24]  |= (m2_q.sb_hit[i] & sb_entry[i].fwd_strb[0]) ? sb_entry[i].wdata[31:24] : '0;
    // end
    resp.rdata = gen_mask_word(resp.rdata[31:0], m2_q.sb_rdata, m2_q.sb_strb);
  end
  if(!ENABLE_64) begin
    // 偏移处理
    resp.rdata = mkrsft(resp.rdata[31:0], m2_q.vaddr, m2_q.msize, m2_q.msigned);
  end
  if(ENABLE_STORE && m2_q.llsc && m2_q.wreq) begin
    resp.rdata = 32'd1;
  end
  // M2 Var 处理
  m2_var = m2_var_q;

  // 主状态
  mod = mod_q;
  fsm = fsm_q;

  // 握手
  m2_stall = '1;

  // 与提交级交互接口
  c_lsu_resp_o = '0; // 产生所有到提交级的响应
`ifdef _VERILATOR
  c_lsu_resp_o.wdata = sb_top.wdata;
`endif
  if(!ICACHE) begin
    c_lsu_resp_o.storebuf_hit = |sb_top.hit;
    c_lsu_resp_o.uncached_load_resp =
  mkrsft(bus_resp_i.rdata[31:0], m2_var_q.unc_paddr, m2_var_q.unc_msize, m2_var_q.unc_msigned);
  end

  // 与总线管理器交互接口
  bus_req_o = '0;
  bus_req_o.target_paddr = m2_q.paddr;
  if(c_lsu_req_i.dbarrier_unlock) begin
      mod = M_NORMAL;
  end
  if(ENABLE_STORE) begin
    // Cache 写操作请求
    for(integer i = 0 ; i < 4 ; i += 1) begin
      bus_req_o.way |= sb_top.hit[i] ? i[1:0] : '0;
    end
    if(c_lsu_req_i.storebuf_commit) begin
      bus_req_o.sram_wb_req = !sb_top.uncached;
      sb_inv = '1;
    end
    bus_req_o.wdata = sb_top.wdata; // 巧合的是，unc也存在这里
    bus_req_o.sram_addr = sb_top.paddr;
    bus_req_o.wstrobe = sb_top.strb;
    bus_req_o.size = m2_var_q.unc_msize;
  end else begin
    bus_req_o.size = m2_q.msize; // 直接提出请求即可
  end
  case (fsm_q)
  default/*S_NORMAL*/: begin
      if(!ICACHE && c_lsu_req_i.valid) begin // 响应来自提交级的请求
        case(1'b1)
        c_lsu_req_i.uncached_load_req: begin
          fsm = S_CUCLOAD;
        end
        c_lsu_req_i.uncached_store_req: begin
          fsm = S_CUCSTRD;
        end
        c_lsu_req_i.refill_store_req: begin
          fsm = S_CREFILL;
        end
        endcase
      end
      else if(mod_q == M_NORMAL) begin // 注意，flush 的时候不能进这些状态
        m2_stall = '0;
        if(m2_valid_q/* && !flush_i*/) begin
          m2_stall = !resp_ready; // resp 不 ready 的时候也得阻塞住
          resp_valid = '1;
          if(!m2_q.found_excp) begin
            if(!m2_q.uncache && m2_q.cacop == RD_ALLOC && 
            (!m2_q.any_rhit || (!m2_q.any_whit && m2_q.llsc)) && 
             !(!m2_q.llsc && m2_q.any_sbhit && m2_q.sb_strb == '1)) begin // 未命中（rhit for all || whit for ll.w）的 cached 读请求
              if(!m2_q.any_sbhit) fsm = S_MREFILL; // 如果命中 store buffer，等着就好了。
              m2_stall = '1;
              resp_valid = '0;
            end else if(!ENABLE_SC_UNCACHE && m2_q.uncache && m2_q.cacop == RD_ALLOC) begin // Uncached 读请求，且弱序
              fsm = S_MUCLOAD;
              m2_stall = '1;
              resp_valid = '0;
            end /*else if(m2_q.wreq && (sb_mhit_mask & m2_q.strb) != '0) begin // 重叠的写请求
              // 由于这条指令在 M2 级别，也就是写入过 SB 的最新指令，后续指令在这条指令前进之前，不会写 SB。
              // mod = M_WAITSB;
              m2_stall = '1;
              resp_valid = '0;
            end */else if(m2_q.cacop inside {HIT_INV, IDX_INIT, IDX_INV}) begin // Cache 无效化请求
              fsm = S_MCACOP;
              m2_stall = '1;
              resp_valid = '0;
            end
          end
          if(!m2_stall && m2_q.dbar) begin
            // 记录产生阻塞效果指令的物理地址（主要是 uncached load/store）
            m2_var.unc_msigned = m2_q.msigned;
            m2_var.unc_msize = m2_q.msize;
            m2_var.unc_paddr = m2_q.paddr;
            // 阻塞住下一条指令
            mod = M_DBAR;
          end
          if(!m2_stall && m2_q.cacop == RD_ALLOC && !m2_q.uncache) begin
            // 这里需要更新下 wkupbuf，是一个命中 Cache 的读请求
            wkup_regslice[wkup_upd_q][31:4] = m2_q.vaddr[31:4];
            wkup_upd = wkup_upd_q + 1;
          end
        end else begin
          resp_valid = '0;
        end
      end
      else if(mod_q == M_HANDLED) begin
        resp_valid = m2_valid_q;
        if(resp_ready || !m2_valid_q) begin
          m2_stall = '0;
          mod = (m2_valid_q && m2_q.dbar) ? M_DBAR : M_NORMAL;
        end
      end
  end
  // M_WAITSB: begin
  //   // 等待重复命中的表项被提交或者冲刷
  //   if(flush_i || (m2_q.sb_hit & sb_valid) == '0) begin
  //     fsm = S_NORMAL;
  //   end
  // end
  S_MREFILL: begin
    bus_req_o.valid = '1;
    bus_req_o.inv_req = (!ICACHE && m2_q.llsc) ? WR_ALLOC : RD_ALLOC; // 对于 ll 指令，需要申请写权限
    if(bus_resp_i.ready) begin
      fsm = S_NORMAL;
      mod = M_HANDLED;
      m2_var.fsm_rdata = bus_resp_i.rdata[SRAM_WIDTH-1:0];
    end
  end
  S_MCACOP: begin
      bus_req_o.valid = '1;
      bus_req_o.inv_req = inv_parm_e'(m2_q.cacop); // 对于 ll 指令，需要申请写权限
      if(bus_resp_i.ready) begin
          fsm = S_NORMAL;
          mod = M_HANDLED;
      end
  end
  S_MUCLOAD: begin
    bus_req_o.valid = '1;
    bus_req_o.uncached_load_req = '1;
    if(bus_resp_i.ready) begin
      fsm = S_NORMAL;
      mod = M_HANDLED;
      m2_var.fsm_rdata = bus_resp_i.rdata[SRAM_WIDTH-1:0];
    end
  end
  S_CUCLOAD: begin
      bus_req_o.valid = '1;
      bus_req_o.uncached_load_req = '1;
      bus_req_o.target_paddr = m2_var_q.unc_paddr;
      c_lsu_resp_o.ready = bus_resp_i.ready;
      if(bus_resp_i.ready) begin
          fsm = S_NORMAL;
      end
  end
  S_CUCSTRD: begin
      bus_req_o.valid = '1;
      bus_req_o.uncached_store_req = '1;
      bus_req_o.target_paddr = m2_var_q.unc_paddr;
      c_lsu_resp_o.ready = bus_resp_i.ready;
      if(bus_resp_i.ready) begin
          fsm = S_NORMAL;
      end
  end
  S_CREFILL: begin
      bus_req_o.valid = '1;
      bus_req_o.inv_req = WR_ALLOC;
      bus_req_o.target_paddr = {sb_top.paddr[31:4], m2_q.paddr[3:0]};
      if(m2_valid_q && m2_q.cacop == RD_ALLOC && sb_top.paddr[31:4] == m2_q.paddr[31:4]) begin
        mod = M_HANDLED;
        m2_var.fsm_rdata = bus_resp_i.rdata[SRAM_WIDTH-1:0];
      end
      c_lsu_resp_o.ready = bus_resp_i.ready;
      if(bus_resp_i.ready) begin
          fsm = S_NORMAL;
      end
  end
  endcase
end

// 输出握手
if(OUTPUT_BUF) begin
  iq_lsu_resp_t resp_q;
  logic resp_valid_q;
  logic [PKG_SIZE-1:0] resp_pkg_q;
  always_ff @(posedge clk) begin
    if(!rst_n || flush_i) begin
      resp_valid_q <= '0;
    end else if(resp_ready) begin
      resp_valid_q <= resp_valid;
    end
  end
  always_ff @(posedge clk) begin 
    if(resp_ready) begin
      resp_q <= resp;
      resp_pkg_q <= resp_pkg;
    end
  end
  assign resp_ready = !resp_valid_q || lsu_resp_ready_i;
  assign lsu_resp_valid_o = resp_valid_q;
  assign lsu_resp_o = resp_q;
  assign lsu_pkg_o = resp_pkg_q;
end else begin
  assign resp_ready = lsu_resp_ready_i;
  assign lsu_resp_valid_o = resp_valid;
  assign lsu_resp_o = resp;
  assign lsu_pkg_o = resp_pkg;
end

// REQ 级检查 WKUP Slice，提前检查命中情况
for(genvar i = 0 ; i < WKUPBUF_LEN ; i += 1) begin
  assign req_regslice_hit[i] = lsu_req_i.vaddr[31:4] == wkup_regslice_q[i][31:4]/* && wkup_regslice_q[3]*/;
  // assign req_regslice_hit[i] = '1;
end
// 检查与 M1 比较的命中情况
assign req_regslice_hit[WKUPBUF_LEN+0] = lsu_req_i.vaddr[31:4] == m1.vaddr[31:4] && m1_valid_q;
// 检查与 M2 比较的命中情况
assign req_regslice_hit[WKUPBUF_LEN+1] = lsu_req_i.vaddr[31:4] == m2_q.vaddr[31:4] && m2_valid_q;

// WKUP_VALID 信号生成
assign wkup_valid_o = !g_stall && m1_valid_q && (|m1_wkup_hit_q) && m1.cacop == RD_ALLOC; // 排除 sc.w 的情况
assign wkup_rid_o = m1.wid;

// 用于追踪 M2 级别唤醒是否进行过唤醒
// 但是实际上没什么用
always_ff @(posedge clk) begin
  if(!m2_stall) begin
    m2_wkup_valid_q <= wkup_valid_o;
    m2_undergo_stall_q <= '0;
  end else begin
    m2_undergo_stall_q <= '1;
  end
end

  // Forward source
  logic[31:0] fwd_src_q;
  always_ff @(posedge clk) fwd_src_q <= resp.rdata[31:0];
  assign wkup_data_o = fwd_src_q;

`ifdef _VERILATOR
  // if(ENABLE_STORE)
  // begin : store_difftest
  //   // 对所有 Store 指令进行提交处理
  //   DifftestStoreEvent DifftestStoreEvent_p (
  //                        .clock     (clk),
  //                        .coreid    ('0 ),
  //                        .index     ('0 ),
  //                        .valid     (c_lsu_req_i.storebuf_commit),
  //                        .storePAddr(sb_top.paddr),
  //                        .storeVAddr(sb_top.vaddr),
  //                        .storeData (sb_top.wdata >> {sb_top.vaddr[1:0],3'd0}) // 恢复已经偏移的写数据
  //                      );
  // end

  // debug 用
  // always_ff @(posedge clk) begin
  //   if(32'h37f01e0 == m2_q.paddr && m2_valid_q && !m2_stall) begin
  //     $display("Hit 0x37f01e0-%x: %x %x", m1_q.vaddr, m2_q.strb, m2_q.wdata);
  //   end
  // end
  always_ff @(posedge clk) begin
    // if(c_lsu_req_i.storebuf_commit && !sb_top.uncached && !(|sb_top.hit)) begin
    //   $display("[%t]Storebuf commit when not hit %x %x %x!!!\n",$time , sb_top.hit, sb_top.vaddr, sb_top.paddr);
    // end
    // if(m2_valid_q && m2_q.paddr == 32'h3fff80 && !m2_stall && resp.rdata == 32'h06) begin
    //   $display("* [%t] DCache catch the read hit:%x strb:%x wdata:%x sb_hit:%x!!!",$time , m2_q.hit, m2_q.strb, m2_q.wdata, m2_q.sb_hit);
    // end
    // if(m2_valid_q && !m2_stall && (m2_q.hit[0] + m2_q.hit[1] + m2_q.hit[2] + m2_q.hit[3] > 1)) begin
    //   $display("!!![%t] Cache multihit @ vaddr:%x paddr:%x wid:%x hit:%x!!!", $time, m2_q.vaddr, m2_q.paddr, m2_q.wid, m2_q.hit);
    //   // $finish;
    // end
    // if($time > 10000000 && bus_req_o.target_paddr[31:4] == 32'h3fff8 && fsm_q == S_MREFILL && bus_req_o.valid && bus_resp_i.ready) begin
    //   $display("##[%t] M Refill @%x: %x. M2 wid:%x.", $time, bus_req_o.target_paddr, bus_resp_i.rdata, m2_q.wid);
    // end
    // if($time > 10000000 && bus_req_o.target_paddr[31:4] == 32'h3fff8 && fsm_q == S_CREFILL && bus_req_o.valid && bus_resp_i.ready) begin
    //   $display("##[%t] C Refill @%x: %x. M2 valid:%d sbhit:%x wid:%x sbtop:%d.",
    //   $time, bus_req_o.target_paddr, bus_resp_i.rdata, m2_valid_q, m2_q.sb_hit, m2_q.wid, sb_top_ptr);
    // end
  end
`endif

endmodule
