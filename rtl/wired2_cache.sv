// dcache cpu side
`include "wired0_defines.svh"


// 2024.4.6 这个模块是前后端通用的 cache 模块
module wired_cache #(
    parameter bit ICACHE = 0,     // 配置是否为 ICache
    parameter bit OUTPUT_BUF = 1, // 状态机输出到 lsu_resp 再打一拍
    parameter int SRAM_WIDTH = 64
  )(
    `_WIRED_GENERAL_DEFINE,

    // CPU LSU IQ 中的请求接口
    input  logic         lsu_req_valid_i,
    output logic         lsu_req_ready_o,
    input  iq_lsu_req_t  lsu_req_i,

    output logic         lsu_resp_valid_o,
    input  logic         lsu_resp_ready_i,
    output iq_lsu_resp_t lsu_resp_o,

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
tlb_s_resp_t m1_tlb_resp;
always_ff @(posedge clk) if(!g_stall) m1_req_q <= lsu_req_i;
wired_addr_trans # (
    .FETCH_ADDR('0)
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
assign m1_tlb_no_excp = m1_req_q.cacop inside {IDX_INIT, IDX_INV};
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
end
// M1 生成
always_comb begin
  m1 = g_stall_q ? m1_q : m1_raw;
  // 转发 snoop 逻辑
  if(snoop_i.daddr[11:4] == m1.paddr[11:4]) begin // 是同一个 Cache line
    if(ENABLE_64 && snoop_i.dstrb[{m1.paddr[3],1'b0}][0]) begin // 写一部分，就是全写了
      m1.data[snoop_i.dway] = {snoop_i.d[{m1.paddr[3],1'b1}], snoop_i.d[{m1.paddr[3],1'b0}]}; // 不存在部分写存在，全部转发即可
    end else begin
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
logic [3:0] sb_valid;     // M2 检查
sb_meta_t [3:0] sb_entry; // 输出表项
logic [1:0] sb_top_ptr;
logic sb_inv;             // M2 控制， TODO:CONN
sb_meta_t sb_top;
assign sb_top = sb_entry[sb_top_ptr]; // M2 使用，顶层表项
if(ENABLE_STORE) begin : StoreBuf
  wired_lsu_sb  wired_lsu_sb_inst (
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
  assign m1_hit[i] = m1.tag[i].p == m1.paddr[31:12]; // 只处理读命中。
  assign m1_rhit[i] = m1_hit[i] & m1.tag[i].rp;
  assign m1_whit[i] = m1_hit[i] & m1.tag[i].wp;
end
// sb Hit 生成
logic [3:0] m1_sb_hit;
for(genvar i = 0 ; i < 4 ; i += 1) begin
    assign m1_sb_hit[i] = sb_entry[i].paddr[31:2] == m1.paddr[31:2] && sb_valid[i];
end
logic [31:0] m1_sb_rdata;
logic [3:0] m1_sb_strb;
assign m1_sb_strb = ({4{m1_sb_hit[0]}} & sb_entry[0].strb) |
                    ({4{m1_sb_hit[1]}} & sb_entry[1].strb) |
                    ({4{m1_sb_hit[2]}} & sb_entry[2].strb) |
                    ({4{m1_sb_hit[3]}} & sb_entry[3].strb);
assign m1_sb_rdata = ({32{m1_sb_hit[0]}} & sb_entry[0].wdata) |
                     ({32{m1_sb_hit[1]}} & sb_entry[1].wdata) |
                     ({32{m1_sb_hit[2]}} & sb_entry[2].wdata) |
                     ({32{m1_sb_hit[3]}} & sb_entry[3].wdata);

// sb_w 项目生成
always_comb begin
  sb_w.paddr = m1.paddr;
  sb_w.hit = m1_whit;
  sb_w.strb = m1.strb;
  sb_w.wdata = m1.wdata;
`ifdef _VERILATOR
  sb_w.vaddr = m1.vaddr;
`endif
end

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
  logic       [3:0] sb_hit;   // store buffer 在 m2 暂停的时候，不会被更新，因此 sb_hit 可以一直使用。
  logic             any_sbhit;
  logic [3:0][SRAM_WIDTH-1:0] data;    // sram data
  logic found_excp;
  lsu_excp_t   d_excp;
  fetch_excp_t f_excp;
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
m2_pack_t m2_q;
always_ff @(posedge clk) if(!m2_stall) m2_q <= m1_gat;
always_comb begin
  m1_gat.vaddr     = m1.vaddr;   m1_gat.paddr     = m1.paddr;
  m1_gat.msize     = m1.msize;   m1_gat.msigned   = m1.msigned;
  m1_gat.wid       = m1.wid;     m1_gat.wreq      = m1.wreq;
  m1_gat.strb      = m1.strb;    m1_gat.wdata     = m1.wdata;
  m1_gat.uncache   = m1.uncache; m1_gat.cacop     = m1.cacop;
  m1_gat.dbar      = m1.dbar || (m1.uncache && ENABLE_SC_UNCACHE);
  m1_gat.llsc      = m1.llsc;    m1_gat.hit       = m1_rhit;
  m1_gat.any_rhit  = |m1_rhit;   m1_gat.any_whit  = |m1_whit;
  m1_gat.sb_hit    =  m1_sb_hit; m1_gat.any_sbhit = |m1_sb_hit;
  m1_gat.data = m1.data;
  m1_gat.d_excp.pil  = m1.inv && !m1.wreq; m1_gat.d_excp.pis  = m1.inv && m1.wreq;
  m1_gat.d_excp.pme  = m1.pme;             m1_gat.d_excp.ppi  = m1.ppi;
  m1_gat.d_excp.ale  = m1.ale;             m1_gat.d_excp.tlbr = m1.tlbr;
  m1_gat.f_excp.adef = m1.ale;             m1_gat.f_excp.tlbr = m1.tlbr;
  m1_gat.f_excp.pif  = m1.inv;             m1_gat.f_excp.ppi  = m1.ppi;
  m1_gat.found_excp = m1.inv | m1.pme | m1.ppi | m1.ale | m1.tlbr;
end

// M2 核心状态机
typedef enum logic[3:0] {
  S_NORMAL,
  S_MWAITSB, // Storebuffer Multihit
  S_MREFILL, // Read miss(LL include) REFILL
  S_MCACOP,  // Cache operation
  S_MUCLOAD, // Uncached load(Weakly ordered)
  S_CUCLOAD, // Uncached load
  S_CUCSTRD, // Uncached store
  S_CREFILL  // Store miss(SC exclude) REFILL
} fsm_e;
fsm_e fsm_q;
fsm_e fsm;
always_ff @(posedge clk) begin
  if(!rst_n) fsm_q <= S_NORMAL;
  else fsm_q <= fsm;
end
typedef enum logic[1:0] {
  M_NORMAL,
  M_HANDLED,
  M_DBAR     // 阻塞 LSU 后续请求，但响应 CPU 请求
} mod_e;
mod_e mod_q;
mod_e mod;
always_ff @(posedge clk) begin
  if(!rst_n || flush_i) mod_q <= M_NORMAL;
  else mod_q <= mod;
end

// 输出接口
logic resp_valid, resp_ready;
iq_lsu_resp_t resp;
// 状态机局部数据存储
typedef struct packed {
  logic      unc_msigned;
  logic [1:0]  unc_msize;
  logic [31:0] unc_paddr;
  logic [SRAM_WIDTH-1:0] fsm_rdata;
} m2_var_t;
m2_var_t m2_var;
m2_var_t m2_var_q;
logic [3:0]  sb_fwd_mask;
logic [31:0] sb_fwd_data;
always_ff @(posedge clk) m2_var_q <= m2_var;
always_comb begin
  // 主要输出及 sb
  sb_inv = '0;     // sb 控制信号
  resp_valid = '0; // 主要输出握手
  resp = '0;       // 主要输出数据
  resp.excp = m2_q.excp;
  resp.uncached = m2_q.uncache;
  resp.vaddr = m2_q.vaddr;
  for(integer i = 0 ; i < 4 ; i += 1) begin
    if(!ENABLE_64) begin
      resp.rdata[31:0] |= m2_q.hit[i] ? m2_q.data[i] : '0;
    end else begin
      resp.rdata       |= m2_q.hit[i] ? m2_q.data[i] : '0;
    end
  end
  resp.wid = m2_q.wid;
  sb_fwd_mask = '0;
  sb_fwd_data = '0;
  if(ENABLE_STORE) begin
    for(integer i = 0 ; i < 4 ; i += 1) begin
      sb_fwd_mask |= m2_q.sb_hit[i] ? sb_entry[i].strb : '0;
      sb_fwd_data |= m2_q.sb_hit[i] ? sb_entry[i].wdata : '0;
    end
    resp.rdata[31:0] = gen_mask_word(resp.rdata[31:0], sb_fwd_data, sb_fwd_mask);
  end
  if(!ENABLE_64) begin
    // 偏移处理
    resp.rdata[31:0] = mkrsft(resp.rdata[31:0], m2_q.vaddr, m2_q.msize, m2_q.msigned);
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
      bus_req_o.sram_wb_req = '1;
      sb_inv = '1;
    end
    bus_req_o.wdata = sb_top.wdata; // 巧合的是，unc也存在这里
    bus_req_o.sram_addr = sb_top.paddr;
    bus_req_o.wstrobe = sb_top.strb;
    bus_req_o.size = m2_var_q.unc_msize;
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
            if(!m2_q.uncache && m2_q.cacop == RD_ALLOC && (!m2_q.any_rhit || (!m2_q.any_whit && m2_q.llsc)) && !(m2_q.any_sbhit && sb_fwd_mask == '1)) begin // 未命中（rhit for all || whit for ll.w）的 cached 读请求
              fsm = S_MREFILL;
              m2_stall = '1;
              resp_valid = '0;
            end else if(!ENABLE_SC_UNCACHE && m2_q.uncache && m2_q.cacop == RD_ALLOC) begin // Uncached 读请求，且弱序
              fsm = S_MUCLOAD;
              m2_stall = '1;
              resp_valid = '0;
            end else if(m2_q.wreq && (m2_q.sb_hit & sb_valid) != '0) begin // 重叠的写请求
              // 由于这条指令在 M2 级别，也就是写入过 SB 的最新指令，后续指令在这条指令前进之前，不会写 SB。
              fsm = S_MWAITSB;
              m2_stall = '1;
              resp_valid = '0;
            end else if(m2_q.cacop) begin // Cache 无效化请求
              fsm = S_MCACOP;
              m2_stall = '1;
              resp_valid = '0;
            end
          end
          if(!m2_stall && m2_q.dbar) begin
            // 记录产生阻塞效果指令的物理地址（主要是 uncached load/store）
            m2_var.unc_msigned = m2_q.msigned;
            m2_var.unc_msize = &m2_q.msize ? 2'd10 : m2_q.msize;
            m2_var.unc_paddr = m2_q.paddr;
            // 阻塞住下一条指令
            mod = M_DBAR;
          end
        end else begin
          resp_valid = '0;
        end
      end
      else if(mod_q == M_HANDLED) begin
        resp_valid = '1;
        resp.rdata[SRAM_WIDTH-1:0] = m2_var_q.fsm_rdata; // 使用 fsm 缓存好的数据即可
        if(resp_ready) begin
          mod = m2_q.dbar ? M_DBAR : M_NORMAL;
        end
      end
  end
  S_MWAITSB: begin
    // 等待重复命中的表项被提交或者冲刷
    if(flush_i || (m2_q.sb_hit & sb_valid) == '0) begin
      fsm = S_NORMAL;
    end
  end
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
      bus_req_o.inv_req = m2_q.cacop; // 对于 ll 指令，需要申请写权限
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
      bus_req_o.target_paddr = sb_top.paddr;
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
  always_ff @(posedge clk) begin
    if(!rst_n || flush_i) begin
      resp_valid_q <= '0;
    end else if(resp_ready) begin
      resp_valid_q <= resp_valid;
    end
  end
  always_ff @(posedge clk) resp_q <= resp;
  assign resp_ready = !resp_valid_q || lsu_resp_ready_i;
  assign lsu_resp_valid_o = resp_valid_q;
  assign lsu_resp_o = resp_q;
end else begin
  assign resp_ready = lsu_resp_ready_i;
  assign lsu_resp_valid_o = resp_valid;
  assign lsu_resp_o = resp;
end

`ifdef _VERILATORS
  if(ENABLE_STORE) : store_difftest
  begin
    // 对所有 Store 指令进行提交处理
    DifftestStoreEvent DifftestStoreEvent_p (
                         .clock     (clk),
                         .coreid    ('0 ),
                         .index     ('0 ),
                         .valid     (c_lsu_req_i.storebuf_commit),
                         .storePAddr(sb_top.paddr),
                         .storeVAddr(sb_top.vaddr),
                         .storeData (sb_top.wdata >> {sb_top.vaddr[1:0],3'd0}) // 恢复已经偏移的写数据
                       );
  end
`endif

endmodule
