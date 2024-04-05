// icache cpu side
`include "wired0_defines.svh"

module wired_icache #(
    parameter integer PACKED_SIZE = 32
  )(
    `_WIRED_GENERAL_DEFINE,

    // CPU FETCH 请求接口
    input  logic                   f_valid_i,
    output logic                   f_ready_o,
    input  logic            [1:0]   f_mask_i,
    input  logic            [31:0]    f_pc_i,
    input  logic [PACKED_SIZE-1 : 0] f_pkg_i,

    // CACHE OP 请求端口
    // 很简单，这就是不需要的
    // ibarrier 一定会使后端写入在前端可见。
    // input  logic                   c_valid_i,
    // output logic                   c_ready_o,
    // input  logic            [31:0]  c_addr_i,
    // input  inv_parm_e               c_parm_i,

    // CPU FETCH 请求返回接口 TODO: CHECK
    output logic                   f_valid_o,
    input  logic                   f_ready_i,
    output logic            [1:0]   f_mask_o,
    output logic            [31:0]    f_pc_o,
    output logic       [1:0][31:0]  f_inst_o,
    output logic [PACKED_SIZE-1 : 0] f_pkg_o,
    output fetch_excp_t             f_excp_o, // 取值错误，显然两条指令是一致的

    // 到总线侧的请求接口 TODO: CHECK
    output lsu_bus_req_t  bus_req_o, // TODO
    input  lsu_bus_resp_t bus_resp_i,

    // 用于 SNOOP 的总线更新接口
    input  dsram_snoop_t  snoop_i,

    // 用于地址翻译更新接口
    input  csr_t            csr_i,
    input  tlb_update_req_t tlb_update_i,

    // SRAM 读端口 TODO CONN
    output logic [11:0]       p_addr_o,
    input  logic [3:0][63:0]  p_rdata_i,
    input  cache_tag_t [3:0]  p_tag_i,

    // 无效化端口
    input  logic              flush_i   // 所有管线中的指令都会被无效化
  );

  typedef struct packed {
            logic            [1:0]   mask;
            logic            [31:0]    pc;
            logic [PACKED_SIZE-1 : 0] pkg;
          } pg_f_t;
  pg_f_t pg_raw;
  always_comb
  begin
    pg_raw.mask = f_mask_i;
    pg_raw.pc   = f_pc_i;
    pg_raw.pkg  = f_pkg_i;
  end

  // F1 信号定义
  pg_f_t pg_f1_q;
  tlb_s_resp_t f1_tlb_resp;
  // 同时开始地址翻译请求与 SRAM 请求。
  // 例化地址翻译模块
  wired_addr_trans # (
                     .FETCH_ADDR('1)
                   )
                   wired_addr_trans_inst (
                     `_WIRED_GENERAL_CONN,
                     .clken_i(f_ready_o),
                     .vaddr_i(f_pc_i),
                     .csr_i(csr_i),
                     .tlb_update_req_i(tlb_update_i),
                     .trans_result_o(f1_tlb_resp)
                   );
  logic f1_skid_busy_q; // F1 级别进入 skid 状态的信号
  assign f_ready_o = !f1_skid_busy_q;
  always_ff @(posedge clk)
  begin
    if(f_ready_o)
    begin
      pg_f1_q <= pg_raw;
    end
  end
  // F2 段到 f1 握手
  logic f1_f2_valid, f1_f2_ready; // TODO: 握手信号，纯组合逻辑驱动
  logic f1_valid_q;
  wire f1_f2_valid = f1_valid_q | f1_skid_busy_q;
  always_ff @(posedge clk)
  begin
    if(!rst_n || flush_i)
    begin
      f1_valid_q <= '0;
    end
    else
    begin
      if(f1_valid_q) begin
        if(f1_f2_ready && f_ready_o) begin
            f1_valid_q <= f_valid_i;
        end
      end else begin
        f1_valid_q <= f_valid_i;
      end
    end
  end
  always_ff @(posedge clk)
  begin
    if(!rst_n || flush_i)
    begin
      f1_skid_busy_q <= '0;
    end
    else
    begin
      if(f1_skid_busy_q)
      begin
        if(f1_f2_ready)
        begin
          f1_skid_busy_q <= '0;
        end
      end
      else
      begin
        if(f1_f2_valid & (!f1_f2_ready))
        begin
          f1_skid_busy_q <= '1;
        end
      end
    end
  end
  typedef struct packed {
            logic [31:0] pc;
            logic [31:0] addr;             // 这里已经是物理地址
            // inv_parm_e   parm;
            logic [1:0]  mask;
            logic     uncache;

            logic [PACKED_SIZE-1 : 0] pkg; // 时刻携带传入信息

            cache_tag_t [3:0]  tag;
            logic [3:0][63:0] data;
            logic inv; logic ppi; logic ale; logic tlbr; // TLB EXCP
          } f1_pack_t;
  f1_pack_t f1_raw, f1_sel, f1; // 组合逻辑
  f1_pack_t f1_skid_q; // 寄存器堆
  logic [3:0] f1_hit;  // 纯组合逻辑，基于 f1 信号

  assign p_addr_o = f1_skid_busy_q ? f1_raw.addr[11:0] : f_pc_i[11:0];
  always_ff @(posedge clk)
  begin
    f1_skid_q <= f1; // 每拍都打
  end
  logic f1_f2_stall_q;
  assign f1_sel = f1_f2_stall_q ? f1_skid_q : f1_raw;
  always_ff @(posedge clk) f1_f2_stall_q <= !f1_f2_ready && f1_f2_valid;

  // SNOP 逻辑开始
  always_comb
  begin
    f1 = f1_sel;
    // TODO: CHECKME
    // 对 DATA SRAM 的 snoop
    if(snoop_i.daddr[11:4] == f1.addr[11:4])
    begin // 是同一个 Cache line
      f1.data[snoop_i.dway] = {snoop_i.d[{f1.addr[3],1'b1}], snoop_i.d[{f1.addr[3],1'b0}]}; // 不存在部分写存在，全部转发即可
    end
    if(snoop_i.taddr[11:4] == f1.addr[11:4])
    begin // 是同一个 Cache line
      for(integer w = 0 ; w < 4 ; w += 1)
      begin // 逐路检查
        if(snoop_i.twe[w])
        begin
          f1.tag[w] = snoop_i.t;
        end
      end
    end
  end

  // F1_RAW 逻辑开始
  always_comb
  begin
    f1_raw = '0;
    // TODO: checkme
    // 基于 f1_tlb_resp pg_f1_q 还有 SRAM 返回值构建即可
    f1_raw.pc      = pg_f1_q.pc;
    f1_raw.addr    = {f1_tlb_resp.value.ppn, pg_f1_q.pc[11:0]};
    // f1_raw.parm = '0;
    f1_raw.mask    = pg_f1_q.mask;
    f1_raw.uncache = !f1_tlb_resp.value.mat[0];
    f1_raw.pkg     = pg_f1_q.pkg;

    f1_raw.tag     = p_tag_i;
    f1_raw.data    = p_rdata_i;
    f1_raw.ale     =  |pg_f1_q.pc[1:0];
    f1_raw.tlbr    = (!f1_raw.ale)  && (!f1_tlb_resp.found);
    f1_raw.inv     = (!f1_raw.tlbr) && (!f1_tlb_resp.value.v);
    f1_raw.ppi     = (!f1_raw.inv)  && (f1_tlb_resp.value.plv == 2'b00) && (csr_i.crmd[`_CRMD_PLV] == 2'd3);
  end

  // HIT 逻辑开始
  for(genvar w = 0 ; w < 4 ; w += 1)
  begin
    assign f1_hit[w] = f1.tag[w].p == f1.addr[31:12] && f1.tag[w].rp; // 生成四路 hit信号。
  end

  /* F2 段开始 */
  logic f2_valid_q; // F2 有效寄存器，状态机使用
  always_ff @(posedge clk)
  begin
    if(!rst_n || flush_i)
      f2_valid_q <= '0;
    if(f1_f2_ready)
      f2_valid_q <= f1_f2_valid;
  end
  typedef struct packed {
            logic [31:0] pc;
            logic [31:0] addr;
            // inv_parm_e   parm;
            logic [1:0]  mask;
            logic     uncache;

            logic [PACKED_SIZE - 1 : 0] pkg; // 时刻携带传入信息

            logic [3:0]   hit; // 是否命中信息
            logic [3:0][63:0] data; // 需要实时更新
            logic inv; logic ppi; logic ale; logic tlbr; // TLB EXCP
          } f2_pack_t;
  logic [1:0][31:0] fsm_data_q;
  logic [1:0][31:0] fsm_data;
  always_ff @(posedge clk) fsm_data_q <= fsm_data;
  f1_pack_t f1_q;
  logic [3:0] f1_hit_q;
  f2_pack_t f2;
  wire has_excp = f1_q.inv | f1_q.ppi | f1_q.ale | f1_q.tlbr;
  always_ff @(posedge clk)
  begin
    if(f1_f2_ready)
    begin
      f1_q <= f1;
      f1_hit_q <= f1_hit;
    end
  end
  always_comb
  begin
    f2.pc      = f1_q.pc;
    f2.addr    = f1_q.addr;
    f2.mask    = f1_q.mask;
    f2.uncache = f1_q.uncache;
    f2.pkg     = f1_q.pkg;
    f2.hit     = f1_hit_q;
    f2.data    = f1_q.data;
    f2.inv     = f1_q.inv;
    f2.ppi     = f1_q.ppi;
    f2.ale     = f1_q.ale;
    f2.tlbr    = f1_q.tlbr;
  end
  typedef enum logic[1:0] {
            S_FREE,
            S_HANDLED, // 已经处理就绪，但是后级暂停
            S_REFILL,
            S_UNC      // UNCACHED LOAD
          } fsm_t;
  fsm_t fsm;
  fsm_t fsm_q;
  always_ff @(posedge clk)
  begin
    if(!rst_n)
    begin
      fsm_q <= S_FREE;
    end
    else
    begin
      fsm_q <= fsm;
    end
  end
  always_comb
  begin
    fsm = fsm_q;
    fsm_data = fsm_data_q;
    bus_req_o = '0;
    bus_req_o.target_paddr = {f2.addr[31:3], 3'd0};
    bus_req_o.size = (&f2.mask) ? 2'd3 : 2'd2;
    f_valid_o = '0;
    f1_f2_ready = '0;
    f_mask_o = f2.mask;
    f_pc_o = f2.pc;
    f_pkg_o = f2.pkg;
    f_excp_o.adef = f2.ale;
    f_excp_o.tlbr = f2.tlbr;
    f_excp_o.pif  = f2.inv;
    f_excp_o.ppi  = f2.ppi;
    f_inst_o = '0;
    for(integer i = 0 ; i < 4 ; i += 1)
    begin
      f_inst_o |= f2.hit[i] ? f2.data[i] : '0;
    end
    // 之后是主要状态机
    case (fsm_q)
      default/*S_FREE*/:
      begin
        f1_f2_ready = f_ready_i;
        f_valid_o = f2_valid_q;
        if(has_excp)
        begin
          f_inst_o = 32'h00100000; // add.w $r0,$r0,$r0
        end
        else if(f2_valid_q && (|f2.mask))
        begin
          if(f2.uncache) begin
            f1_f2_ready = '0;
            f_valid_o = '0;
            fsm = S_UNC;
          end else if(!(|f2.hit)) begin
            f1_f2_ready = '0;
            f_valid_o = '0;
            fsm = S_REFILL;
          end
        end
      end
      S_HANDLED:
      begin
        f1_f2_ready = f_ready_i || !f2_valid_q;
        f_valid_o = f2_valid_q;
        f_inst_o = fsm_data_q;
        if(f_ready_i || !f2_valid_q) // 被 flush 了。
        begin
          fsm = S_FREE;
        end
      end
      S_REFILL:
      begin
        // 重填完成后返回
        bus_req_o.valid = '1;
        bus_req_o.inv_req = RD_ALLOC;
        fsm_data = bus_resp_i.rdata;
        if(bus_resp_i.ready)
        begin
          fsm = /*f_ready_i ? S_FREE : */S_HANDLED;
        end
      end
      S_UNC:
      begin
        // 重填完成后返回
        bus_req_o.valid = '1;
        bus_req_o.uncached_load_req = '1;
        bus_req_o.target_paddr[2] = f2.mask[0] ? '0 : '1;
        bus_req_o.size = (&f2.mask) ? 2'd3 : 2'd2;
        fsm_data = bus_resp_i.rdata;
        if(bus_resp_i.ready)
        begin
          fsm = /*f_ready_i ? S_FREE : */S_HANDLED;
        end
      end
    endcase
  end


endmodule
