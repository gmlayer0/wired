// dcache cpu side
`include "wired0_defines.svh"

module wired_lsu(
    `_WIRED_GENERAL_DEFINE,

    // CPU LSU IQ 中的请求接口
    input  logic         lsu_req_valid_i,
    output logic         lsu_req_ready_o,
    input  iq_lsu_req_t  lsu_req_i,

    output logic         lsu_resp_valid_o,
    input  logic         lsu_resp_ready_i,
    output iq_lsu_resp_t lsu_resp_o,

    // CPU COMMIT 中的交互端口
    input  commit_lsu_req_t  commit_lsu_req_i,
    output commit_lsu_resp_t commit_lsu_resp_o,

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
    input  logic [3:0][31:0]  p_rdata_i,
    input  cache_tag_t [3:0]  p_tag_i,

    // 无效化端口
    input  logic              flush_i

);

    // IQ-M1 真握手信号
    logic iq_m1_ready; // iq -> m1 的通路就绪
    // M1-M2 真握手信号
    logic m1_m2_ready, m1_m2_valid;
    // STOREBUFFER 握手信号
    logic store_buffer_ready;

    // IQ-M1 握手生成
    assign lsu_req_ready_o = iq_m1_ready;

    // M1 信号定义
    tlb_s_resp_t m1_tlb_resp;
    // 同时开始地址翻译请求与 SRAM 请求。
    // 例化地址翻译模块
    wired_addr_trans # (
        .FETCH_ADDR('0)
    )
    wired_addr_trans_inst (
        `_WIRED_GENERAL_CONN,
        .clken_i(iq_m1_ready),
        .vaddr_i(lsu_req_i.vaddr),
        .csr_i(csr_i),
        .tlb_update_req_i(tlb_update_i),
        .trans_result_o(m1_tlb_resp)
    );

    // M1 主要流水
    logic m1_valid_q, m1_skid_valid_q;
    assign m1_m2_valid = m1_valid_q | m1_skid_valid_q;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            m1_skid_valid_q <= '0;
        end else begin
            if(m1_skid_valid_q) begin
                m1_skid_valid_q <= !m1_m2_ready;
            end else begin 
                if(!m1_m2_ready && m1_m2_valid) begin
                    m1_skid_valid_q <= '1;
                end
            end
        end
    end
    // M1 请求结构体
    typedef struct packed {
        logic             wreq;   // 写请求
        logic       [3:0] strb;    // 写掩码
        logic       [2:0] cacop;   // cache 请求
        logic             dbar;    // 产生 dbar 效果
        logic             llsc;    // llsc 指令，读时需要申请写权限
        logic       [1:0] msize;   // 访存大小-1
        logic      [31:0] paddr;   // 请求物理地址
        logic      [31:0] vaddr;   // 请求虚拟地址
        logic             uncache; // Uncached 请求
        logic      [31:0] wdata;   // 写数据
        cache_tag_t [3:0] tag;     // sram tag
        logic [3:0][31:0] data;    // sram data
        logic inv; logic pme; logic ppi; logic ale; logic tlbr; // TLB EXCP
    } m1_pack_t;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            m1_valid_q <= '0;
        end else begin
            if(iq_m1_ready) m1_valid_q <= lsu_req_valid_i;
        end
    end
    iq_lsu_req_t m1_req_q;
    always_ff @(posedge clk) begin
        if(iq_m1_ready) m1_req_q <= lsu_req_i;
    end
    m1_pack_t m1_raw, m1_nosnop, m1; // m1_raw 直接来自输入打一拍， m1_nosnop 在 skid 与 raw 之间进行选择，经过 snoop 得到 m1
    m1_pack_t m1_skid_q;
    assign iq_m1_ready = !m1_skid_valid_q;
    assign p_addr_o = m1_skid_valid_q ? m1_req_q.vaddr[11:0] : lsu_req_i.vaddr[11:0];

    // m1_raw 逻辑
    logic m1_tlb_no_excp; // TODO:对于 (sc && llbit == '0) || (cacheop && no_addr_trans)不触发异常
    assign m1_tlb_no_excp = m1_req_q.cacop inside {IDX_INIT, IDX_INV};
    always_comb begin
        m1_raw.wreq = |m1_req_q.strb;
        m1_raw.strb = m1_req_q.strb;
        m1_raw.cacop = m1_req_q.cacop;
        m1_raw.dbar = m1_req_q.dbar || (!m1_tlb_resp.value.mat[0]); // TODO: MAKE SURE UNCACHED INST CAN ALSO CAUSE DBAR
        m1_raw.llsc = m1_req_q.llsc;
        m1_raw.msize = m1_req_q.msize;
        m1_raw.paddr = {m1_tlb_resp.value.ppn, m1_req_q.vaddr[11:0]};
        m1_raw.vaddr = m1_req_q.vaddr;
        m1_raw.uncached = !m1_tlb_resp.value.mat[0];
        m1_raw.wdata = m1_req_q.wdata;
        m1_raw.tag = p_tag_i;
        m1_raw.data = p_rdata_i;
        m1_raw.ale = (m1_req_q.msize == 2'd0) ? '0 :
                     ((m1_req_q.msize == 2'd1) ?  m1_req_q.vaddr[0] :
                                                (|m1_req_q.vaddr[1:0]));
        m1_raw.tlbr = !m1_raw.ale && !m1_tlb_resp.found && !m1_tlb_no_excp;
        m1_raw.inv = !m1_raw.tlbr && !m1_tlb_resp.value.v && !m1_tlb_no_excp;
        m1_raw.ppi = !m1_raw.inv && (m1_tlb_resp.value.plv == 2'b00) && (csr_i.crmd[`_CRMD_PLV] == 2'd3) && !m1_tlb_no_excp;
        m1_raw.pme = !m1_raw.ppi && !m1_tlb_resp.value.d && (|m1_req_q.strb) && !m1_tlb_no_excp;
    end

    // m1_nosnop 逻辑
    assign m1_no_snop = m1_skid_valid_q ? m1_skid_q : m1_raw;

    // m1_skid_q 逻辑
    always_ff @(posedge clk) begin
        m1_skid_q <= m1;
    end

    // m1 逻辑，主要是 snoop sram 的所有写入
    always_comb begin
        m1 = m1_nosnop;
        // 对 DATA SRAM 的 snoop
        if(snoop_i.daddr[11:4] == m1.paddr[11:4]) begin // 是同一个 Cache line
            m1.data[snoop_i.dway] = gen_mask_word(m1.data[snoop_i.dway], snoop_i.d[m1.paddr[3:2]], snoop_i.dstrb[m1.paddr[3:2]]);
        end
        if(snoop_i.taddr[11:4] == m1.paddr[11:4]) begin // 是同一个 Cache line
            for(integer w = 0 ; w < 4 ; w += 1) begin // 逐路检查
                if(snoop_i.twe[w]) begin
                    m1.tag[w] = snoop_i.t;
                end
            end
        end
    end
    sb_meta_t sb_w, sb_top;
    
    logic sb_inv; // TODO: CONNECT ME
    logic [3:0] sb_valid;
    sb_meta_t [3:0] sb_entry;
    // storebuffer 定义
    wired_lsu_sb  wired_lsu_sb_inst (
    `_WIRED_GENERAL_CONN,
    .flush_i(flush_i),
    .ready_o(store_buffer_ready),
    .valid_i(m1_m2_ready && m1_m2_valid && m1.wreq),  /* 当且仅当 m1-m2 握手成功时，更新 storebuffer */
    .meta_i(sb_w),
    .valid_o(sb_valid),
    .meta_o(sb_entry),
    .invalid_i(sb_inv),
    .top_hit_o(commit_lsu_resp_o.storebuf_hit),
    .top_meta_o(sb_top),
    .snoop_i(snoop_i)
  );

    // M1-M2 部分
    // hit 生成
    logic [3:0] m1_hit, m1_rhit, m1_whit;
    for(genvar i = 0 ; i < 4 ; i += 1) begin
        assign m1_hit[i] = m1.tag[w].p == m1.paddr[31:12]; // 只处理读命中。
        assign m1_rhit[i] = m1_hit[i] & m1.tag[w].rp;
        assign m1_whit[i] = m1_hit[i] & m1.tag[w].wp;
    end
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
    // sbhit 生成：生成与 storebuf 的碰撞情况，（注意重复碰撞是不被允许的，故存在 hit 的 uncached store 指令会暂停住管线）
    logic [3:0] m1_sb_hit;
    // 检查四项 sb_entry
    for(genvar i = 0 ; i < 4 ; i += 1) begin
        assign m1_sb_hit[i] = sb_entry[i].paddr[31:2] == m1.paddr[31:2] && sb_valid[i];
    end
    // 由于约束， 至多存在一项 hit，也仅有一组 strb。
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

    // 注： M1 天然带有 skid buf 属性，当 M2 暂停后的第一个周期，M1 依然可以接受一条请求。
    // M1 此时的请求将被压入 M1 的 skid buf 中记录，并实时检查 snoop 进行更新。
    // 这里的 snoop 主要是为了避免重复 refill

    // M2 部分开始，涉及主状态机
    // M2 请求结构体
    typedef struct packed {
        logic             wreq;   // 写请求
        logic       [3:0] strb;    // 写掩码
        logic       [2:0] cacop;   // cache 请求
        logic             dbar;    // 产生 dbar 效果
        logic             llsc;    // llsc 指令，读时需要申请写权限
        logic       [1:0] msize;   // 访存大小-1
        logic      [31:0] paddr;   // 请求物理地址
        logic             uncache; // Uncached 请求
        logic      [31:0] wdata;   // 写数据
        logic       [3:0] hit;     // 这里不用更新，到达此处的命中指令被认为已经完成访存
        logic       [3:0] sb_hit;  // store buffer 在 m2 暂停的时候，不会被更新，因此 sb_hit 可以一直使用。
        logic [3:0][31:0] data;    // sram data
        logic inv; logic pme; logic ppi; logic ale; logic tlbr; // TLB EXCP
    } m2_pack_t;
    logic m2_c_valid_q;
    m2_pack_t m2;
    always_ff @(posedge clk) begin
        if(m1_m2_ready) begin
            m2 <= m1;
        end
    end
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            m2_c_valid_q <= '0;
        end else if(m1_m2_ready) begin
            m2_c_valid_q <= m1_m2_valid;
        end
    end

    // M2 主状态机
    typedef logic[3:0] fsm_t;
    localparam fsm_t S_NORMAL  = 0;
    localparam fsm_t S_HANDLED = 1; // 已经处理就绪，但是后级暂停
    localparam fsm_t S_MWAITSB = 2; // Storebuffer Multihit
    localparam fsm_t S_MREFILL = 3; // Read miss(LL include) REFILL
    localparam fsm_t S_MCACOP  = 4;  // Cache operation
    localparam fsm_t S_MDBAR   = 5;   // DBarrier
    localparam fsm_t S_CUCLOAD = 6; // Uncached load
    localparam fsm_t S_CUCSTRD = 7; // Uncached store
    localparam fsm_t S_CREFILL = 8; // Store miss(SC exclude) REFILL

    fsm_t fsm_q;
    fsm_t fsm;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            fsm_q <= S_NORMAL;
        end else begin
            fsm_q <= fsm;
        end
    end
    // 主状态机转移逻辑
    always_comb begin
        fsm = fsm_q;
        m1_m2_ready = '0;
        bus_req_o = '0;         // 产生所有到总线管理器的请求
        commit_lsu_resp_o = '0; // 产生所有到提交级的响应
        case (fsm_q)
        default/*S_NORMAL*/: begin
            
        end
        S_HANDLED: begin
            
        end
        S_MWAITSB: begin
            
        end
        S_MREFILL: begin
            
        end
        S_MCACOP: begin
            
        end
        S_MDBAR: begin
            
        end
        S_CUCLOAD: begin
            
        end
        S_CUCSTRD: begin
            
        end
        S_CREFILL: begin
            
        end
        endcase
    end
    
    // 对所有 Store 指令进行提交处理
    DifftestStoreEvent DifftestStoreEvent_p (
      .clock     (clk),
      .coreid    ('0 ),
      .index     ('0 ),
      .valid     (commit_lsu_req_i.storebuf_commit),
      .storePAddr(sb_top.paddr),
      .storeVAddr(sb_top.vaddr),
      .storeData (sb_top.wdata)
    );

endmodule
