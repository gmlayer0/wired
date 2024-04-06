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
    input  logic [3:0][31:0]  p_rdata_i,
    input  cache_tag_t [3:0]  p_tag_i,

    // 无效化端口
    input  logic              flush_i

);

    // M1-M2 真握手信号
    logic m1_m2_ready, m1_m2_valid;
    logic m1_m2_stall_q;
    always_ff @(posedge clk) m1_m2_stall_q <= !m1_m2_ready && m1_m2_valid;
    // STOREBUFFER 握手信号
    logic store_buffer_ready;
    // M1 信号定义
    tlb_s_resp_t m1_tlb_resp;
    // 同时开始地址翻译请求与 SRAM 请求。
    // 例化地址翻译模块
    wired_addr_trans # (
        .FETCH_ADDR('0)
    )
    wired_addr_trans_inst (
        `_WIRED_GENERAL_CONN,
        .clken_i(lsu_req_ready_o),
        .vaddr_i(lsu_req_i.vaddr),
        .csr_i(csr_i),
        .tlb_update_req_i(tlb_update_i),
        .trans_result_o(m1_tlb_resp)
    );

    // M1 主要流水
    logic m1_valid_q, m1_skid_busy_q;
    assign m1_m2_valid = m1_valid_q;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            m1_skid_busy_q <= '0;
        end else begin
            if(m1_skid_busy_q) begin
                m1_skid_busy_q <= !m1_m2_ready;
            end else begin 
                if(!m1_m2_ready && m1_valid_q) begin
                    m1_skid_busy_q <= lsu_req_valid_i;
                end
            end
        end
    end
    // M1 请求结构体
    typedef struct packed {
        logic             wreq;   // 写请求
        rob_rid_t         wid;    // 写回地址
        logic       [3:0] strb;    // 写掩码
        logic       [2:0] cacop;   // cache 请求
        logic             dbar;    // 产生 dbar 效果
        logic             llsc;    // llsc 指令，读时需要申请写权限
        logic             msigned; // 有符号扩展
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
        if(!rst_n || flush_i) begin
            m1_valid_q <= '0;
        end else begin
            if(m1_valid_q) begin
                if(m1_m2_ready && lsu_req_ready_o) begin
                    m1_valid_q <= lsu_req_valid_i;
                end
            end else begin
                m1_valid_q <= lsu_req_valid_i;
            end
            // if(lsu_req_ready_o && m1_m2_ready) m1_valid_q <= lsu_req_valid_i;
        end
    end
    iq_lsu_req_t m1_req_q;
    always_ff @(posedge clk) begin
        if(lsu_req_ready_o) m1_req_q <= lsu_req_i;
    end
    m1_pack_t m1_raw, m1_nosnop, m1; // m1_raw 直接来自输入打一拍， m1_nosnop 在 skid 与 raw 之间进行选择，经过 snoop 得到 m1
    m1_pack_t m1_buf_q;
    assign lsu_req_ready_o = !m1_skid_busy_q;
    assign p_addr_o = m1_skid_busy_q ? m1_req_q.vaddr[11:0] : lsu_req_i.vaddr[11:0];

    // m1_raw 逻辑
    logic m1_tlb_no_excp; // TODO:对于 (sc && llbit == '0) || (cacheop && no_addr_trans)不触发异常
    assign m1_tlb_no_excp = m1_req_q.cacop inside {IDX_INIT, IDX_INV};
    always_comb begin
        m1_raw.wid  = m1_req_q.wid;
        m1_raw.wreq = |m1_req_q.strb;
        m1_raw.strb = m1_req_q.strb;
        m1_raw.cacop = m1_req_q.cacop;
        m1_raw.dbar = m1_req_q.dbar;
        m1_raw.llsc = m1_req_q.llsc;
        m1_raw.msigned = m1_req_q.msigned;
        m1_raw.msize = m1_req_q.msize;
        m1_raw.paddr = {m1_tlb_resp.value.ppn, m1_req_q.vaddr[11:0]};
        m1_raw.vaddr = m1_req_q.vaddr;
        m1_raw.uncache = !m1_tlb_resp.value.mat[0];
        m1_raw.wdata = m1_req_q.wdata; // 已对齐
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
    assign m1_nosnop = m1_m2_stall_q ? m1_buf_q : m1_raw;

    // m1_buf_q 逻辑
    always_ff @(posedge clk) begin
        m1_buf_q <= m1;
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
    logic sb_top_hit;
    logic [1:0] sb_top_ptr;
    sb_meta_t sb_w, sb_top;
    
    logic sb_inv; // TODO: CONNECT ME
    logic [3:0] sb_valid;
    logic [3:0] sb_valid_fwd;
    sb_meta_t [3:0] sb_entry;
    // storebuffer 定义
    wired_lsu_sb  wired_lsu_sb_inst (
    `_WIRED_GENERAL_CONN,
    .flush_i(flush_i),
    .ready_o(store_buffer_ready),
    .valid_i(m1_m2_ready && m1_m2_valid && m1.wreq),  /* 当且仅当 m1-m2 握手成功时，更新 storebuffer */
    .meta_i(sb_w),
    .valid_o(sb_valid),
    .valid_fwd_o(sb_valid_fwd),
    .meta_o(sb_entry),
    .top_o(sb_top_ptr),
    .invalid_i(sb_inv),
    .top_hit_o(sb_top_hit),
    .top_meta_o(sb_top),
    .snoop_i(snoop_i)
  );

    // M1-M2 部分
    // hit 生成
    logic [3:0] m1_hit, m1_rhit, m1_whit;
    for(genvar i = 0 ; i < 4 ; i += 1) begin
        assign m1_hit[i] = m1.tag[i].p == m1.paddr[31:12]; // 只处理读命中。
        assign m1_rhit[i] = m1_hit[i] & m1.tag[i].rp;
        assign m1_whit[i] = m1_hit[i] & m1.tag[i].wp;
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
        assign m1_sb_hit[i] = sb_entry[i].paddr[31:2] == m1.paddr[31:2] && sb_valid[i]; // TODO: CHECKME
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
        logic             wreq;     // 写请求
        rob_rid_t         wid;    // 写回地址
        logic       [3:0] strb;     // 写掩码
        logic       [2:0] cacop;    // cache 请求
        logic             dbar;     // 产生 dbar 效果
        logic             llsc;     // llsc 指令，读时需要申请写权限
        logic             msigned;  // 有符号扩展
        logic       [1:0] msize;    // 访存大小-1
        logic      [31:0] paddr;    // 请求物理地址
        logic      [31:0] vaddr;    // 请求物理地址
        logic             uncache;  // Uncached 请求
        logic      [31:0] wdata;    // 写数据
        logic       [3:0] hit;      // 这里不用更新，到达此处的命中指令被认为已经完成访存
        logic             any_rhit;
        logic             any_whit; // 专供 ll 指令使用
        logic       [3:0] sb_hit;   // store buffer 在 m2 暂停的时候，不会被更新，因此 sb_hit 可以一直使用。
        logic             any_sbhit; // 专供 ll 指令使用
        logic [3:0][31:0] data;     // sram data
        lsu_excp_t  excp;
        logic found_excp;
    } m2_pack_t;
    logic m2_valid_q;
    m2_pack_t m2;
    m2_pack_t m2_q;
    always_comb begin
        m2 = '0;
        m2.wid = m1.wid;
        m2.wreq = m1.wreq;
        m2.strb = m1.strb;
        m2.cacop = m1.cacop;
        m2.dbar = m1.dbar || m1.uncache; // TODO: MAKE SURE UNCACHED INST CAN ALSO CAUSE DBAR
        m2.llsc = m1.llsc;
        m2.msigned = m1.msigned;
        m2.msize = m1.msize;
        m2.paddr = m1.paddr;
        m2.vaddr = m1.vaddr;
        m2.uncache = m1.uncache;
        m2.wdata = m1.wdata;
        m2.hit = m1_rhit;
        m2.any_rhit = |m1_rhit;
        m2.any_whit = |m1_whit;
        m2.sb_hit = m1_sb_hit;
        m2.any_sbhit = |m1_sb_hit;
        m2.data = m1.data;
        m2.excp.pil = m1.inv && !m1.wreq;
        m2.excp.pis = m1.inv && m1.wreq;
        m2.excp.pme = m1.pme;
        m2.excp.ppi = m1.ppi;
        m2.excp.ale = m1.ale;
        m2.excp.tlbr = m1.tlbr;
        m2.found_excp = m1.inv | m1.pme | m1.ppi | m1.ale | m1.tlbr;
    end
    always_ff @(posedge clk) begin
        if(m1_m2_ready) begin
            m2_q <= m2;
        end
    end
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            m2_valid_q <= '0;
        end else if(m1_m2_ready) begin
            m2_valid_q <= m1_m2_valid;
        end
    end

    // M2 主状态机
    logic m2_c_valid_q;
    logic m2_c_valid, m2_c_ready;
    iq_lsu_resp_t m2_c;
    iq_lsu_resp_t m2_c_q;
    assign lsu_resp_valid_o = m2_c_valid_q;
    assign lsu_resp_o = m2_c_q;
    assign m2_c_ready = !m2_c_valid_q || lsu_resp_ready_i;
    always_ff @(posedge clk) if(m2_c_ready) m2_c_q <= m2_c;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            m2_c_valid_q <= '0;
        end else begin
            if(m2_c_ready) begin
                m2_c_valid_q <= m2_c_valid;
            end
        end
    end
    typedef enum logic[3:0] {
        S_NORMAL,
        S_MWAITSB, // Storebuffer Multihit
        S_MREFILL, // Read miss(LL include) REFILL
        S_MCACOP,  // Cache operation
        S_CUCLOAD, // Uncached load
        S_CUCSTRD, // Uncached store
        S_CREFILL  // Store miss(SC exclude) REFILL
    } fsm_e;
    typedef enum logic[1:0] {
        M_NORMAL,
        M_HANDLED,
        M_DBAR     // 阻塞 LSU 后续请求，但响应 CPU 请求
    } mod_e;
    logic unc_msigned;
    logic unc_msigned_q;
    logic [1:0]  unc_msize;
    logic [1:0]  unc_msize_q;
    logic [31:0] unc_paddr;
    logic [31:0] unc_paddr_q;
    logic [31:0] fsm_rdata;
    logic [31:0] fsm_rdata_q;
    always_ff @(posedge clk) unc_msigned_q <= unc_msigned;
    always_ff @(posedge clk) unc_msize_q <= unc_msize;
    always_ff @(posedge clk) unc_paddr_q <= unc_paddr;
    always_ff @(posedge clk) fsm_rdata_q <= fsm_rdata;
    mod_e mod_q;
    mod_e mod;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            mod_q <= M_NORMAL;
        end else begin
            mod_q <= mod;
        end
    end

    fsm_e fsm_q;
    fsm_e fsm;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            fsm_q <= S_NORMAL;
        end else begin
            fsm_q <= fsm;
        end
    end
    // 主状态机转移逻辑
    logic  [3:0] sb_fwd_mask;
    logic [31:0] sb_fwd_data;
    always_comb begin
        m2_c_valid = '0; // 主要输出握手
        m2_c = '0;       // 主要输出数据
        sb_inv = '0;
        m2_c.excp = m2_q.excp;
        m2_c.uncached = m2_q.uncache;
        m2_c.vaddr = m2_q.vaddr;
        m2_c.rdata = '0;
        for(integer i = 0 ; i < 4 ; i += 1) begin
            m2_c.rdata |= m2_q.hit[i] ? m2_q.data[i] : '0;
        end
        m2_c.wid = m2_q.wid;
        sb_fwd_mask = '0;
        sb_fwd_data = '0;
        for(integer i = 0 ; i < 4 ; i += 1) begin
            sb_fwd_mask |= m2_q.sb_hit[i] ? sb_entry[i].strb : '0;
            sb_fwd_data |= m2_q.sb_hit[i] ? sb_entry[i].wdata : '0;
        end
        m2_c.rdata = gen_mask_word(m2_c.rdata, sb_fwd_data, sb_fwd_mask);
        // 偏移处理
        m2_c.rdata = mkrsft(m2_c.rdata, m2_q.vaddr, m2_q.msize, m2_q.msigned);
        unc_msigned = unc_msigned_q;
        unc_msize = unc_msize_q;
        unc_paddr = unc_paddr_q;
        fsm_rdata = fsm_rdata_q;
        mod = mod_q;
        fsm = fsm_q;
        m1_m2_ready = '0;
        c_lsu_resp_o = '0; // 产生所有到提交级的响应
        c_lsu_resp_o.storebuf_hit = sb_top_hit;
        c_lsu_resp_o.uncached_load_resp = mkrsft(bus_resp_i.rdata[31:0], unc_paddr_q, unc_msize_q, unc_msigned_q);
        bus_req_o = '0;         // 产生所有到总线管理器的请求
        if(c_lsu_req_i.dbarrier_unlock) begin
            mod = M_NORMAL;
        end
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
        bus_req_o.size = unc_msize_q;
        bus_req_o.target_paddr = m2_q.paddr;
        case (fsm_q)
        default/*S_NORMAL*/: begin
            if(c_lsu_req_i.valid) begin // 响应来自提交级的请求
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
                m1_m2_ready = store_buffer_ready;
                if(m2_valid_q && !flush_i) begin
                    m1_m2_ready = store_buffer_ready && m2_c_ready; // m2_c 不 ready 的时候也得阻塞住
                    m2_c_valid = '1;
                    if(!m2_q.found_excp && !m2_q.uncache && m2_q.cacop == RD_ALLOC && (!m2_q.any_rhit || (!m2_q.any_whit && m2_q.llsc))) begin // 未命中（rhit for all || whit for ll.w）的 cached 读请求
                        fsm = S_MREFILL;
                        m1_m2_ready = '0;
                        m2_c_valid = '0;
                    end else if(!m2_q.found_excp && m2_q.wreq && (m2_q.sb_hit & sb_valid) != '0) begin // 重叠的写请求
                        // 由于这条指令在 M2 级别，也就是写入过 SB 的最新指令，后续指令在这条指令前进之前，不会写 SB。
                        fsm = S_MWAITSB;
                        m1_m2_ready = '0;
                        m2_c_valid = '0;
                    end else if(!m2_q.found_excp && m2_q.cacop) begin // Cache 无效化请求
                        fsm = S_MCACOP;
                        m1_m2_ready = '0;
                        m2_c_valid = '0;
                    end
                    if(m1_m2_ready && m2_q.dbar) begin
                        // 记录产生阻塞效果指令的物理地址（主要是 uncached load/store）
                        unc_msigned = m2_q.msigned;
                        unc_msize = &m2_q.msize ? 2'd10 : m2_q.msize;
                        unc_paddr = m2_q.paddr;
                        // 阻塞住下一条指令
                        mod = M_DBAR;
                    end
                end else begin
                    m2_c_valid = '0;
                end
            end
            else if(mod_q == M_HANDLED) begin
                m2_c_valid = '1;
                m2_c.rdata = fsm_rdata_q; // 使用 fsm 缓存好的数据即可
                if(m2_c_ready) begin
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
            bus_req_o.inv_req = m2_q.llsc ? WR_ALLOC : RD_ALLOC; // 对于 ll 指令，需要申请写权限
            if(bus_resp_i.ready) begin
                fsm = S_NORMAL;
                mod = M_HANDLED;
                fsm_rdata = bus_resp_i.rdata[31:0];
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
        S_CUCLOAD: begin
            bus_req_o.valid = '1;
            bus_req_o.uncached_load_req = '1;
            bus_req_o.target_paddr = unc_paddr_q;
            c_lsu_resp_o.ready = bus_resp_i.ready;
            if(bus_resp_i.ready) begin
                fsm = S_NORMAL;
            end
        end
        S_CUCSTRD: begin
            bus_req_o.valid = '1;
            bus_req_o.uncached_store_req = '1;
            bus_req_o.target_paddr = unc_paddr_q;
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

endmodule
