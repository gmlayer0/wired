// dcache bus side
`include "wired0_defines.svh"

// include tilelink header
`include "tl_util.svh"

module wired_tl_adapter import tl_pkg::*; #(
    parameter int unsigned SOURCE_WIDTH  = 1,
    parameter int unsigned SINK_WIDTH    = 1,
    parameter bit [SOURCE_WIDTH-1:0] SOURCE_BASE  = 0
)(
    `_WIRED_GENERAL_DEFINE,
    // 到 CPU 侧的请求接口
    input  lsu_bus_req_t  bus_req_i,
    output lsu_bus_resp_t bus_resp_o,

    // 用于 SNOOP 的总线更新接口
    output dsram_snoop_t  snoop_o,

    // DSRAM 端口
    output  logic [1:0]  m_way_o,
    output  logic [11:0] m_addr_o,       // 状态机侧访问地址，读写某 way 的整行
    output  logic [3:0][3:0] m_wstrb_o,  // 写掩码，全 0 为读
    output  logic [3:0][31:0] m_wdata_o,
    input   logic [3:0][31:0] m_rdata_i,   // 整行，已内部对齐（注：组合逻辑对齐）

    // TSRAM 端口
    output  logic [11:4] t_addr_o,
    output  logic  [3:0] t_we_o,
    output  cache_tag_t  t_wtag_o,
    input   cache_tag_t  [3:0] t_rtag_i,

    `TL_DECLARE_HOST_PORT(128, 32, SOURCE_WIDTH, SINK_WIDTH, tl) // tl_a_o
);
    assign snoop_o.daddr = m_addr_o;
    assign snoop_o.dway  = m_way_o;
    assign snoop_o.dstrb = m_wstrb_o;
    assign snoop_o.d     = m_wdata_o;
    assign snoop_o.taddr = {t_addr_o,4'd0};
    assign snoop_o.twe   = t_we_o;
    assign snoop_o.t     = t_wtag_o;

    /* --- --- --- --- --- --- ---  I-FSM-CALL Begin  --- --- --- --- --- --- --- --- */
    // Inter-FSM Called signals
    // - prb begin
    // --- prb call inv
    logic prb_inv_cal; // prb drive
    logic prb_inv_ret; // inv drive
    // payload
    inv_parm_e   prb_inv_parm; // prb drive
    logic [31:0] prb_inv_addr; // prb drive
    // - crq begin
    // --- crq call inv
    logic        crq_inv_cal; // crq drive
    logic        crq_inv_ret; // inv drive
    logic [2:0]  crq_inv_sel; // high indecate-write hit inv drive
    // payload
    inv_parm_e   crq_inv_parm; // crq drive
    logic [31:0] crq_inv_addr; // crq drive
    // --- crq call acq
    logic crq_acq_cal; // crq drive
    logic crq_acq_ret; // acq drive
    // payload
    logic             crq_acq_wp;   // crq drive, wether to get write permission
    logic [1:0]       crq_acq_way;  // crq drive
    logic [31:4]      crq_acq_addr; // crq drive
    logic [3:0][31:0] crq_acq_data; // acq drive
    // --- crq call unc
    logic crq_unc_cal; // crq drive
    logic crq_unc_ret; // unc drive
    // payload
    logic        crq_unc_wreq; // crq drive, write/read selection.
    logic  [3:0] crq_unc_histrb; // crq drive
    logic  [3:0] crq_unc_strb; // crq drive
    logic  [1:0] crq_unc_size; // crq drive
    logic [31:0] crq_unc_addr; // crq drive
    logic [31:0] crq_unc_wdata; // crq drive
    logic [3:0][31:0] crq_unc_data; // unc drive

    /* --- --- --- --- --- --- --- --- FSM Defines Begin  --- --- --- --- --- --- --- --- */

    // 有意思的是，只有 A 需要仲裁，C、E 均为独享。
    typedef `TL_A_STRUCT(128, 32, SOURCE_WIDTH, SINK_WIDTH) tl_a_t;
    typedef `TL_B_STRUCT(128, 32, SOURCE_WIDTH, SINK_WIDTH) tl_b_t;
    typedef `TL_C_STRUCT(128, 32, SOURCE_WIDTH, SINK_WIDTH) tl_c_t;
    typedef `TL_D_STRUCT(128, 32, SOURCE_WIDTH, SINK_WIDTH) tl_d_t;
    typedef `TL_E_STRUCT(128, 32, SOURCE_WIDTH, SINK_WIDTH) tl_e_t;

    // Probe       状态机 - prb - B - read SRAM-TAG
    /* - tl - */
    logic prb_b_valid, prb_b_ready;
    tl_b_t prb_b;
    /* - sram - */
    // logic prb_tag_valid, prb_tag_ready;
    // logic [11:4] prb_tag_set;
    // cache_tag_t [3:0] prb_tag;

    // CPU_Request 状态机 - crq - - write SRAM-DATA
    /* - data sram - */
    logic             crq_data_valid, crq_data_ready;
    logic  [1:0]      crq_data_way;
    logic [11:0]      crq_data_addr;
    logic [3:0][3:0]  crq_data_wstrb;
    logic [3:0][31:0] crq_data_wdata;

    // Invalid     状态机 - inv - C、D - read/write SRAM-TAG, read SRAM-DATA
    /* - tl - */
    logic inv_c_valid, inv_c_ready;
    tl_c_t inv_c;
    logic inv_d_valid, inv_d_ready;
    tl_d_t inv_d;
    /* - tag sram - */
    logic inv_tag_valid, inv_tag_ready;
    logic [11:4] inv_tag_set;
    logic  [3:0] inv_tag_we;
    cache_tag_t  inv_tag;
    cache_tag_t [3:0] inv_rtag;
    /* - data sram - */
    logic        inv_data_valid, inv_data_ready;
    logic  [1:0] inv_data_way;
    logic [11:0] inv_data_addr;
    logic [3:0][31:0] inv_data;

    // Acquire     状态机 - acq - A、D、E - write SRAM-TAG, write SRAM-DATA
    /* - tl - */
    logic acq_a_valid, acq_a_ready;
    tl_a_t acq_a;
    logic acq_d_valid, acq_d_ready;
    tl_d_t acq_d;
    logic acq_e_valid, acq_e_ready;
    tl_e_t acq_e;
    /* - tag sram - */
    logic acq_tag_valid, acq_tag_ready;
    logic [11:4] acq_tag_set;
    logic  [3:0] acq_tag_we;
    cache_tag_t  acq_tag;
    /* - data sram - */
    logic             acq_data_valid, acq_data_ready;
    logic  [1:0]      acq_data_way;
    logic [11:0]      acq_data_addr;
    logic [3:0][3:0]  acq_data_wstrb;
    logic [3:0][31:0] acq_data_wdata;

    // Uncached    状态机 - unc - A、D
    logic unc_a_valid, unc_a_ready;
    tl_a_t unc_a;
    logic unc_d_valid, unc_d_ready;
    tl_d_t unc_d;

    /*
    --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
                      ##   ##    ####   ### ##   ### ###  ### ##   
                      ##   ##     ##     ##  ##   ##  ##   ##  ##  
                      ##   ##     ##     ##  ##   ##       ##  ##  
                      ## # ##     ##     ## ##    ## ##    ##  ##  
                      # ### #     ##     ## ##    ##       ##  ##  
                       ## ##      ##     ##  ##   ##  ##   ##  ##  
                      ##   ##    ####   #### ##  ### ###  ### ##   
                                                                   
    #### ##    ####   ####     ### ###           ####       ####   ###  ##  ##  ###  
    # ## ##     ##     ##       ##  ##            ##         ##      ## ##  ##  ##   
      ##        ##     ##       ##                ##         ##     # ## #  ## ##    
      ##        ##     ##       ## ##    #####    ##         ##     ## ##   ## ##    
      ##        ##     ##       ##                ##         ##     ##  ##  ## ###   
      ##        ##     ##  ##   ##  ##            ##  ##     ##     ##  ##  ##  ##   
     ####      ####   ### ###  ### ###           ### ###    ####   ###  ##  ##  ###  
                                                                                     
                               ### ###   ## ##   ##   ##  
                                ##  ##  ##   ##   ## ##   
                                ##      ####     # ### #  
                                ## ##    #####   ## # ##  
                                ##          ###  ##   ##  
                                ##      ##   ##  ##   ##  
                               ####      ## ##   ##   ##  
    --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
    */
    // Probe       状态机 - prb - B - read SRAM-TAG
    if(1) begin : prb_fsm
        /* 资源监视器，监视 acq 状态 */
        logic [31:4] last_fetch_addr_q;
        logic [5:0] last_fetch_cnt_q;
        always_ff @(posedge clk) begin
            if(crq_acq_ret) begin
                last_fetch_cnt_q  <= 6'd32; // 保护 32 个周期
                last_fetch_addr_q <= crq_acq_addr;
            end
            else if(last_fetch_cnt_q) begin
                last_fetch_cnt_q <= last_fetch_cnt_q - 1;
            end
        end
        /* PROBE 状态机，状态定义 */
        typedef enum logic[1:0] {
            S_FREE,
            S_PROT,
            S_INV // Call inv
        } fsm_e;
        fsm_e fsm;
        fsm_e fsm_q;
        typedef struct packed {
            logic [31:0] addr;
            logic [2:0] parm;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys d;
        registerd_entrys q;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                q <= init;
                fsm_q <= S_FREE;
            end else begin
                q <= d;
                fsm_q <= fsm;
            end
        end
        always_comb begin
            // ALL DRIVEN SIGNAL
            d = q;
            fsm = fsm_q;
            prb_b_ready = '0;
            prb_inv_parm = PRB_HIT_ADDR_N;
            prb_inv_addr = q.addr;
            prb_inv_cal = '0;
            case (fsm_q)
                /*S_FREE*/default: begin
                    prb_b_ready = '1;
                    if(prb_b_valid) begin
                        // TODO: ADD RESOURCE PROTECTION, BUT NOT TODAY
                        // 4.8: I encounter live lock problem, so I add a resource protection here
                        // fsm = S_INV;
                        fsm = S_PROT;
                        d.addr = prb_b.address;
                        d.parm = prb_b.param;
                    end
                end
                S_PROT: begin
                    // 检查需要 Probe 的 Block 是否是最近 64 个周期内 Acquire 过来的，若是，则等它 64 个周期再继续 prb
                    if(last_fetch_addr_q != q.addr[31:4] || last_fetch_cnt_q == '0) begin
                        fsm = S_INV;
                    end
                end
                S_INV: begin
                    prb_inv_parm = q.parm == tl_pkg::toB ? PRB_HIT_ADDR_B : PRB_HIT_ADDR_N;
                    prb_inv_addr = q.addr;
                    prb_inv_cal = '1;
                    if(prb_inv_ret) begin
                        fsm = S_FREE;
                    end
                end
            endcase
        end
    end

    // CPU_Request 状态机 - crq - - write SRAM-DATA
    if(1) begin : crq_fsm
        /* CPU Request 状态机，状态定义 */
        typedef enum logic[2:0] {
            S_FREE,
            S_ULD, // Call unc
            S_UST, // Call unc
            S_INV, // Call inv
            S_ACQ, // Call acq
            S_RET  // Return value
        } fsm_e;
        fsm_e fsm;
        fsm_e fsm_q;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                fsm_q <= S_FREE;
            end else begin
                fsm_q <= fsm;
            end
        end
        typedef struct packed {
            logic  [1:0] size;
            logic [31:0] addr; // 目标地址
            logic [63:0] data;
            logic  [2:0] inv_sel;
            inv_parm_e   parm;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys q;
        registerd_entrys d;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                q <= init;
                fsm_q <= S_FREE;
            end else begin
                q <= d;
                fsm_q <= fsm;
            end
        end
        always_comb begin
            fsm = fsm_q;
            d   = q;
            // inv call
            crq_inv_cal  = '0;
            crq_inv_parm = q.parm;
            crq_inv_addr = q.addr;
            // acq call
            crq_acq_cal  = '0;
            crq_acq_wp   = q.parm == WR_ALLOC;
            crq_acq_way  = q.inv_sel;
            crq_acq_addr = q.addr[31:4];
            // unc call
            crq_unc_cal   = '0; // crq drive
            crq_unc_wreq  = '0;
            crq_unc_wdata = q.data;
            crq_unc_histrb = (&q.size) ? 4'b1111 : 4'b0000;
            crq_unc_strb = q.size[1] ? 4'b1111 : (q.size[0] ? 
                                    (q.addr[1] ? 4'b1100 : 4'b0011) : 
                                    (q.addr[1] ? (q.addr[0] ? 4'b1000 : 4'b0100) : (q.addr[0] ? 4'b0010 : 4'b0001)));
            crq_unc_size = q.size;
            crq_unc_addr = q.addr;
            // cpu responded
            bus_resp_o   = '0;
            bus_resp_o.rdata = q.data;
            // cpu writeback
            crq_data_valid = '0;
            crq_data_way = bus_req_i.way;
            crq_data_addr = {bus_req_i.sram_addr[11:4], 4'd0};
            crq_data_wstrb = '0;
            crq_data_wdata = '0;
            crq_data_wdata[bus_req_i.sram_addr[3:2]] = bus_req_i.wdata;
            if(bus_req_i.sram_wb_req) begin // 最最最高优先级，一定保证
                crq_data_valid = '1;
                for(integer i = 0 ; i < 4 ; i += 1) crq_data_wstrb[bus_req_i.sram_addr[3:2]][i[1:0]] = bus_req_i.wstrobe[i];
            end
            case (fsm_q)
            /*S_FREE*/default:begin
                if(bus_req_i.valid) begin
                    if(bus_req_i.uncached_load_req) begin
                            fsm = S_ULD;
                            d.size = bus_req_i.size;
                            d.addr = bus_req_i.target_paddr;
                    end
                    if(bus_req_i.uncached_store_req) begin
                            fsm = S_UST;
                            d.size = bus_req_i.size;
                            d.addr = bus_req_i.target_paddr;
                            d.data[31:0] = bus_req_i.wdata;
                    end
                    if(bus_req_i.inv_req != NOT_VALID_INV_PARM) begin
                            fsm = S_INV;
                            d.size = bus_req_i.size;
                            d.addr = bus_req_i.target_paddr;
                            d.parm = bus_req_i.inv_req;
                    end
                end
            end
            S_ULD: begin
                crq_unc_cal = '1;
                d.data = {crq_unc_data[{q.addr[3], 1'd1}], crq_unc_data[q.addr[3:2]]};
                if(crq_unc_ret) begin
                    fsm = S_RET;
                end
            end
            S_UST: begin
                crq_unc_cal = '1;
                crq_unc_wreq = '1;
                if(crq_unc_ret) begin
                    bus_resp_o.ready = '1;
                    fsm = S_FREE;
                end
            end
            S_INV: begin
                crq_inv_cal = '1;
                d.inv_sel = crq_inv_sel;
                if(crq_inv_ret) begin
                    if(q.parm inside {WR_ALLOC, RD_ALLOC}) begin
                        fsm = S_ACQ;
                    end else begin
                        bus_resp_o.ready = '1;
                        fsm = S_FREE;
                    end
                end
            end
            S_ACQ: begin
                crq_acq_cal = '1;
                d.data = {crq_acq_data[{q.addr[3], 1'd1}], crq_acq_data[q.addr[3:2]]};
                if(crq_acq_ret) begin
                    fsm = S_RET;
                end
            end
            S_RET: begin
                bus_resp_o.ready = '1;
                fsm = S_FREE;
            end
            endcase
        end
    end

    // Invalid     状态机 - inv - C、D - read/write SRAM-TAG, read SRAM-DATA
    if(1) begin : inv_fsm
        /* Invalid 状态机，状态定义 */
        logic [7:0] rnd_value_q;
        always_ff @(posedge clk) begin
            if(~rst_n) begin
                rnd_value_q <= 8'h24;
            end else begin
                rnd_value_q <= {rnd_value_q[5:0], ~{rnd_value_q[7] ^ rnd_value_q[5] ^ rnd_value_q[4] ^ rnd_value_q[3]},
                                              ~{rnd_value_q[6] ^ rnd_value_q[4] ^ rnd_value_q[3] ^ rnd_value_q[2]}};
            end
        end
        wire [1:0] rnd_sel = rnd_value_q[7:6];
        wire [3:0] rnd_sel_oh = {rnd_sel == 2'd3, rnd_sel == 2'd2, rnd_sel == 2'd1, rnd_sel == 2'd0};
        typedef enum logic[3:0] {
            S_FREE  ,
            S_RTAG0 , // Read tags request
            S_RTAG1 , // Read tags fetch result
            S_ASEL  , // Alloc select
            S_HSEL  , // Hit select
            S_WTAG  , // Write tag sram
            S_SEL   , // 从 MASK 中拿到第一个请求，并无效化对应 MASK
            S_RDAT0 , // Read data request
            S_RDAT1 , // Read data fetch result
            S_TLC   , // TL-C
            S_TLD     // TL-D
        } fsm_e;
        fsm_e fsm;
        fsm_e fsm_q;
        typedef struct packed {
            logic     prb_sel; // 选择来自 prb 的请求
            logic [31:0] addr;
            logic [31:0] taddr;
            inv_parm_e  parm;
            cache_tag_t [3:0] tags;
            logic [3:0] mask;
            logic [1:0] perm;
            logic [3:0][31:0] data;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys q;
        registerd_entrys d;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                q <= init;
                fsm_q <= S_FREE;
            end else begin
                q <= d;
                fsm_q <= fsm;
            end
        end
        always_comb begin
            fsm = fsm_q;
            d = q;
            prb_inv_ret = '0;
            crq_inv_ret = '0;
            crq_inv_sel = '0;
            inv_c_valid = '0;
            inv_c = '0;
            inv_c.opcode = tl_pkg::Release;
            inv_c.source = SOURCE_BASE;
            inv_c.size = 4; // 16Bytes == 128bits == 2^4Bytes
            inv_c.address = {q.taddr[31:4], 4'd0};
            inv_c.data = q.data;
            inv_d_ready = '0;
            inv_data_valid = '0;
            inv_data_addr = {q.taddr[11:4], 4'd0};
            inv_data_way = q.taddr[1:0];
            inv_tag_valid = '0;
            inv_tag_we = '0;
            inv_tag_set = q.addr[11:4];
            inv_tag = '0;
            case (fsm_q)
                /*S_FREE*/default: begin
                    if(prb_inv_cal) begin // 这个优先级比较高哦
                        fsm = S_RTAG0;
                        d.prb_sel = '1;
                        d.taddr = prb_inv_addr;
                        d.addr = prb_inv_addr;
                        d.parm = prb_inv_parm;
                        d.mask = '0;
                    end else if(crq_inv_cal) begin
                        fsm = S_RTAG0;
                        d.prb_sel = '0;
                        d.addr = crq_inv_addr;
                        d.parm = crq_inv_parm;
                        // d.mask = d.addr[17:14]; // 直接索引形式
                        d.mask = '0;
                        if(d.parm inside {IDX_INV, IDX_INIT}) begin
                            d.mask[d.addr[13:12]] = '1;
                        end
                    end
                end
                S_RTAG0: begin
                    inv_tag_valid = '1;
                    inv_tag_set = q.addr[11:4];
                    if(inv_tag_ready) begin
                        fsm = S_RTAG1;
                    end
                end
                S_RTAG1: begin
                    d.tags = inv_rtag;
                    // 这两个不需要 match 生成 mask
                    if(q.parm inside {IDX_INV, IDX_INIT}) begin
                        fsm = S_WTAG;
                    end else begin
                        if(q.parm == RD_ALLOC) begin
                            fsm = S_ASEL;
                        end else begin
                            fsm = S_HSEL;
                        end
                    end
                end
                S_ASEL: begin
                    // 生成 分配项目的 mask
                    for(integer i = 0 ; i < 4 ; i += 1) begin
                        if(!q.tags[i].rp && (d.mask == '0)) begin
                            d.mask[i] = '1;
                            crq_inv_sel = {1'b0, i[1:0]};
                        end
                    end
                    if(d.mask == '0) begin
                        d.mask = rnd_sel_oh[3:0];
                        // 这里还是需要去释放下旧表项的
                        fsm = S_WTAG;
                    end else begin
                        // 不用再继续走了，返回空闲项目就行
                        // crq_inv_sel 已经填写
                        crq_inv_ret = '1;
                        fsm = S_FREE;
                    end
                end
                S_HSEL: begin // 以命中的方式选择表项
                    for(integer i = 0 ; i < 4 ; i += 1) begin
                        if(q.tags[i].rp && q.tags[i].p == q.addr[31:12]) begin
                            d.mask[i] = '1; // hit logic 
                            crq_inv_sel = {1'b1, i[1:0]};
                        end
                    end
                    if((d.mask == '0)) begin
                        if(q.parm inside {PRB_HIT_ADDR_N, PRB_HIT_ADDR_B}) begin
                            // 没有命中项目，对于 probe 请求，直接返回 C 即可 NtoN
                            d.perm = '0;
                            fsm = S_TLC;
                        end else begin
                            // 对于写请求，继续进行 ASEL 工作
                            fsm = S_ASEL;
                        end
                    end else begin
                        // 存在命中项目，即至少存在 rp == '1
                        if(q.parm == WR_ALLOC) begin
                            // 直接返回
                            // crq_inv_sel 已经填写
                            crq_inv_ret = '1;
                            fsm = S_FREE;
                        end else begin
                            fsm = S_WTAG;
                        end
                    end
                end
                S_WTAG: begin
                    // 所有 mask 的 TAG 都需要在这里被无效化
                    inv_tag_valid = '1;
                    inv_tag_we = q.mask; // 对于 match 不可能 multi hit，但对于 index 有可能。
                    inv_tag_set = q.addr[11:4];
                    inv_tag = '0;
                    inv_tag.p = q.addr[31:12]; // Prb hit B still have write permission.
                    inv_tag.rp = q.parm == PRB_HIT_ADDR_B ? '1 : '0;
                    if(inv_tag_ready) begin
                        if(q.parm == IDX_INIT) begin
                            crq_inv_ret = '1;
                            fsm = S_FREE;
                        end else begin
                            // 继续前进！
                            fsm = S_SEL;
                        end
                    end
                end
                S_SEL: begin
                    for(integer i = 3 ; i >= 0 ; i -= 1) begin
                        if(q.mask[i]) begin
                            d.perm  = {q.tags[i].wp, q.tags[i].rp};
                            d.taddr = {q.tags[i].p, q.addr[11:4],2'd0,i[1:0]};
                        end
                    end
                    d.mask[d.taddr[1:0]] = '0;
                    if(d.perm[1]) fsm = S_RDAT0;
                    else fsm = S_TLC;
                end
                S_RDAT0: begin
                    inv_data_valid = '1;
                    inv_data_addr = {q.taddr[11:4], 4'd0};
                    inv_data_way = q.taddr[1:0];
                    if(inv_data_ready) fsm = S_RDAT1;
                end
                S_RDAT1: begin
                    d.data = inv_data;
                    fsm = S_TLC;
                end
                S_TLC: begin
                    // form tlc request
                    inv_c_valid = '1;
                    inv_c.opcode = q.parm inside {PRB_HIT_ADDR_B, PRB_HIT_ADDR_N} ? (
                        q.perm[1] ? tl_pkg::ProbeAckData : tl_pkg::ProbeAck
                    ) : (
                        q.perm[1] ? tl_pkg::ReleaseData : tl_pkg::Release
                    );
                    inv_c.param = q.parm == PRB_HIT_ADDR_B ? (
                        q.perm[1] ? tl_pkg::TtoB : (q.perm[0] ? tl_pkg::BtoB : tl_pkg::NtoN)
                    ) : (
                        q.perm[1] ? tl_pkg::TtoN : (q.perm[0] ? tl_pkg::BtoN : tl_pkg::NtoN)
                    );
                    if(inv_c_ready) begin
                        // Probe 到此结束，否之等待 response
                        if(q.parm inside {PRB_HIT_ADDR_B, PRB_HIT_ADDR_N}) begin
                            prb_inv_ret = '1;
                            fsm = S_FREE;
                        end else begin
                            fsm = S_TLD;
                        end
                    end
                end
                S_TLD: begin
                    // 等待
                    if(inv_d_valid && inv_d.opcode == tl_pkg::ReleaseAck) begin
                        inv_d_ready = '1;
                        if(q.mask != 0) begin
                            fsm = S_SEL;
                        end
                        else begin
                            crq_inv_sel = {1'b0, q.taddr[1:0]};
                            crq_inv_ret = '1;
                            fsm = S_FREE;
                        end
                    end
                end
            endcase
        end
    end

    // Acquire     状态机 - acq - A、D、E - write SRAM-TAG, write SRAM-DATA
    if(1) begin : acq_fsm
        /* Acquire 状态机，状态定义 */
        typedef enum logic[2:0] {
            S_FREE ,
            S_TLA  , // Issue A request
            S_TLD  , // Waiting for D response, and lock B
            S_WRAM , // Write data&tag sram
            S_TLE    // Issue E response
        } fsm_e;
        fsm_e fsm;
        fsm_e fsm_q;
        typedef struct packed {
            logic [3:0][31:0]      data;
            logic [31:0]           addr;
            logic [1:0]             way;
            logic [SINK_WIDTH-1:0] sink;
            logic wp;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys q;
        registerd_entrys d;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                fsm_q <= S_FREE;
            end else begin
                fsm_q <= fsm;
            end
        end
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                q <= init;
            end else begin
                q <= d;
            end
        end
        always_comb begin
            fsm = fsm_q;
            d = q;
            crq_acq_ret = '0;
            crq_acq_data = q.data;
            acq_a_valid = '0;
            acq_a = '0;
            acq_a.opcode  = tl_pkg::AcquireBlock;
            acq_a.param   = tl_pkg::NtoB;
            acq_a.size    = 4;
            acq_a.address = {q.addr[31:4], 4'd0};
            acq_a.source  = SOURCE_BASE;
            acq_a.mask    = '1;
            acq_a.corrupt = '0;

            acq_d_ready = '0;

            acq_e_valid = '0;
            acq_e = '0;
            acq_e.sink    = q.sink;

            // SRAM 控制信号
            acq_tag_valid = '0;
            acq_tag_set   = q.addr[11:4];
            acq_tag_we    = '0;
            acq_tag       = '0;
            acq_tag.p     = q.addr[31:12];
            acq_data_valid = '0;
            acq_data_way   = q.way;
            acq_data_addr  = q.addr[11:0];
            acq_data_wstrb = '0;
            acq_data_wdata = q.data;
            case (fsm_q)
            default/*S_FREE*/: begin
                if(crq_acq_cal) begin
                    d.addr = {crq_acq_addr, 4'd0};
                    d.way  = crq_acq_way;
                    d.wp   = crq_acq_wp;
                    fsm    = S_TLA;
                end
            end 
            S_TLA: begin
                acq_a_valid = '1;
                if(q.wp) begin
                    acq_a.param = tl_pkg::NtoT;
                end
                if(acq_a_ready) begin
                    fsm = S_TLD;
                end
            end
            S_TLD: begin
                d.data = acq_d.data;
                acq_d_ready = '1;
                d.sink = acq_d.sink; // 记录 sink 给 E 通道使用
                if(acq_d_valid && acq_d.opcode == tl_pkg::GrantData) begin
                    fsm = S_WRAM;
                end
            end
            S_WRAM: begin
                acq_tag_valid = '1; // 这里只是为了锁住 inv，不允许其提前进入 RTAG 状态
                acq_tag_we[q.way] = '1;
                acq_data_valid = '1;
                acq_data_wstrb = '1;
                if(acq_data_ready) begin
                    fsm = S_TLE;
                end
            end
            S_TLE: begin
                acq_tag_valid = '1;
                acq_tag_we[q.way] = '1;
                acq_e_valid = '1;
                // Keep writing to tag sram to block inv from continue.
                if(acq_e_ready) begin
                    fsm = S_FREE;
                    acq_tag.wp = q.wp;
                    acq_tag.rp = '1; // 这里才更新状态
                    crq_acq_ret = '1;
                end
            end
            endcase
        end
    end

    // Uncached 状态机
    if(1) begin : unc_fsm
        /* PROBE 状态机，状态定义 */
        typedef enum logic[1:0] {
            S_FREE ,
            S_TLA  , // Call Get / PutFullData
            S_WTLD , // Wait AccessAck / AccessAckData
            S_RTLD   // Wait AccessAck / AccessAckData
        } fsm_e;
        fsm_e fsm;
        fsm_e fsm_q;
        typedef struct packed {
            logic [31:0] addr;
            logic        wreq;
            logic      [1:0]   size;
            logic [3:0][3:0]   strb;
            logic [3:0][31:0]  data;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys d;
        registerd_entrys q;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                q <= init;
                fsm_q <= S_FREE;
            end else begin
                q <= d;
                fsm_q <= fsm;
            end
        end
        always_comb begin
            // ALL DRIVEN SIGNAL
            d = q;
            fsm = fsm_q;
            unc_a_valid = '0;
            unc_a = '0;
            unc_a.opcode  = q.wreq ? tl_pkg::PutFullData : tl_pkg::Get;
            unc_a.param   = '0;
            unc_a.size    = q.size;
            unc_a.address = q.addr[31:0];
            unc_a.source  = SOURCE_BASE;
            unc_a.mask    = q.strb;
            unc_a.corrupt = '0;
            unc_a.data    = q.data;
            unc_d_ready = '0;
            crq_unc_data = unc_d.data;
            crq_unc_ret = '0;
            case (fsm_q)
                /*S_FREE*/default: begin
                    if(crq_unc_cal) begin
                        fsm = S_TLA;
                        d.addr = crq_unc_addr;
                        d.wreq = crq_unc_wreq;
                        d.size = crq_unc_size;
                        d.strb = '0;
                        d.strb[{crq_unc_addr[3],1'd1}] = crq_unc_histrb;
                        d.strb[crq_unc_addr[3:2]]      = crq_unc_strb;
                        d.data = '0;
                        d.data[crq_unc_addr[3:2]]      = crq_unc_wdata;
                    end
                end
                S_TLA: begin
                    unc_a_valid = '1;
                    if(unc_a_ready) begin
                        fsm = q.wreq ? S_WTLD : S_RTLD;
                    end
                end
                S_WTLD: begin
                    if(unc_d_valid && unc_d.opcode == tl_pkg::AccessAck) begin
                        unc_d_ready = '1;
                        crq_unc_ret = '1;
                        fsm = S_FREE;
                    end
                end
                S_RTLD: begin
                    if(unc_d_valid && unc_d.opcode == tl_pkg::AccessAckData) begin
                        unc_d_ready = '1;
                        crq_unc_ret = '1;
                        fsm = S_FREE;
                    end
                end
            endcase
        end
    end


    // SRAM 仲裁器，固定优先级
    // - tag 仲裁器，两写一读 - prb / inv acq -
    logic [2:0] sram_tag_valid_mult, sram_tag_ready_mult;
    logic [2:0][11:4] sram_tag_addr_mult;
    logic [2:0][3:0]  sram_tag_we_mult;
    cache_tag_t [2:0] sram_tag_w_mult;
    assign sram_tag_valid_mult[0] = /*prb_tag_valid*/ '0;
    assign sram_tag_valid_mult[1] = acq_tag_valid;
    assign sram_tag_valid_mult[2] = inv_tag_valid;
    assign sram_tag_addr_mult[0] = /*prb_tag_set*/ '0;
    assign sram_tag_addr_mult[1] = acq_tag_set;
    assign sram_tag_addr_mult[2] = inv_tag_set;
    // assign prb_tag_ready = sram_tag_ready_mult[0];
    assign acq_tag_ready = sram_tag_ready_mult[1];
    assign inv_tag_ready = sram_tag_ready_mult[2];
    assign sram_tag_we_mult[0] = '0;
    assign sram_tag_we_mult[1] = acq_tag_we;
    assign sram_tag_we_mult[2] = inv_tag_we;
    assign sram_tag_w_mult[0] = '0;
    assign sram_tag_w_mult[1] = acq_tag;
    assign sram_tag_w_mult[2] = inv_tag;
    // assign prb_tag = t_rtag_i;
    assign inv_rtag = t_rtag_i;
    assign sram_tag_ready_mult[0] = '1;
    assign sram_tag_ready_mult[1] = ~sram_tag_valid_mult[0];
    assign sram_tag_ready_mult[2] = ~sram_tag_valid_mult[0] & ~sram_tag_valid_mult[1];
    assign t_addr_o = sram_tag_valid_mult[0] ? sram_tag_addr_mult[0] : 
                      sram_tag_valid_mult[1] ? sram_tag_addr_mult[1] :
                                               sram_tag_addr_mult[2];
    assign t_we_o   = sram_tag_valid_mult[0] ? sram_tag_we_mult[0] : 
                      sram_tag_valid_mult[1] ? sram_tag_we_mult[1] :
                                               sram_tag_we_mult[2];
    assign t_wtag_o = sram_tag_valid_mult[0] ? sram_tag_w_mult[0] : 
                      sram_tag_valid_mult[1] ? sram_tag_w_mult[1] :
                                               sram_tag_w_mult[2];
    // - data 仲裁器，两写一读 - inv / crq acq -
    logic [2:0] sram_data_valid_mult, sram_data_ready_mult;
    logic [2:0][1:0]  sram_data_way_mult;
    logic [2:0][11:0] sram_data_addr_mult;
    logic [2:0][3:0][3:0]  sram_data_strb_mult;
    logic [2:0][3:0][31:0] sram_data_w_mult;
    assign sram_data_valid_mult[0] = crq_data_valid;
    assign sram_data_valid_mult[1] = acq_data_valid;
    assign sram_data_valid_mult[2] = inv_data_valid;
    assign sram_data_way_mult[0] = crq_data_way;
    assign sram_data_way_mult[1] = acq_data_way;
    assign sram_data_way_mult[2] = inv_data_way;
    assign sram_data_addr_mult[0] = crq_data_addr;
    assign sram_data_addr_mult[1] = acq_data_addr;
    assign sram_data_addr_mult[2] = inv_data_addr;
    assign crq_data_ready = sram_data_ready_mult[0];
    assign acq_data_ready = sram_data_ready_mult[1];
    assign inv_data_ready = sram_data_ready_mult[2];
    assign sram_data_strb_mult[0] = crq_data_wstrb;
    assign sram_data_strb_mult[1] = acq_data_wstrb;
    assign sram_data_strb_mult[2] = '0;
    assign sram_data_w_mult[0] = crq_data_wdata;
    assign sram_data_w_mult[1] = acq_data_wdata;
    assign sram_data_w_mult[2] = '0;
    assign inv_data = m_rdata_i;
    assign sram_data_ready_mult[0] = '1;
    assign sram_data_ready_mult[1] = ~sram_data_valid_mult[0];
    assign sram_data_ready_mult[2] = ~sram_data_valid_mult[0] & ~sram_data_valid_mult[1];
    assign m_way_o   = sram_data_valid_mult[0] ? sram_data_way_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_way_mult[1] :
                                                sram_data_way_mult[2];
    assign m_addr_o  = sram_data_valid_mult[0] ? sram_data_addr_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_addr_mult[1] :
                                                sram_data_addr_mult[2];
    assign m_wstrb_o = sram_data_valid_mult[0] ? sram_data_strb_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_strb_mult[1] :
                                                sram_data_strb_mult[2];
    assign m_wdata_o = sram_data_valid_mult[0] ? sram_data_w_mult[0] :
                       /*sram_data_valid_mult[1] ? */sram_data_w_mult[1]/* :
                                                sram_data_w_mult[2]*/;

    // TILELINK 仲裁器，固定优先级

    // A - acq unc
    tl_a_t [1:0] tl_a_mult;
    logic  [1:0] tl_a_mult_valid;
    logic  [1:0] tl_a_mult_ready;
    assign tl_a_mult[0] = acq_a;
    assign tl_a_mult_valid[0] = acq_a_valid;
    assign acq_a_ready = tl_a_mult_ready[0];
    assign tl_a_mult[1] = unc_a;
    assign tl_a_mult_valid[1] = unc_a_valid;
    assign unc_a_ready = tl_a_mult_ready[1];

    // 接出，固定优先级
    assign tl_a_valid_o = |tl_a_mult_valid;
    assign tl_a_o = tl_a_mult_valid[0] ? tl_a_mult[0] : tl_a_mult[1];
    assign tl_a_mult_ready[0] = tl_a_ready_i;
    assign tl_a_mult_ready[1] = tl_a_mult_valid[0] ? '0 : tl_a_ready_i;

    // D - inv acq unc
    assign inv_d = tl_d_i;
    assign acq_d = tl_d_i;
    assign unc_d = tl_d_i;
    assign inv_d_valid = tl_d_valid_i;
    assign acq_d_valid = tl_d_valid_i;
    assign unc_d_valid = tl_d_valid_i;
    assign tl_d_ready_o = inv_d_ready | acq_d_ready | unc_d_ready;

    // B - prb
    assign prb_b = tl_b_i;
    assign prb_b_valid = tl_b_valid_i;
    assign tl_b_ready_o = prb_b_ready;

    // C - inv
    assign tl_c_o = inv_c;
    assign tl_c_valid_o = inv_c_valid;
    assign inv_c_ready = tl_c_ready_i;

    // E - acq
    assign tl_e_o = acq_e;
    assign tl_e_valid_o = acq_e_valid;
    assign acq_e_ready = tl_e_ready_i;

    // dbg observe points
    tl_a_t dbg_a;
    tl_b_t dbg_b;
    tl_c_t dbg_c;
    tl_d_t dbg_d;
    tl_e_t dbg_e;
    assign dbg_a = tl_a_o;
    assign dbg_b = tl_b_i;
    assign dbg_c = tl_c_o;
    assign dbg_d = tl_d_i;
    assign dbg_e = tl_e_o;

    // debug 用
    parameter string CacheName = (SOURCE_BASE % 2) == 0 ? "DCache" : "ICache";
    parameter integer CoreID = SOURCE_BASE / 2;
    parameter string ColorTable[4] = {"\033[40;97m", "\033[41;97m", "\033[43;97m", "\033[44;97m"};
    parameter string ColorID = ColorTable[SOURCE_BASE];
    parameter logic[31:4] WATCH_ADDR = 28'h40000;
    always_ff @(posedge clk) begin
    //   if($time > 10000000 && bus_req_i.sram_addr[31:0] == 32'h3fff80 && bus_req_i.sram_wb_req &&
    //   bus_req_i.wstrobe[0] && (m_wdata_o[0][7:0] == 8'h06 || m_wdata_o[0][7:0] == 8'h00)) begin
    //     $display("[%t] TL Adapter catch the theif %x %x %x %x %x!!!", $time, sram_data_valid_mult, sram_data_ready_mult, m_wstrb_o, m_wdata_o[0], m_way_o);
    //   end
        // // 请求监视器
        // if((SOURCE_BASE % 2) == 0) begin
        // if(dbg_a.address[31:4] == WATCH_ADDR && tl_a_ready_i && tl_a_valid_o && dbg_a.opcode inside {tl_pkg::AcquireBlock, tl_pkg::Get, tl_pkg::PutFullData}) begin
        //     $display("%s↓↓[%-8t] Core%1d-%s TL A \033[1;97m%s @%x with mask %x.\033[0m", ColorID, $time, CoreID, CacheName, 
        //      dbg_a.opcode == tl_pkg::AcquireBlock       ? (dbg_a.param == tl_pkg::NtoB ? "Acq CRead " : "Acq CWrite") :
        //     (dbg_a.opcode == tl_pkg::Get                ? "Acq URead " : "Acq UWrite"),
        //     dbg_a.address, dbg_a.mask);
        //     if(dbg_a.opcode == tl_pkg::PutFullData) $display("%s||[--------] With Write Data %x.\033[0m",ColorID, dbg_a.data);
        // end
        // // 无效化监视器
        // if(inv_fsm.q.taddr[31:4] == WATCH_ADDR && inv_c_ready && inv_c_valid) begin
        //     $display("%s||[%-8t]--- Core%1d-%s TL C \033[1;33m%s inv@%x in way%d, %s.\033[0m", ColorID , $time, CoreID, CacheName, inv_c.opcode inside {tl_pkg::ProbeAckData,tl_pkg::ProbeAck} ? "probed" : "volunt",
        //     {inv_fsm.q.taddr[31:4],4'd0}, inv_fsm.q.taddr[1:0],
        //      inv_c.param == tl_pkg::TtoB ? "TtoB" : 
        //     (inv_c.param == tl_pkg::TtoN ? "TtoN" : 
        //     (inv_c.param == tl_pkg::BtoB ? "BtoB" : 
        //     (inv_c.param == tl_pkg::BtoN ? "BtoN" : "NtoN"))));
        //     if(inv_c.opcode inside {tl_pkg::ProbeAckData,tl_pkg::ReleaseData}) begin
        //         $display("%s||[--------]--- With data: %x\033[0m",ColorID, inv_c.data);
        //     end
        // end
        // // 重填监视器
        // if(acq_fsm.q.addr[31:4] == WATCH_ADDR && tl_d_ready_o && tl_d_valid_i && dbg_d.opcode inside {tl_pkg::GrantData, tl_pkg::Grant}) begin
        //     $display("%s↑↑[%-8t] Core%1d-%s TL D \033[1;32m%s @%x in way%d with data %x.\033[0m", ColorID, $time, CoreID, CacheName, 
        //     "Refill", {acq_fsm.q.addr[31:4],4'd0}, acq_fsm.q.way, dbg_d.data);
        //     if(acq_fsm.q.wp) $display("%s↑↑[--------] With Write Permission.\033[0m",ColorID);
        // end
        // if(unc_fsm.q.addr[31:4] == WATCH_ADDR && tl_d_ready_o && tl_d_valid_i && dbg_d.opcode inside {tl_pkg::AccessAck, tl_pkg::AccessAckData}) begin
        //     $display("%s↑↑[%-8t] Core%1d-%s TL D \033[1;97mUncached Respond.\033[0m", ColorID, $time, CoreID, CacheName);
        // end
        // end
    end
  

endmodule
