// dcache bus side
`include "wired0_defines"

// include tilelink header
`include "tl_util.svh"

module wired_tl_adapter import tl_pkg::*; #(
    parameter int unsigned SourceWidth = 1,
    parameter int unsigned SinkWidth   = 1,
    parameter bit [SourceWidth-1:0] SourceBase  = 0,
)(
    `_WIRED_GENERAL_DEFINE,
    // 到 CPU 侧的请求接口
    input  lsu_bus_req_t  bus_req_i,
    output lsu_bus_resp_t bus_resp_o,

    // 用于 SNOOP 的总线更新接口
    output dsram_snoop_t  snoop_i,

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

    `TL_DECLARE_HOST_PORT(DataWidth, PhysAddrLen, SourceWidth, SinkWidth, tl) // tl_a_o
);

    /* --- --- --- --- --- --- ---  I-FSM-CALL Begin  --- --- --- --- --- --- --- --- */
    // Inter-FSM Called signals
    // - prb begin
    // --- prb call inv
    logic prb_inv_cal; // prb drive
    logic prb_inv_ret; // inv drive
    typedef enum logic[2:0] {
        PRB_HIT_ADDR_N, // for prb inv toN
        PRB_HIT_ADDR_B, // for prb inv toB
        RD_ALLOC,       // for read inv, 找到一个合适的行释放并返回
        WR_ALLOC,       // for write miss, check whether read hit, if hit, just return.
                        // otherwise, invalidate random way and return.
        IDX_INV,        // INDEX INVALID WRITE BACK
        IDX_INIT        // INDEXED INIT
    } inv_parm_e;
    // payload
    inv_parm_e   prb_inv_parm; // prb drive
    logic [31:0] prb_inv_addr; // prb drive
    // - crq begin
    // --- crq call inv
    logic        crq_inv_cal; // crq drive
    logic        crq_inv_ret; // inv drive
    logic [1:0]  crq_inv_sel; // inv drive
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
    logic  [3:0] crq_unc_strb; // crq drive
    logic  [1:0] crq_unc_size; // crq drive
    logic [31:0] crq_unc_addr; // crq drive
    logic [31:0] crq_unc_data; // unc drive

    /* --- --- --- --- --- --- --- --- FSM Defines Begin  --- --- --- --- --- --- --- --- */

    // 有意思的是，只有 A 需要仲裁，C、E 均为独享。
    typedef `TL_A_STRUCT(128, 32, SourceWidth, SinkWidth) tl_a_t;
    typedef `TL_B_STRUCT(128, 32, SourceWidth, SinkWidth) tl_b_t;
    typedef `TL_C_STRUCT(128, 32, SourceWidth, SinkWidth) tl_c_t;
    typedef `TL_D_STRUCT(128, 32, SourceWidth, SinkWidth) tl_d_t;
    typedef `TL_E_STRUCT(128, 32, SourceWidth, SinkWidth) tl_e_t;

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
    logic [3:0][31:0] inv_data_rdata;

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
        /* PROBE 状态机，状态定义 */
        typedef logic[1:0] fsm_t;
        fsm_t fsm;
        fsm_t fsm_q;
        localparam fsm_t S_FREE = 0;
        localparam fsm_t S_INV = 1; // Call inv
        typedef struct packed {
            logic [31:0] addr;
            logic [2:0] parm;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys d;
        registerd_entrys q;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                d <= init;
                fsm_q <= S_FREE;
            end else begin
                d <= q;
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
                        fsm = S_INV;
                        d.addr = prb_b.address;
                        d.parm = prb_b.parm;
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
        /* PROBE 状态机，状态定义 */
        typedef logic[2:0] fsm_t;
        fsm_t fsm;
        fsm_t fsm_q;
        localparam fsm_t S_FREE = 0;
        localparam fsm_t S_ULD = 1; // Call unc
        localparam fsm_t S_UST = 2; // Call unc
        localparam fsm_t S_COP = 3; // Call inv
        localparam fsm_t S_INV = 4; // Call inv
        localparam fsm_t S_ACQ = 5; // Call acq
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                fsm_q <= S_FREE;
            end else begin
                fsm_q <= fsm;
            end
        end
        typedef struct packed {
            logic     sel_prb; // 响应来自 prb 的请求，否之则是 crq
            logic [31:0] addr;
        } registerd_entrys;
        always_comb begin
            
        end
    end

    // Invalid     状态机 - inv - C、D - read/write SRAM-TAG, read SRAM-DATA
    if(1) begin : inv_fsm
        /* PROBE 状态机，状态定义 */
        typedef logic[2:0] fsm_t;
        fsm_t fsm;
        fsm_t fsm_q;
        localparam fsm_t S_FREE = 0;
        localparam fsm_t S_RTAG0 = 1; // Read tags request
        localparam fsm_t S_RTAG1 = 2; // Read tags fetch result
        localparam fsm_t S_MTAG = 3; // Match tags
        localparam fsm_t S_WTAG = 4; // Write tag sram
        localparam fsm_t S_RDAT = 5; // Read data sram
        localparam fsm_t S_TLC  = 6; // TL-C
        localparam fsm_t S_TLD  = 7; // TL-D
        typedef struct packed {
            logic     prb_sel; // 选择来自 prb 的请求
            logic [31:0] addr;
            inv_parm_e  parm;
            cache_tag_t [3:0] tags;
            logic [3:0] mask;
            logic [1:0] perm;
        } registerd_entrys;
        registerd_entrys init = '0;
        registerd_entrys q;
        registerd_entrys d;
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
            prb_inv_ret = '0;
            crq_inv_ret = '0;
            crq_inv_sel = '0;
            inv_c_valid = '0;
            inv_c = '0;
            inv_data_valid = '0;
            inv_data_addr = '0;
            inv_data_way = '0;
            inv_tag_valid = '0;
            inv_tag_we = '0;
            inv_tag_set = '0;
            inv_tag = '0;
            case (fsm_q)
                /*S_FREE*/default: begin
                    if(prb_inv_ret) begin // 这个优先级比较高哦
                        d.prb_sel = '1;
                        d.addr = prb_inv_addr;
                        d.parm = prb_inv_parm;
                        if(d.parm inside {IDX_INV, IDX_INIT}) begin
                            d.mask = d.addr[15:12];
                        end else begin
                            d.mask = '0;
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
                    // RD_ALLOC 不需要 match，只需要找空行
                    if(q.parm inside {RD_ALLOC, IDX_INV, IDX_INIT}) begin
                        fsm = S_WTAG;
                    end else begin
                        fsm = S_MTAG;
                    end
                end
                S_MTAG: begin
                    for(integer i = 0 ; i < 4 ; i += 1) begin
                        if(q.tags[i].rp && q.tags[i].p == q.addr[31:12]) begin
                            d.mask[i] = '1; // hit logic 
                            d.perm = {q.tags[i].rp, q.tags[i].wp};
                            if(q.parm = WR_ALLOC) begin
                                fsm = S_FREE;
                                crq_inv_ret = '1;
                                crq_inv_sel = i[1:0];
                            end
                        end
                    end
                    if(!crq_inv_ret) begin
                        fsm = S_WTAG;
                    end
                end
                S_WTAG: begin
                    // WR_ALLOC | RD_ALLOC
                    // PRB_HIT_ADDR_N
                    // PRB_HIT_ADDR_B
                end
            endcase
        end
    end

    // Acquire     状态机 - acq - A、D、E - write SRAM-TAG, write SRAM-DATA
    if(1) begin : acq_fsm
        /* PROBE 状态机，状态定义 */
        typedef logic[2:0] fsm_t;
        fsm_t fsm;
        fsm_t fsm_q;
        localparam fsm_t S_FREE = 0;
        localparam fsm_t S_TLA  = 1; // Issue A request
        localparam fsm_t S_TLD  = 2; // Waiting for D response, and lock B
        localparam fsm_t S_WRAM = 3; // Write data&tag sram
        localparam fsm_t S_TLE  = 4; // Issue E response
        typedef struct packed {
            logic [31:0] addr;
            logic [2:0] parm;
        } registerd_entrys;
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
    assign sram_data_w_mult[1] = acq_data_wstrb;
    assign sram_data_w_mult[2] = '0;
    assign inv_data_rdata = m_rdata_i;
    assign sram_data_ready_mult[0] = '1;
    assign sram_data_ready_mult[1] = ~sram_data_valid_mult[0];
    assign sram_data_ready_mult[2] = ~sram_data_valid_mult[0] & ~sram_data_valid_mult[1];
    assign m_way_o   = sram_data_valid_mult[0] ? sram_data_way_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_way_mult[1] :
                                                sram_data_way_mult[2];
    assign m_addr_o  = sram_data_valid_mult[0] ? sram_data_addr_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_addr_mult[1] :
                                                sram_data_addr_mult[2];
    assign m_strb_o  = sram_data_valid_mult[0] ? sram_data_strb_mult[0] :
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

endmodule
