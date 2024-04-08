`include "wired0_defines.svh"

// Fuction module for Wired project
// Commit module, fetch all static and dynamic information about instruction execution.
// Futhermore, this module will change CSR / ARF / NPC status to commit instruction.
// This module can commit(retire) at most two inst / cycle.
// Because our CDB is 2-width, this is already enough.
module wired_commit #(
    parameter int ENABLE_DIFFTEST = 1,
    parameter int CPU_ID = 0
)(
    `_WIRED_GENERAL_DEFINE,

    // Part 1: ROB 读端口
    output rob_rid_t         [1:0] c_rrrid_o,
    input  logic             [1:0] c_rob_valid_i,
    input  rob_entry_t       [1:0] c_rob_entry_i,
    output logic             [1:0] c_retire_o,

    // Part 2: LSU 提交请求端口
    output commit_lsu_req_t        c_lsu_req_o,
    input  commit_lsu_resp_t       c_lsu_resp_i,

    // Part 3: ARF / Rename 更新端口
    output      logic        [1:0] l_retire_o, // FOR RENAME
    output      logic        [1:0] l_commit_o, // FOR ARF
    output     word_t        [1:0] l_data_o,
    output arch_rid_t        [1:0] l_warid_o,
    output  rob_rid_t        [1:0] l_wrrid_o,
    output      logic        [1:0] l_tier_id_o,

    // Part 4: 前端更新端口（跳转/flush/异常中断处理）
    output bpu_correct_t           f_upd_o,
    input       logic        [8:0] f_interrupt_i,  // 8 位硬件中断及核心间中断（最高位）

    // Part 5: flush 端口，刷新流水线用
    output      logic              l_flush_o,
    input       logic              rename_empty_i, // RENAME 级清空，说明 ROB 均已退休，ROB 空，可以继续。

    // Part 6: CSR 及地址翻译相关控制接口
    output      csr_t              csr_o,
    output tlb_update_req_t        tlb_update_req_o
    // 注意，提交模块分为三级，分别是 Fetch ROB(F) | Handle(H) | Commit(C)
);

    // 第一级流水，F，需要判定是否可双提交。
    // 对于可能需要状态机处理或者修改 CSR 、产生跳转、刷新流水线效果的指令，仅允许在 SLOT0 提交。
    // 其它指令则无所谓
    wire slot1_ctrl_conflict, slot1_bank_conflict;
    wire [1:0] f_valid = c_rob_valid_i & {((~slot1_ctrl_conflict) | l_flush_o) & ~slot1_bank_conflict & c_rob_valid_i[0], 1'd1};
    rob_rid_t f_rob_ptr0_q;
    rob_rid_t f_rob_ptr1_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            f_rob_ptr0_q <= `_WIRED_PARAM_ROB_LEN'd0;
            f_rob_ptr1_q <= `_WIRED_PARAM_ROB_LEN'd1;
        end else if(f_skid_ready_q) begin
            f_rob_ptr0_q <= f_rob_ptr0_q + f_valid[0] + f_valid[1];
            f_rob_ptr1_q <= f_rob_ptr1_q + f_valid[0] + f_valid[1];
        end
    end
    assign c_rrrid_o = {f_rob_ptr1_q, f_rob_ptr0_q};
    reg f_skid_ready_q;
    rob_entry_t [1:0] f_skid_entry_q;
    rob_rid_t [1:0] f_skid_wrrid_q;
    assign slot1_bank_conflict = c_rob_entry_i[1].wreg[0] == c_rob_entry_i[0].wreg[0] &&
                                 c_rob_entry_i[0].wreg    != '0;
    assign slot1_ctrl_conflict = c_rob_entry_i[1].di.slot0 ||
                                 c_rob_entry_i[1].excp_found ||
                                 c_rob_entry_i[1].bpu_predict.taken || // 错误预测的跳转，也强制送到第一条管线检查
                                 c_rob_entry_i[1].uncached; // 写指令 / uncached 指令需要特殊处理
    assign c_retire_o = f_valid & {f_skid_ready_q, f_skid_ready_q};

    always_ff @(posedge clk) begin
        if(f_skid_ready_q) begin
            f_skid_entry_q <= c_rob_entry_i;
            f_skid_wrrid_q <= {f_rob_ptr1_q, f_rob_ptr0_q};
        end
    end

    // 一二级流水之间插入的 Skid Buffer
    logic h_ready;    // H 级准备好接受来自 F 级别的数据
    reg [1:0] f_skid_valid_q;
    rob_entry_t [1:0] h_entry;
    rob_rid_t [1:0] h_wrrid;
    logic[31:0] h_flushtarget;
    assign h_entry = f_skid_ready_q ? c_rob_entry_i : f_skid_entry_q;
    assign h_wrrid = f_skid_ready_q ? {f_rob_ptr1_q, f_rob_ptr0_q} : f_skid_wrrid_q;
    wire [1:0] h_valid = f_skid_ready_q ? f_valid : f_skid_valid_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            f_skid_ready_q <= '1;
            f_skid_valid_q <= '0;
        end else if(f_skid_ready_q) begin
            if(!h_ready && (|f_valid)) begin
                f_skid_ready_q <= '0;
                f_skid_valid_q <= f_valid;
            end
        end else begin
            if(h_ready) begin
                f_skid_ready_q <= '1;
                f_skid_valid_q <= '0;
            end
        end
    end
    logic [31:0] h_csr_rdata;
    logic [63:0] timer_64_q;
    // 组合逻辑读取 CSR
    always_comb begin
        h_csr_rdata = '0;
        h_flushtarget = h_entry[0].pc + 4;
        if(h_entry[0].di.ertn_inst) begin
            h_flushtarget = csr_q.era;
        end
        case(h_entry[0].csr_id[8:0])
        `_CSR_CRMD      : h_csr_rdata = csr_q.crmd;
        `_CSR_PRMD      : h_csr_rdata = csr_q.prmd;
        `_CSR_EUEN      : h_csr_rdata = csr_q.euen;
        `_CSR_ECTL      : h_csr_rdata = csr_q.ectl;
        `_CSR_ESTAT     : h_csr_rdata = csr_q.estat;
        `_CSR_ERA       : h_csr_rdata = csr_q.era;
        `_CSR_BADV      : h_csr_rdata = csr_q.badv;
        `_CSR_EENTRY    : h_csr_rdata = csr_q.eentry;
        `_CSR_TLBIDX    : h_csr_rdata = csr_q.tlbidx;
        `_CSR_TLBEHI    : h_csr_rdata = csr_q.tlbehi;
        `_CSR_TLBELO0   : h_csr_rdata = csr_q.tlbelo0;
        `_CSR_TLBELO1   : h_csr_rdata = csr_q.tlbelo1;
        `_CSR_ASID      : h_csr_rdata = csr_q.asid;
        `_CSR_PGDL      : h_csr_rdata = csr_q.pgdl;
        `_CSR_PGDH      : h_csr_rdata = csr_q.pgdh;
        `_CSR_PGD       : h_csr_rdata = {csr_q.badv[31] ? csr_q.pgdh[31:12] : csr_q.pgdl[31:12] , 12'd0};
        `_CSR_CPUID     : h_csr_rdata = csr_q.cpuid;
        `_CSR_SAVE0     : h_csr_rdata = csr_q.save0;
        `_CSR_SAVE1     : h_csr_rdata = csr_q.save1;
        `_CSR_SAVE2     : h_csr_rdata = csr_q.save2;
        `_CSR_SAVE3     : h_csr_rdata = csr_q.save3;
        `_CSR_TID       : h_csr_rdata = csr_q.tid;
        `_CSR_TCFG      : h_csr_rdata = csr_q.tcfg;
        `_CSR_TVAL      : h_csr_rdata = csr_q.tval;
        `_CSR_TICLR     : h_csr_rdata = '0;
        `_CSR_LLBCTL    : h_csr_rdata = {csr_q.llbctl, 1'b0, csr_q.llbit};
        `_CSR_TLBRENTRY : h_csr_rdata = csr_q.tlbrentry;
        `_CSR_DMW0      : h_csr_rdata = csr_q.dmw0;
        `_CSR_DMW1      : h_csr_rdata = csr_q.dmw1;
        endcase
        // 特殊处理 rdcnt 命令
        if(h_entry[0].di.csr_rdcnt[0] /*== `_RDCNT_ID_VLOW*/) h_csr_rdata = (|h_entry[0].op_code) ? csr_q.tid : timer_64_q[31:0];
        else if(h_entry[0].di.csr_rdcnt[1] /*== `_RDCNT_VHIGH*/) h_csr_rdata = timer_64_q[63:32];
    end

    // TLBSRCH / TLBRD / INVTLB PRE-RUN.
    // 这里例化一个管理 TLB，执行所有 TLB 相关的操作。
    tlb_entry_t [`_WIRED_PARAM_TLB_CNT-1:0] tlb_entrys_q;
    for(genvar i = 0 ; i < `_WIRED_PARAM_TLB_CNT ; i += 1) begin
        always_ff @(posedge clk) begin
            if(tlb_update_req_o.tlb_we[i]) begin
                tlb_entrys_q[i] <= tlb_update_req_o.tlb_w_entry;
            end
        end
    end
    logic [`_WIRED_PARAM_TLB_CNT-1:0] h_tlb_hit_srch;
    logic [`_WIRED_PARAM_TLB_CNT-1:0] h_tlb_hit_invtlb; // 需要打流水
    tlb_entry_t h_tlb_rd; // for tlbrd 需要打流水
    logic [$clog2(`_WIRED_PARAM_TLB_CNT):0] h_tlb_srch_idx; // for tlbsrch 需要打流水，最高位是 FOUND
    // tlb_entry_t h_tlb_srch; // for tlbsrch 需要打流水
    // TLB HIT LOGIC HERE
    for(genvar i = 0 ; i < `_WIRED_PARAM_TLB_CNT ; i+=1) begin
        wire h_tlb_hit_asid = (tlb_entrys_q[i].key.asid == h_entry[0].target_addr[9:0]);
        wire h_tlb_hit_vppn = (tlb_entrys_q[i].key.vppn[18:10] == h_entry[0].target_addr[31:23]) && // HIGH HIT
                              (tlb_entrys_q[i].key.vppn[9:0] == h_entry[0].target_addr[22:13] || tlb_entrys_q[i].key.huge_page);   // LOW  HIT

        always_comb begin
            h_tlb_hit_invtlb[i] = '0;
            if(h_entry[0].di.invtlb_en) begin
              if(h_entry[0].op_code == 0 || h_entry[0].op_code == 1) begin
                h_tlb_hit_invtlb[i] = '1;
              end
              if(h_entry[0].op_code == 2) begin
                h_tlb_hit_invtlb[i] = tlb_entrys_q[i].key.g;
              end
              if(h_entry[0].op_code == 3) begin
                h_tlb_hit_invtlb[i] = !tlb_entrys_q[i].key.g;
              end
              if(h_entry[0].op_code == 4) begin
                h_tlb_hit_invtlb[i] = !tlb_entrys_q[i].key.g && h_tlb_hit_asid;
              end
              if(h_entry[0].op_code == 5) begin
                h_tlb_hit_invtlb[i] = !tlb_entrys_q[i].key.g && h_tlb_hit_asid && h_tlb_hit_vppn;
              end
              if(h_entry[0].op_code == 6) begin
                h_tlb_hit_invtlb[i] = (tlb_entrys_q[i].key.g || h_tlb_hit_asid) && h_tlb_hit_vppn;
              end
            end
        end
        // tlbsrch
        // 注意，这个路径可以配置为 multicycle 以优化性能。
        assign h_tlb_hit_srch[i] = (tlb_entrys_q[i].key.e) && // E
                                   (tlb_entrys_q[i].key.g || tlb_entrys_q[i].key.asid == csr_q.asid[9:0]) && // ASID MATHC
                                   (tlb_entrys_q[i].key.vppn[18:10] == csr_q.tlbehi[31:23]) && // HI-VPN MATCH
                                   (tlb_entrys_q[i].key.vppn[9:0] == csr_q.tlbehi[22:13] || tlb_entrys_q[i].key.huge_page) // LO-VPN MATCH
                                   ;
    end
    // tlbrd
    assign h_tlb_rd = tlb_entrys_q[csr_q.tlbidx[`_TLBIDX_INDEX]];
    // tlbsrch
    always_comb begin
        h_tlb_srch_idx = '0;
        // h_tlb_srch = '0;
        for(integer i = 0 ; i < `_WIRED_PARAM_TLB_CNT ; i += 1) begin
            h_tlb_srch_idx |= h_tlb_hit_srch[i] ? {1'b1, i[`_TLBIDX_INDEX]} : '0;
            // h_tlb_srch |= h_tlb_hit_srch[i] ? tlb_entrys_q[i].key : '0;
        end
    end

    // 第二级流水（实质是一个状态机）
    logic [31:0] h_csr_rdata_q;
    rob_entry_t [1:0] h_entry_q;
    rob_rid_t [1:0] h_wrrid_q;
    logic [1:0]  h_valid_inst_q;
    logic [31:0] h_flushtarget_q;
    logic [`_WIRED_PARAM_TLB_CNT-1:0] h_tlb_hit_invtlb_q; // 需要打流水
    tlb_entry_t h_tlb_rd_q; // for tlbrd 需要打流水
    logic [$clog2(`_WIRED_PARAM_TLB_CNT):0] h_tlb_srch_idx_q; // for tlbsrch 需要打流水，最高位是 FOUND
    // tlb_entry_t h_tlb_srch_q; // for tlbsrch 需要打流水
    always_ff @(posedge clk) begin
        if(h_ready) begin
            h_entry_q <= h_entry;
            h_wrrid_q <= h_wrrid;
            h_csr_rdata_q <= h_csr_rdata;
            h_flushtarget_q <= h_flushtarget;
            // TLB RELATED
            h_tlb_hit_invtlb_q <= h_tlb_hit_invtlb;
            h_tlb_rd_q <= h_tlb_rd;
            h_tlb_srch_idx_q <= h_tlb_srch_idx;
            // h_tlb_srch_q <= h_tlb_srch;
        end
    end
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            h_valid_inst_q <= '0;
        end else begin
            if(h_ready) begin
                h_valid_inst_q <= h_valid;
            end
        end
    end

    // 主提交状态机
    // 需要考虑：
    // 1. Uncached 请求（向 LSU 发出请求，等待 Uncached 读数据 / 写完成）
    // （注意， Uncached Load 及 Uncached Store 均需要立即发出请求，且均需要刷新流水线）
    // 2. Store 请求，等待 Cache 重填后再执行（暂停等待）
    // 3. Store Conditional 请求，不再等待 Cache 重填，如果不可执行，则直接刷新流水线
    // 4. Branch 类型指令，预测错误时跳转并更新 BPU
    // 5. CSRWR/CSRRD 指令，直接刷新管线
    // （注意， Store Conditional 请求需要检查 store buffer 中的情况）
    // 经过之前的约束，所有可能造成暂停的指令（Uncache load/store | Missed Store）均一定在槽一
    // 可能产生流水线刷新的指令也是同样的情况
    csr_t csr, csr_init;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            timer_64_q <= '0;
        end else begin
            timer_64_q <= timer_64_q + 64'b1;
        end
    end
    always_comb begin
        csr_init = '0;
        csr_init.crmd[`_CRMD_DA] = 1'd1; // 初始化要求非0的 CSR 寄存器值
        csr_init.asid[31:10] = 22'h280;
        csr_init.cpuid = CPU_ID;
        csr_init.tid = CPU_ID;
    end
    csr_t csr_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            csr_q <= csr_init; // 初始化 CSR
        end else if(h_ready) begin
            csr_q <= csr;
        end
    end

    // 注意从此开始不再需要握手，因为之后的流水线一路通畅。
    logic            l_flush;  // 流水线需要冲刷
    logic      [1:0] l_retire; // 标识指令的信息需要进入 Rename，注意，所有进入 ROB 的指令无论实际是否执行一定需要 retire。
    logic      [1:0] l_commit; // 标识指令的结果需要写入 ARF
    word_t     [1:0] l_data;
    arch_rid_t [1:0] l_warid;
    rob_rid_t  [1:0] l_wrrid;
    logic      [1:0] l_tier_id;
    bpu_correct_t    f_upd;
    always_ff @(posedge clk) begin
        l_flush_o <= l_flush;
        l_retire_o <= l_retire;
        l_commit_o <= l_commit;
        l_data_o <= l_data;
        l_warid_o <= l_warid;
        l_wrrid_o <= l_wrrid;
        l_tier_id_o <= l_tier_id;
        f_upd_o <= f_upd;
    end

    // 主状态机定义
    typedef enum logic[2:0] {
    S_NORMAL,
    S_WAIT_ULOAD,    // 这个要刷流水线，还要修改写入 ARF 的数据
    S_WAIT_USTORE,   // 这个不用刷流水线，但需要解除 dbar（uncached 会设置 barrier）
    S_WAIT_MSTORE,   // 这个不用刷流水线
    S_WAIT_FLUSH,    // 这个是用来刷流水线的
    S_WAIT_INTERRUPT // 这个是用来等待中断的
    } commit_fsm_e;
    commit_fsm_e fsm_q;
    commit_fsm_e fsm;
    logic h_tid;
    logic h_tid_q; // 跳转使用的 id 标识
    logic timer_en;
    logic timer_en_q;  // CSR 时钟使能
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            fsm_q <= S_NORMAL;
            h_tid_q <= '0;
            timer_en_q <= '0;
        end else begin
            fsm_q <= fsm;
            h_tid_q <= h_tid;
            timer_en_q <= timer_en;
        end
    end

    // 获取指令 0 所属跳转目标类型
    // 0: 无跳转
    // 1: 函数调用；2: 函数返回；3: 调用栈无关类
    logic [1:0] slot0_target_type;
    always_comb begin
        slot0_target_type = '0;
        if(h_entry_q[0].di.jump_inst && h_entry_q[0].op_code[4]) begin // 1 -> CALL JIRL BL
            slot0_target_type = 2'd1;
        end else if(h_entry_q[0].di.jump_inst && h_entry_q[0].op_code[3]) begin // 2 -> RETURN
            slot0_target_type = 2'd2;
        end else if(h_entry_q[0].di.jump_inst) begin  // 3 -> IMM
            slot0_target_type = 2'd3;
        end
    end

    // CSR 写辅助指令
    logic [31:0] csr_wmask, csr_wdata, csr_rwdata;

    // 主状态机组合逻辑
    excp_t excp;
    // for(genvar i = 0 ; i < 2 ; i += 1) begin
    assign excp = gather_excp(h_entry_q[0].static_excp, h_entry_q[0].lsu_excp);
    // end
    // 打拍后的中断向量
    logic timer_interrupt;
    logic [1:0] soft_interrupt;
    logic [1:0] soft_interrupt_q;
    logic [8:0] f_interrupt_q;
    always_ff @(posedge clk) f_interrupt_q <= f_interrupt_i;
    wire  [12:0] int_vec = {f_interrupt_q[8] ,timer_interrupt, 1'd0, f_interrupt_q[7:0], /*csr_q.estat[1:0]*/ soft_interrupt_q};
    wire  [12:0] int_mask = csr_q.ectl[12:0] & {13{csr_q.crmd[`_CRMD_IE]}};
    wire  [12:0] masked_int = int_mask & int_vec;
    logic int_pending_q;
    logic [12:0] int_vec_q;
    always_ff @(posedge clk) begin
        int_vec_q <= int_vec;
        int_pending_q <= |masked_int;
        soft_interrupt_q <= soft_interrupt;
    end
    // SUPER SUPER HUGE LOGIC BEGIN !
    // MAIN PROCESSOR STATES ARE MAINTAINED HERE !
    always_comb begin
        tlb_update_req_o = '0;
        timer_interrupt = int_vec_q[11];
        soft_interrupt = soft_interrupt_q;
        
        tlb_update_req_o.tlb_w_entry.key.vppn = csr_q.tlbehi[31:13];
        // tlb_update_req_o.tlb_w_entry.key.ps   = csr_q.tlbidx[29:24]; // P72
        tlb_update_req_o.tlb_w_entry.key.huge_page = csr_q.tlbidx[29:24] == 6'd22; // P72
        tlb_update_req_o.tlb_w_entry.key.g    = csr_q.tlbelo0[6] && csr_q.tlbelo1[6];
        tlb_update_req_o.tlb_w_entry.key.asid = csr_q.asid[9:0];
        tlb_update_req_o.tlb_w_entry.key.e    = !csr_q.tlbidx[31] || csr_q.estat[21]; // P73

        tlb_update_req_o.tlb_w_entry.value[0].ppn = csr_q.tlbelo0[27:8]; // P74
        tlb_update_req_o.tlb_w_entry.value[0].v   = csr_q.tlbelo0[0];
        tlb_update_req_o.tlb_w_entry.value[0].d   = csr_q.tlbelo0[1];
        tlb_update_req_o.tlb_w_entry.value[0].plv = csr_q.tlbelo0[3:2];
        tlb_update_req_o.tlb_w_entry.value[0].mat = csr_q.tlbelo0[5:4];

        tlb_update_req_o.tlb_w_entry.value[1].ppn = csr_q.tlbelo1[27:8]; // P74
        tlb_update_req_o.tlb_w_entry.value[1].v   = csr_q.tlbelo1[0];
        tlb_update_req_o.tlb_w_entry.value[1].d   = csr_q.tlbelo1[1];
        tlb_update_req_o.tlb_w_entry.value[1].plv = csr_q.tlbelo1[3:2];
        tlb_update_req_o.tlb_w_entry.value[1].mat = csr_q.tlbelo1[5:4];
        c_lsu_req_o = '0;
        csr = csr_q;
        h_tid = h_tid_q;
        fsm = fsm_q;
        timer_en = timer_en_q;
        h_ready = '1;
        l_flush = '0;  // 流水线需要冲刷
        l_retire = '0; // 标识指令的信息需要进入 Rename，注意，所有进入 ROB 的指令无论实际是否执行一定需要 retire。
        l_commit = '0; // 标识指令的结果需要进入 CSR / 写入 ARF
        for(integer i = 0 ; i < 2 ; i+=1) begin
            l_data[i] = h_entry_q[i].wdata;
            l_warid[i] = h_entry_q[i].wreg;
            l_wrrid[i] = h_wrrid_q[i];
            l_tier_id[i] = h_entry_q[i].wtier;
        end
        f_upd = '0;
        f_upd.tid = h_tid_q;
        f_upd.pc = h_entry_q[0].pc;
        f_upd.true_taken = h_entry_q[0].need_jump;
        f_upd.true_target = h_flushtarget_q; // 这里已经更新过了 // 对于分支指令，这就是最终目标，对于非分支指令，这里不会使用，若跳转类型错误，后面会做纠正
        f_upd.btb_target  = h_entry_q[0].target_addr;
        f_upd.lphr = h_entry_q[0].bpu_predict.lphr;
        f_upd.history = h_entry_q[0].bpu_predict.history;
        f_upd.true_target_type = bpu_target_type_e'(slot0_target_type);
        f_upd.true_conditional_jmp = (|h_entry_q[0].di.cmp_type[3:1]) && !(&h_entry_q[0].di.cmp_type[3:1]);
        f_upd.ras_ptr = h_entry_q[0].bpu_predict.ras_ptr;
        if(slot0_target_type == 2'd1 && (slot0_target_type != h_entry_q[0].bpu_predict.target_type)) begin
            f_upd.ras_miss_type = '1;
            f_upd.ras_ptr = h_entry_q[0].bpu_predict.ras_ptr + 1;
        end
        if(slot0_target_type == 2'd2 && (slot0_target_type != h_entry_q[0].bpu_predict.target_type)) begin
            f_upd.ras_miss_type = '1;
            f_upd.ras_ptr = h_entry_q[0].bpu_predict.ras_ptr - 1;
        end
        // CSR mask 获得
        csr_wmask = h_entry_q[0].target_addr;
        if(h_entry_q[0].op_code == '0) csr_wmask = '0;
        if(h_entry_q[0].op_code == 5'd1) csr_wmask = '1;
        csr_rwdata =  csr_wmask & h_entry_q[0].wdata;
        csr_wdata  = (csr_wmask & h_entry_q[0].wdata) | ((~csr_wmask) & h_csr_rdata_q);
        // 时钟处理
        if(/*csr_q.tcfg[`_TCFG_EN]*/ timer_en_q) begin
            if(csr_q.tval != '0) begin
                csr.tval = csr_q.tval - 1;
            end else if(csr_q.tcfg[`_TCFG_PERIODIC]) begin
                csr.tval[`_TCFG_INITVAL] = csr_q.tcfg[`_TCFG_INITVAL];
                timer_interrupt = '1;
            end else begin
                // tval == '0, en =='1, periodic == '0;
                /*csr.tcfg[`_TCFG_EN]*/timer_en = '0;
                timer_interrupt = '1;
            end
        end
        case (fsm_q)
            S_NORMAL: begin
                f_upd.need_update = h_valid_inst_q[0] && ((|slot0_target_type) || (slot0_target_type != h_entry_q[0].bpu_predict.target_type));
                l_flush = '0;
                l_retire = h_valid_inst_q;
                l_commit = h_valid_inst_q;
                // 对 inst0 的中断异常情况做 judge
                if(h_valid_inst_q[0] && (h_entry_q[0].excp_found || int_pending_q)) begin
                    // 第 0 条指令有问题
                    l_commit[1] = '0;
                    l_commit[0] = '0;
                    f_upd.redirect = '1;
                    f_upd.true_target = csr_q.eentry;
                    csr.era = h_entry_q[0].pc;
                    csr.crmd[`_CRMD_PLV] = 2'b0;
                    csr.crmd[`_CRMD_IE] = 1'b0;
                    csr.prmd[2:0] = csr_q.crmd[2:0];
                    fsm = S_WAIT_FLUSH;
                    // 先更新 ecode
                    case(1'b1)
                        int_pending_q: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h00;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                        end // None Masked Interruption founded, if founded, this instruction is forced to issue in ALU slot
                        excp.adef: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h08;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].pc;
                        end
                        excp.itlbr: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h3f;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.crmd[`_CRMD_DA] = 1'b1;
                            csr.crmd[`_CRMD_PG] = 1'b0;
                            csr.badv = h_entry_q[0].pc;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].pc[`_TLBEHI_VPPN];
                            f_upd.true_target = csr_q.tlbrentry;
                        end
                        excp.pif: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h03;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].pc;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].pc[`_TLBEHI_VPPN];
                        end
                        excp.ippi: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h07;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].pc;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].pc[`_TLBEHI_VPPN];
                        end
                        excp.ine: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h0d;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                        end
                        excp.ipe: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h0e;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                        end
                        excp.sys: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h0b;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                        end
                        excp.brk: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h0c;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                        end
                        excp.ale: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h09;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].target_addr;
                        end
                        excp.tlbr: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h3f;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.crmd[`_CRMD_DA] = 1'b1;
                            csr.crmd[`_CRMD_PG] = 1'b0;
                            csr.badv = h_entry_q[0].target_addr;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].target_addr[`_TLBEHI_VPPN];
                            f_upd.true_target = csr_q.tlbrentry;
                        end
                        excp.pis: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h02;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].target_addr;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].target_addr[`_TLBEHI_VPPN];
                        end
                        excp.pil: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h01;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].target_addr;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].target_addr[`_TLBEHI_VPPN];
                        end
                        excp.ppi: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h07;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].target_addr;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].target_addr[`_TLBEHI_VPPN];
                        end
                        excp.pme: begin
                            csr.estat[`_ESTAT_ECODE] = 6'h04;
                            csr.estat[`_ESTAT_ESUBCODE] = '0;
                            csr.badv = h_entry_q[0].target_addr;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[0].target_addr[`_TLBEHI_VPPN];
                        end
                    endcase
                    // end
                end else if(h_valid_inst_q[0]) begin
                    // 不存在异常的部分
                    // 0. 恢复 DBAR
                    if(h_entry_q[0].di.dbarrier) begin
                        // 可能进入这里的，一定不会暂停，对应指令：DBAR
                        c_lsu_req_o.dbarrier_unlock = '1;
                    end
                    // 访存部分处理
                    if(h_entry_q[0].di.lsu_inst) begin
                        if(h_entry_q[0].uncached) begin
                            // 1. Uncached 请求（向 LSU 发出请求，等待 Uncached 读数据 / 写完成）
                            // （注意， Uncached Load 及 Uncached Store 均需要立即发出请求，且 Uncached Load 需要刷新流水线）
                            l_commit = '0;
                            l_retire = '0;
                            h_ready = '0;
                            if(h_entry_q[0].di.mem_write) begin
                                fsm = S_WAIT_USTORE;
                            end else begin
                                fsm = S_WAIT_ULOAD;
                            end
                        end else begin
                            if(h_entry_q[0].di.mem_write) begin
                                // 3. Store Conditional 请求，不再等待 Cache 重填，如果不可执行，则直接刷新流水线
                                if(h_entry_q[0].di.llsc_inst) begin
                                    if((!c_lsu_resp_i.storebuf_hit) || (!csr_q.llbit)) begin // 其它 Cache Coherent Master probe 了这行，不再原子
                                        l_commit = 2'b01;
                                        l_data[0] = 32'd0;
                                        f_upd.redirect = '1;
                                        fsm = S_WAIT_FLUSH; // 对于失败的 SC ，需要 refresh 流水线
                                    end else begin
                                        // 成功的 SC，弹栈
                                        c_lsu_req_o.storebuf_commit = '1;
                                    end
                                    csr.llbit = '0; // 下周期清理 llbit
                                end else begin
                                    // 2. 非 SC 的 Store 请求 MISS ，等待 Cache 重填后再执行（暂停等待）
                                    if(!c_lsu_resp_i.storebuf_hit) begin
                                        h_ready = '0;
                                        l_commit = '0;
                                        l_retire = '0;
                                        fsm = S_WAIT_MSTORE;
                                    end else begin
                                        c_lsu_req_o.storebuf_commit = '1;
                                    end
                                end
                            end else begin
                                // load inst
                                if(h_entry_q[0].di.llsc_inst) begin
                                    csr.llbit = '1;
                                end
                            end
                        end
                    end
                    // end
                    
                    // CSR 读写指令处理，注意刷新管线
                    // 所有指令的读取实际在 ALU 中已经完成了，这里只需要检查读结果是否有效并写入，合适的刷新管线即可
                    // 所有对 CSR 产生写操作（状态改变）的指令都需要刷新管线。
                    // 其实只有一条指令，csrwrxchg，操作是根据掩码写寄存器之后再读出
`define _MW(csr_name, mask) csr.``csr_name``[mask] = csr_wdata[mask]
                    if(h_entry_q[0].di.csr_op_en) begin
                        l_data[0] = h_csr_rdata_q; // 强制刷新 csr 数据
                        case(h_entry_q[0].csr_id[8:0])
                        `_CSR_CRMD:      begin `_MW(crmd, `_CRMD_PLV);`_MW(crmd, `_CRMD_IE);`_MW(crmd, `_CRMD_DA);`_MW(crmd, `_CRMD_PG);`_MW(crmd, `_CRMD_DATF);`_MW(crmd, `_CRMD_DATM); end
                        `_CSR_PRMD:      begin `_MW(prmd, `_PRMD_PPLV);`_MW(prmd, `_PRMD_PIE); end
                        `_CSR_EUEN:      begin `_MW(euen, `_EUEN_FPE); end
                        `_CSR_ECTL:      begin `_MW(ectl, `_ECTL_LIE1);`_MW(ectl, `_ECTL_LIE2); end
                        `_CSR_ESTAT:     begin /*`_MW(estat, 1:0);*/ soft_interrupt = csr_wdata[1:0]; end
                        `_CSR_ERA:       begin `_MW(era, 31:0); end
                        `_CSR_BADV:      begin `_MW(badv, 31:0); end
                        `_CSR_EENTRY:    begin `_MW(eentry, `_EENTRY_VA); end
                        `_CSR_TLBIDX:    begin `_MW(tlbidx, `_TLBIDX_INDEX);`_MW(tlbidx, `_TLBIDX_PS);`_MW(tlbidx, `_TLBIDX_NE); end
                        `_CSR_TLBEHI:    begin `_MW(tlbehi, `_TLBEHI_VPPN); end
                        `_CSR_TLBELO0:   begin `_MW(tlbelo0, 31:8);`_MW(tlbelo0, 6:0); end
                        `_CSR_TLBELO1:   begin `_MW(tlbelo1, 31:8);`_MW(tlbelo1, 6:0); end
                        `_CSR_ASID:      begin `_MW(asid, `_ASID); end
                        `_CSR_PGDL:      begin `_MW(pgdl, `_PGD_BASE); end
                        `_CSR_PGDH:      begin `_MW(pgdh, `_PGD_BASE); end
                        // `_CSR_PGD:       begin `_MW(); end // 只读
                        // `_CSR_CPUID:     begin `_MW(); end // 只读
                        `_CSR_SAVE0:     begin `_MW(save0, 31:0); end
                        `_CSR_SAVE1:     begin `_MW(save1, 31:0); end
                        `_CSR_SAVE2:     begin `_MW(save2, 31:0); end
                        `_CSR_SAVE3:     begin `_MW(save3, 31:0); end
                        `_CSR_TID:       begin `_MW(tid, 31:0); end
                        `_CSR_TCFG:      begin `_MW(tcfg,31:0);csr.tval[1:0] = '0;`_MW(tval,`_TCFG_INITVAL); if(csr_wmask & 32'd1) timer_en = h_entry_q[0].wdata[0]; end
                        // `_CSR_TVAL:      begin `_MW(); end // 只读
                        `_CSR_TICLR:     begin if(csr_rwdata[0]) timer_interrupt = '0; end
                        `_CSR_LLBCTL:    begin `_MW(llbctl, `_LLBCT_KLO); if(csr_wdata[`_LLBCT_WCLLB]) csr.llbit = 1'b0; end
                        `_CSR_TLBRENTRY: begin `_MW(tlbrentry, `_TLBRENTRY_PA); end
                        `_CSR_DMW0:      begin `_MW(dmw0,`_DMW_PLV0);`_MW(dmw0,`_DMW_PLV3);`_MW(dmw0,`_DMW_MAT);`_MW(dmw0,`_DMW_PSEG);`_MW(dmw0,`_DMW_VSEG); end
                        `_CSR_DMW1:      begin `_MW(dmw1,`_DMW_PLV0);`_MW(dmw1,`_DMW_PLV3);`_MW(dmw1,`_DMW_MAT);`_MW(dmw1,`_DMW_PSEG);`_MW(dmw1,`_DMW_VSEG); end
                        endcase
                    end
`undef _MW
                    // 实现 RDCNT 指令
                    if(h_entry_q[0].di.csr_rdcnt != '0) begin
                        // RDCNT LOW | ID
                        l_data[0] = h_csr_rdata_q; // 强制刷新 csr 数据
                    end
                    // 实现 TLB 控制指令
                    // - INVTLB，直接无效化表项即可，先前已进行检查
                    if(h_entry_q[0].di.invtlb_en) begin
                        tlb_update_req_o.tlb_w_entry.key.e = '0;
                        tlb_update_req_o.tlb_we = h_tlb_hit_invtlb_q;
                    end
                    // - TLBWR
                    if(h_entry_q[0].di.tlbwr_en) begin
                        tlb_update_req_o.tlb_we[csr_q.tlbidx[`_TLBIDX_INDEX]] = '1;
                    end
                    // - TLBFILL
                    if(h_entry_q[0].di.tlbfill_en) begin
                        tlb_update_req_o.tlb_we[timer_64_q[`_TLBIDX_INDEX]] = '1;
                    end
                    // - TLBSRCH
                    if(h_entry_q[0].di.tlbsrch_en) begin
                        csr.tlbidx[`_TLBIDX_INDEX] = h_tlb_srch_idx_q[`_TLBIDX_INDEX];
                        csr.tlbidx[`_TLBIDX_NE] = !h_tlb_srch_idx_q[$clog2(`_WIRED_PARAM_TLB_CNT)];
                    end
                    // - TLBRD
                    if(h_entry_q[0].di.tlbrd_en) begin
                        csr.tlbidx[`_TLBIDX_NE] = ~h_tlb_rd_q.key.e;
                        if(h_tlb_rd_q.key.e) begin
                            csr.tlbidx[`_TLBIDX_PS] = h_tlb_rd_q.key.huge_page ? 6'd22 : 6'd12;
                            csr.tlbehi[`_TLBEHI_VPPN] = h_tlb_rd_q.key.vppn;
                            csr.tlbelo0[`_TLBELO_TLB_V]   = h_tlb_rd_q.value[0].v;
                            csr.tlbelo0[`_TLBELO_TLB_D]   = h_tlb_rd_q.value[0].d;
                            csr.tlbelo0[`_TLBELO_TLB_PLV] = h_tlb_rd_q.value[0].plv;
                            csr.tlbelo0[`_TLBELO_TLB_MAT] = h_tlb_rd_q.value[0].mat;
                            csr.tlbelo0[`_TLBELO_TLB_G]   = h_tlb_rd_q.key.g;
                            csr.tlbelo0[`_TLBELO_TLB_PPN] = h_tlb_rd_q.value[0].ppn;

                            csr.tlbelo1[`_TLBELO_TLB_V]   = h_tlb_rd_q.value[1].v;
                            csr.tlbelo1[`_TLBELO_TLB_D]   = h_tlb_rd_q.value[1].d;
                            csr.tlbelo1[`_TLBELO_TLB_PLV] = h_tlb_rd_q.value[1].plv;
                            csr.tlbelo1[`_TLBELO_TLB_MAT] = h_tlb_rd_q.value[1].mat;
                            csr.tlbelo1[`_TLBELO_TLB_G]   = h_tlb_rd_q.key.g;
                            csr.tlbelo1[`_TLBELO_TLB_PPN] = h_tlb_rd_q.value[1].ppn;
                            csr.asid[`_ASID]              =  h_tlb_rd_q.key.asid;
                        end else begin
                            csr.tlbidx[`_TLBIDX_PS] = '0;
                            csr.tlbehi[`_TLBEHI_VPPN] = '0;
                            csr.tlbelo0[`_TLBELO_TLB_V]   = '0;
                            csr.tlbelo0[`_TLBELO_TLB_D]   = '0;
                            csr.tlbelo0[`_TLBELO_TLB_PLV] = '0;
                            csr.tlbelo0[`_TLBELO_TLB_MAT] = '0;
                            csr.tlbelo0[`_TLBELO_TLB_G]   = '0;
                            csr.tlbelo0[`_TLBELO_TLB_PPN] = '0;

                            csr.tlbelo1[`_TLBELO_TLB_V]   = '0;
                            csr.tlbelo1[`_TLBELO_TLB_D]   = '0;
                            csr.tlbelo1[`_TLBELO_TLB_PLV] = '0;
                            csr.tlbelo1[`_TLBELO_TLB_MAT] = '0;
                            csr.tlbelo1[`_TLBELO_TLB_G]   = '0;
                            csr.tlbelo1[`_TLBELO_TLB_PPN] = '0;
                            csr.asid[`_ASID]              = '0;
                        end
                    end
                    // flush / ertn 逻辑 及 idle 逻辑
                    if(l_commit[0] && (h_entry_q[0].di.refetch)) begin
                        if(h_entry_q[0].di.ertn_inst) begin
                            // 执行异常返回操作，更新寄存器
                            csr.crmd[`_CRMD_PLV] = csr_q.prmd[`_PRMD_PPLV];
                            csr.crmd[`_CRMD_IE]  = csr_q.prmd[`_PRMD_PIE];
                            if(csr_q.llbctl[`_LLBCT_KLO]) begin
                                csr.llbctl[`_LLBCT_KLO] = '0;
                            end else begin
                                csr.llbit = 1'b0;
                            end
                            if(csr_q.estat[21]) begin
                                csr.crmd[`_CRMD_DA] = '0;
                                csr.crmd[`_CRMD_PG] = '1;
                            end
                        end
                        l_commit = 2'b01;
                        f_upd.redirect = '1;
                        f_upd.true_target = h_flushtarget_q;
                        fsm = h_entry_q[0].di.wait_inst ? S_WAIT_INTERRUPT : S_WAIT_FLUSH;
                    end
                end
            end
            S_WAIT_ULOAD: begin
                h_ready = '0;
                c_lsu_req_o.valid = '1;
                c_lsu_req_o.uncached_load_req = '1;
                if(c_lsu_resp_i.ready) begin
                    h_ready = '1;
                    l_retire = h_valid_inst_q;
                    l_commit = 2'b01;
                    l_data[0] = c_lsu_resp_i.uncached_load_resp;
                    f_upd.redirect = '1;
                    fsm = S_WAIT_FLUSH; // 对于 Uncached load ，需要 refresh 流水线
                    // 由于流水线被刷新，对于 uncache load，不一定设置 dbar，更不需要解除 dbar
                end
            end
            S_WAIT_USTORE: begin
                h_ready = '0;
                c_lsu_req_o.valid = '1;
                c_lsu_req_o.uncached_store_req = '1;
                if(c_lsu_resp_i.ready) begin
                    h_ready = '1;
                    l_retire = h_valid_inst_q;
                    l_commit = h_valid_inst_q;
                    fsm = S_NORMAL; // 对于 Uncached store ，不需要 refresh 流水线
                    c_lsu_req_o.dbarrier_unlock = '1; // 但是需要解除 dbar
                    c_lsu_req_o.storebuf_commit = '1;
                end
            end
            S_WAIT_MSTORE: begin
                h_ready = '0;
                c_lsu_req_o.valid = '1;
                c_lsu_req_o.refill_store_req = '1;
                if(c_lsu_resp_i.ready) begin
                    h_ready = '1;
                    l_retire = h_valid_inst_q;
                    l_commit = h_valid_inst_q;
                    fsm = S_NORMAL; // 对于 Uncached load ，需要 refresh 流水线
                    c_lsu_req_o.storebuf_commit = '1;
                end
            end
            S_WAIT_FLUSH: begin
                h_ready = '1;
                l_flush  = '1;
                l_retire = h_valid_inst_q;
                l_commit = '0;
                if(rename_empty_i) begin
                    fsm = S_NORMAL;
                end
            end
            S_WAIT_INTERRUPT: begin
                h_ready = '1;
                l_flush  = '1;
                l_retire = h_valid_inst_q;
                l_commit = '0;
                if(rename_empty_i && int_pending_q) begin
                    fsm = S_NORMAL;
                end
            end
        endcase
        // 跳转指令处理，注意刷新管线
        // if(h_entry_q[0].di.jump_inst) begin // 对非访存指令，也可能会误预测。
        // 特殊处理未命中情况，刷新流水线，重定向控制流
        // 注意：到达这里的指令，可能下一拍不一定正常执行，检查 l_commit[0]
        if(  l_commit[0] &&
          ((h_entry_q[0].need_jump && (!h_entry_q[0].bpu_predict.taken || h_entry_q[0].bpu_predict.predict_pc != h_entry_q[0].target_addr)) ||
          (!h_entry_q[0].need_jump &&   h_entry_q[0].bpu_predict.taken))) begin
            l_commit = 2'b01;
            f_upd.miss = '1;
            f_upd.need_update = '1;
            f_upd.redirect = '1;
            if(h_entry_q[0].need_jump) f_upd.true_target = h_entry_q[0].target_addr;
            fsm = S_WAIT_FLUSH;
        end
        if(h_ready && f_upd.redirect) begin
            h_tid = ~h_tid_q;
            f_upd.tid = ~h_tid_q;
        end
        if(h_ready) csr.estat[12:0] = int_vec_q[12:0];
    end

    // CSR 输出
    assign csr_o = csr_q;

    // 连接差分测试
if(ENABLE_DIFFTEST) begin
  logic[31:0][31:0] ref_regs;
    for(genvar i = 0 ; i < 32 ; i ++) begin
      always_ff @(posedge clk) begin
        if(!rst_n) begin
          ref_regs[i] <= '0;
        end
        else if(l_commit[0] && l_warid[0] == i[4:0] && i != 0) begin
          ref_regs[i] <= l_data[0];
        end
        else if(l_commit[1] && l_warid[1] == i[4:0] && i != 0) begin
          ref_regs[i] <= l_data[1];
        end
      end
    end
    DifftestGRegState DifftestGRegState (
        .clock (clk         ),
        .coreid(0           ),
        .gpr_0 (ref_regs[0] ),
        .gpr_1 (ref_regs[1] ),
        .gpr_2 (ref_regs[2] ),
        .gpr_3 (ref_regs[3] ),
        .gpr_4 (ref_regs[4] ),
        .gpr_5 (ref_regs[5] ),
        .gpr_6 (ref_regs[6] ),
        .gpr_7 (ref_regs[7] ),
        .gpr_8 (ref_regs[8] ),
        .gpr_9 (ref_regs[9] ),
        .gpr_10(ref_regs[10]),
        .gpr_11(ref_regs[11]),
        .gpr_12(ref_regs[12]),
        .gpr_13(ref_regs[13]),
        .gpr_14(ref_regs[14]),
        .gpr_15(ref_regs[15]),
        .gpr_16(ref_regs[16]),
        .gpr_17(ref_regs[17]),
        .gpr_18(ref_regs[18]),
        .gpr_19(ref_regs[19]),
        .gpr_20(ref_regs[20]),
        .gpr_21(ref_regs[21]),
        .gpr_22(ref_regs[22]),
        .gpr_23(ref_regs[23]),
        .gpr_24(ref_regs[24]),
        .gpr_25(ref_regs[25]),
        .gpr_26(ref_regs[26]),
        .gpr_27(ref_regs[27]),
        .gpr_28(ref_regs[28]),
        .gpr_29(ref_regs[29]),
        .gpr_30(ref_regs[30]),
        .gpr_31(ref_regs[31])
        );
    rob_entry_t [1:0] df_entry_q;
    always_ff @(posedge clk) df_entry_q <= h_entry_q;
    logic [`_TLBIDX_INDEX] dbg_tlb_rndsel;
    logic [63:0] dbg0_timer_64_q, dbg_timer_64_q;
    wire h_excp_inst = h_valid_inst_q[0] && h_entry_q[0].excp_found && fsm_q == S_NORMAL;
    wire h_int_inst  = h_valid_inst_q[0] && int_pending_q && fsm_q == S_NORMAL;
    logic dbg_excp_inst, dbg_int_inst;
    always_ff @(posedge clk) begin
        dbg_tlb_rndsel <= timer_64_q[`_TLBIDX_INDEX];
        dbg0_timer_64_q <= timer_64_q;
        dbg_timer_64_q <= dbg0_timer_64_q;
        dbg_excp_inst  <= h_excp_inst;
        dbg_int_inst   <= h_int_inst;
    end
    for(genvar p = 0 ; p < 2 ; p += 1) begin
                    // 假的地址翻译模块，用于拿到提交load指令的物理地址
                    tlb_s_resp_t dbg_tlb_resp;
                    wired_addr_trans # (
                        .FETCH_ADDR('0)
                    )
                    wired_addr_trans_inst (
                        `_WIRED_GENERAL_CONN,
                        .clken_i('1),
                        .vaddr_i(h_entry_q[p].target_addr),
                        .csr_i(csr_q),
                        .tlb_update_req_i(tlb_update_req_o),
                        .trans_result_o(dbg_tlb_resp)
                    );
                    DifftestInstrCommit DifftestInstrCommit_p (
                        .clock         (clk             ),
                        .coreid        ('0              ),
                        .index         (p               ),
                        .valid         (l_commit_o[p]   ),
                        .pc            (df_entry_q[p].pc),
                        .instr         (df_entry_q[p].di.inst),
                        .skip          ('0              ),
                        .is_TLBFILL    (df_entry_q[p].di.tlbfill_en && l_commit_o[p]), // TODO: CHECK
                        .TLBFILL_index (dbg_tlb_rndsel  ),
                        .is_CNTinst    ((df_entry_q[p].di.csr_rdcnt != '0) && l_commit_o[p]),
                        .timer_64_value(dbg_timer_64_q  ),
                        .wen           (l_warid_o[p] == '0 ? '0 : l_commit_o[p]),
                        .wdest         (l_warid_o[p]),
                        .wdata         (l_warid_o[p] == '0 ? '0 : l_data_o[p]),
                        .csr_rstat     (df_entry_q[p].di.csr_op_en && df_entry_q[p].csr_id[8:0] == `_CSR_ESTAT && l_commit_o[p]),
                        .csr_data      (l_data_o[p]),
                        .is_SC_W       (df_entry_q[p].di.llsc_inst && df_entry_q[p].di.mem_write && l_commit_o[p]),
                        .scw_llbit     (l_data_o[p][0])
                      );
                      DifftestLoadEvent DifftestLoadEvent_p (
                        .clock (clk),
                        .coreid(0),
                        .index (p),
                        .valid (l_commit_o[p] && df_entry_q[p].di.mem_read),
                        .paddr ({dbg_tlb_resp.value.ppn, df_entry_q[p].target_addr[11:0]}),
                        .vaddr (df_entry_q[p].target_addr)
                      );
        end
                  DifftestExcpEvent DifftestExcpEvent (
                    .clock     (clk                                       ),
                    .coreid    (0                                         ),
                    .excp_valid(dbg_int_inst | dbg_excp_inst              ),
                    // .excp_valid         ('0),
                    .eret      (l_commit_o[0] && df_entry_q[0].di.ertn_inst),
                    // .eret               ('0),
                    .intrNo    (csr_q.estat[12:2]),
                    .cause     (csr_q.estat[21:16]),
                    .exceptionPC(df_entry_q[0].pc), 
                    .exceptionInst(df_entry_q[0].di.inst)  
                  );
                
                  DifftestTrapEvent DifftestTrapEvent (
                    .clock   (clk       ),
                    .coreid  (0         ),
                    .valid   ('0/*TODO*/),
                    .code    ('0/*TODO*/),
                    .pc      ('0/*TODO*/),
                    .cycleCnt('0/*TODO*/),
                    .instrCnt('0/*TODO*/)
                  );

                  DifftestCSRRegState DifftestCSRRegState_inst (
                    .clock    (clk                            ),
                    .coreid   (0                              ),
                    .crmd     (csr_q.crmd                     ),
                    .prmd     (csr_q.prmd                     ),
                    .euen     (csr_q.euen                     ),
                    .ecfg     (csr_q.ectl                     ),
                    .estat    (csr_q.estat                    ),
                    .era      (csr_q.era                      ),
                    .badv     (csr_q.badv                     ),
                    .eentry   (csr_q.eentry                   ),
                    .tlbidx   (csr_q.tlbidx                   ),
                    .tlbehi   (csr_q.tlbehi                   ),
                    .tlbelo0  (csr_q.tlbelo0                  ),
                    .tlbelo1  (csr_q.tlbelo1                  ),
                    .asid     (csr_q.asid                     ),
                    .pgdl     (csr_q.pgdl                     ),
                    .pgdh     (csr_q.pgdh                     ),
                    .save0    (csr_q.save0                    ),
                    .save1    (csr_q.save1                    ),
                    .save2    (csr_q.save2                    ),
                    .save3    (csr_q.save3                    ),
                    .tid      (csr_q.tid                      ),
                    .tcfg     (csr_q.tcfg                     ),
                    .tval     (csr_q.tval                     ),
                    .ticlr    (csr_q.ticlr                    ),
                    .tlbrentry(csr_q.tlbrentry                ),
                    .dmw0     (csr_q.dmw0                     ),
                    .dmw1     (csr_q.dmw1                     ),
                    .llbctl   ({csr_q.llbctl,1'b0,csr_q.llbit})
                  );
end

  // dbg 使用
  parameter string ColorTable[4] = {"\033[40;97m", "\033[41;97m", "\033[43;97m", "\033[44;97m"};
  parameter string ColorID = ColorTable[CPU_ID];
  always_ff @(posedge clk) begin
    // if(l_commit[0] && h_entry_q[0].di.llsc_inst && h_entry_q[0].di.mem_write) begin
    //     $display("%s[Core%d] %s with data %d\033[0m",ColorID, CPU_ID, l_data[0] ?
    //     "SUCC" : "FAIL", c_lsu_resp_i.wdata);
    // end
  end

endmodule
