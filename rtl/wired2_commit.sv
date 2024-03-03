`include "wired0_defines"

// Fuction module for Wired project
// Commit module, fetch all static and dynamic information about instruction execution.
// Futhermore, this module will change CSR / ARF / NPC status to commit instruction.
// This module can commit(retire) at most two inst / cycle.
// Because our CDB is 2-width, this is already enough.
module wired_commit #(
    parameter int INST_PAYLOAD_SIZE = 128
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

    // Part 5: flush 端口，刷新流水线用
    output      logic              l_flush_o,
    input       logic              rename_empty_i, // RENAME 级清空，说明 ROB 均已退休，ROB 空，可以继续。

    // Part 6: CSR 及地址翻译相关控制接口
    // 注意，提交模块分为三级，分别是 Fetch ROB(F) | Handle(H) | Commit(C)
);

    // 第一级流水，F，需要判定是否可双提交。
    // 对于可能需要状态机处理或者修改 CSR 、产生跳转、刷新流水线效果的指令，仅允许在 SLOT0 提交。
    // 其它指令则无所谓
    rob_rid_t f_rob_ptr0_q;
    rob_rid_t f_rob_ptr1_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            f_rob_ptr0_q <= `_WIRED_PARAM_ROB_LENd'0;
            f_rob_ptr1_q <= `_WIRED_PARAM_ROB_LENd'1;
        end else if(f_skid_ready_q) begin
            f_rob_ptr0_q <= f_rob_ptr0_q + f_valid[0] + f_valid[1];
            f_rob_ptr1_q <= f_rob_ptr1_q + f_valid[0] + f_valid[1];
        end
    end
    reg f_skid_ready_q;
    rob_entry_t [1:0] f_skid_entry_q;
    rob_rid_t [1:0] f_skid_wrrid_q;
    wire slot1_bank_conflict = c_rob_entry_i[1].wreg[0] == c_rob_entry_i[0].wreg[0];
    wire slot1_cannot_fire = c_rob_entry_i[1].decode_info.slot0 || slot1_bank_conflict1;
    wire [1:0] f_valid = c_rob_valid_i & {~slot1_cannot_fire, 1'd1};
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
    // 组合逻辑读取 CSR
    always_comb begin
        h_csr_rdata = '0;
        case(h_entry[0].csr_id[8:0])
        `_CSR_CRMD      h_csr_rdata = csr_q.crmd;
        `_CSR_PRMD      h_csr_rdata = csr_q.prmd;
        `_CSR_EUEN      h_csr_rdata = csr_q.euen;
        `_CSR_ECTL      h_csr_rdata = csr_q.ectl;
        `_CSR_ESTAT     h_csr_rdata = csr_q.estat;
        `_CSR_ERA       h_csr_rdata = csr_q.era;
        `_CSR_BADV      h_csr_rdata = csr_q.badv;
        `_CSR_EENTRY    h_csr_rdata = csr_q.eentry;
        `_CSR_TLBIDX    h_csr_rdata = csr_q.tlbidx;
        `_CSR_TLBEHI    h_csr_rdata = csr_q.tlbehi;
        `_CSR_TLBELO0   h_csr_rdata = csr_q.tlbelo0;
        `_CSR_TLBELO1   h_csr_rdata = csr_q.tlbelo1;
        `_CSR_ASID      h_csr_rdata = csr_q.asid;
        `_CSR_PGDL      h_csr_rdata = csr_q.pgdl;
        `_CSR_PGDH      h_csr_rdata = csr_q.pgdh;
        `_CSR_PGD       h_csr_rdata = {csr_q.badv[31] ? csr_q.pgdh[31:12] : csr_q.pgdl[31:12] , 12'd0};
        `_CSR_CPUID     h_csr_rdata = csr_q.cpuid;
        `_CSR_SAVE0     h_csr_rdata = csr_q.save0;
        `_CSR_SAVE1     h_csr_rdata = csr_q.save1;
        `_CSR_SAVE2     h_csr_rdata = csr_q.save2;
        `_CSR_SAVE3     h_csr_rdata = csr_q.save3;
        `_CSR_TID       h_csr_rdata = csr_q.tid;
        `_CSR_TCFG      h_csr_rdata = csr_q.tcfg;
        `_CSR_TVAL      h_csr_rdata = csr_q.tval;
        `_CSR_TICLR     h_csr_rdata = csr_q.ticlr;
        `_CSR_LLBCTL    h_csr_rdata = {csr_q.llbctl, 1'b0, csr_q.llbit};
        `_CSR_TLBRENTRY h_csr_rdata = csr_q.tlbrentry;
        `_CSR_DMW0      h_csr_rdata = csr_q.dmw0;
        `_CSR_DMW1      h_csr_rdata = csr_q.dmw1;
        
        endcase
    end

    // 第二级流水（实质是一个状态机）
    logic [31:0] h_csr_rdata_q;
    rob_entry_t [1:0] h_entry_q;
    rob_rid_t [1:0] h_wrrid_q;
    logic [1:0]  h_valid_inst_q;
    rob_rid_t [1:0] h_wrrid_q;
    always_ff @(posedge clk) begin
        if(h_ready) begin
            h_entry_q <= h_entry;
            h_wrrid_q <= h_wrrid;
            h_csr_rdata_q <= h_csr_rdata;
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
    logic[63:0] timer_64_q;
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
    `define __FSM_BIT_LEN 5
    typedef logic[`__FSM_BIT_LEN-1:0] commit_fsm_t;
    parameter commit_fsm_t S_NORMAL      = `__FSM_BIT_LEN'd0;
    parameter commit_fsm_t S_WAIT_ULOAD  = `__FSM_BIT_LEN'd1; // 这个要刷流水线，还要修改写入 ARF 的数据
    parameter commit_fsm_t S_WAIT_USTORE = `__FSM_BIT_LEN'd2; // 这个不用刷流水线，但需要解除 dbar（uncached 会设置 barrier）
    parameter commit_fsm_t S_WAIT_MSTORE = `__FSM_BIT_LEN'd3; // 这个不用刷流水线
    parameter commit_fsm_t S_WAIT_FLUSH  = `__FSM_BIT_LEN'd4; // 这个是用来刷流水线的
    `undef __FSM_BIT_LEN
    commit_fsm_t fsm_q;
    commit_fsm_t fsm;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            fsm_q <= S_NORMAL;
        end else begin
            fsm_q <= fsm;
        end
    end

    // 获取指令 0 所属跳转目标类型
    // 0: 无跳转
    // 1: 函数调用；2: 函数返回；3: 调用栈无关类
    logic [1:0] slot0_target_type;
    always_comb begin
        slot0_target_type = '0;
        if(h_entry_q[0].decode_info.jump_inst && h_entry_q[0].op_code[4]) begin // 1 -> CALL JIRL BL
            slot0_target_type = 2'd1;
        end else if(h_entry_q[0].decode_info.jump_inst && h_entry_q[0].op_code[3]) begin // 2 -> RETURN
            slot0_target_type = 2'd2;
        end else if(h_entry_q[0].decode_info.jump_inst) begin  // 3 -> IMM
            slot0_target_type = 2'd3;
        end
    end

    // CSR 写辅助指令
    logic [31:0] csr_wmask, csr_wdata;

    // 主状态机组合逻辑
    excp_t [1:0] excp;
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        assign excp[i] = gather_excp(h_entry_q[i].static_excp, h_entry_q[i].lsu_excp);
    end
    // SUPER SUPER HUGE LOGIC BEGIN !
    // MAIN PROCESSOR STATES ARE MAINTAINED HERE !
    always_comb begin
        c_lsu_req_o = '0;
        csr = csr_q;
        fsm = fsm_q;
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
        f_upd.pc = h_entry_q[0].pc;
        f_upd.true_taken = slot0_target_type;
        f_upd.true_target = h_entry_q[0].target_addr;
        f_upd.lphr = h_entry_q[0].bpu_predict.lphr;
        f_upd.history = h_entry_q[0].bpu_predict.history;
        f_upd.true_target_type = slot0_target_type;
        f_upd.true_conditional_jmp = (|h_entry_q[0].decode_info.cmp_type[3:1]) && !(&h_entry_q[0].decode_info.cmp_type[3:1]);
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
        csr_wdata = (csr_wmask & h_entry_q[0].wdata) | ((~csr_wmask) & h_csr_rdata_q);
        case (fsm_q)
            S_NORMAL: begin
                f_upd.need_update = h_valid_inst_q[0] && ((|slot0_target_type) || (slot0_target_type != h_entry_q[0].bpu_predict.target_type));
                l_flush = '0;
                l_retire = h_valid_inst_q;
                l_commit = h_valid_inst_q;
                // 对 inst 的中断异常情况做 judge
                if(h_valid_inst_q[0] && h_entry_q[0].excp_found || h_valid_inst_q[1] && h_entry_q[1].excp_found) begin
                    for(integer i = 1 ; i >= 0 ; i --) begin
                        if(h_valid_inst_q[i] && h_entry_q[i].excp_found) begin
                            // 第 i 条指令有问题
                            l_commit[1] = '0;
                            l_commit[i] = '0;
                            f_upd.redirect = '1;
                            f_upd.true_target = csr_q.eentry;
                            fsm = S_WAIT_FLUSH;
                            if(i != 1 || (!|excp[0])) begin
                                // 对于 0 一定可以修改，对于1，一定在 0 无异常时可以修改
                                // 先更新 ecode
                                case(1'b1)
                                    excp[i].fetch_int: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h00;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                    end      // None Masked Interruption founded, if founded, this instruction is forced to issue in ALU slot
                                    excp[i].adef: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h08;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].pc;
                                    end
                                    excp[i].itlbr: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h3f;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.crmd[`_CRMD_DA] = 1'b0;
                                        csr.crmd[`_CRMD_PG] = 1'b1;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].pc;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                        f_upd.true_target = csr_q.tlbrentry;
                                    end
                                    excp[i].pif: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h03;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].pc;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                    end
                                    excp[i].ippi: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h07;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].pc;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                    end
                                    excp[i].ine: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h0d;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                    end
                                    excp[i].ipe: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h0e;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                    end
                                    excp[i].sys: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h0b;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                    end
                                    excp[i].brk: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h0c;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                    end
                                    excp[i].ale: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h09;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].target_addr;
                                    end
                                    excp[i].tlbr: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h3f;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.crmd[`_CRMD_DA] = 1'b0;
                                        csr.crmd[`_CRMD_PG] = 1'b1;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].target_addr;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                        f_upd.true_target = csr_q.tlbrentry;
                                    end
                                    excp[i].pis: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h02;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].target_addr;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                    end
                                    excp[i].pil: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h01;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].target_addr;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                    end
                                    excp[i].ppi: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h07;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].target_addr;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                    end
                                    excp[i].pme: begin
                                        csr.estat[`_ESTAT_ECODE] = 6'h04;
                                        csr.estat[`_ESTAT_ESUBCODE] = '0;
                                        csr.era = h_entry_q[i].pc;
                                        csr.crmd[`_CRMD_PLV] = 2'b0;
                                        csr.crmd[`_CRMD_IE] = 1'b0;
                                        csr.prmd[2:0] = csr_q.crmd[2:0];
                                        csr.badv = h_entry_q[i].target_addr;
                                        csr.tlbehi[`_TLBEHI_VPPN] = h_entry_q[i].pc[`_TLBEHI_VPPN];
                                    end
                                endcase
                            end
                        end
                    end
                end else if(h_valid_inst_q[0]) begin
                    // 不存在异常的部分
                    // 0. 恢复 DBAR
                    if(h_entry_q[0].decode_info.dbarrier) begin
                        // 可能进入这里的，一定不会暂停
                        c_lsu_req_o.dbarrier_unlock = '1;
                    end
                    // 访存部分处理
                    if(h_entry_q[0].lsu_inst) begin
                        if(h_entry_q[0].uncached) begin
                            // 1. Uncached 请求（向 LSU 发出请求，等待 Uncached 读数据 / 写完成）
                            // （注意， Uncached Load 及 Uncached Store 均需要立即发出请求，且 Uncached Load 需要刷新流水线）
                            h_ready = '0;
                            if(h_entry_q[0].decode_info.mem_write) begin
                                fsm = S_WAIT_USTORE;
                            end else begin
                                fsm = S_WAIT_ULOAD;
                            end
                        end else begin
                            if(h_entry_q[0].decode_info.mem_write) begin
                                // 3. Store Conditional 请求，不再等待 Cache 重填，如果不可执行，则直接刷新流水线
                                if(h_entry_q[0].decode_info.llsc_inst) begin
                                    if((!c_lsu_resp_i.storebuf_hit) || (!csr_q.llbit)) begin // 其它 Cache Coherent Master probe 了这行，不再原子
                                        l_commit = 2'b01;
                                        l_data[0] = 32'd0;
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
                                        fsm = S_WAIT_USTORE;
                                    end else begin
                                        c_lsu_req_o.storebuf_commit = '1;
                                    end
                                end
                            end
                        end
                    end
                    // 跳转指令处理，注意刷新管线
                    if(h_entry_q[0].jump_inst) begin
                        // 特殊处理未命中情况，刷新流水线，重定向控制流
                        if( (h_entry_q[0].need_jump && (!h_entry_q[0].bpu_predict.taken || h_entry_q[0].bpu_predict.predict_pc != h_entry_q[0].target_addr)) ||
                        ||  (!h_entry_q[0].need_jump && h_entry_q[0].bpu_predict.taken)) begin
                            l_commit = 2'b01;
                            f_upd.miss = '1;
                            f_upd.redirect = '1;
                            fsm = S_WAIT_FLUSH;
                        end
                    end
                    
                    // CSR 读写指令处理，注意刷新管线
                    // 所有指令的读取实际在 ALU 中已经完成了，这里只需要检查读结果是否有效并写入，合适的刷新管线即可
                    // 所有对 CSR 产生写操作（状态改变）的指令都需要刷新管线。
                    // 其实只有一条指令，csrwrxchg，操作是根据掩码写寄存器之后再读出
`define _MW(csr_name, mask) csr.``csr_name``[mask] = csr_wdata[mask]
                    if(h_entry_q[0].decode_info.csr_op_en) begin
                        l_commit = 2'b01;
                        f_upd.redirect = '1;
                        fsm = S_WAIT_FLUSH;
                        case(h_entry_q[0].csr_id[8:0])
                        `_CSR_CRMD:      begin _MW(crmd, `_CRMD_PLV);_MW(crmd, `_CRMD_IE);_MW(crmd, `_CRMD_DA);_MW(crmd, `_CRMD_PG);_MW(crmd, `_CRMD_DATF);_MW(crmd, `_CRMD_DATM); end
                        `_CSR_PRMD:      begin _MW(prmd, `_PRMD_PPLV);_MW(prmd, `_PRMD_PIE); end
                        `_CSR_EUEN:      begin _MW(euen, `_EUEN_FPE); end
                        `_CSR_ECTL:      begin _MW(ectl, `_ECTL_LIE1);_MW(ectl, `_ECTL_LIE2); end
                        // `_CSR_ESTAT:     begin _MW(); end // TODO: FIXME: WRITE TO FRONTEND, NOT DIRECTLY ESTAT
                        `_CSR_ERA:       begin _MW(era, 31:0); end
                        `_CSR_BADV:      begin _MW(badv, 31:0); end
                        `_CSR_EENTRY:    begin _MW(eentry, `_EENTRY_VA); end
                        `_CSR_TLBIDX:    begin _MW(tlbidx, `_TLBIDX_INDEX);_MW(tlbidx, `_TLBIDX_PS);_MW(tlbidx, `_TLBIDX_NE); end
                        `_CSR_TLBEHI:    begin _MW(tlbehi, `_TLBEHI_VPPN); end
                        `_CSR_TLBELO0:   begin _MW(tlbelo0, 31:8);_MW(tlbelo0, 6:0); end
                        `_CSR_TLBELO1:   begin _MW(tlbelo1, 31:8);_MW(tlbelo1, 6:0); end
                        `_CSR_ASID:      begin _MW(asid, `_ASID); end
                        `_CSR_PGDL:      begin _MW(pgdl, `_PGD_BASE); end
                        `_CSR_PGDH:      begin _MW(pgdh, `_PGD_BASE); end
                        // `_CSR_PGD:       begin _MW(); end // 只读
                        // `_CSR_CPUID:     begin _MW(); end // 只读
                        `_CSR_SAVE0:     begin _MW(save0, 31:0); end
                        `_CSR_SAVE1:     begin _MW(save1, 31:0); end
                        `_CSR_SAVE2:     begin _MW(save2, 31:0); end
                        `_CSR_SAVE3:     begin _MW(save3, 31:0); end
                        `_CSR_TID:       begin _MW(tid, 31:0); end
                        `_CSR_TCFG:      begin _MW(tcfg,31:0);csr.tval[1:0] = '0;_MW(tval,`_TCFG_INITVAL); end
                        // `_CSR_TVAL:      begin _MW(); end // 只读
                        // `_CSR_TICLR:     begin _MW(); end // TODO: FIXME: WRITE TO FRONTEND, NOT DIRECTLY ESTAT
                        `_CSR_LLBCTL:    begin _MW(llbctl, `_LLBCT_KLO); if(csr_wdata[`_LLBCT_WCLLB]) csr.llbit = 1'b0; end
                        `_CSR_TLBRENTRY: begin _MW(tlbrentry, `_TLBRENTRY_PA); end
                        `_CSR_DMW0:      begin _MW(dmw0,`_DMW_PLV0);_MW(dmw0,`_DMW_PLV3);_MW(dmw0,`_DMW_MAT);_MW(dmw0,`_DMW_PSEG);_MW(dmw0,`_DMW_VSEG); end
                        `_CSR_DMW1:      begin _MW(dmw1,`_DMW_PLV0);_MW(dmw1,`_DMW_PLV3);_MW(dmw1,`_DMW_MAT);_MW(dmw1,`_DMW_PSEG);_MW(dmw1,`_DMW_VSEG); end
                        endcase
                    end
                end
`undef _MW

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
        endcase
    end

endmodule
