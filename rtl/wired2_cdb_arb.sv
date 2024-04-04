`include "wired0_defines.svh"

// Fuction module for Wired project
// Commit module, fetch all static and dynamic information about instruction execution.
// Futhermore, this module will change CSR / ARF / NPC status to commit instruction.
// This module can commit(retire) at most two inst / cycle.
// Because our CDB is 2-width, this is already enough.
module wired_cdb_arb #(
    parameter int CDB_PORT_CNT = 4
)(
    `_WIRED_GENERAL_DEFINE,
    // port 0 has the most significant pio than other.
    input  pipeline_cdb_t [CDB_PORT_CNT-1:0] cdb_i,
    output logic          [CDB_PORT_CNT-1:0] ready_o,
    output pipeline_cdb_t [1:0]              cdb_o
);

    // 注意， CDB ARB 输入上存在一个 SKID BUF，暂存握手失败的 cdb 请求等待重放。
    // 握手成功的请求在一个寄存器中，通过 cdb_o 输出给 ROB。

    // ROB 采用分 BANK 策略，因此万幸此模块仅需要做一个 bank split -> n-1 arb(fixed-pio) 即可。
    logic          [1:0][CDB_PORT_CNT-1:0] req_valid; // 输入的 bank split 有效，做了 skid
    pipeline_cdb_t [CDB_PORT_CNT-1:0] req_cdb;
    logic          [1:0][CDB_PORT_CNT-1:0] req_ready; // 输入的 bank split 被接受，实际 ready
    pipeline_cdb_t [1:0] sel_cdb;
    // 仲裁器
    for(genvar b = 0 ; b < 2 ; b += 1) begin
        always_comb begin
            req_ready[b] = '0;
            for(integer i = CDB_PORT_CNT - 1 ; i >= 0; i -= 1) begin
                if(req_valid[b][i]) begin
                    req_ready[b] = '0;
                    req_ready[b][i] = '1;
                end
            end
        end
    end
    for(genvar b = 0 ; b < 2 ; b += 1) begin
        always_comb begin
            sel_cdb[b] = '0;
            for(integer i = 0 ; i < CDB_PORT_CNT ; i += 1) begin
                sel_cdb[b] |= req_ready[b][i] ? req_cdb[i] : '0;
            end
        end 
    end
    always_ff @(posedge clk) begin
        cdb_o <= sel_cdb;
    end

    for(genvar i = 0 ; i < CDB_PORT_CNT ; i += 1) begin
        logic [1:0] bank_ready;
        for(genvar b = 0 ; b < 2 ; b += 1) begin
            wire req_valid_pre = cdb_i[i].valid && (cdb_i[i].wid[0] == b[0]);
            reg req_skid_q;
            always_ff @(posedge clk) begin
                if(!rst_n) begin
                    req_skid_q <= '0;
                end else begin
                    if(req_skid_q) begin
                        req_skid_q <= !req_ready[b][i];
                    end else begin
                        req_skid_q <= !req_ready[b][i] && req_valid_pre;
                    end
                end
            end
            assign req_valid[b][i] = req_valid_pre | req_skid_q; // 需要
            assign bank_ready[b] = !req_skid_q;
        end
        pipeline_cdb_t req_cdb_q;
        reg huge_skid_q;
        always_ff @(posedge clk) begin
            if(!rst_n) begin
                huge_skid_q <= '0;
            end else begin
                if(huge_skid_q) begin
                    huge_skid_q <= !req_ready[0][i] && !req_ready[1][i];
                end else begin
                    huge_skid_q <= !req_ready[0][i] && !req_ready[1][i] && cdb_i[i].valid;
                end
            end
        end
        always_ff @(posedge clk) begin
            if(!huge_skid_q) begin
                req_cdb_q <= cdb_i[i];
            end
        end
        assign req_cdb[i] = huge_skid_q ? req_cdb_q : cdb_i[i];
        assign ready_o[i] = !huge_skid_q;
    end

endmodule
