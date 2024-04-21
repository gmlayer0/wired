`include "wired0_defines.svh"

// Fuction module for Wired project
// Single issue queue entry

module wired_iq_entry_static #(
    parameter int PAYLOAD_SIZE = 32
)(
    `_WIRED_GENERAL_DEFINE,

    input logic sel_i,     // 指令被发射标记
    input logic updata_i,  // 新的指令加入标记
    input logic [PAYLOAD_SIZE-1:0] payload_i, // 新指令的控制数据


    output logic [PAYLOAD_SIZE-1:0] payload_o,
    output logic  empty_o  // IQ 项目有效
);

    // 标记 IQ Entry 中存储的是一条有效的指令
    logic valid_inst_q, empty_inst_q;
    logic [PAYLOAD_SIZE-1:0] payload_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            valid_inst_q <= '0;
            empty_inst_q <= '1;
        end else begin
            if(updata_i) begin
                valid_inst_q <= '1;
                empty_inst_q <= '0;
            end else if(sel_i) begin
                valid_inst_q <= '0;
                empty_inst_q <= '1;
            end
        end
    end
    always_ff @(posedge clk) begin
        if(updata_i) begin
            payload_q <= payload_i;
        end
    end
    assign empty_o = empty_inst_q; // OK
    assign payload_o = payload_q;  // OK

endmodule
