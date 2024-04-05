`include "wired0_defines.svh"

module wired_ex_divider (
    `_WIRED_GENERAL_DEFINE,
    input  wire flush_i,

    input  wire valid_i,
    output wire ready_o,
    input  iq_mdu_req_t req_i,

    input  wire ready_i,
    output wire valid_o,
    output iq_mdu_resp_t resp_o
);

    logic valid_q;
    logic [1:0] op_q;
    rob_rid_t wid_q;
    logic busy;
    always_ff @(posedge clk) begin
        if(flush_i || !rst_n) begin
            valid_q <= '0;
        end else if(ready_o) begin
            valid_q <= valid_i;
        end
    end
    always_ff @(posedge clk) begin
        if(ready_o) begin
            op_q <= req_i.op[1:0];
            wid_q <= req_i.wid;
        end
    end
    logic [31:0] rem, quo;
    wired_div_fast divider_core (
        `_WIRED_GENERAL_CONN,
        .A(req_i.r1),
        .B(req_i.r0),
        .start(valid_i & ready_o),
        .sign(~req_i.op[0]),
        .busy(busy),
        .rem(rem),
        .quo(quo)
    );

    assign valid_o = valid_q & !busy;
    assign ready_o = ready_i || !valid_q;
    assign resp_o.wid = wid_q;
    assign resp_o.result = op_q[1] ? rem : quo;

endmodule