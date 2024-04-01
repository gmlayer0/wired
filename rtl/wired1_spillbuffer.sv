`include "wired0_defines.svh"

module wired_spillbuffer #(
    parameter int DATA_WIDTH = 32,
    parameter type T = logic[DATA_WIDTH - 1 : 0]
)(
    `_WIRED_GENERAL_DEFINE,
    `_WIRED_INPORT_DEFINE(inport, T),
    `_WIRED_OUTPORT_DEFINE(outport, T)
);

    `_WIRED_HANDSHAKE_DEFINE_CONN(inport, outport, T)

    // ready 逻辑
    reg spilled_ready_q;
    always_ff @(posedge clk) begin
        if(!rst_n) spilled_ready_q <= '1;
        else spilled_ready_q <= (!inport_valid & spilled_ready_q) | outport_ready;
    end

    // payload 逻辑
    `_WIRED_FF_EN(inport_payload, spilled_ready_q)

    // 输出逻辑
    assign inport_ready = spilled_ready_q;
    assign outport_payload = spilled_ready_q ? inport_payload : inport_payload_q;
    assign outport_valid = inport_valid | !spilled_ready_q;

endmodule
