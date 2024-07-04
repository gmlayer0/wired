`include "wired0_defines.svh"

module wired_fifo #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter bit BYPASS = 0,

    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,
    `_WIRED_INPORT_DEFINE(inport, T),
    `_WIRED_OUTPORT_DEFINE(outport, T)
);

    `_WIRED_HANDSHAKE_DEFINE_CONN(inport, outport, T)

    // push pop 逻辑
    wire push = inport_valid & inport_ready;
    wire pop = outport_valid & outport_ready;

    // 读写指针
    wire [ADDR_DEPTH-1:0] wptr, rptr;
    wire [ADDR_DEPTH:0] cnt;
    wire empty;

    // 指针更新
    `_WIRED_FF_RSTABLE_EN(wptr, '0, '1)
    `_WIRED_FF_RSTABLE_EN(rptr, '0, '1)
    `_WIRED_FF_RSTABLE_EN(cnt, '0, '1)
    `_WIRED_FF_RSTABLE_EN(empty, '0, '1)
    assign wptr = push ? (wptr_q + 1'd1) : wptr_q;
    assign rptr = pop  ? (rptr_q + 1'd1) : rptr_q;
    assign cnt  = cnt_q + (push ? 1'd1 : 1'd0) - (pop ? 1'd1 : 1'd0);
    assign empty = BYPASS && (cnt == '0);

    // 握手信号
    wire ready, valid;
    assign ready = cnt < DEPTH;
    assign valid = cnt > 1'd0;
    `_WIRED_FF_RSTABLE(ready, '1)
    `_WIRED_FF_RSTABLE(valid, '0)

    // mem 堆 -> lutram | distributed ram
    T [DEPTH - 1 : 0] mem;
    T data;
    assign data = ((cnt_q == 0 && push) || (cnt_q == 1 && push && pop)) ? inport_payload : mem[rptr];
    `_WIRED_FF(data)

    always_ff @(posedge clk) begin
        if(push) mem[wptr_q] <= inport_payload;
    end

    // 接线
    assign inport_ready = ready_q;
    assign outport_valid = empty_q ? inport_valid : valid_q;
    assign outport_payload = empty_q ? inport_payload : data_q;

endmodule
