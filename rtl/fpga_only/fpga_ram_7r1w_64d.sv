// THIS MODULE IS FPGA ONLY
`include "wired0_defines.svh"
module fpga_ram_7r1w_64d#(
    parameter int WIDTH = 32
)( 
    input                          clk,
    input  [5 : 0] addr0,
    input  [5 : 0] addr1,
    input  [5 : 0] addr2,
    input  [5 : 0] addr3,
    input  [5 : 0] addr4,
    input  [5 : 0] addr5,
    input  [5 : 0] addr6,
    input  [5 : 0] addrw,
    output [WIDTH - 1:0]           dout0,
    output [WIDTH - 1:0]           dout1,
    output [WIDTH - 1:0]           dout2,
    output [WIDTH - 1:0]           dout3,
    output [WIDTH - 1:0]           dout4,
    output [WIDTH - 1:0]           dout5,
    output [WIDTH - 1:0]           dout6,
    input  [WIDTH - 1:0]           din,
    input                          wea
);

    for(genvar i = 0 ; i < WIDTH; i++) begin : gen_opram
        opram_64x1 qpram_inst(
            .CLK(clk),
            .CEN(1'b1), 
            .WEN(wea),
            .A0(addr0),
            .A1(addr1),
            .A2(addr2),
            .A3(addr3),
            .A4(addr4),
            .A5(addr5),
            .A6(addr6),
            .AW(addrw),
            .DI(  din[i]),
            .Q0(dout0[i]),
            .Q1(dout1[i]),
            .Q2(dout2[i]),
            .Q3(dout3[i]),
            .Q4(dout4[i]),
            .Q5(dout5[i]),
            .Q6(dout6[i])
        );
    end

endmodule