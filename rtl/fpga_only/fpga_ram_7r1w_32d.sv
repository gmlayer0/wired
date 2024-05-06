// THIS MODULE IS FPGA ONLY
`include "wired0_defines.svh"
module fpga_ram_7r1w_32d#(
    parameter int WIDTH = 32
)( 
    input                          clk,
    input  [4 : 0] addr0,
    input  [4 : 0] addr1,
    input  [4 : 0] addr2,
    input  [4 : 0] addr3,
    input  [4 : 0] addr4,
    input  [4 : 0] addr5,
    input  [4 : 0] addr6,
    input  [4 : 0] addrw,
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

    for(genvar i = 0 ; i < WIDTH / 2; i++) begin : gen_opram
        opram_32x2 qpram_inst(
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
            .DI(  din[2 * i + 1 : 2 * i]),
            .Q0(dout0[2 * i + 1 : 2 * i]),
            .Q1(dout1[2 * i + 1 : 2 * i]),
            .Q2(dout2[2 * i + 1 : 2 * i]),
            .Q3(dout3[2 * i + 1 : 2 * i]),
            .Q4(dout4[2 * i + 1 : 2 * i]),
            .Q5(dout5[2 * i + 1 : 2 * i]),
            .Q6(dout6[2 * i + 1 : 2 * i])
        );
    end
    if(WIDTH % 2 != 0) begin : gen_addition_one
        opram_32x2 qpram_inst(
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
            .DI(  din[(2*(WIDTH/2))]),
            .Q0(dout0[(2*(WIDTH/2))]),
            .Q1(dout1[(2*(WIDTH/2))]),
            .Q2(dout2[(2*(WIDTH/2))]),
            .Q3(dout3[(2*(WIDTH/2))]),
            .Q4(dout4[(2*(WIDTH/2))]),
            .Q5(dout5[(2*(WIDTH/2))]),
            .Q6(dout6[(2*(WIDTH/2))])
        );
    end

endmodule