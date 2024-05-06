module opram_64x1(
    input  wire        CLK,
    input  wire        CEN, 
    input  wire        WEN,
    input  wire [5:0] A0,
    input  wire [5:0] A1,
    input  wire [5:0] A2,
    input  wire [5:0] A3,
    input  wire [5:0] A4,
    input  wire [5:0] A5,
    input  wire [5:0] A6,
    input  wire [5:0] AW,
    input  wire       DI,
    output wire       Q0,
    output wire       Q1,
    output wire       Q2,
    output wire       Q3,
    output wire       Q4,
    output wire       Q5,
    output wire       Q6
);

wire Q7; // Not used

`ifdef _FPGA 
RAM64M8 #(
   .INIT_A(64'h0000000000000000), // Initial contents of A Port
   .INIT_B(64'h0000000000000000), // Initial contents of B Port
   .INIT_C(64'h0000000000000000), // Initial contents of C Port
   .INIT_D(64'h0000000000000000), // Initial contents of D Port
   .INIT_E(64'h0000000000000000), // Initial contents of E Port
   .INIT_F(64'h0000000000000000), // Initial contents of F Port
   .INIT_G(64'h0000000000000000), // Initial contents of G Port
   .INIT_H(64'h0000000000000000), // Initial contents of H Port
   .IS_WCLK_INVERTED(1'b0)        // Specifies active high/low WCLK
) RAM64M8_inst (
   .DOA(Q0),     // Read port A 1-bit output
   .DOB(Q1),     // Read port B 1-bit output
   .DOC(Q2),     // Read port C 1-bit output
   .DOD(Q3),     // Read port D 1-bit output
   .DOE(Q4),     // Read port E 1-bit output
   .DOF(Q5),     // Read port F 1-bit output
   .DOG(Q6),     // Read port G 1-bit output
   .DOH(Q7),     // Read/write port H 1-bit output
   .DIA(DI),     // RAM 1-bit data write input addressed by ADDRD,read addressed by ADDRA
   .DIB(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRB
   .DIC(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRC
   .DID(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRD
   .DIE(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRE
   .DIF(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRF
   .DIG(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRG
   .DIH(DI),     // RAM 1-bit data write input addressed by ADDRD, read addressed by ADDRH
   .ADDRA(A0), // Read port A 6-bit address input
   .ADDRB(A1), // Read port B 6-bit address input
   .ADDRC(A2), // Read port C 6-bit address input
   .ADDRD(A3), // Read port D 6-bit address input
   .ADDRE(A4), // Read port E 6-bit address input
   .ADDRF(A5), // Read port F 6-bit address input
   .ADDRG(A6), // Read port G 6-bit address input
   .ADDRH(AW), // Read/write port H 6-bit address input
   .WE(WEN),       // Write enable input
   .WCLK(CLK)    // Write clock input
);
`endif

`ifndef _FPGA

    reg ram [63:0];
    assign Q0 = ram[A0];
    assign Q1 = ram[A1];
    assign Q2 = ram[A2];
    assign Q3 = ram[A3];
    assign Q4 = ram[A4];
    assign Q5 = ram[A5];
    assign Q6 = ram[A6];
    assign Q7 = ram[A7];
    always @(posedge CLK) begin
        if(WEN) begin
            ram[AW] <= DI;
        end
    end

`endif

endmodule
