`include "axi_util.svh"

module core_top
  (
    input           aclk,
    input           aresetn,
    input    [ 7:0] intrpt,
    //AXI interface
    //read reqest
    output   [ 3:0] arid,
    output   [31:0] araddr,
    output   [ 7:0] arlen,
    output   [ 2:0] arsize,
    output   [ 1:0] arburst,
    output   [ 1:0] arlock,
    output   [ 3:0] arcache,
    output   [ 2:0] arprot,
    output          arvalid,
    input           arready,
    //read back
    input    [ 3:0] rid,
    input    [31:0] rdata,
    input    [ 1:0] rresp,
    input           rlast,
    input           rvalid,
    output          rready,
    //write request
    output   [ 3:0] awid,
    output   [31:0] awaddr,
    output   [ 7:0] awlen,
    output   [ 2:0] awsize,
    output   [ 1:0] awburst,
    output   [ 1:0] awlock,
    output   [ 3:0] awcache,
    output   [ 2:0] awprot,
    output          awvalid,
    input           awready,
    //write data
    output   [ 3:0] wid,
    output   [31:0] wdata,
    output   [ 3:0] wstrb,
    output          wlast,
    output          wvalid,
    input           wready,
    //write back
    input    [ 3:0] bid,
    input    [ 1:0] bresp,
    input           bvalid,
    output          bready,

    output [31:0] debug0_wb_pc,
    output [ 3:0] debug0_wb_rf_wen,
    output [ 4:0] debug0_wb_rf_wnum,
    output [31:0] debug0_wb_rf_wdata,
    output [31:0] debug1_wb_pc,
    output [ 3:0] debug1_wb_rf_wen,
    output [ 4:0] debug1_wb_rf_wnum,
    output [31:0] debug1_wb_rf_wdata
  );
  reg         resetn;
  always @(posedge aclk) resetn <= aresetn;

  `AXI_DECLARE(32, 32, 4, mem);

  wired_sp cpu(
             .clk(aclk),
             .rst_n(resetn),
             .interrupt_i(intrpt),
             `AXI_CONNECT_HOST_PORT(mem, mem)
           );
  assign arid = mem_ar.id;
  assign araddr = mem_ar.addr;
  assign arlen = mem_ar.len;
  assign arsize = mem_ar.size;
  assign arburst = mem_ar.burst;
  assign arlock = mem_ar.lock;
  assign arcache = mem_ar.cache;
  assign arprot = mem_ar.prot;
  assign awid = mem_aw.id;
  assign awaddr = mem_aw.addr;
  assign awlen = mem_aw.len;
  assign awsize = mem_aw.size;
  assign awburst = mem_aw.burst;
  assign awlock = mem_aw.lock;
  assign awcache = mem_aw.cache;
  assign awprot = mem_aw.prot;
  reg [3:0] mem_aw_id_q;
  always_ff @(posedge aclk) if(mem_aw_valid) mem_aw_id_q <= mem_aw.id;
  assign wid =   mem_aw_valid ? mem_aw.id : mem_aw_id_q;
  assign wdata = mem_w.data;
  assign wstrb = mem_w.strb;
  assign wlast = mem_w.last;

  assign mem_r.id = rid;
  assign mem_r.data = rdata;
  assign mem_r.resp = rresp;
  assign mem_r.last = rlast;
  assign mem_b.id = bid;
  assign mem_b.resp = bresp;
  assign arvalid = mem_ar_valid;
  assign mem_ar_ready = arready;
  assign mem_r_valid = rvalid;
  assign rready = mem_r_ready;
  assign awvalid = mem_aw_valid;
  assign mem_aw_ready = awready;
  assign wvalid = mem_w_valid;
  assign mem_w_ready = wready;
  assign mem_b_valid = bvalid;
  assign bready = mem_b_ready;


  assign debug0_wb_pc = 0;
  assign debug0_wb_rf_wen = 0;
  assign debug0_wb_rf_wnum = 0;
  assign debug0_wb_rf_wdata = 0;
  assign debug1_wb_pc = 0;
  assign debug1_wb_rf_wen = 0;
  assign debug1_wb_rf_wnum = 0;
  assign debug1_wb_rf_wdata = 0;

endmodule
