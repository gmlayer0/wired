`include "wired0_defines.svh"

// Fuction module for Wired project
// Pack Instruction in group
module wired_packer #(
    parameter int PKG_SIZE = 32
  )(
    `_WIRED_GENERAL_DEFINE,
    // 更新 tier id
    input flush_i,

    // 来自对齐解码级
    input  logic                     valid_i,
    output logic                     ready_o, //TODO 剩余指令数 <= 1 时前级可以就绪
    input  logic                [1:0]   nz_i,
    input  logic                [1:0] bank_i,
    input  logic  [1:0][PKG_SIZE-1:0]  pkg_i,
    input  logic                [1:0] mask_i,

    // 到 FIFO
    output logic                     valid_o, //TODO
    input  logic                     ready_i,
    output logic  [1:0][PKG_SIZE-1:0]  pkg_o,
    output logic                [1:0] mask_o  //TODO
  );

  wire [1:0] mask;
  assign mask[0] = mask_i[0] & valid_i;
  assign mask[1] = mask_i[1] & valid_i;
  logic skid_busy_q;
  logic skid_nz_q;
  logic skid_bank_q;
  logic[PKG_SIZE-1:0] skid_pkg_q;
  logic skid_en, skid_sel;
  logic skid_busy;
  always_ff @(posedge clk) begin
    if(!rst_n || flush_i) begin
      skid_busy_q <= '0;
    end else begin
      skid_busy_q <= skid_busy;
    end
  end
  always_ff @(posedge clk) begin
    if(skid_en) begin
      skid_nz_q   <= nz_i[skid_sel];
      skid_bank_q <= bank_i[skid_sel];
      skid_pkg_q  <= pkg_i[skid_sel];
    end
  end
  
  logic [1:0][1:0][PKG_SIZE-1:0] sel_pkg;
  assign sel_pkg[0] = {pkg_i[0], skid_pkg_q};
  assign sel_pkg[1] = {pkg_i[1], pkg_i[0]};
  assign pkg_o = skid_busy_q ? sel_pkg[0] : sel_pkg[1];

  wire [1:0] c;
`ifdef _WIRED_TDP_ARF
  assign c = '0;
`else
  assign c[0] = (skid_bank_q == bank_i[0]) && (skid_nz_q & nz_i[0]);
  assign c[1] = (bank_i[0] == bank_i[1]) && (nz_i[0] & nz_i[1]);
`endif

  assign valid_o = skid_busy_q | (|mask);

  always_comb begin
    ready_o = '0;
    skid_busy = skid_busy_q;
    skid_en = '0;
    skid_sel = '0;
    mask_o  = '0;
    // case ({ready_i, skid_busy_q, c, mask}) // LUT6
    if(!ready_i) begin
      if(!skid_busy_q) begin
        // FIFO 不就绪，没有未发射的指令，唯一可能导致状态变化的就是再来了一条指令，这时可以接受它，让它进 skidbuf。
        if(mask[0] & !mask[1]) begin
          ready_o = '1;
          skid_busy = '1;
          skid_en = '1;
        end
      end
    end else begin
      if(!skid_busy_q) begin
        // FIFO 就绪，没有未发射的指令
        // 此时，一定可以接受解码级过来的指令，拉高 ready
        ready_o = '1;
        // 若有进入 skid 的指令，一定是第二条
        skid_sel = '1;
        // 先假设任意指令都可以发射
        mask_o = mask;
        if(mask[1] & c[1]) begin
          // 只有一种特例不能发射，即第二条指令有效且冲突
          // 将其送入 skid buf，只发射第一条
          skid_busy = '1;
          skid_en = '1;
          mask_o[1] = '0;
        end
      end else begin
        // FIFO 就绪，且有未发射的指令
        // mask_o[0] 一定会被发射，就是目前 skid 中的指令
        mask_o = {mask[0], 1'b1};
        // 目前检查第一条指令是否存在。
        if(!mask[0]) begin
          // 不存在第一条指令，那么可以直接解除下周期的 SKID 状态
          skid_busy = '0;
        end else begin
          // 存在第一条指令，分类讨论。
          if(c[0]) begin
            // 第一条指令不能发射
            mask_o[1] = '0;
            if(!mask[1])begin
              // 只有 C0 一条指令，直接送入 SKID 中
              ready_o = '1;
              skid_en = '1;
            end else begin
              // 有两条指令，下周期再处理
              skid_busy = '0;
            end
          end else begin
            // 第一条指令可以发射
            ready_o = '1;
            if(mask[1]) begin
              // 有两条指令
              skid_sel = '1;
              skid_en = '1;
            end else begin
              // 只有这一条指令
              skid_busy = '0;
            end
          end
        end
      end
    end
    // endcase
  end

endmodule
