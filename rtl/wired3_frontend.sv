`include "wired0_defines"

// Fuction module for Wired project
// Frontend module
module wired_frontend #(
    parameter CPU_ID = 1'd0
  )(
    `_WIRED_GENERAL_DEFINE,

    // 连接到后端

    // 连接到内存总线（TILELINK-C）

  );

  logic w_skid_valid;
  logic w_skid_ready;
  logic skid_f_valid, skid_f_ready;
  typedef struct packed {
            logic        [31:0]      pc;
            logic         [1:0]    mask;
            bpu_predict_t [1:0] predict;
          } w_f_t;
  w_f_t w_skid;
  w_f_t skid_f;
  w_f_t w_f;

  wired_pcgen  wired_pcgen_inst (
                 `_WIRED_GENERAL_CONN,
                 .p_correct_i(p_correct_i),
                 .p_ready_i(w_skid_ready),
                 .p_valid_o(w_skid_valid),
                 .p_pc_o(w_skid.pc),
                 .p_mask_o(w_skid.mask),
                 .p_predict_o(w_skid.predict)
               );

  // w_f skid buf
  if( 1 )
  begin : W_F_SKIDBUF
    always_ff @(posedge clk)
    begin
      if(!rst_n)
      begin
        w_skid_ready <= '1;
      end
      else
      begin
        w_skid_ready <= !(w_skid_valid & !skid_f_ready);
      end
    end

    always_ff @(posedge clk)
    begin
      if(w_skid_ready)
      begin
        skid_f <= w_skid;
      end
    end

    assign w_f = w_skid_ready ? w_skid : skid_f;
  end

  // icache
  wired_icache # (
                 .PACKED_SIZE(2 * $bits(bpu_predict_t))
               )
               wired_icache_inst (
                 `_WIRED_GENERAL_CONN,
                 .f_valid_i(skid_f_valid),
                 .f_ready_o(skid_f_ready),
                 .f_mask_i(w_f.mask),
                 .f_pc_i(w_f.pc),
                 .f_pkg_i(w_f.predict),
                 .c_valid_i(),
                 .c_ready_o(),
                 .c_addr_i(),
                 .c_parm_i(),
                 .f_valid_o(),
                 .f_ready_i(),
                 .f_mask_o(),
                 .f_pc_o(),
                 .f_inst_o(),
                 .f_pkg_o(),
                 .bus_req_o(),
                 .bus_resp_i(),
                 .snoop_i(),
                 .csr_i(csr_i),
                 .tlb_update_i(tlb_update_i),
                 .p_addr_o(),
                 .p_rdata_i(),
                 .p_tag_i(),
                 .p_sll_i(),
                 .flush_i()
               );

  // MMIO FIFO
  

endmodule
