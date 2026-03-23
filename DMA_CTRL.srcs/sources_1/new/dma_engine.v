// ============================================================
// dma_engine.v — Multi-channel DMA engine
// Layer : Reusable IP (top của thư viện)
// `include: dma_defines.vh
//
// Đây là module bạn instantiate trong project.
// Layer 3 wrapper chỉ map port names và chọn parameters.
//
// Parameters — TẤT CẢ đều override được
//   N_CH         : Số kênh DMA (1..8)            [default 4]
//   N_CH_W       : $clog2(N_CH) — tính tự động   [KHÔNG override]
//   ADDR_W       : AXI địa chỉ                   [default 32]
//   LEN_W        : AXLEN bits                    [default 4]
//   ID_W         : AXI ID width (≥ N_CH_W+1)     [default 4]
//   FIFO_DEPTH   : FIFO depth mỗi kênh            [default 16]
//   MAX_BURST    : Max bytes/burst                [default 64]
//   BURST_W      : Bits field burst_max           [default 7]
//   TOKEN_W      : Token counter width            [default 4]
//   OUT_W        : Outstanding counter width      [default 4]
//   TIMEOUT_W    : Watchdog width                 [default 10]
//   PERIPH_NUM_W : Peripheral index width         [default 5]
//   LEN_FIELD_W  : Transfer length register       [default 16]
//   APB_ADDR_W   : APB paddr width                [default 13]
//
// APB register map
//   paddr[APB_ADDR_W-1 : `DMA_CH_STRIDE] = channel index
//   paddr[`DMA_CH_STRIDE-1 : 0]          = register offset (xem dma_defines.vh)
//   paddr = `REG_GLOBAL_CTRL             = 0xF00 (global control)
//
// AXI interface
//   1 shared AXI master port — round-robin giữa N_CH kênh
// ============================================================
`include "dma_define.vh"

module dma_engine #(
    parameter N_CH         = 4,
    parameter N_CH_W       = 2,    // PHẢI = $clog2(N_CH), tính thủ công vì Verilog-2001
    parameter ADDR_W       = 32,
    parameter LEN_W        = 4,
    parameter ID_W         = 4,
    parameter FIFO_DEPTH   = 16,
    parameter MAX_BURST    = 64,
    parameter BURST_W      = 7,
    parameter TOKEN_W      = 4,
    parameter OUT_W        = 4,
    parameter TIMEOUT_W    = 10,
    parameter PERIPH_NUM_W = 5,
    parameter LEN_FIELD_W  = 16,
    parameter APB_ADDR_W   = 13,
    // Giá trị default cho cfg_tokens và cfg_out_max của các kênh.
    // Phải là hằng số (không thể dùng expression trong Verilog-2001 port connection).
    // DEF_TOKENS ≤ 2^TOKEN_W - 1, DEF_OUTS ≤ 2^OUT_W - 1
    parameter DEF_TOKENS   = 4,    // default outstanding rd burst tokens/kênh
    parameter DEF_OUTS     = 4     // default max outstanding commands/kênh
) (
    input  wire              clk,
    input  wire              rst_n,

    //  APB Slave
    input  wire                    apb_psel,
    input  wire                    apb_penable,
    input  wire                    apb_pwrite,
    input  wire [APB_ADDR_W-1:0]   apb_paddr,
    input  wire [31:0]             apb_pwdata,
    output reg  [31:0]             apb_prdata,
    output wire                    apb_pslverr,
    output reg                     apb_pready,

    //  IRQ (1 bit per channel) 
    output wire [N_CH-1:0]  irq,

    //  Peripheral triggers 
    input  wire [31:1]       periph_req,
    output wire [31:1]       periph_clr,

    //  AXI4 Master — AR channel 
    output wire [ADDR_W-1:0]      ARADDR,
    output wire [LEN_W-1:0]       ARLEN,
    output wire [2:0]             ARSIZE,
    output wire [ID_W-1:0]        ARID,
    output wire                   ARVALID,
    input  wire                   ARREADY,

    //  AXI4 Master — R channel 
    input  wire [`AXI_DATA_W-1:0] RDATA,
    input  wire [ID_W-1:0]        RID,
    input  wire [1:0]             RRESP,
    input  wire                   RLAST,
    input  wire                   RVALID,
    output wire                   RREADY,

    //  AXI4 Master — AW channel 
    output wire [ADDR_W-1:0]      AWADDR,
    output wire [LEN_W-1:0]       AWLEN,
    output wire [2:0]             AWSIZE,
    output wire [ID_W-1:0]        AWID,
    output wire                   AWVALID,
    input  wire                   AWREADY,

    //  AXI4 Master — W channel 
    output wire [`AXI_DATA_W-1:0]  WDATA,
    output wire [`AXI_STRB_W-1:0]  WSTRB,
    output wire                    WLAST,
    output wire                    WVALID,
    input  wire                    WREADY,

    //  AXI4 Master — B channel 
    input  wire [ID_W-1:0]        BID,
    input  wire [1:0]             BRESP,
    input  wire                   BVALID,
    output wire                   BREADY
);

    // localparam & function
    // function phải khai báo TRƯỚC localparam dùng nó (Verilog-2001)
    function integer clog2_fn;
        input integer v;
        integer i;
        begin
            clog2_fn = 0;
            for (i = v-1; i > 0; i = i >> 1)
                clog2_fn = clog2_fn + 1;
        end
    endfunction

    localparam REG_PER_CH = (1 << `DMA_CH_STRIDE); // bytes/channel

    // CTRL register bit positions (dùng localparam thay vì hardcode số)
    // Layout: [0]=start [BURST_W:1]=burst_max [8]=src_incr [9]=dst_incr
    //         [9+PERIPH_NUM_W:10]=periph_num
    localparam CTRL_START_BIT  = 0;
    localparam CTRL_BMAX_LSB   = 1;
    localparam CTRL_BMAX_MSB   = BURST_W;           // = CTRL_BMAX_LSB + BURST_W - 1
    localparam CTRL_SI_BIT     = BURST_W + 1;        // = 8 khi BURST_W=7
    localparam CTRL_DI_BIT     = BURST_W + 2;        // = 9 khi BURST_W=7
    localparam CTRL_PNUM_LSB   = BURST_W + 3;        // = 10 khi BURST_W=7
    localparam CTRL_PNUM_MSB   = BURST_W + 2 + PERIPH_NUM_W; // = 14 khi BURST_W=7, PERIPH_NUM_W=5

    // STATUS register bit positions
    localparam STAT_ACTIVE_BIT = 0;
    localparam STAT_DONE_BIT   = 1;
    localparam STAT_ERR_LSB    = 2;
    localparam STAT_ERR_MSB    = 3;  // 2-bit err code
    // Per-channel registers
    reg [ADDR_W-1:0]      ch_src    [0:N_CH-1];
    reg [ADDR_W-1:0]      ch_dst    [0:N_CH-1];
    reg [LEN_FIELD_W-1:0] ch_len    [0:N_CH-1];
    reg [BURST_W-1:0]     ch_bmax   [0:N_CH-1];
    reg                   ch_si     [0:N_CH-1];   // src_incr
    reg                   ch_di     [0:N_CH-1];   // dst_incr
    reg [PERIPH_NUM_W-1:0]ch_pnum   [0:N_CH-1];   // periph_num
    reg                   ch_start_r[0:N_CH-1];   // 1-cycle pulse
    reg                   ch_int_en [0:N_CH-1];
    reg [1:0]             ch_int_st [0:N_CH-1];   // W1C: [0]=done [1]=err

    wire                  ch_active [0:N_CH-1];
    wire                  ch_done   [0:N_CH-1];
    wire [1:0]            ch_err    [0:N_CH-1];

    // APB decode
    wire apb_wr  = apb_psel & ~apb_penable & apb_pwrite;
    wire apb_rd  = apb_psel & ~apb_penable & ~apb_pwrite;

    // channel index = paddr[APB_ADDR_W-1:`DMA_CH_STRIDE]
    wire [N_CH_W-1:0] ch_idx = apb_paddr[N_CH_W+`DMA_CH_STRIDE-1:`DMA_CH_STRIDE];
    wire [7:0]        reg_off = apb_paddr[7:0];
    wire is_global = (apb_paddr[11:0] == `REG_GLOBAL_CTRL);
    localparam [N_CH_W:0] N_CH_CMP = N_CH;   // N_CH_W+1 bits, holds 0..N_CH safely
    wire is_ch_reg = ~is_global & (ch_idx < N_CH_CMP);
    reg soft_rst_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) soft_rst_r <= 1'b0;
        else if (apb_wr & is_global & apb_pwdata[0]) soft_rst_r <= 1'b1;
        else soft_rst_r <= 1'b0;   // auto-clear after 1 cycle
    end
    // Combine với hard rst_n: nếu một trong hai active → reset kênh
    wire ch_rst_n = rst_n & ~soft_rst_r;
    always @(posedge clk or negedge rst_n) begin
        apb_pready <= !rst_n ? 1'b0 : (apb_psel & apb_penable);
    end
    assign apb_pslverr = 1'b0;

    // APB write
    integer n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || soft_rst_r) begin
            // Hard reset OR soft reset: clear all channel control registers
            for (n = 0; n < N_CH; n = n + 1) begin
                ch_start_r[n] <= 1'b0;
                ch_int_st[n]  <= 2'b00;
            end
            // On hard reset only: also clear config registers
            if (!rst_n) begin
                for (n = 0; n < N_CH; n = n + 1) begin
                    ch_src[n]    <= {ADDR_W{1'b0}};
                    ch_dst[n]    <= {ADDR_W{1'b0}};
                    ch_len[n]    <= {LEN_FIELD_W{1'b0}};
                    ch_bmax[n]   <= MAX_BURST[BURST_W-1:0];
                    ch_si[n]     <= 1'b1;
                    ch_di[n]     <= 1'b1;
                    ch_pnum[n]   <= {PERIPH_NUM_W{1'b0}};
                    ch_int_en[n] <= 1'b1;
                end
            end
        end else begin
            // Clear start pulses mỗi cycle
            for (n = 0; n < N_CH; n = n + 1)
                ch_start_r[n] <= 1'b0;

            // Cập nhật interrupt status
            for (n = 0; n < N_CH; n = n + 1) begin
                if (ch_done[n] & ch_int_en[n])
                    ch_int_st[n][0] <= 1'b1;
                if (ch_done[n] & (ch_err[n] != `DMA_ERR_NONE) & ch_int_en[n])
                    ch_int_st[n][1] <= 1'b1;
            end

            // APB channel register writes
            if (apb_wr & is_ch_reg) begin
                case (reg_off)
                    `REG_SRC_ADDR: ch_src[ch_idx]  <= apb_pwdata[ADDR_W-1:0];
                    `REG_DST_ADDR: ch_dst[ch_idx]  <= apb_pwdata[ADDR_W-1:0];
                    `REG_LEN:      ch_len[ch_idx]  <= apb_pwdata[LEN_FIELD_W-1:0];
                    `REG_CTRL: begin
                        ch_start_r[ch_idx] <= apb_pwdata[CTRL_START_BIT];
                        ch_bmax[ch_idx]    <= apb_pwdata[CTRL_BMAX_MSB:CTRL_BMAX_LSB];
                        ch_si[ch_idx]      <= apb_pwdata[CTRL_SI_BIT];
                        ch_di[ch_idx]      <= apb_pwdata[CTRL_DI_BIT];
                        ch_pnum[ch_idx]    <= apb_pwdata[CTRL_PNUM_MSB:CTRL_PNUM_LSB];
                    end
                    `REG_INT_EN:   ch_int_en[ch_idx]  <= apb_pwdata[0];
                    `REG_INT_STAT: ch_int_st[ch_idx]  <= ch_int_st[ch_idx] & ~apb_pwdata[1:0];
                    default: ;
                endcase
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apb_prdata <= 32'h0;
        end else if (apb_rd) begin
            if (is_ch_reg) begin
                case (reg_off)
                    `REG_SRC_ADDR: apb_prdata <= {{(32-ADDR_W){1'b0}}, ch_src[ch_idx]};
                    `REG_DST_ADDR: apb_prdata <= {{(32-ADDR_W){1'b0}}, ch_dst[ch_idx]};
                    `REG_LEN:      apb_prdata <= {{(32-LEN_FIELD_W){1'b0}}, ch_len[ch_idx]};
                    `REG_CTRL:     apb_prdata <= {
                                       {(32-PERIPH_NUM_W-CTRL_PNUM_LSB){1'b0}},
                                       ch_pnum[ch_idx],
                                       ch_di[ch_idx],
                                       ch_si[ch_idx],
                                       ch_bmax[ch_idx],
                                       1'b0 };       // [0]=write-only start
                    // STATUS: [0]=active, [1]=done (latched = ch_int_st[0]),
                    //         [3:2]=err
                    // Dùng ch_int_st[n][0] thay vì ch_done (1-cycle pulse)
                    // để STATUS.done=1 bền vững cho đến khi SW W1C.
                    `REG_STATUS:   apb_prdata <= {
                                       {(32-STAT_ERR_MSB-1){1'b0}},
                                       ch_err[ch_idx],
                                       ch_int_st[ch_idx][0],
                                       ch_active[ch_idx] };
                    `REG_INT_EN:   apb_prdata <= {31'h0, ch_int_en[ch_idx]};
                    `REG_INT_STAT: apb_prdata <= {30'h0, ch_int_st[ch_idx]};
                    default:       apb_prdata <= 32'h0;
                endcase
            end else begin
                apb_prdata <= 32'h0;
            end
        end
    end

    // IRQ output
    genvar gi;
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_irq
            assign irq[gi] = |ch_int_st[gi];
        end
    endgenerate

    // RD cmd bus
    wire [N_CH-1:0]         ch_rd_cmd_valid;
    wire [N_CH-1:0]         ch_rd_cmd_ready;
    wire [N_CH*ADDR_W-1:0]  ch_rd_cmd_addr;
    wire [N_CH*LEN_W-1:0]   ch_rd_cmd_len;
    wire [N_CH*3-1:0]       ch_rd_cmd_size;
    wire [N_CH*ID_W-1:0]    ch_rd_cmd_id;

    // RD data bus
    wire [N_CH-1:0]               ch_rd_dat_valid;
    wire [N_CH-1:0]               ch_rd_dat_ready;
    wire [N_CH*`AXI_DATA_W-1:0]   ch_rd_dat_data;
    wire [N_CH-1:0]               ch_rd_dat_last;

    // RD response bus
    wire [N_CH-1:0]         ch_rd_rsp_valid;
    wire [N_CH*ID_W-1:0]    ch_rd_rsp_id;
    wire [N_CH*2-1:0]       ch_rd_rsp_err;

    // WR cmd bus
    wire [N_CH-1:0]         ch_wr_cmd_valid;
    wire [N_CH-1:0]         ch_wr_cmd_ready;
    wire [N_CH*ADDR_W-1:0]  ch_wr_cmd_addr;
    wire [N_CH*LEN_W-1:0]   ch_wr_cmd_len;
    wire [N_CH*3-1:0]       ch_wr_cmd_size;
    wire [N_CH*ID_W-1:0]    ch_wr_cmd_id;

    // WR data bus
    wire [N_CH-1:0]               ch_wr_dat_valid;
    wire [N_CH-1:0]               ch_wr_dat_ready;
    wire [N_CH*`AXI_DATA_W-1:0]   ch_wr_dat_data;

    // WR response bus
    wire [N_CH-1:0]         ch_wr_rsp_valid;
    wire [N_CH*ID_W-1:0]    ch_wr_rsp_id;
    wire [N_CH*2-1:0]       ch_wr_rsp_err;

    // Periph clr per channel
    wire [N_CH*31-1:0]      ch_periph_clr_bus;

    // Timeout per channel
    wire [N_CH-1:0]         ch_timeout_rd;
    wire [N_CH-1:0]         ch_timeout_aw;
    wire [N_CH-1:0]         ch_timeout_w;

    // Channel instances
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_ch
            dma_channel #(
                .ADDR_W       (ADDR_W),
                .LEN_W        (LEN_W),
                .ID_W         (ID_W),
                .N_CH_W       (N_CH_W),   // FIX ID: channel index bits
                .CH_IDX       (gi),        // FIX ID: this channel's index
                .FIFO_DEPTH   (FIFO_DEPTH),
                .MAX_BURST    (MAX_BURST),
                .BURST_W      (BURST_W),
                .TOKEN_W      (TOKEN_W),
                .OUT_W        (OUT_W),
                .TIMEOUT_W    (TIMEOUT_W),
                .PERIPH_NUM_W (PERIPH_NUM_W),
                .LEN_FIELD_W  (LEN_FIELD_W)
            ) u_ch (
                .clk            (clk),
                .rst_n          (ch_rst_n),   // soft reset support
                .cfg_src_addr   (ch_src[gi]),
                .cfg_dst_addr   (ch_dst[gi]),
                .cfg_len        (ch_len[gi]),
                .cfg_burst_max  (ch_bmax[gi]),
                .cfg_tokens     (DEF_TOKENS[TOKEN_W-1:0]),
                .cfg_rd_out_max (DEF_OUTS[OUT_W-1:0]),
                .cfg_wr_out_max (DEF_OUTS[OUT_W-1:0]),
                .cfg_src_incr   (ch_si[gi]),
                .cfg_dst_incr   (ch_di[gi]),
                .cfg_periph_num (ch_pnum[gi]),
                .start          (ch_start_r[gi]),
                .done           (ch_done[gi]),
                .active         (ch_active[gi]),
                .err            (ch_err[gi]),
                .periph_req     (periph_req),
                .periph_clr     (ch_periph_clr_bus[gi*31+30 : gi*31]),
                // RD cmd
                .rd_cmd_valid   (ch_rd_cmd_valid[gi]),
                .rd_cmd_ready   (ch_rd_cmd_ready[gi]),
                .rd_cmd_addr    (ch_rd_cmd_addr[gi*ADDR_W+:ADDR_W]),
                .rd_cmd_len     (ch_rd_cmd_len[gi*LEN_W+:LEN_W]),
                .rd_cmd_size    (ch_rd_cmd_size[gi*3+:3]),
                .rd_cmd_id      (ch_rd_cmd_id[gi*ID_W+:ID_W]),
                // RD data
                .rd_dat_valid   (ch_rd_dat_valid[gi]),
                .rd_dat_ready   (ch_rd_dat_ready[gi]),
                .rd_dat_data    (ch_rd_dat_data[gi*`AXI_DATA_W+:`AXI_DATA_W]),
                .rd_dat_last    (ch_rd_dat_last[gi]),
                // RD rsp
                .rd_rsp_valid   (ch_rd_rsp_valid[gi]),
                .rd_rsp_id      (ch_rd_rsp_id[gi*ID_W+:ID_W]),
                .rd_rsp_err     (ch_rd_rsp_err[gi*2+:2]),
                // WR cmd
                .wr_cmd_valid   (ch_wr_cmd_valid[gi]),
                .wr_cmd_ready   (ch_wr_cmd_ready[gi]),
                .wr_cmd_addr    (ch_wr_cmd_addr[gi*ADDR_W+:ADDR_W]),
                .wr_cmd_len     (ch_wr_cmd_len[gi*LEN_W+:LEN_W]),
                .wr_cmd_size    (ch_wr_cmd_size[gi*3+:3]),
                .wr_cmd_id      (ch_wr_cmd_id[gi*ID_W+:ID_W]),
                // WR data
                .wr_dat_valid   (ch_wr_dat_valid[gi]),
                .wr_dat_ready   (ch_wr_dat_ready[gi]),
                .wr_dat_data    (ch_wr_dat_data[gi*`AXI_DATA_W+:`AXI_DATA_W]),
                // WR rsp
                .wr_rsp_valid   (ch_wr_rsp_valid[gi]),
                .wr_rsp_id      (ch_wr_rsp_id[gi*ID_W+:ID_W]),
                .wr_rsp_err     (ch_wr_rsp_err[gi*2+:2]),
                // Timeouts
                .timeout_rd     (ch_timeout_rd[gi]),
                .timeout_wr_aw  (ch_timeout_aw[gi]),
                .timeout_wr_w   (ch_timeout_w[gi])
            );
        end
    endgenerate

    // periph_clr: OR của tất cả kênh
    // ch_periph_clr_bus là [N_CH*31-1:0], mỗi kênh chiếm 31 bits
    // periph_clr output là [31:1] (31 bits)
    integer pi;
    reg [30:0] pclr_or;   // 31 bits nội bộ (index 0..30 = periph 1..31)
    always @(*) begin
        pclr_or = 31'h0;
        for (pi = 0; pi < N_CH; pi = pi + 1)
            pclr_or = pclr_or | ch_periph_clr_bus[pi*31 +: 31];
    end
    assign periph_clr = pclr_or;

    // Round-robin arbiter — RD commands -> axi4_master_rd
    wire [N_CH-1:0] rd_grant;

    rr_arbiter #(.N(N_CH)) u_rd_arb (
        .clk   (clk),
        .rst_n (ch_rst_n),
        .req   (ch_rd_cmd_valid),
        .grant (rd_grant)
    );

    // Assign rd_cmd_ready dựa vào grant
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_rd_ready
            assign ch_rd_cmd_ready[gi] = rd_grant[gi] & rd_cmd_ready_top;
        end
    endgenerate

    // Mux AR signals từ kênh được grant
    wire [ADDR_W-1:0] mux_rd_addr;
    wire [LEN_W-1:0]  mux_rd_len;
    wire [2:0]        mux_rd_size;
    wire [ID_W-1:0]   mux_rd_id;

    onehot_mux #(.DATA_W(ADDR_W), .N(N_CH)) mux_araddr (
        .din(ch_rd_cmd_addr), .sel(rd_grant), .dout(mux_rd_addr));
    onehot_mux #(.DATA_W(LEN_W),  .N(N_CH)) mux_arlen (
        .din(ch_rd_cmd_len),  .sel(rd_grant), .dout(mux_rd_len));
    onehot_mux #(.DATA_W(3),      .N(N_CH)) mux_arsize (
        .din(ch_rd_cmd_size), .sel(rd_grant), .dout(mux_rd_size));
    onehot_mux #(.DATA_W(ID_W),   .N(N_CH)) mux_arid (
        .din(ch_rd_cmd_id),   .sel(rd_grant), .dout(mux_rd_id));

    wire rd_cmd_valid_top = |(rd_grant & ch_rd_cmd_valid);
    wire rd_cmd_ready_top;
    wire rd_rsp_valid_top;
    wire [ID_W-1:0] rd_rsp_id_top;
    wire [1:0]      rd_rsp_err_top;
    wire            rd_timeout_top;

    // RD data: broadcast đến kênh phù hợp theo RID[N_CH_W-1:0]
    wire                   rd_dout_valid;
    wire                   rd_dout_ready;
    wire [`AXI_DATA_W-1:0] rd_dout_data;
    wire                   rd_dout_last;
    wire [ID_W-1:0]        rd_dout_id;

    axi4_master_rd #(
        .ADDR_W    (ADDR_W),
        .ID_W      (ID_W),
        .LEN_W     (LEN_W),
        .CMD_DEPTH (N_CH),     // 1 slot/channel
        .TIMEOUT_W (TIMEOUT_W)
    ) u_rd_master (
        .clk         (clk),
        .rst_n       (ch_rst_n),   // soft reset clears cmd_fifo & AXI state
        .cmd_valid   (rd_cmd_valid_top),
        .cmd_ready   (rd_cmd_ready_top),
        .cmd_addr    (mux_rd_addr),
        .cmd_len     (mux_rd_len),
        .cmd_size    (mux_rd_size),
        .cmd_id      (mux_rd_id),
        .dout_valid  (rd_dout_valid),
        .dout_ready  (rd_dout_ready),
        .dout_data   (rd_dout_data),
        .dout_last   (rd_dout_last),
        .dout_id     (rd_dout_id),
        .rsp_valid   (rd_rsp_valid_top),
        .rsp_id      (rd_rsp_id_top),
        .rsp_err     (rd_rsp_err_top),
        .timeout_out (rd_timeout_top),
        .ARADDR      (ARADDR),
        .ARLEN       (ARLEN),
        .ARSIZE      (ARSIZE),
        .ARID        (ARID),
        .ARVALID     (ARVALID),
        .ARREADY     (ARREADY),
        .RDATA       (RDATA),
        .RID         (RID),
        .RRESP       (RRESP),
        .RLAST       (RLAST),
        .RVALID      (RVALID),
        .RREADY      (RREADY)
    );

    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_rd_route
            assign ch_rd_dat_valid[gi] =
                rd_dout_valid & (rd_dout_id[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_rd_dat_data[gi*`AXI_DATA_W+:`AXI_DATA_W] = rd_dout_data;
            assign ch_rd_dat_last[gi]  = rd_dout_last;

            assign ch_rd_rsp_valid[gi] =
                rd_rsp_valid_top & (rd_rsp_id_top[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_rd_rsp_id[gi*ID_W+:ID_W]   = rd_rsp_id_top;
            assign ch_rd_rsp_err[gi*2+:2]         = rd_rsp_err_top;
            assign ch_timeout_rd[gi] = rd_timeout_top &
                (ARVALID ? (ARID[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0])
                          : rd_grant[gi]);
        end
    endgenerate

    // rd_dout_ready: chỉ assert khi kênh ĐÍCH (khớp dout_id) có FIFO ready.
    // BUG FIX: dùng `|(valid & ready)` thay vì `|ready`.
    // Nếu dùng `|ch_rd_dat_ready`, RREADY=1 khi BẤT KỲ kênh nào ready —
    // ngay cả khi FIFO của kênh đích đầy → beat bị chấp nhận bởi AXI slave
    // nhưng KHÔNG được ghi vào FIFO đúng → mất data.
    assign rd_dout_ready = |(ch_rd_dat_valid & ch_rd_dat_ready);
    wire [N_CH-1:0] wr_grant_pulse;  // registered output từ rr_arbiter

    rr_arbiter #(.N(N_CH)) u_wr_arb (
        .clk   (clk),
        .rst_n (ch_rst_n),
        .req   (ch_wr_cmd_valid),
        .grant (wr_grant_pulse)
    );

    reg  [N_CH-1:0] wr_grant_r;
    wire            wr_cmd_fire_top;  // forward ref, driven after wr_cmd_ready_top

    always @(posedge clk or negedge ch_rst_n) begin
        if (!ch_rst_n) begin
            wr_grant_r <= {N_CH{1'b0}};
        end else begin
            if (wr_cmd_fire_top) begin
                // Handshake xong: advance đến kênh tiếp theo
                wr_grant_r <= wr_grant_pulse;
            end else if (wr_cmd_ready_top & ~|wr_dat_grant_r & ~wr_cmd_valid_top) begin
                wr_grant_r <= wr_grant_pulse;
            end
            // else: giữ nguyên (đang chờ AWREADY, hoặc kênh đang phát W data)
        end
    end

    wire [N_CH-1:0] wr_grant = wr_grant_r;

    // wr_burst_gate: cho phép AW mới chỉ khi không có W burst đang chạy,
    // HOẶC đúng cycle WLAST (pipeline). Dùng chung cho CẢ ch_wr_cmd_ready
    // VÀ wr_cmd_valid_gated để dma_channel và u_wr_master luôn đồng thuận.
    wire wr_burst_gate = ~|wr_dat_grant_r | wr_wlast_done;

    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_wr_ready
            assign ch_wr_cmd_ready[gi] = wr_grant[gi] & wr_cmd_ready_top & wr_burst_gate;
        end
    endgenerate

    wire [ADDR_W-1:0] mux_wr_addr;
    wire [LEN_W-1:0]  mux_wr_len;
    wire [2:0]        mux_wr_size;
    wire [ID_W-1:0]   mux_wr_id;

    onehot_mux #(.DATA_W(ADDR_W), .N(N_CH)) mux_awaddr (
        .din(ch_wr_cmd_addr), .sel(wr_grant), .dout(mux_wr_addr));
    onehot_mux #(.DATA_W(LEN_W),  .N(N_CH)) mux_awlen (
        .din(ch_wr_cmd_len),  .sel(wr_grant), .dout(mux_wr_len));
    onehot_mux #(.DATA_W(3),      .N(N_CH)) mux_awsize (
        .din(ch_wr_cmd_size), .sel(wr_grant), .dout(mux_wr_size));
    onehot_mux #(.DATA_W(ID_W),   .N(N_CH)) mux_awid (
        .din(ch_wr_cmd_id),   .sel(wr_grant), .dout(mux_wr_id));
    reg  [N_CH-1:0] wr_dat_grant_r;
    wire            wr_wlast_done = WLAST & WVALID & WREADY;
    wire            aw_accepted   = AWVALID & AWREADY;

    wire [N_CH-1:0] aw_ch_onehot;
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_aw_ch_dec
            assign aw_ch_onehot[gi] =
                (AWID[ID_W-1 : ID_W-N_CH_W] == gi[N_CH_W-1:0]);
        end
    endgenerate

    always @(posedge clk or negedge ch_rst_n) begin
        if (!ch_rst_n) begin
            wr_dat_grant_r <= {N_CH{1'b0}};
        end else begin
            if (wr_wlast_done & wr_cmd_fire_top) begin
                // Pipeline: burst kết thúc, AW mới accepted cùng cycle
                wr_dat_grant_r <= wr_grant;
            end else if (wr_wlast_done) begin
                wr_dat_grant_r <= {N_CH{1'b0}};
            end else if (~|wr_dat_grant_r & wr_cmd_fire_top) begin
                wr_dat_grant_r <= wr_grant;
            end
            // else: giữ nguyên cho đến WLAST
        end
    end

    // Sel: W phase dùng wr_dat_grant_r; giữa các burst dùng wr_grant
    wire [N_CH-1:0] wr_dat_sel = (|wr_dat_grant_r) ? wr_dat_grant_r : wr_grant;
    // WR data mux: từ kênh đang active trong W phase
    wire [`AXI_DATA_W-1:0] mux_wr_data;
    onehot_mux #(.DATA_W(`AXI_DATA_W), .N(N_CH)) mux_wdata (
        .din(ch_wr_dat_data), .sel(wr_dat_sel), .dout(mux_wr_data));

    // Same fix as rd_cmd_valid_top: gate with actual channel valid.
    wire wr_cmd_valid_top = |(wr_grant & ch_wr_cmd_valid);
    wire wr_cmd_ready_top;
    wire wr_cmd_valid_gated = wr_cmd_valid_top & wr_burst_gate;

    // wr_cmd_fire_top: forward-declared above, driven here
    // Dùng wr_cmd_valid_gated thay vì wr_cmd_valid_top
    assign wr_cmd_fire_top = wr_cmd_valid_gated & wr_cmd_ready_top;
    wire wr_din_valid = |(ch_wr_dat_valid & wr_dat_sel);
    wire wr_din_ready;
    wire wr_rsp_valid_top;
    wire [ID_W-1:0] wr_rsp_id_top;
    wire [1:0]      wr_rsp_err_top;
    wire            wr_timeout_aw_top, wr_timeout_w_top;

    // wr_dat_ready: chỉ kênh được sticky grant nhận ready
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_wr_dat_ready
            assign ch_wr_dat_ready[gi] = wr_dat_sel[gi] & wr_din_ready;
        end
    endgenerate

    axi4_master_wr #(
        .ADDR_W    (ADDR_W),
        .ID_W      (ID_W),
        .LEN_W     (LEN_W),
        .CMD_DEPTH (N_CH),
        .TIMEOUT_W (TIMEOUT_W)
    ) u_wr_master (
        .clk         (clk),
        .rst_n       (ch_rst_n),   // soft reset clears internal state
        .cmd_valid   (wr_cmd_valid_gated),   // FIX: dùng gated version
        .cmd_ready   (wr_cmd_ready_top),
        .cmd_addr    (mux_wr_addr),
        .cmd_len     (mux_wr_len),
        .cmd_size    (mux_wr_size),
        .cmd_id      (mux_wr_id),
        .din_valid   (wr_din_valid),
        .din_ready   (wr_din_ready),
        .din_data    (mux_wr_data),
        .rsp_valid   (wr_rsp_valid_top),
        .rsp_id      (wr_rsp_id_top),
        .rsp_err     (wr_rsp_err_top),
        .timeout_aw  (wr_timeout_aw_top),
        .timeout_w   (wr_timeout_w_top),
        .AWADDR      (AWADDR),
        .AWLEN       (AWLEN),
        .AWSIZE      (AWSIZE),
        .AWID        (AWID),
        .AWVALID     (AWVALID),
        .AWREADY     (AWREADY),
        .WDATA       (WDATA),
        .WSTRB       (WSTRB),
        .WLAST       (WLAST),
        .WVALID      (WVALID),
        .WREADY      (WREADY),
        .BID         (BID),
        .BRESP       (BRESP),
        .BVALID      (BVALID),
        .BREADY      (BREADY)
    );

    // Route WR rsp về đúng kênh
    // FIX ID routing: dùng N_CH_W bit CAO của BID
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_wr_rsp_route
            assign ch_wr_rsp_valid[gi] =
                wr_rsp_valid_top & (wr_rsp_id_top[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_wr_rsp_id[gi*ID_W+:ID_W] = wr_rsp_id_top;
            assign ch_wr_rsp_err[gi*2+:2]       = wr_rsp_err_top;
            assign ch_timeout_aw[gi] = wr_timeout_aw_top &
                                       (AWID[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_timeout_w[gi]  = wr_timeout_w_top & wr_dat_grant_r[gi];
        end
    endgenerate

endmodule