// ============================================================
// axi4_master_wr.v — AXI4 Write Master (32-bit data bus)
// Layer : Reusable IP
// `include: dma_defines.vh
//
// Parameters
//   ADDR_W      : AXI address width               [default 32]
//   ID_W        : AXI ID width                    [default 4]
//   LEN_W       : AWLEN width                     [default 4]
//   CMD_DEPTH   : Outstanding burst FIFO depth    [default 4]
//   TIMEOUT_W   : Watchdog bit width              [default 10]
//
// Upstream — burst descriptor (valid/ready)
//   cmd_valid, cmd_ready
//   cmd_addr, cmd_len, cmd_size, cmd_id
//
// Upstream — data stream (valid/ready)
//   din_valid, din_ready
//   din_data [`AXI_DATA_W-1:0]
//   din_last  (optional: module tự tính từ AWLEN nếu din_last=0)
//
// Response
//   rsp_valid, rsp_id, rsp_err
//
// Timeout
//   timeout_aw : AW channel watchdog
//   timeout_w  : W channel watchdog
// ============================================================
`include "dma_defines.vh"

module axi4_master_wr #(
    parameter ADDR_W    = 32,
    parameter ID_W      = 4,
    parameter LEN_W     = 4,
    parameter CMD_DEPTH = 4,
    parameter TIMEOUT_W = 10
) (
    input  wire              clk,
    input  wire              rst_n,

    //  Upstream: burst descriptor
    input  wire              cmd_valid,
    output wire              cmd_ready,
    input  wire [ADDR_W-1:0] cmd_addr,
    input  wire [LEN_W-1:0]  cmd_len,
    input  wire [2:0]        cmd_size,
    input  wire [ID_W-1:0]   cmd_id,

    //  Upstream: data stream
    input  wire                    din_valid,
    output wire                    din_ready,
    input  wire [`AXI_DATA_W-1:0]  din_data,

    //  Response
    output wire              rsp_valid,
    output wire [ID_W-1:0]   rsp_id,
    output wire [1:0]        rsp_err,

    //  Timeout
    output wire              timeout_aw,
    output wire              timeout_w,

    //  AXI4 AW channel
    output reg  [ADDR_W-1:0] AWADDR,
    output reg  [LEN_W-1:0]  AWLEN,
    output reg  [2:0]        AWSIZE,
    output reg  [ID_W-1:0]   AWID,
    output reg               AWVALID,
    input  wire              AWREADY,

    //  AXI4 W channel 
    output wire [`AXI_DATA_W-1:0]   WDATA,
    output wire [`AXI_STRB_W-1:0]   WSTRB,
    output wire                      WLAST,
    output wire                      WVALID,
    input  wire                      WREADY,

    //  AXI4 B channel 
    input  wire [ID_W-1:0]   BID,
    input  wire [1:0]        BRESP,
    input  wire              BVALID,
    output wire              BREADY
);

    // localparam
    function integer clog2_fn;
        input integer v;
        integer i;
        begin
            clog2_fn = 0;
            for (i = v-1; i > 0; i = i >> 1)
                clog2_fn = clog2_fn + 1;
        end
    endfunction

    localparam CMD_D_W = clog2_fn(CMD_DEPTH);  // = PTR_W cần truyền vào sync_fifo

    // AW channel FSM
    // Tách AW và W để tối đa bandwidth (AW phát trước W data)
    assign cmd_ready = ~AWVALID | AWREADY;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            AWVALID <= 1'b0;
            AWADDR  <= {ADDR_W{1'b0}};
            AWLEN   <= {LEN_W{1'b0}};
            AWSIZE  <= `AXI_SIZE_4B;
            AWID    <= {ID_W{1'b0}};
        end else begin
            if (cmd_valid & cmd_ready) begin
                AWVALID <= 1'b1;
                AWADDR  <= cmd_addr;
                AWLEN   <= cmd_len;
                AWSIZE  <= cmd_size;
                AWID    <= cmd_id;
            end else if (AWVALID & AWREADY) begin
                AWVALID <= 1'b0;
            end
        end
    end

    // W channel — beat counter
    // Khi AW accepted -> capture len -> phát W beats
    // FIFO lưu {len, id} để W channel dùng sau khi AW accepted
    localparam WF_W = LEN_W + ID_W;

    wire              wf_push  = AWVALID & AWREADY;
    wire              wf_pop;
    wire              wf_empty, wf_full;
    wire [WF_W-1:0]   wf_rdata;

    sync_fifo #(
        .DATA_W    (WF_W),
        .DEPTH     (CMD_DEPTH),
        .PTR_W     (CMD_D_W),
        .AFULL_TH  (1),
        .AEMPTY_TH (1),
        .OUTREG    (0)
    ) u_winfo_fifo (
        .clk         (clk),
        .rst_n       (rst_n),
        .wr_en       (wf_push),
        .wr_data     ({AWLEN, AWID}),
        .rd_en       (wf_pop),
        .rd_data     (wf_rdata),
        .full        (wf_full),
        .empty       (wf_empty),
        .almost_full (),
        .almost_empty(),
        .count       ()
    );

    wire [LEN_W-1:0] cur_len = wf_rdata[WF_W-1 : ID_W];
    wire [ID_W-1:0]  cur_id  = wf_rdata[ID_W-1  : 0];

    // Beat counter
    reg [LEN_W-1:0]  beat_cnt;
    reg              w_active;

    wire w_beat = WVALID & WREADY;
    wire w_last = w_beat & WLAST;

    assign wf_pop = w_last;   // giải phóng FIFO slot sau burst xong

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_active <= 1'b0;
            beat_cnt <= {LEN_W{1'b0}};
        end else begin
            if (!w_active & !wf_empty & din_valid) begin
                // Bắt đầu burst mới
                w_active <= 1'b1;
                beat_cnt <= cur_len;
            end else if (w_active & w_last) begin
                w_active <= 1'b0;
                beat_cnt <= {LEN_W{1'b0}};
            end else if (w_active & w_beat) begin
                beat_cnt <= beat_cnt - 1'b1;
            end
        end
    end

    // W outputs
    assign WVALID  = w_active & din_valid;
    assign WDATA   = din_data;
    assign WLAST   = w_active & (beat_cnt == {LEN_W{1'b0}});
    assign WSTRB   = {`AXI_STRB_W{1'b1}};   // full-width write
    assign din_ready = w_active & WREADY;

    // B channel
    // FIFO lưu AWID để map BID -> rsp_id (hỗ trợ out-of-order B)
    wire              bf_push  = AWVALID & AWREADY;
    wire              bf_pop   = BVALID & BREADY;
    wire              bf_empty;
    wire [ID_W-1:0]   bf_rdata;

    sync_fifo #(
        .DATA_W    (ID_W),
        .DEPTH     (CMD_DEPTH),
        .PTR_W     (CMD_D_W),
        .AFULL_TH  (1),
        .AEMPTY_TH (1),
        .OUTREG    (0)
    ) u_bid_fifo (
        .clk         (clk),
        .rst_n       (rst_n),
        .wr_en       (bf_push),
        .wr_data     (AWID),
        .rd_en       (bf_pop),
        .rd_data     (bf_rdata),
        .full        (),
        .empty       (bf_empty),
        .almost_full (),
        .almost_empty(),
        .count       ()
    );

    assign BREADY = 1'b1;   // DMA luôn chấp nhận B response

    reg              rsp_valid_r;
    reg [ID_W-1:0]   rsp_id_r;
    reg [1:0]        rsp_err_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_valid_r <= 1'b0;
            rsp_id_r    <= {ID_W{1'b0}};
            rsp_err_r   <= 2'b00;
        end else begin
            rsp_valid_r <= bf_pop;
            if (bf_pop) begin
                rsp_id_r  <= bf_rdata;
                rsp_err_r <= BRESP;
            end
        end
    end

    assign rsp_valid = rsp_valid_r;
    assign rsp_id    = rsp_id_r;
    assign rsp_err   = rsp_err_r;

    // Timeout watchdogs
    timeout_cnt #(
        .CNT_W     (TIMEOUT_W),
        .AUTO_HOLD (1)
    ) u_aw_to (
        .clk     (clk),
        .rst_n   (rst_n),
        .valid   (AWVALID & ~AWREADY),
        .ack     (AWVALID & AWREADY),
        .timeout (timeout_aw)
    );

    timeout_cnt #(
        .CNT_W     (TIMEOUT_W),
        .AUTO_HOLD (1)
    ) u_w_to (
        .clk     (clk),
        .rst_n   (rst_n),
        .valid   (WVALID & ~WREADY),
        .ack     (WVALID & WREADY & WLAST),
        .timeout (timeout_w)
    );

endmodule