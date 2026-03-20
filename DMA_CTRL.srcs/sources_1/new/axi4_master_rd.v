// ============================================================
// axi4_master_rd.v — AXI4 Read Master (32-bit data bus)
// Layer : Reusable IP
// `include: dma_defines.vh
//
// Chức năng
//   Nhận burst descriptor từ upstream, phát AR channel,
//   nhận R channel, đẩy data ra downstream qua valid/ready.
//
// Parameters
//   ADDR_W      : Địa chỉ AXI (32 / 64)          [default 32]
//   ID_W        : AXI ID width                    [default 4]
//   LEN_W       : ARLEN width (4=AXI4-Lite/16beat, 8=AXI4/256beat) [default 4]
//   CMD_DEPTH   : Số outstanding burst            [default 4]
//   TIMEOUT_W   : Bits counter watchdog AR        [default 10]
//
// Upstream — burst descriptor (valid/ready)
//   cmd_valid, cmd_ready
//   cmd_addr [ADDR_W-1:0]  : địa chỉ bắt đầu burst
//   cmd_len  [LEN_W-1:0]   : ARLEN (beats-1)
//   cmd_size [2:0]         : ARSIZE (thường = `AXI_SIZE_4B cho 32-bit)
//   cmd_id   [ID_W-1:0]    : tag — được echo trong rsp_id
//
// Downstream — data stream (valid/ready)
//   dout_valid, dout_ready
//   dout_data [`AXI_DATA_W-1:0]
//   dout_last                  : last beat của burst
//   dout_id   [ID_W-1:0]       : tag từ cmd_id
//
// Response — 1 pulse khi burst kết thúc
//   rsp_valid, rsp_id [ID_W-1:0], rsp_err [1:0]
//
// Timeout
//   timeout_out : AR đang chờ quá TIMEOUT_W cycles
// ============================================================
`include "dma_defines.vh"

module axi4_master_rd #(
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

    //  Downstream: data stream 
    output wire                    dout_valid,
    input  wire                    dout_ready,
    output wire [`AXI_DATA_W-1:0]  dout_data,
    output wire                    dout_last,
    output wire [ID_W-1:0]         dout_id,

    //  Response 
    output wire              rsp_valid,
    output wire [ID_W-1:0]   rsp_id,
    output wire [1:0]        rsp_err,

    //  Timeout 
    output wire              timeout_out,

    //  AXI4 AR channel 
    output reg  [ADDR_W-1:0] ARADDR,
    output reg  [LEN_W-1:0]  ARLEN,
    output reg  [2:0]        ARSIZE,
    output reg  [ID_W-1:0]   ARID,
    output reg               ARVALID,
    input  wire              ARREADY,

    //  AXI4 R channel 
    input  wire [`AXI_DATA_W-1:0] RDATA,
    input  wire [ID_W-1:0]        RID,
    input  wire [1:0]             RRESP,
    input  wire                   RLAST,
    input  wire                   RVALID,
    output wire                   RREADY
);

    // localparam
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

    // CMD_DEPTH phải là lũy thừa 2 — kiểm tra tại elaboration
    localparam CMD_D_W = clog2_fn(CMD_DEPTH);

    // Outstanding command FIFO — lưu cmd_id để map vào rsp
    // push khi AR accepted, pop khi R LAST received
    wire              cmd_fifo_push  = ARVALID & ARREADY;
    wire              cmd_fifo_pop   = RVALID & RREADY & RLAST;
    wire              cmd_fifo_full;
    wire              cmd_fifo_empty;
    wire [ID_W-1:0]   cmd_fifo_rdata;

    sync_fifo #(
        .DATA_W    (ID_W),
        .DEPTH     (CMD_DEPTH),
        .PTR_W     (CMD_D_W),  // = clog2(CMD_DEPTH), khớp với parameter PTR_W của sync_fifo
        .AFULL_TH  (1),
        .AEMPTY_TH (1),
        .OUTREG    (0)         // 0-latency cần thiết cho dout_id
    ) u_cmd_fifo (
        .clk         (clk),
        .rst_n       (rst_n),
        .wr_en       (cmd_fifo_push),
        .wr_data     (cmd_id),        // lưu cmd_id khi gửi AR
        .rd_en       (cmd_fifo_pop),
        .rd_data     (cmd_fifo_rdata),
        .full        (cmd_fifo_full),
        .empty       (cmd_fifo_empty),
        .almost_full (),
        .almost_empty(),
        .count       ()
    );

    // AR channel FSM
    // cmd_ready = AR slot trống (ARVALID=0 hoặc sắp accepted)
    //           AND cmd_fifo chưa đầy (outstanding limit)
    assign cmd_ready = (~ARVALID | ARREADY) & ~cmd_fifo_full;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ARVALID <= 1'b0;
            ARADDR  <= {ADDR_W{1'b0}};
            ARLEN   <= {LEN_W{1'b0}};
            ARSIZE  <= `AXI_SIZE_4B;
            ARID    <= {ID_W{1'b0}};
        end else begin
            // Load mới khi: cmd có sẵn AND slot trống AND FIFO chưa đầy
            if (cmd_valid & cmd_ready & ~cmd_fifo_full) begin
                ARVALID <= 1'b1;
                ARADDR  <= cmd_addr;
                ARLEN   <= cmd_len;
                ARSIZE  <= cmd_size;
                ARID    <= cmd_id;
            end else if (ARVALID & ARREADY) begin
                ARVALID <= 1'b0;
            end
        end
    end

    // R channel — forward trực tiếp ra downstream
    // RREADY backpressure từ downstream
    assign RREADY     = dout_ready;
    assign dout_valid = RVALID;
    assign dout_data  = RDATA;
    assign dout_last  = RLAST;
    // dout_id: lấy từ FIFO (0-latency, available ngay khi RVALID)
    assign dout_id    = cmd_fifo_empty ? {ID_W{1'b0}} : cmd_fifo_rdata;

    // Response: 1 cycle sau khi burst kết thúc
    reg              rsp_valid_r;
    reg [ID_W-1:0]   rsp_id_r;
    reg [1:0]        rsp_err_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_valid_r <= 1'b0;
            rsp_id_r    <= {ID_W{1'b0}};
            rsp_err_r   <= 2'b00;
        end else begin
            rsp_valid_r <= cmd_fifo_pop;
            if (cmd_fifo_pop) begin
                rsp_id_r  <= cmd_fifo_rdata;
                rsp_err_r <= RRESP;
            end
        end
    end

    assign rsp_valid = rsp_valid_r;
    assign rsp_id    = rsp_id_r;
    assign rsp_err   = rsp_err_r;

    // Timeout watchdog — AR channel
    timeout_cnt #(
        .CNT_W     (TIMEOUT_W),
        .AUTO_HOLD (1)
    ) u_ar_timeout (
        .clk     (clk),
        .rst_n   (rst_n),
        .valid   (ARVALID & ~ARREADY),   // chờ ARREADY
        .ack     (ARVALID & ARREADY),    // accepted
        .timeout (timeout_out)
    );

endmodule