// ============================================================
// dma_channel.v — 1 kênh DMA (src -> FIFO -> dst)
// Layer : Reusable IP
// `include: dma_defines.vh
//
// Parameters — MỌI giá trị đều có thể override
//   ADDR_W       : AXI address bits              [default 32]
//   LEN_W        : AXLEN width                   [default 4]
//   ID_W         : AXI ID width                  [default 4]
//   FIFO_DEPTH   : Internal data FIFO depth      [default 16]
//   MAX_BURST    : Byte tối đa mỗi burst         [default 64]
//   BURST_W      : Bits cho burst_max field      [default 7]
//   TOKEN_W      : Bits cho token counter        [default 4]
//   OUT_W        : Bits cho outstanding counter  [default 4]
//   TIMEOUT_W    : Bits cho watchdog counter     [default 10]
//   PERIPH_NUM_W : Bits cho peripheral index     [default 5]
//   LEN_FIELD_W  : Bits cho transfer length reg  [default 16]
//
// Giao diện với dma_engine
//   cfg_*   : cấu hình (ổn định trong suốt transfer)
//   start   : 1-cycle pulse kích hoạt
//   done    : 1-cycle pulse khi hoàn thành
//   active  : 1 khi đang chạy
//   err     : mã lỗi khi done
//
// Giao diện với axi4_master_rd (qua dma_engine mux)
//   rd_cmd_*  : lệnh đọc
//   rd_dat_*  : data nhận về
//   rd_rsp_*  : phản hồi cuối burst
//
// Giao diện với axi4_master_wr
//   wr_cmd_*  : lệnh ghi
//   wr_dat_*  : data gửi đi
//   wr_rsp_*  : phản hồi cuối burst
// ============================================================
`include "dma_define.vh"

module dma_channel #(
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
    parameter LEN_FIELD_W  = 16
) (
    input  wire              clk,
    input  wire              rst_n,

    //  Cấu hình (stable khi active=1) 
    input  wire [ADDR_W-1:0]       cfg_src_addr,
    input  wire [ADDR_W-1:0]       cfg_dst_addr,
    input  wire [LEN_FIELD_W-1:0]  cfg_len,          // tổng bytes
    input  wire [BURST_W-1:0]      cfg_burst_max,    // bytes/burst ≤ MAX_BURST
    input  wire [TOKEN_W-1:0]      cfg_tokens,       // max inflight rd bursts
    input  wire [OUT_W-1:0]        cfg_rd_out_max,   // max outstanding rd cmds
    input  wire [OUT_W-1:0]        cfg_wr_out_max,   // max outstanding wr cmds
    input  wire                    cfg_src_incr,     // 1 = increment src addr
    input  wire                    cfg_dst_incr,     // 1 = increment dst addr
    input  wire [PERIPH_NUM_W-1:0] cfg_periph_num,   // 0 = memory (always ready)

    //  Control 
    input  wire              start,
    output wire              done,
    output wire              active,
    output reg  [1:0]        err,

    //  Peripheral trigger 
    input  wire [31:1]       periph_req,
    output reg  [31:1]       periph_clr,

    //   RD command (-> axi4_master_rd) 
    output wire              rd_cmd_valid,
    input  wire              rd_cmd_ready,
    output wire [ADDR_W-1:0] rd_cmd_addr,
    output wire [LEN_W-1:0]  rd_cmd_len,
    output wire [2:0]        rd_cmd_size,
    output wire [ID_W-1:0]   rd_cmd_id,

    //  RD data (← axi4_master_rd) 
    input  wire                    rd_dat_valid,
    output wire                    rd_dat_ready,
    input  wire [`AXI_DATA_W-1:0]  rd_dat_data,
    input  wire                    rd_dat_last,

    //  RD response 
    input  wire              rd_rsp_valid,
    input  wire [ID_W-1:0]   rd_rsp_id,
    input  wire [1:0]        rd_rsp_err,

    //  WR command (-> axi4_master_wr) 
    output wire              wr_cmd_valid,
    input  wire              wr_cmd_ready,
    output wire [ADDR_W-1:0] wr_cmd_addr,
    output wire [LEN_W-1:0]  wr_cmd_len,
    output wire [2:0]        wr_cmd_size,
    output wire [ID_W-1:0]   wr_cmd_id,

    //  WR data (-> axi4_master_wr) 
    output wire                    wr_dat_valid,
    input  wire                    wr_dat_ready,
    output wire [`AXI_DATA_W-1:0]  wr_dat_data,

    //  WR response 
    input  wire              wr_rsp_valid,
    input  wire [ID_W-1:0]   wr_rsp_id,
    input  wire [1:0]        wr_rsp_err,

    //  Timeout flags (từ AXI masters) 
    input  wire              timeout_rd,
    input  wire              timeout_wr_aw,
    input  wire              timeout_wr_w
);

    // localparam
    localparam BYTES_PER_BEAT = `AXI_BYTES;     // = 4 cho 32-bit
    // FIFO phải là lũy thừa 2 — dùng generate để kiểm tra?
    // ASSERT: FIFO_DEPTH phải là lũy thừa 2

    function integer clog2_fn;
        input integer v;
        integer i;
        begin
            clog2_fn = 0;
            for (i = v-1; i > 0; i = i >> 1)
                clog2_fn = clog2_fn + 1;
        end
    endfunction

    // BEAT_BITS = log2(BYTES_PER_BEAT) = 2 cho 32-bit bus
    localparam BEAT_BITS  = clog2_fn(BYTES_PER_BEAT);  // = 2

    // FIFO_CNT_W: phải bằng PTR_W+1 của sync_fifo.
    // sync_fifo.count là [PTR_W:0] = PTR_W+1 bits,
    // với PTR_W = clog2(FIFO_DEPTH).
    // Vì vậy FIFO_CNT_W = clog2_fn(FIFO_DEPTH) + 1.
    localparam FIFO_PTR_W = clog2_fn(FIFO_DEPTH);      // = PTR_W trong sync_fifo
    localparam FIFO_CNT_W = FIFO_PTR_W + 1;            // = width của count port

    // State machine
    localparam [2:0]
        ST_IDLE  = 3'd0,
        ST_RUN   = 3'd1,
        ST_DRAIN = 3'd2,   // rd xong, đợi FIFO drain + wr xong
        ST_DONE  = 3'd3;   // 1-cycle done pulse

    reg [2:0] state;

    // Address và remaining counters
    reg [ADDR_W-1:0]      rd_addr, wr_addr;
    reg [LEN_FIELD_W-1:0] rd_remain, wr_remain;

    // Tính burst size thực tế (min của remain và cfg_burst_max)
    // Dùng combo logic để luôn reflect giá trị hiện tại
    wire [BURST_W-1:0] rd_burst_bytes =
        (rd_remain < {{(LEN_FIELD_W-BURST_W){1'b0}}, cfg_burst_max})
        ? rd_remain[BURST_W-1:0]
        : cfg_burst_max;

    wire [BURST_W-1:0] wr_burst_bytes =
        (wr_remain < {{(LEN_FIELD_W-BURST_W){1'b0}}, cfg_burst_max})
        ? wr_remain[BURST_W-1:0]
        : cfg_burst_max;

    // ARLEN = (burst_bytes / BYTES_PER_BEAT) - 1
    // Dùng BEAT_BITS (localparam = clog2(BYTES_PER_BEAT)) thay vì hardcode 2
    wire [LEN_W-1:0] rd_burst_len =
        (rd_burst_bytes[BURST_W-1:BEAT_BITS] == {(BURST_W-BEAT_BITS){1'b0}})
        ? {LEN_W{1'b0}}
        : rd_burst_bytes[BURST_W-1:BEAT_BITS] - 1'b1;

    wire [LEN_W-1:0] wr_burst_len =
        (wr_burst_bytes[BURST_W-1:BEAT_BITS] == {(BURST_W-BEAT_BITS){1'b0}})
        ? {LEN_W{1'b0}}
        : wr_burst_bytes[BURST_W-1:BEAT_BITS] - 1'b1;

    // Outstanding command counters
    reg [OUT_W-1:0] rd_outs, wr_outs;

    wire rd_cmd_fire = rd_cmd_valid & rd_cmd_ready;
    wire wr_cmd_fire = wr_cmd_valid & wr_cmd_ready;

    wire rd_stall = (cfg_rd_out_max != {OUT_W{1'b0}}) &
                    (rd_outs >= cfg_rd_out_max);
    wire wr_stall = (cfg_wr_out_max != {OUT_W{1'b0}}) &
                    (wr_outs >= cfg_wr_out_max);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_outs <= {OUT_W{1'b0}};
            wr_outs <= {OUT_W{1'b0}};
        end else begin
            // rd_outs: +1 khi phát lệnh, -1 khi nhận response
            rd_outs <= rd_outs + rd_cmd_fire - rd_rsp_valid;
            wr_outs <= wr_outs + wr_cmd_fire - wr_rsp_valid;
        end
    end

    // Token counter — rate-match RD vs WR để tránh FIFO overflow
    // Mỗi token = 1 phép RD burst chưa được WR xử lý
    reg [TOKEN_W-1:0] tokens;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == ST_IDLE)
            tokens <= cfg_tokens;
        else if (rd_cmd_fire & ~wr_rsp_valid)
            tokens <= (tokens == {TOKEN_W{1'b0}}) ? {TOKEN_W{1'b0}} : tokens - 1'b1;
        else if (~rd_cmd_fire & wr_rsp_valid)
            tokens <= tokens + 1'b1;
    end

    wire token_ok = (tokens != {TOKEN_W{1'b0}}) | (wr_remain == {LEN_FIELD_W{1'b0}});

    // Peripheral ready (registered 1 cycle)
    // bit 0 của periph_req_ext = memory = selalu 1
    wire [31:0] periph_req_ext = {periph_req, 1'b1};
    reg         periph_rdy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) periph_rdy <= 1'b0;
        else periph_rdy <= periph_req_ext[cfg_periph_num];
    end

    // periph_clr: pulse 1 cycle sau mỗi rd burst
    // Bit trong periph_clr[31:1] tương ứng periph_num 1..31
    // periph_clr[periph_num-1] = 1 khi clr periph thứ periph_num
    // Khi periph_num=0 (memory) → không clr gì cả
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            periph_clr <= {31{1'b0}};
        end else begin
            periph_clr <= {31{1'b0}};
            if (rd_cmd_fire && (cfg_periph_num != {PERIPH_NUM_W{1'b0}})) begin
                // Dịch an toàn: cfg_periph_num >= 1 ở đây
                // periph_clr là [31:1], bit i tương ứng periph_num = i
                // periph_clr[periph_num - 1] = 1
                periph_clr <= (31'b1 << (cfg_periph_num - {{(PERIPH_NUM_W-1){1'b0}}, 1'b1}));
            end
        end
    end

    // Internal data FIFO
    wire              fifo_wr_en  = rd_dat_valid & rd_dat_ready;
    wire              fifo_rd_en  = wr_dat_valid & wr_dat_ready;
    wire              fifo_full, fifo_empty, fifo_afull;
    wire [`AXI_DATA_W-1:0] fifo_rdata;
    // count port của sync_fifo là [PTR_W:0] = [FIFO_PTR_W:0] = FIFO_CNT_W bits
    wire [FIFO_CNT_W-1:0]  fifo_count;

    assign rd_dat_ready = ~fifo_full;

    sync_fifo #(
        .DATA_W    (`AXI_DATA_W),
        .DEPTH     (FIFO_DEPTH),
        .PTR_W     (FIFO_PTR_W),  // phải khớp với clog2(FIFO_DEPTH)
        .AFULL_TH  (4),            // giữ 4 slot dự phòng
        .AEMPTY_TH (1),
        .OUTREG    (0)             // 0-latency cho WR data path
    ) u_data_fifo (
        .clk         (clk),
        .rst_n       (rst_n),
        .wr_en       (fifo_wr_en),
        .wr_data     (rd_dat_data),
        .rd_en       (fifo_rd_en),
        .rd_data     (fifo_rdata),
        .full        (fifo_full),
        .empty       (fifo_empty),
        .almost_full (fifo_afull),
        .almost_empty(),
        .count       (fifo_count)
    );

    // State machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            rd_addr   <= {ADDR_W{1'b0}};
            wr_addr   <= {ADDR_W{1'b0}};
            rd_remain <= {LEN_FIELD_W{1'b0}};
            wr_remain <= {LEN_FIELD_W{1'b0}};
            err       <= 2'b00;
        end else begin
            case (state)

                ST_IDLE: begin
                    if (start) begin
                        state     <= ST_RUN;
                        rd_addr   <= cfg_src_addr;
                        wr_addr   <= cfg_dst_addr;
                        rd_remain <= cfg_len;
                        wr_remain <= cfg_len;
                        err       <= `DMA_ERR_NONE;
                    end
                end

                ST_RUN: begin
                    // Cập nhật address / remaining
                    if (rd_cmd_fire) begin
                        if (cfg_src_incr)
                            rd_addr <= rd_addr + {{(ADDR_W-BURST_W){1'b0}}, rd_burst_bytes};
                        rd_remain <= rd_remain - {{(LEN_FIELD_W-BURST_W){1'b0}}, rd_burst_bytes};
                    end
                    if (wr_cmd_fire) begin
                        if (cfg_dst_incr)
                            wr_addr <= wr_addr + {{(ADDR_W-BURST_W){1'b0}}, wr_burst_bytes};
                        wr_remain <= wr_remain - {{(LEN_FIELD_W-BURST_W){1'b0}}, wr_burst_bytes};
                    end

                    // Thu thập lỗi
                    if (rd_rsp_valid & (rd_rsp_err != `AXI_RESP_OKAY))
                        err <= (rd_rsp_err == `AXI_RESP_SLVERR)
                               ? `DMA_ERR_SLVERR : `DMA_ERR_DECERR;
                    if (wr_rsp_valid & (wr_rsp_err != `AXI_RESP_OKAY))
                        err <= (wr_rsp_err == `AXI_RESP_SLVERR)
                               ? `DMA_ERR_SLVERR : `DMA_ERR_DECERR;
                    if (timeout_rd | timeout_wr_aw | timeout_wr_w)
                        err <= `DMA_ERR_TIMEOUT;

                    // Lỗi → dừng ngay
                    if (err != `DMA_ERR_NONE)
                        state <= ST_DRAIN;

                    // Hết RD → chờ drain
                    if (rd_remain == {LEN_FIELD_W{1'b0}} && !rd_cmd_fire)
                        state <= ST_DRAIN;
                end

                ST_DRAIN: begin
                    // Đợi FIFO empty và WR xong
                    if (fifo_empty && wr_outs == {OUT_W{1'b0}} &&
                        wr_remain == {LEN_FIELD_W{1'b0}})
                        state <= ST_DONE;
                end

                ST_DONE: begin
                    state <= ST_IDLE;   // 1-cycle pulse
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

    // RD command generation
    reg [ID_W-1:0] rd_id_cnt;   // burst sequence counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == ST_IDLE) rd_id_cnt <= {ID_W{1'b0}};
        else if (rd_cmd_fire) rd_id_cnt <= rd_id_cnt + 1'b1;
    end

    assign rd_cmd_valid = (state == ST_RUN) &
                          (rd_remain != {LEN_FIELD_W{1'b0}}) &
                          ~rd_stall     &
                          ~fifo_afull   &   // đủ chỗ FIFO cho burst này
                          token_ok      &
                          periph_rdy;

    assign rd_cmd_addr  = rd_addr;
    assign rd_cmd_len   = rd_burst_len;
    assign rd_cmd_size  = `AXI_SIZE_4B;
    assign rd_cmd_id    = rd_id_cnt;

    // WR command generation
    // Chờ FIFO đủ data cho 1 burst trước khi phát AW
    reg [ID_W-1:0] wr_id_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == ST_IDLE) wr_id_cnt <= {ID_W{1'b0}};
        else if (wr_cmd_fire) wr_id_cnt <= wr_id_cnt + 1'b1;
    end

    // Số beats cần thiết cho burst hiện tại
    // wr_burst_len là [LEN_W-1:0], +1 → tối đa LEN_W bits cần +1 = LEN_W+1 bits
    // fifo_count là [FIFO_CNT_W-1:0]
    // Để so sánh an toàn, mở rộng cả hai về max(FIFO_CNT_W, LEN_W+1) bits
    localparam CMP_W = (FIFO_CNT_W > LEN_W) ? FIFO_CNT_W : LEN_W + 1;

    wire [CMP_W-1:0] beats_needed =
        {{(CMP_W-LEN_W){1'b0}}, wr_burst_len} + 1'b1;

    wire fifo_has_enough = ({{(CMP_W-FIFO_CNT_W){1'b0}}, fifo_count} >= beats_needed);

    assign wr_cmd_valid = ((state == ST_RUN) | (state == ST_DRAIN)) &
                          (wr_remain != {LEN_FIELD_W{1'b0}}) &
                          ~wr_stall &
                          fifo_has_enough;

    assign wr_cmd_addr  = wr_addr;
    assign wr_cmd_len   = wr_burst_len;
    assign wr_cmd_size  = `AXI_SIZE_4B;
    assign wr_cmd_id    = wr_id_cnt;

    // WR data: feed trực tiếp từ FIFO
    // WR data active khi AW đã được accepted (tracked bằng beat counter)
    reg [LEN_W-1:0] wr_beat_cnt;
    reg             wr_dat_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_dat_active <= 1'b0;
            wr_beat_cnt   <= {LEN_W{1'b0}};
        end else begin
            if (~wr_dat_active & wr_cmd_valid & wr_cmd_ready) begin
                wr_dat_active <= 1'b1;
                wr_beat_cnt   <= wr_burst_len;
            end else if (wr_dat_valid & wr_dat_ready) begin
                if (wr_beat_cnt == {LEN_W{1'b0}})
                    wr_dat_active <= 1'b0;
                else
                    wr_beat_cnt <= wr_beat_cnt - 1'b1;
            end
        end
    end

    assign wr_dat_valid = wr_dat_active & ~fifo_empty;
    assign wr_dat_data  = fifo_rdata;

    // Status outputs
    assign active = (state != ST_IDLE);
    assign done   = (state == ST_DONE);

endmodule