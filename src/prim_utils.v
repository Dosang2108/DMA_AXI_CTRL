// ============================================================
// prim_utils.v — Collection of small primitives
//   1. shift_reg      Pipeline delay register
//   2. timeout_cnt    Countdown watchdog
//   3. onehot_mux     N:1 mux with one-hot select
//   4. rr_arbiter     Round-robin arbiter
//
// ============================================================


// ------------------------------------------------------------
// 1. shift_reg
//    Pipeline delay: DATA_W bits, DELAY cycles
//    Dùng thay cho chuỗi prgen_delay #(N) của code gốc
// ------------------------------------------------------------
module shift_reg #(
    parameter DATA_W = 1,
    parameter DELAY  = 1       // ≥ 1
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DATA_W-1:0] din,
    output wire [DATA_W-1:0] dout
);
    // Mảng DELAY tầng flip-flop
    reg [DATA_W-1:0] pipe [0:DELAY-1];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin : rst_blk
            integer j;
            for (j = 0; j < DELAY; j = j + 1)
                pipe[j] <= {DATA_W{1'b0}};
        end else begin
            pipe[0] <= din;
            for (i = 1; i < DELAY; i = i + 1)
                pipe[i] <= pipe[i-1];
        end
    end

    assign dout = pipe[DELAY-1];

endmodule


// ------------------------------------------------------------
// 2. timeout_cnt
//    Đếm ngược CNT_W bit khi valid=1, reload khi ack=1
//    timeout = 1 khi counter == 0 (pulse một chu kỳ rồi tự giữ)
//
//    Parameters
//      CNT_W     : số bit counter → timeout sau 2^CNT_W - 1 cycles
//      AUTO_HOLD : 1 = timeout giữ nguyên cho đến khi ack
//                  0 = timeout chỉ pulse 1 cycle
// ------------------------------------------------------------
module timeout_cnt #(
    parameter CNT_W     = 10,
    parameter AUTO_HOLD = 1
) (
    input  wire clk,
    input  wire rst_n,
    input  wire valid,        // 1 = đang chờ phản hồi
    input  wire ack,          // 1 = giao dịch hoàn thành → reload
    output wire timeout
);
    reg [CNT_W-1:0] cnt;
    reg             timed_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= {CNT_W{1'b1}};
            timed_out<= 1'b0;
        end else if (ack) begin
            cnt      <= {CNT_W{1'b1}};
            timed_out<= 1'b0;
        end else if (valid && !timed_out) begin
            cnt <= cnt - 1'b1;
            if (cnt == {{(CNT_W-1){1'b0}}, 1'b1})  // cnt sắp về 0
                timed_out <= 1'b1;
        end
    end

    generate
        if (AUTO_HOLD == 1)
            assign timeout = timed_out;
        else
            assign timeout = valid & (cnt == {CNT_W{1'b0}});
    endgenerate

endmodule


// ------------------------------------------------------------
// 3. onehot_mux
//    N:1 mux, select là one-hot encoding
//    Dùng thay cho prgen_mux8 / prgen_or8 của code gốc
//
//    Parameters
//      DATA_W : chiều rộng mỗi input
//      N      : số input
//
//    Ports
//      din    : packed input [N*DATA_W-1:0]
//               din[i*DATA_W +: DATA_W] = input thứ i
//      sel    : one-hot select [N-1:0]
//      dout   : output [DATA_W-1:0]
//
//    Nếu sel = 0 → dout = 0 (safe default)
//    Nếu sel có nhiều bit set → dout = OR của các input được chọn
// ------------------------------------------------------------
module onehot_mux #(
    parameter DATA_W = 32,
    parameter N      = 4
) (
    input  wire [N*DATA_W-1:0] din,
    input  wire [N-1:0]        sel,
    output reg  [DATA_W-1:0]   dout
);
    integer i;
    always @(*) begin
        dout = {DATA_W{1'b0}};
        for (i = 0; i < N; i = i + 1)
            if (sel[i])
                dout = dout | din[i*DATA_W +: DATA_W];
    end

endmodule


// ------------------------------------------------------------
// 4. rr_arbiter
//    Round-robin arbiter, N requestors
//    Grant là one-hot, 1 grant mỗi cycle khi có request
//
//    Parameters
//      N        : số requestor [default 4]
//
//    Behaviour
//      - Sau khi grant requestor i, lần sau bắt đầu tìm từ i+1
//      - Nếu không có request → grant = 0
//      - Grant có hiệu lực ngay trong cùng cycle với req (combinational)
//        và được register lại để giữ stable
// ------------------------------------------------------------
module rr_arbiter #(
    parameter N = 4
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [N-1:0] req,
    output reg  [N-1:0] grant
);
    // Tính N_W = clog2(N) để làm pointer
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam N_W = clog2(N);

    reg [N_W-1:0] last_grant;   // index của grant gần nhất

    // ---- Combinational: tìm grant tiếp theo sau last_grant ----
    // req_dbl = {req, req}; scan từ last_grant+1..last_grant+N
    wire [2*N-1:0] req_dbl = {req, req};

    // Tìm bit 1 đầu tiên trong req_dbl bắt đầu từ vị trí last_grant+1
    // BUG FIX: dùng cờ `found` thay vì sentinel value.
    // Sentinel = {(N_W+1){1'b1}} = 2*N-1 cho N lũy thừa 2, trùng với
    // index hợp lệ cuối cùng trong req_dbl → kênh N-1 bị bỏ qua khi
    // last_grant = N-1 và chỉ kênh N-1 đang request.
    reg  [N_W:0]   grant_idx_dbl;  // index trong req_dbl
    reg  [N-1:0]   grant_next;
    reg            found;

    integer k;
    always @(*) begin
        grant_idx_dbl = {(N_W+1){1'b0}};
        grant_next    = {N{1'b0}};
        found         = 1'b0;
        for (k = 0; k < N; k = k + 1) begin
            // kiểm tra từ (last_grant+1) đến (last_grant+N)
            if (!found && req_dbl[last_grant + 1 + k]) begin
                grant_idx_dbl = last_grant + 1 + k;
                found         = 1'b1;
            end
        end
        // Chuyển index trong req_dbl về index thực [0..N-1]
        if (found)
            grant_next[grant_idx_dbl % N] = 1'b1;
    end

    // ---- Register grant và cập nhật last_grant ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant      <= {N{1'b0}};
            last_grant <= {N_W{1'b0}};
        end else begin
            if (|req) begin
                grant <= grant_next;
                // Cập nhật last_grant = index của grant_next
                begin : update_last
                    integer m;
                    for (m = 0; m < N; m = m + 1)
                        if (grant_next[m])
                            last_grant <= m[N_W-1:0];
                end
            end else begin
                grant <= {N{1'b0}};
            end
        end
    end

endmodule