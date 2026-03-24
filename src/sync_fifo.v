// ============================================================
// sync_fifo.v — Synchronous FIFO (single-clock)
// Layer : Primitive
//
// Parameters (tất cả đều override được)
//   DATA_W   : chiều rộng mỗi entry                [default 32]
//   DEPTH    : số entry — PHẢI là lũy thừa 2        [default 8]
//   AFULL_TH : almost_full khi count >= DEPTH-TH    [default 2]
//   AEMPTY_TH: almost_empty khi count <= TH         [default 2]
//   OUTREG   : 1 = output register (1 cycle latency, Fmax cao hơn)
//              0 = output combinational (0 latency)  [default 1]
//
// Ports
//   clk, rst_n            : clock, active-low reset
//   wr_en, wr_data        : write port
//   rd_en, rd_data        : read port
//   full, empty           : full/empty flags
//   almost_full           : count >= DEPTH - AFULL_TH
//   almost_empty          : count <= AEMPTY_TH
//   count [CNT_W:0]       : số entry hiện tại (0..DEPTH)
//
// Behaviour
//   - wr_en khi full  → bị bỏ qua, không wrap
//   - rd_en khi empty → rd_data giữ giá trị cũ
//   - wr_en+rd_en cùng lúc → count không đổi, data forward
module sync_fifo #(
    parameter DATA_W    = 32,
    parameter DEPTH     = 8,
    parameter AFULL_TH  = 2,
    parameter AEMPTY_TH = 2,
    parameter OUTREG    = 1,
    // PTR_W = clog2(DEPTH). Khai báo là parameter để dùng được trong port list.
    // chỉ được khai báo bên trong module body. Người dùng KHÔNG override tham số này.
    parameter PTR_W     = 3            // default = clog2(8). Override khi DEPTH thay đổi:
                                       //   DEPTH=4  -> PTR_W=2
                                       //   DEPTH=8  -> PTR_W=3
                                       //   DEPTH=16 -> PTR_W=4
                                       //   DEPTH=32 -> PTR_W=5
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              wr_en,
    input  wire [DATA_W-1:0] wr_data,

    input  wire              rd_en,
    output wire [DATA_W-1:0] rd_data,

    output wire              full,
    output wire              empty,
    output wire              almost_full,
    output wire              almost_empty,
    output wire [PTR_W:0]    count      // PTR_W+1 bits: 0..DEPTH
);

    // Kiểm tra nhất quán PTR_W giữa DEPTH bằng functio
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    // Dùng localparam nội bộ để tính — không export ra port
    localparam PTR_W_CALC = clog2(DEPTH); // phải bằng PTR_W parameter

    // Storage
    reg [DATA_W-1:0] mem [0:DEPTH-1];

    // Pointers — dùng PTR_W_CALC (localparam, tính từ DEPTH) cho
    // tất cả logic nội bộ. PTR_W parameter chỉ dùng cho port count.
    reg [PTR_W_CALC:0] wr_ptr;   // PTR_W_CALC+1 bits (extra MSB = wrap bit)
    reg [PTR_W_CALC:0] rd_ptr;

    wire do_wr = wr_en & ~full;
    wire do_rd = rd_en & ~empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {(PTR_W_CALC+1){1'b0}};
            rd_ptr <= {(PTR_W_CALC+1){1'b0}};
        end else begin
            if (do_wr) begin
                mem[wr_ptr[PTR_W_CALC-1:0]] <= wr_data;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (do_rd)
                rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // Count (wrap-safe subtraction) — internal wire dùng PTR_W_CALC
    wire [PTR_W_CALC:0] count_int = wr_ptr - rd_ptr;

    // Export ra port count (width = PTR_W từ parameter)
    assign count = count_int[PTR_W:0];

    // Flags — dùng count_int để tránh width mismatch
    assign full         = (count_int == DEPTH[PTR_W_CALC:0]);
    assign empty        = (count_int == {(PTR_W_CALC+1){1'b0}});
    assign almost_full  = (count_int >= (DEPTH - AFULL_TH));
    assign almost_empty = (count_int <= AEMPTY_TH[PTR_W_CALC:0]);

    // Read data output
    generate
        if (OUTREG == 1) begin : gen_outreg
            // Registered output: thêm 1 cycle latency, Fmax tốt hơn
            reg [DATA_W-1:0] rd_data_r;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    rd_data_r <= {DATA_W{1'b0}};
                else if (do_rd)
                    rd_data_r <= mem[rd_ptr[PTR_W_CALC-1:0]];
            end
            assign rd_data = rd_data_r;
        end else begin : gen_combo
            // Combinational output: 0 latency, dùng cho low-latency path
            assign rd_data = mem[rd_ptr[PTR_W_CALC-1:0]];
        end
    endgenerate

endmodule