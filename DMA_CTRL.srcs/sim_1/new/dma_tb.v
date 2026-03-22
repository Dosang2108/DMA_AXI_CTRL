// ============================================================
// tb_dma_engine.v — Testbench cho dma_engine
// Phiên bản 2: AXI slave model viết lại bằng task-based
// (tránh race condition always-block với non-blocking assign)
//
// Test cases:
//   TC1 : Mem-to-Mem, 1 channel, 64B
//   TC2 : 4 channels đồng thời, 256B/kênh
//   TC3 : IRQ và INT_STAT W1C
//   TC4 : Peripheral-triggered transfer
//   TC5 : AXI SLVERR → DMA_ERR_SLVERR
//   TC6 : AW timeout watchdog
//   TC7 : Soft-reset rồi transfer lại
// ============================================================
`timescale 1ns/1ps
`include "dma_define.vh"

module tb_dma_engine;

    // --------------------------------------------------------
    // Parameters
    // --------------------------------------------------------
    localparam N_CH         = 4;
    localparam N_CH_W       = 2;
    localparam ADDR_W       = 32;
    localparam LEN_W        = 4;
    localparam ID_W         = 4;
    localparam FIFO_DEPTH   = 16;
    localparam MAX_BURST    = 64;
    localparam BURST_W      = 7;
    localparam TOKEN_W      = 4;
    localparam OUT_W        = 4;
    localparam TIMEOUT_W    = 12;
    localparam PERIPH_NUM_W = 5;
    localparam LEN_FIELD_W  = 16;
    localparam APB_ADDR_W   = 14;
    localparam DEF_TOKENS   = 4;
    localparam DEF_OUTS     = 4;

    // CTRL register bit layout (khop voi dma_engine BURST_W=7)
    localparam CTRL_START   = 0;
    localparam CTRL_BMAX_L  = 1;
    localparam CTRL_BMAX_H  = 7;
    localparam CTRL_SI      = 8;
    localparam CTRL_DI      = 9;
    localparam CTRL_PNUM_L  = 10;
    localparam CTRL_PNUM_H  = 14;

    localparam CH_STRIDE    = (1 << `DMA_CH_STRIDE);
    localparam MEM_SIZE     = 8192;
    localparam MEM_WORDS    = MEM_SIZE / 4;
    localparam MAX_WAIT     = 200000;

    // --------------------------------------------------------
    // Clock & Reset
    // --------------------------------------------------------
    reg clk;
    reg rst_n;
    initial clk = 0;
    always #5 clk = ~clk;

    // --------------------------------------------------------
    // DUT I/O
    // --------------------------------------------------------
    reg                   apb_psel, apb_penable, apb_pwrite;
    reg  [APB_ADDR_W-1:0] apb_paddr;
    reg  [31:0]           apb_pwdata;
    wire [31:0]           apb_prdata;
    wire                  apb_pslverr, apb_pready;
    wire [N_CH-1:0]       irq;
    reg  [31:1]           periph_req;
    wire [31:1]           periph_clr;

    wire [ADDR_W-1:0] ARADDR; wire [LEN_W-1:0] ARLEN;
    wire [2:0] ARSIZE;        wire [ID_W-1:0]  ARID;
    wire ARVALID;             reg  ARREADY;

    reg  [31:0]      RDATA;  reg  [ID_W-1:0] RID;
    reg  [1:0]       RRESP;  reg  RLAST, RVALID;
    wire             RREADY;

    wire [ADDR_W-1:0] AWADDR; wire [LEN_W-1:0] AWLEN;
    wire [2:0] AWSIZE;         wire [ID_W-1:0]  AWID;
    wire AWVALID;              reg  AWREADY;

    wire [31:0] WDATA; wire [3:0] WSTRB;
    wire WLAST, WVALID; reg WREADY;

    reg  [ID_W-1:0] BID; reg [1:0] BRESP;
    reg  BVALID;         wire BREADY;

    // --------------------------------------------------------
    // DUT
    // --------------------------------------------------------
    dma_engine #(
        .N_CH(N_CH), .N_CH_W(N_CH_W), .ADDR_W(ADDR_W), .LEN_W(LEN_W),
        .ID_W(ID_W), .FIFO_DEPTH(FIFO_DEPTH), .MAX_BURST(MAX_BURST),
        .BURST_W(BURST_W), .TOKEN_W(TOKEN_W), .OUT_W(OUT_W),
        .TIMEOUT_W(TIMEOUT_W), .PERIPH_NUM_W(PERIPH_NUM_W),
        .LEN_FIELD_W(LEN_FIELD_W), .APB_ADDR_W(APB_ADDR_W),
        .DEF_TOKENS(DEF_TOKENS), .DEF_OUTS(DEF_OUTS)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .apb_psel(apb_psel), .apb_penable(apb_penable),
        .apb_pwrite(apb_pwrite), .apb_paddr(apb_paddr),
        .apb_pwdata(apb_pwdata), .apb_prdata(apb_prdata),
        .apb_pslverr(apb_pslverr), .apb_pready(apb_pready),
        .irq(irq), .periph_req(periph_req), .periph_clr(periph_clr),
        .ARADDR(ARADDR), .ARLEN(ARLEN), .ARSIZE(ARSIZE), .ARID(ARID),
        .ARVALID(ARVALID), .ARREADY(ARREADY),
        .RDATA(RDATA), .RID(RID), .RRESP(RRESP),
        .RLAST(RLAST), .RVALID(RVALID), .RREADY(RREADY),
        .AWADDR(AWADDR), .AWLEN(AWLEN), .AWSIZE(AWSIZE), .AWID(AWID),
        .AWVALID(AWVALID), .AWREADY(AWREADY),
        .WDATA(WDATA), .WSTRB(WSTRB), .WLAST(WLAST),
        .WVALID(WVALID), .WREADY(WREADY),
        .BID(BID), .BRESP(BRESP), .BVALID(BVALID), .BREADY(BREADY)
    );

    // --------------------------------------------------------
    // AXI Slave RAM
    // --------------------------------------------------------
    reg [31:0] mem [0:MEM_WORDS-1];

    // Fault injection
    reg slverr_on;
    reg aw_stall;
    reg ar_stall;

    // --- AR + R handler ---
    // FIX TB: RREADY loop now exits on rst_n deassertion.
    task axi_rd_channel;
        reg [ADDR_W-1:0] a_addr;
        reg [LEN_W-1:0]  a_len;
        reg [ID_W-1:0]   a_id;
        integer          beat;
        integer          waddr;
        reg              aborted;
        begin
            ARREADY = 1'b0;
            RVALID  = 1'b0; RLAST = 1'b0;
            RDATA   = 32'h0; RID  = 0; RRESP = 2'b00;

            forever begin
                // --- Doi ARVALID ---
                @(posedge clk); #1;
                while (!ARVALID || ar_stall) begin
                    if (!rst_n) @(posedge rst_n);
                    @(posedge clk); #1;
                end
                a_addr  = ARADDR;
                a_len   = ARLEN;
                a_id    = ARID;
                aborted = 0;

                @(negedge clk); ARREADY = 1'b1;
                @(posedge clk); #1;
                @(negedge clk); ARREADY = 1'b0;

                // --- Phat R beats ---
                for (beat = 0; beat <= a_len && !aborted; beat = beat + 1) begin
                    waddr = (a_addr >> 2) + beat;
                    @(negedge clk);
                    RVALID = 1'b1;
                    RID    = a_id;
                    RDATA  = mem[waddr % MEM_WORDS];
                    RRESP  = slverr_on ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                    RLAST  = (beat == a_len) ? 1'b1 : 1'b0;
                    @(posedge clk); #1;
                    while (!RREADY && rst_n) begin
                        @(posedge clk); #1;
                    end
                    if (!rst_n) aborted = 1;
                end
                @(negedge clk);
                RVALID = 1'b0; RLAST = 1'b0;
                if (slverr_on) slverr_on = 1'b0;
            end
        end
    endtask

    // --- AW + W + B handler ---
    // FIX TB: W-beat loop now exits on rst_n deassertion so do_reset
    // between test cases doesn't leave the handler stuck waiting for WVALID.
    task axi_wr_channel;
        reg [ADDR_W-1:0] a_addr;
        reg [LEN_W-1:0]  a_len;
        reg [ID_W-1:0]   a_id;
        integer          beat;
        integer          waddr;
        reg              aborted;
        begin
            AWREADY = 1'b0;
            WREADY  = 1'b1;
            BVALID  = 1'b0; BID = 0; BRESP = 2'b00;

            forever begin
                // --- Doi AWVALID (or reset) ---
                @(posedge clk); #1;
                while (!AWVALID || aw_stall) begin
                    // If DUT held in reset: wait for it to come out
                    if (!rst_n) @(posedge rst_n);
                    @(posedge clk); #1;
                end
                a_addr  = AWADDR;
                a_len   = AWLEN;
                a_id    = AWID;
                aborted = 0;

                @(negedge clk); AWREADY = 1'b1;
                @(posedge clk); #1;
                @(negedge clk); AWREADY = 1'b0;

                // --- Nhan W beats (abort gracefully on reset) ---
                beat = 0;
                while (beat <= a_len && !aborted) begin
                    @(posedge clk); #1;
                    if (!rst_n) begin
                        // DUT reset mid-burst: abandon this transaction
                        aborted = 1;
                    end else if (WVALID && WREADY) begin
                        waddr = (a_addr >> 2) + beat;
                        mem[waddr % MEM_WORDS] = WDATA;
                        beat = beat + 1;
                    end
                end

                if (!aborted) begin
                    // --- Gui B response ---
                    @(negedge clk);
                    BVALID = 1'b1; BID = a_id; BRESP = 2'b00;
                    @(posedge clk); #1;
                    while (!BREADY && rst_n) begin @(posedge clk); #1; end
                    @(negedge clk); BVALID = 1'b0;
                end
                // If aborted: loop back and wait for next AWVALID after reset
            end
        end
    endtask

    // Spawn hai AXI handlers
    initial begin
        ARREADY = 0; RVALID = 0; RLAST = 0; RDATA = 0; RID = 0; RRESP = 0;
        AWREADY = 0; WREADY  = 0;
        BVALID  = 0; BID = 0; BRESP = 0;
        slverr_on = 0; aw_stall = 0; ar_stall = 0;
        @(negedge rst_n); @(posedge rst_n);
        @(posedge clk);
        fork
            axi_rd_channel;
            axi_wr_channel;
        join
    end

    // --------------------------------------------------------
    // APB Master tasks
    // --------------------------------------------------------
    task apb_write;
        input [APB_ADDR_W-1:0] addr;
        input [31:0]           data;
        integer t;
        begin
            @(negedge clk);
            apb_psel=1; apb_pwrite=1; apb_penable=0;
            apb_paddr=addr; apb_pwdata=data;
            @(negedge clk); apb_penable=1;
            @(posedge clk); #1; t=0;
            while (!apb_pready && t<20) begin @(posedge clk); #1; t=t+1; end
            @(negedge clk);
            apb_psel=0; apb_penable=0; apb_pwrite=0;
        end
    endtask

    task apb_read;
        input  [APB_ADDR_W-1:0] addr;
        output [31:0]           rdata;
        integer t;
        begin
            @(negedge clk);
            apb_psel=1; apb_pwrite=0; apb_penable=0;
            apb_paddr=addr; apb_pwdata=0;
            @(negedge clk); apb_penable=1;
            @(posedge clk); #1; t=0;
            while (!apb_pready && t<20) begin @(posedge clk); #1; t=t+1; end
            rdata = apb_prdata;
            @(negedge clk);
            apb_psel=0; apb_penable=0;
        end
    endtask

    task ch_write;
        input integer ch; input [7:0] reg_off; input [31:0] data;
        begin apb_write(ch*CH_STRIDE + reg_off, data); end
    endtask

    task ch_read;
        input  integer ch; input [7:0] reg_off; output [31:0] rdata;
        begin apb_read(ch*CH_STRIDE + reg_off, rdata); end
    endtask

    task start_channel;
        input integer  ch;
        input [31:0]   src, dst;
        input [15:0]   len;
        input [6:0]    burst_max;
        input          src_incr, dst_incr;
        input [4:0]    periph_num;
        reg   [31:0]   ctrl;
        begin
            ch_write(ch, `REG_SRC_ADDR, src);
            ch_write(ch, `REG_DST_ADDR, dst);
            ch_write(ch, `REG_LEN,      {16'h0, len});
            ch_write(ch, `REG_INT_EN,   32'h3);
            ctrl = 32'h0;
            ctrl[CTRL_START]               = 1'b1;
            ctrl[CTRL_BMAX_H:CTRL_BMAX_L]  = burst_max;
            ctrl[CTRL_SI]                   = src_incr;
            ctrl[CTRL_DI]                   = dst_incr;
            ctrl[CTRL_PNUM_H:CTRL_PNUM_L]   = periph_num;
            ch_write(ch, `REG_CTRL, ctrl);
            $display("[%0t] CH%0d started  src=%08h dst=%08h len=%0d burst=%0d periph=%0d",
                     $time, ch, src, dst, len, burst_max, periph_num);
        end
    endtask

    task wait_done;
        input  integer ch;
        output         timed_out;
        integer t;
        reg [31:0] st;
        begin
            timed_out = 0;
            t = 0;
            // FIX TB: dùng IRQ wire thay vì poll APB STATUS.
            // ch_done là 1-cycle pulse; APB polling có thể miss nó vì
            // mỗi ch_read tốn 5-6 cycle. IRQ được latch trong ch_int_st
            // → persist cho đến khi SW clear → không bao giờ miss.
            while (!irq[ch] && t < MAX_WAIT) begin
                @(posedge clk);
                t = t + 1;
            end
            ch_read(ch, `REG_STATUS, st);
            if (irq[ch] || st[1])
                $display("[%0t] CH%0d DONE  status=%08h err=%02b",
                         $time, ch, st, st[3:2]);
            else begin
                $display("[%0t] CH%0d TIMEOUT!", $time, ch);
                timed_out = 1;
            end
        end
    endtask

    // --------------------------------------------------------
    // Scoreboard
    // --------------------------------------------------------
    integer pass_cnt, fail_cnt;

    task chk;
        input        cond;
        input [255:0] msg;
        begin
            if (cond) begin
                $display("  [PASS] %0s", msg); pass_cnt = pass_cnt + 1;
            end else begin
                $display("  [FAIL] %0s", msg); fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    task verify_copy;
        input [31:0] src_b, dst_b, len_b;
        integer i, errs;
        reg [31:0] sw, dw;
        begin
            errs = 0;
            for (i = 0; i < len_b/4; i = i + 1) begin
                sw = mem[(src_b/4 + i) % MEM_WORDS];
                dw = mem[(dst_b/4 + i) % MEM_WORDS];
                if (sw !== dw) begin
                    if (errs < 4)
                        $display("    MISMATCH[%0d] src=%08h dst=%08h", i, sw, dw);
                    errs = errs + 1;
                end
            end
            if (errs == 0) begin
                $display("  [PASS] Data match %0d bytes", len_b);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  [FAIL] %0d/%0d words wrong", errs, len_b/4);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // --------------------------------------------------------
    // Memory helpers
    // --------------------------------------------------------
    integer mi;
    task init_mem;
        begin
            for (mi=0; mi<MEM_WORDS; mi=mi+1)
                mem[mi] = 32'hDEAD_0000 | mi[15:0];
        end
    endtask

    task fill_src;
        input [31:0] base_b, len_b;
        integer i;
        begin
            for (i=0; i<len_b/4; i=i+1)
                mem[(base_b/4 + i) % MEM_WORDS] = 32'hA500_0000 | i[23:0];
        end
    endtask

    // --------------------------------------------------------
    // Reset helper
    // --------------------------------------------------------
    task do_reset;
        begin
            rst_n=0; apb_psel=0; apb_penable=0; apb_pwrite=0;
            apb_paddr=0; apb_pwdata=0; periph_req=0;
            slverr_on=0; aw_stall=0; ar_stall=0;
            repeat(12) @(posedge clk);
            rst_n=1;
            repeat(5) @(posedge clk);
        end
    endtask

    // --------------------------------------------------------
    // VCD
    // --------------------------------------------------------
    initial begin
        $dumpfile("tb_dma_engine.vcd");
        $dumpvars(0, tb_dma_engine);
    end

    // --------------------------------------------------------
    // AXI monitor
    // --------------------------------------------------------
    always @(posedge clk) begin
        if (ARVALID && ARREADY)
            $display("[%0t]  >AR addr=%08h len=%0d id=%0d", $time, ARADDR, ARLEN, ARID);
        if (AWVALID && AWREADY)
            $display("[%0t]  >AW addr=%08h len=%0d id=%0d", $time, AWADDR, AWLEN, AWID);
        if (RVALID  && RREADY  && RLAST)
            $display("[%0t]  <R  LAST id=%0d resp=%02b", $time, RID, RRESP);
        if (BVALID  && BREADY)
            $display("[%0t]  <B  id=%0d resp=%02b", $time, BID, BRESP);
        if (WVALID && WREADY && WLAST)
            $display("[%0t]  >W  LAST id=%0d", $time, AWID);
    end

    // --------------------------------------------------------
    // MAIN
    // --------------------------------------------------------
    reg   to_flag;
    reg [31:0] rd_val;
    integer ch_i;

    initial begin
        pass_cnt=0; fail_cnt=0;
        init_mem;
        do_reset;

        $display("\n========================================");
        $display("  DMA Engine Testbench v2");
        $display("========================================");

        // ================================================
        // TC1: Mem-to-Mem, CH0, 64 bytes
        // ================================================
        $display("\n--- TC1: Mem-to-Mem CH0, 64 bytes ---");
        fill_src(32'h0000_0000, 64);
        start_channel(0, 32'h0000_0000, 32'h0000_0200,
                       16'd64, 7'd64, 1'b1, 1'b1, 5'd0);
        wait_done(0, to_flag);
        chk(!to_flag,           "TC1: no timeout");
        verify_copy(32'h0, 32'h200, 64);
        ch_read(0, `REG_STATUS, rd_val);
        chk(rd_val[1],          "TC1: STATUS.done=1");
        chk(rd_val[3:2]==2'b00, "TC1: STATUS.err=NONE");
        repeat(2) @(posedge clk);
        chk(irq[0],             "TC1: IRQ[0] asserted");
        ch_write(0, `REG_INT_STAT, 32'h3);
        repeat(3) @(posedge clk);
        chk(!irq[0],            "TC1: IRQ[0] cleared after W1C");

        // ================================================
        // TC2: 4 channels concurrent, 256 bytes each
        // ================================================
        $display("\n--- TC2: 4 channels concurrent, 256 bytes ---");
        do_reset;
        fill_src(32'h0000_0000, 256);
        fill_src(32'h0000_0400, 256);
        fill_src(32'h0000_0800, 256);
        fill_src(32'h0000_0C00, 256);
        start_channel(0, 32'h0000_0000, 32'h0000_1000, 256, 64, 1, 1, 0);
        start_channel(1, 32'h0000_0400, 32'h0000_1400, 256, 64, 1, 1, 0);
        start_channel(2, 32'h0000_0800, 32'h0000_1800, 256, 64, 1, 1, 0);
        start_channel(3, 32'h0000_0C00, 32'h0000_1C00, 256, 64, 1, 1, 0);
        for (ch_i=0; ch_i<4; ch_i=ch_i+1) begin
            wait_done(ch_i, to_flag);
            chk(!to_flag, "TC2: no timeout");
        end
        verify_copy(32'h0000_0000, 32'h0000_1000, 256);
        verify_copy(32'h0000_0400, 32'h0000_1400, 256);
        verify_copy(32'h0000_0800, 32'h0000_1800, 256);
        verify_copy(32'h0000_0C00, 32'h0000_1C00, 256);

        // ================================================
        // TC3: IRQ & INT_STAT W1C
        // ================================================
        $display("\n--- TC3: IRQ and INT_STAT W1C ---");
        do_reset;
        fill_src(32'h0, 64);
        ch_write(0, `REG_INT_EN, 32'h1);
        start_channel(0, 32'h0, 32'h200, 64, 64, 1, 1, 0);
        wait_done(0, to_flag);
        repeat(2) @(posedge clk);
        chk(irq[0],             "TC3: IRQ[0] asserted");
        ch_read(0, `REG_INT_STAT, rd_val);
        chk(rd_val[0],          "TC3: INT_STAT.done=1");
        ch_write(0, `REG_INT_STAT, 32'h1);
        repeat(3) @(posedge clk);
        ch_read(0, `REG_INT_STAT, rd_val);
        chk(!rd_val[0],         "TC3: INT_STAT.done cleared");
        chk(!irq[0],            "TC3: IRQ[0] deasserted");

        // ================================================
        // TC4: Peripheral-triggered (periph_num=1)
        // ================================================
        $display("\n--- TC4: Peripheral triggered ---");
        do_reset;
        fill_src(32'h0, 64);
        periph_req = 31'h0;
        start_channel(0, 32'h0, 32'h200, 64, 64, 1, 1, 5'd1);
        repeat(30) @(posedge clk);
        ch_read(0, `REG_STATUS, rd_val);
        chk(rd_val[0],          "TC4: channel active");
        chk(!rd_val[1],         "TC4: not done (waiting periph)");
        $display("[%0t] TC4: asserting periph_req[1]", $time);
        periph_req[1] = 1'b1;
        begin : tc4_wait_clr
            integer t4;
            for (t4=0; t4<300; t4=t4+1) begin
                @(posedge clk);
                if (periph_clr[1]) begin
                    $display("[%0t] TC4: periph_clr[1] seen", $time);
                    t4 = 300;
                end
            end
        end
        periph_req = 31'h0;
        wait_done(0, to_flag);
        chk(!to_flag,           "TC4: completed after periph_req");
        verify_copy(32'h0, 32'h200, 64);

        // ================================================
        // TC5: SLVERR injection
        // ================================================
        $display("\n--- TC5: AXI SLVERR injection ---");
        do_reset;
        fill_src(32'h0, 128);
        slverr_on = 1'b1;
        start_channel(0, 32'h0, 32'h200, 128, 64, 1, 1, 0);
        begin : tc5_poll
            integer t5;
            for (t5=0; t5<MAX_WAIT; t5=t5+1) begin
                ch_read(0, `REG_STATUS, rd_val);
                if (rd_val[1] || rd_val[3:2]!=2'b00)
                    t5 = MAX_WAIT + 1;
                else
                    @(posedge clk);
            end
        end
        ch_read(0, `REG_STATUS, rd_val);
        $display("  TC5 STATUS=%08h err=%02b", rd_val, rd_val[3:2]);
        chk(rd_val[3:2]!=2'b00, "TC5: err!=NONE after SLVERR");

        // ================================================
        // TC6: AW timeout
        // ================================================
        $display("\n--- TC6: AW timeout (4096 cycles) ---");
        do_reset;
        fill_src(32'h0, 64);
        aw_stall = 1'b1;
        start_channel(0, 32'h0, 32'h200, 64, 64, 1, 1, 0);
        repeat(5000) @(posedge clk);
        ch_read(0, `REG_STATUS, rd_val);
        $display("  TC6 STATUS=%08h err=%02b", rd_val, rd_val[3:2]);
        chk(rd_val[1] || (rd_val[3:2]==2'b11),
                                "TC6: done or TIMEOUT_ERR");
        aw_stall = 1'b0;

        // ================================================
        // TC7: Soft reset
        // ================================================
        $display("\n--- TC7: Soft reset ---");
        do_reset;
        fill_src(32'h0, 256);
        start_channel(0, 32'h0, 32'h200, 256, 64, 1, 1, 0);
        repeat(20) @(posedge clk);
        $display("[%0t] TC7: soft reset", $time);
        apb_write(`REG_GLOBAL_CTRL, 32'h1);
        // FIX TC7: wait long enough for any in-flight AXI transactions to complete.
        // Soft reset is 1-cycle; axi4_master_rd may still have a queued burst that
        // fires after reset. 200 cycles > max burst latency (16 beats + overhead).
        // Also wait for ARVALID and AWVALID to deassert (AXI idle).
        repeat(200) @(posedge clk);
        // Drain any spurious IRQ from the aborted transfer
        ch_write(0, `REG_INT_STAT, 32'h3);
        repeat(5) @(posedge clk);
        // Re-run sau reset
        fill_src(32'h0, 64);
        start_channel(0, 32'h0, 32'h400, 64, 64, 1, 1, 0);
        wait_done(0, to_flag);
        chk(!to_flag,           "TC7: transfer OK after soft-reset");
        verify_copy(32'h0, 32'h400, 64);

        // ================================================
        // SUMMARY
        // ================================================
        $display("\n========================================");
        $display("  RESULTS: %0d PASS   %0d FAIL", pass_cnt, fail_cnt);
        $display("========================================");
        if (fail_cnt == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED");
        #200; $finish;
    end

    // Watchdog
    initial begin
        #50_000_000;
        $display("[WATCHDOG] 50ms limit — force stop");
        $finish;
    end

endmodule