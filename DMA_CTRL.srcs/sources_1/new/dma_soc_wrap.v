// ============================================================
// dma_soc_wrap.v — Layer 3 project wrapper (ví dụ)
// Dự án: SoC với 4 kênh DMA, AXI4 32-bit, APB config
//
// File này CHỈ làm 2 việc:
//   1. Chọn parameters cho dự án cụ thể
//   2. Map tên port theo convention của SoC
//
// ============================================================
`include "dma_define.vh"

module dma_soc_wrap (
    // SoC clock/reset (tên theo convention dự án)
    input  wire         clk_bus,
    input  wire         rst_bus_n,

    // APB từ CPU subsystem
    input  wire         s_apb_psel,
    input  wire         s_apb_penable,
    input  wire         s_apb_pwrite,
    input  wire [12:0]  s_apb_paddr,   // 13-bit: 4 kênh × 0x1000
    input  wire [31:0]  s_apb_pwdata,
    output wire [31:0]  s_apb_prdata,
    output wire         s_apb_pslverr,
    output wire         s_apb_pready,

    // IRQ → interrupt controller
    output wire [3:0]   dma_irq,

    // Peripheral DMA triggers (e.g. UART, SPI, I2C)
    input  wire [31:1]  periph_dma_req,
    output wire [31:1]  periph_dma_clr,

    // AXI4 master → memory interconnect
    output wire [31:0]  m_axi_araddr,
    output wire [3:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [3:0]   m_axi_arid,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [31:0]  m_axi_rdata,
    input  wire [3:0]   m_axi_rid,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready,

    output wire [31:0]  m_axi_awaddr,
    output wire [3:0]   m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [3:0]   m_axi_awid,
    output wire         m_axi_awvalid,
    input  wire         m_axi_awready,
    output wire [31:0]  m_axi_wdata,
    output wire [3:0]   m_axi_wstrb,
    output wire         m_axi_wlast,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,
    input  wire [3:0]   m_axi_bid,
    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output wire         m_axi_bready
);

    dma_engine #(
        .N_CH         (4),          // 4 kênh DMA
        .N_CH_W       (2),          // clog2(4) = 2
        .ADDR_W       (32),         // AXI4 32-bit address
        .LEN_W        (4),          // ARLEN 4-bit = max 16 beats/burst
        .ID_W         (4),          // 4-bit ID
        .FIFO_DEPTH   (16),         // 16 × 4B = 64B FIFO/channel
        .MAX_BURST    (64),         // 64 bytes = 16 beats max
        .BURST_W      (7),          // clog2(128) = 7
        .TOKEN_W      (4),          // max 15 outstanding rd bursts
        .OUT_W        (4),          // max 15 outstanding cmds
        .TIMEOUT_W    (12),         // 4096 cycles timeout
        .DEF_TOKENS   (4),          // default 4 tokens/kênh
        .DEF_OUTS     (4),          // default 4 outstanding cmds/kênh
        .PERIPH_NUM_W (5),          // 32 peripheral slots
        .LEN_FIELD_W  (16),         // max transfer = 64KB
        .APB_ADDR_W   (14)          // 4 channels × 2^12 = 2^14 → 14-bit địa chỉ
    ) u_dma_engine (
        .clk          (clk_bus),
        .rst_n        (rst_bus_n),
        .apb_psel     (s_apb_psel),
        .apb_penable  (s_apb_penable),
        .apb_pwrite   (s_apb_pwrite),
        .apb_paddr    (s_apb_paddr),
        .apb_pwdata   (s_apb_pwdata),
        .apb_prdata   (s_apb_prdata),
        .apb_pslverr  (s_apb_pslverr),
        .apb_pready   (s_apb_pready),
        .irq          (dma_irq),
        .periph_req   (periph_dma_req),
        .periph_clr   (periph_dma_clr),
        .ARADDR       (m_axi_araddr),
        .ARLEN        (m_axi_arlen),
        .ARSIZE       (m_axi_arsize),
        .ARID         (m_axi_arid),
        .ARVALID      (m_axi_arvalid),
        .ARREADY      (m_axi_arready),
        .RDATA        (m_axi_rdata),
        .RID          (m_axi_rid),
        .RRESP        (m_axi_rresp),
        .RLAST        (m_axi_rlast),
        .RVALID       (m_axi_rvalid),
        .RREADY       (m_axi_rready),
        .AWADDR       (m_axi_awaddr),
        .AWLEN        (m_axi_awlen),
        .AWSIZE       (m_axi_awsize),
        .AWID         (m_axi_awid),
        .AWVALID      (m_axi_awvalid),
        .AWREADY      (m_axi_awready),
        .WDATA        (m_axi_wdata),
        .WSTRB        (m_axi_wstrb),
        .WLAST        (m_axi_wlast),
        .WVALID       (m_axi_wvalid),
        .WREADY       (m_axi_wready),
        .BID          (m_axi_bid),
        .BRESP        (m_axi_bresp),
        .BVALID       (m_axi_bvalid),
        .BREADY       (m_axi_bready)
    );

endmodule