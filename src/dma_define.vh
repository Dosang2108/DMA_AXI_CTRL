// ============================================================
// dma_defines.vh
// File `define toàn cục — include một lần ở đầu mỗi file .v
//
//
// Cách include:
//   `include "dma_defines.vh"
// ============================================================
`ifndef DMA_DEFINES_VH
`define DMA_DEFINES_VH

// AXI4 bus
`define AXI_DATA_W   32          // data width — cố định cho thư viện này
`define AXI_STRB_W   4           // = AXI_DATA_W/8
`define AXI_BYTES    4           // số byte mỗi beat = AXI_DATA_W/8

// AXI ARSIZE/AWSIZE encoding cho 32-bit bus
`define AXI_SIZE_1B  3'b000
`define AXI_SIZE_2B  3'b001
`define AXI_SIZE_4B  3'b010      // default cho 32-bit bus

// AXI RRESP/BRESP codes
`define AXI_RESP_OKAY    2'b00
`define AXI_RESP_EXOKAY  2'b01
`define AXI_RESP_SLVERR  2'b10
`define AXI_RESP_DECERR  2'b11

// DMA channel error codes (2-bit, field trong STATUS register)
`define DMA_ERR_NONE    2'b00
`define DMA_ERR_SLVERR  2'b01
`define DMA_ERR_DECERR  2'b10
`define DMA_ERR_TIMEOUT 2'b11

// Register offsets (mỗi kênh chiếm 0x100 byte)
// Dùng cùng địa chỉ cho mọi project — không thay đổi
`define REG_SRC_ADDR   8'h00
`define REG_DST_ADDR   8'h04
`define REG_LEN        8'h08     // số byte cần transfer [15:0]
`define REG_CTRL       8'h0C     // [0]=start [7:1]=burst_max [8]=src_incr
                                  // [9]=dst_incr [14:10]=periph_num
`define REG_STATUS     8'h10     // RO: [0]=active [1]=done [3:2]=err
`define REG_INT_EN     8'h14     // [0]=done_ie [1]=err_ie
`define REG_INT_STAT   8'h18     // W1C: [0]=done_int [1]=err_int

// Global control (offset từ engine base, không phải channel base)
`define REG_GLOBAL_CTRL 12'hF00  // [0]=soft_rst

// Misc
`define DMA_CH_STRIDE  12        // bits: mỗi kênh cách nhau 2^12 = 0x1000
                                  // paddr[11+N_CH_W : 12] = channel index

`endif // DMA_DEFINES_VH
