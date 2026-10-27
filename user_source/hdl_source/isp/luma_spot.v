/******************************************************************************
 * luma_spot.v — 暗光亮点掩码 (纯比较器, 与 color_mask 同构)
 *
 * 位置: user_source/hdl_source/isp/luma_spot.v
 * 时钟: 与 isp_top 同源 (axi4s_video_aclk = S_csi_rx_clk)
 *
 * 功能: 在 AWB 输出 RGB96 (每拍 4 像素 × RGB888) 上逐像素提取高亮亮点:
 *   ① 亮度 Y = R/4 + G/2 + B/4   (移位相加, 与 color_mask 同算法, 8bit 不溢出)
 *   ② 亮点   = (Y >= LUM_TH)
 *
 * 设计依据 (室内烟头检测):
 *   - 点燃的烟头/暗光中的火点呈现为高亮白炽点 (Y 高、面积小、孤立)
 *   - 白墙/灯光等大面积亮区虽然也 Y 高, 但连通域面积大 ——
 *     由 detect_top 的"面积上限"排除 (不算烟头), 本模块只做纯阈值
 *   - 火焰主体由 color_mask (R>G>B 高饱和) 负责, 本模块只负责暗光亮点
 *
 * 输出:
 *   O_mask   : 亮点掩码, 每拍 4 bit (O_mask[0] 对应最先像素)
 *   O_tdata  : 8bit 伪彩可视化 (命中=高亮白, 未命中=暗灰), RGB96 流
 *   O_tlast/O_tuser/O_tvalid : 与 O_tdata/O_mask 同拍对齐
 *****************************************************************************/
module luma_spot #(
    parameter IMG_WIDTH  = 1280,   // 图像宽度 (保留, 便于扩展)
    parameter IMG_HEIGHT = 720,    // 图像高度 (保留, 便于扩展)
    parameter LUM_TH     = 8'd150  // 亮点亮度下限 (暗光亮点; 纯白过曝由面积上限排除)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire        I_tvalid,
    input  wire [95:0] I_tdata,    // AWB 输出 RGB96: [23:0]=像素1, [95:72]=像素4
    input  wire [7:0]  I_lum_th,   // 亮点亮度下限 (按键标定, csi 域寄存器; parameter 为复位默认)

    output reg  [3:0]  O_mask,     // 亮点掩码 (每拍 4 像素)
    output reg  [95:0] O_tdata,    // 伪彩 RGB96
    output reg         O_tlast,
    output reg         O_tuser,
    output reg         O_tvalid
);

    // ---- 拆像素: px_a=像素1(最先) ... px_d=像素4 ----
    wire [23:0] px_a = I_tdata[23:0];
    wire [23:0] px_b = I_tdata[47:24];
    wire [23:0] px_c = I_tdata[71:48];
    wire [23:0] px_d = I_tdata[95:72];

    // ---- 亮度 Y = R/4 + G/2 + B/4 (与 color_mask 同算法) ----
    wire [7:0] y_a = {2'b00,px_a[23:18]} + {1'b0,px_a[15:9]} + {2'b00,px_a[7:2]};
    wire [7:0] y_b = {2'b00,px_b[23:18]} + {1'b0,px_b[15:9]} + {2'b00,px_b[7:2]};
    wire [7:0] y_c = {2'b00,px_c[23:18]} + {1'b0,px_c[15:9]} + {2'b00,px_c[7:2]};
    wire [7:0] y_d = {2'b00,px_d[23:18]} + {1'b0,px_d[15:9]} + {2'b00,px_d[7:2]};

    // ---- 亮点判定 (纯比较) ----
    wire m_a = (y_a >= I_lum_th);
    wire m_b = (y_b >= I_lum_th);
    wire m_c = (y_c >= I_lum_th);
    wire m_d = (y_d >= I_lum_th);

    // ---- 8bit 伪彩: 命中=高亮白(烟头火光), 未命中=暗灰 ----
    wire [23:0] o_a = m_a ? 24'hE8F0FF : 24'h101010;
    wire [23:0] o_b = m_b ? 24'hE8F0FF : 24'h101010;
    wire [23:0] o_c = m_c ? 24'hE8F0FF : 24'h101010;
    wire [23:0] o_d = m_d ? 24'hE8F0FF : 24'h101010;

    // ================= 一级寄存器输出 (掩码/伪彩/同步同拍) =================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_mask   <= 4'd0;
            O_tdata  <= 96'd0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
        end else begin
            O_mask   <= {m_d, m_c, m_b, m_a};     // O_mask[0] = 像素1
            O_tdata  <= {o_d, o_c, o_b, o_a};     // [95:72]=像素4, [23:0]=像素1
            O_tlast  <= I_tlast;
            O_tuser  <= I_tuser;
            O_tvalid <= I_tvalid;
        end
    end

endmodule
