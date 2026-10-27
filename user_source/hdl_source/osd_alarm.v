module osd_alarm(
    input               I_clk,
    input               I_rst_n,
    input               I_hsync,
    input               I_vsync,
    input               I_alarm,
    input               I_quad_mode,   // 1=四分屏: 文字 1x 缩至右下象限右上角 (1224,368)

    output reg [23:0]   O_osd_pixel,
    output reg [3:0]    O_osd_alpha,
    output reg          O_de_out,
    output reg          O_hsync_out,
    output reg          O_vsync_out
);

// ============================================================
// 告警OSD: I_alarm=1 时在画面右上角叠加红色"ALARM"点阵文字
// 字模: 8x16 基础点阵经 2x2 整数放大为 16x32 (源像素1点->屏幕2x2块,
//       屏幕坐标右移1位还原字模坐标, 无乘法/无重采样, 边缘锐利)
// 纯PL像素流水线, 无帧缓存/不访问DDR/零丢帧, 像素时钟 I_clk
// 时序参数与本工程 720p VTC (uivtc 1650x750@60) 一致:
//   H_ActiveSize=1280 H_FrameSize=1650 H_SyncStart=1390 H_SyncEnd=1430
//   V_ActiveSize=720  V_FrameSize=750  V_SyncStart=725  V_SyncEnd=730
// 无外部 de 输入, de 由模块内同步相位锚定计数器生成:
//   hsync上升沿锚定 x:=1391 (下一拍即真实相位, 每行自校准);
//   vsync上升沿锚定 y:=725 (VTC 整行 y=725 保持 vsync 高, 不能锚 726,
//   否则帧内永久 +1 漂移、帧末回绕判据失效);
//   locked 首次 vsync 前置位, 之前禁止 de (防上电首帧消隐区误输出)
// I_alarm 在 vsync 上升沿锁存, 帧内稳定 (锁存点位于消隐区 y=725,
//   第N帧显示第N-1帧锁存值, 标准一帧显示延迟)
// 输出: 1拍流水, O_osd_pixel/O_osd_alpha 与打1拍的 de/hsync/vsync 同拍,
//   保证与视频流水线对齐
// Alpha约定 (外部混合): out = (video*(16-a) + osd*a + 8) >> 4
//   a=0 全透明(视频直通无损失), a=15 近似不透明(15/16)
// 无组合逻辑环: 计数器->纯组合命中->寄存器输出, 环路全部由D触发器打断
// ============================================================

    // ---- 时序/显示参数 ----
    localparam [10:0] H_FRAME      = 11'd1650; // 行总拍数
    localparam [10:0] H_ACTIVE     = 11'd1280; // 行有效像素
    localparam [10:0] H_SYNC_START = 11'd1390; // hsync 起始相位(锚点)
    localparam [9:0]  V_FRAME      = 10'd750;  // 场总行数
    localparam [9:0]  V_ACTIVE     = 10'd720;  // 场有效行
    localparam [9:0]  V_SYNC_START = 10'd725;  // vsync 起始相位(锚点)
    localparam        FRAME_EN     = 1'b0;     // 屏幕边缘空心警示框: 1=叠加(旧版功能) 0=仅文字
    localparam [10:0] BORDER_T     = 11'd4;    // 警示框线宽 (FRAME_EN=1 时生效)
    localparam [10:0] TEXT_RIGHT   = 11'd1272; // 文本右边缘x(不含)
    localparam [9:0]  TEXT_TOP     = 10'd16;   // 文本顶行y
    localparam [3:0]  ALPHA_OSD    = 4'hF;     // 文字叠加alpha (15=不透明)
    localparam [4:0]  CHAR_W       = 5'd16;    // 字符宽 (8x16字模2x放大)
    localparam [5:0]  CHAR_H       = 6'd32;    // 字符高 (2x放大)
    localparam [3:0]  CHAR_GAP     = 4'd6;     // 字符间距 (放大后适当增宽)
    localparam [2:0]  N_CHAR       = 3'd5;     // "ALARM" 共5字符
    // 字符单元宽 = CHAR_W+CHAR_GAP = 22; 文本总宽 = 5*16 + 4*6 = 104
    // (用字面量 11'd104: Verilog 按操作数位宽计算, 5*16 会截断, 不能写成乘法形式)
    // 文本区域: [1168,1271] x [16,47]
    localparam [10:0] TEXT_X0      = TEXT_RIGHT - 11'd104;

    // ---- 同步脉冲上升沿检测 ----
    reg hs_d1, vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            hs_d1 <= 1'b0;
            vs_d1 <= 1'b0;
        end else begin
            hs_d1 <= I_hsync;
            vs_d1 <= I_vsync;
        end
    end
    wire hsync_rise = I_hsync && !hs_d1;
    wire vsync_rise = I_vsync && !vs_d1;

    // ---- 告警信号帧头锁存 (帧内稳定) ----
    reg r_alarm;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            r_alarm <= 1'b0;
        else if(vsync_rise)
            r_alarm <= I_alarm;
    end

    // ---- 内部时序计数器 (无外部de, 相位锚定 + 自由运行) ----
    // x: 每拍+1, hsync上升沿锚定到 H_SYNC_START+1, 行末回绕
    // y: 行末+1, vsync上升沿锚定到 V_SYNC_START, 场末回绕
    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    reg        locked;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            x_cnt <= 11'd0;
        else if(hsync_rise)
            x_cnt <= H_SYNC_START + 11'd1;
        else if(x_cnt == H_FRAME - 11'd1)
            x_cnt <= 11'd0;
        else
            x_cnt <= x_cnt + 11'd1;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            y_cnt  <= 10'd0;
            locked <= 1'b0;
        end else begin
            if(vsync_rise) begin
                y_cnt  <= V_SYNC_START;
                locked <= 1'b1;
            end else if(x_cnt == H_FRAME - 11'd1) begin
                if(y_cnt == V_FRAME - 10'd1)
                    y_cnt <= 10'd0;
                else
                    y_cnt <= y_cnt + 10'd1;
            end
        end
    end

    // ---- 内部生成的像素有效 ----
    wire de_w = (x_cnt < H_ACTIVE);   // 列在有效区
    wire de_h = (y_cnt < V_ACTIVE);   // 行在有效区
    wire de   = locked && de_w && de_h;

    // ---- 屏幕边缘空心警示框命中 (FRAME_EN=1 时启用, 默认关闭) ----
    // 边框区 = 距屏幕边缘 < BORDER_T; 内部完全不命中 -> 空心不填充
    wire in_frame_h = (x_cnt < BORDER_T) || (x_cnt >= (H_ACTIVE - BORDER_T));
    wire in_frame_v = (y_cnt < BORDER_T) || (y_cnt >= (V_ACTIVE - BORDER_T));
    wire frame_hit  = FRAME_EN && de && (in_frame_h || in_frame_v);

    // ---- 8x16 点阵字模 "ALARM" (bit7=左, bit0=右, 行0=顶, 行14/15空) ----
    function [7:0] font_byte;
        input [2:0] ch;    // 字符索引 0..4: A L A R M
        input [3:0] row;   // 行 0..15
        begin
            case (ch)
            3'd0: case (row)   // 'A'
                4'd0 : font_byte = 8'h18;
                4'd1 : font_byte = 8'h3C;
                4'd2 : font_byte = 8'h66;
                4'd3 : font_byte = 8'hC3;
                4'd4 : font_byte = 8'hC3;
                4'd5 : font_byte = 8'hFF;
                4'd6 : font_byte = 8'hFF;
                4'd7 : font_byte = 8'hC3;
                4'd8 : font_byte = 8'hC3;
                4'd9 : font_byte = 8'hC3;
                4'd10: font_byte = 8'hC3;
                4'd11: font_byte = 8'hC3;
                4'd12: font_byte = 8'hC3;
                4'd13: font_byte = 8'hC3;
                default: font_byte = 8'h00;
            endcase
            3'd1: case (row)   // 'L'
                4'd0 : font_byte = 8'hC0;
                4'd1 : font_byte = 8'hC0;
                4'd2 : font_byte = 8'hC0;
                4'd3 : font_byte = 8'hC0;
                4'd4 : font_byte = 8'hC0;
                4'd5 : font_byte = 8'hC0;
                4'd6 : font_byte = 8'hC0;
                4'd7 : font_byte = 8'hC0;
                4'd8 : font_byte = 8'hC0;
                4'd9 : font_byte = 8'hC0;
                4'd10: font_byte = 8'hC0;
                4'd11: font_byte = 8'hC0;
                4'd12: font_byte = 8'hC0;
                4'd13: font_byte = 8'hFF;
                default: font_byte = 8'h00;
            endcase
            3'd2: case (row)   // 'A'
                4'd0 : font_byte = 8'h18;
                4'd1 : font_byte = 8'h3C;
                4'd2 : font_byte = 8'h66;
                4'd3 : font_byte = 8'hC3;
                4'd4 : font_byte = 8'hC3;
                4'd5 : font_byte = 8'hFF;
                4'd6 : font_byte = 8'hFF;
                4'd7 : font_byte = 8'hC3;
                4'd8 : font_byte = 8'hC3;
                4'd9 : font_byte = 8'hC3;
                4'd10: font_byte = 8'hC3;
                4'd11: font_byte = 8'hC3;
                4'd12: font_byte = 8'hC3;
                4'd13: font_byte = 8'hC3;
                default: font_byte = 8'h00;
            endcase
            3'd3: case (row)   // 'R'
                4'd0 : font_byte = 8'hFE;
                4'd1 : font_byte = 8'hC3;
                4'd2 : font_byte = 8'hC3;
                4'd3 : font_byte = 8'hC3;
                4'd4 : font_byte = 8'hC3;
                4'd5 : font_byte = 8'hC3;
                4'd6 : font_byte = 8'hFE;
                4'd7 : font_byte = 8'hFC;
                4'd8 : font_byte = 8'hC6;
                4'd9 : font_byte = 8'hC3;
                4'd10: font_byte = 8'hC3;
                4'd11: font_byte = 8'hC3;
                4'd12: font_byte = 8'hC3;
                4'd13: font_byte = 8'hC3;
                default: font_byte = 8'h00;
            endcase
            3'd4: case (row)   // 'M'
                4'd0 : font_byte = 8'hC3;
                4'd1 : font_byte = 8'hE7;
                4'd2 : font_byte = 8'hFF;
                4'd3 : font_byte = 8'hFF;
                4'd4 : font_byte = 8'hDB;
                4'd5 : font_byte = 8'hDB;
                4'd6 : font_byte = 8'hC3;
                4'd7 : font_byte = 8'hC3;
                4'd8 : font_byte = 8'hC3;
                4'd9 : font_byte = 8'hC3;
                4'd10: font_byte = 8'hC3;
                4'd11: font_byte = 8'hC3;
                4'd12: font_byte = 8'hC3;
                4'd13: font_byte = 8'hC3;
                default: font_byte = 8'h00;
            endcase
            default: font_byte = 8'h00;
            endcase
        end
    endfunction

    // ---- 文字命中判定 (纯组合, 无除法, 无组合环) ----
    // 非QUAD: 2x2 整数放大, 屏幕坐标 >>1 还原为 8x16 字模坐标
    //   字符单元宽 = CHAR_W+CHAR_GAP = 22, 各字符单元左边界: 0/22/44/66/88
    //   (用字面量阈值, 不能用乘法形式: Verilog 按操作数位宽计算)
    // QUAD: 1x 原生尺寸 (单元宽10, 阈值 10/20/30/40/48), 位置移右下象限
    // 坐标/阈值 mux 使两模式共用同一解码链
    wire [10:0] TX0   = I_quad_mode ? 11'd1224 : TEXT_X0;
    wire [9:0]  TY0   = I_quad_mode ? 10'd368  : TEXT_TOP;
    wire [10:0] TCHW  = I_quad_mode ? 11'd8    : CHAR_W;    // 字符宽
    wire [5:0]  TCHH  = I_quad_mode ? 6'd16    : CHAR_H;    // 文本高
    wire [10:0] TW    = I_quad_mode ? 11'd48   : 11'd104;   // 文本总宽
    wire [10:0] T1    = I_quad_mode ? 11'd10   : 11'd22;    // 单元宽 x1
    wire [10:0] T2    = I_quad_mode ? 11'd20   : 11'd44;
    wire [10:0] T3    = I_quad_mode ? 11'd30   : 11'd66;
    wire [10:0] T4    = I_quad_mode ? 11'd40   : 11'd88;
    wire [10:0] T5    = I_quad_mode ? 11'd48   : 11'd104;
    wire in_text_x = (x_cnt >= TX0) && (x_cnt < TX0 + TW);
    wire in_text_y = (y_cnt >= TY0) && (y_cnt < TY0 + TCHH);
    wire [10:0] dx       = x_cnt - TX0;   // 文本区内 0..TW-1, 区外回绕大值(被门控)
    wire [9:0]  cy       = y_cnt - TY0;   // 文本区内 0..TCHH-1
    wire [3:0]  font_row = I_quad_mode ? cy[3:0] : cy[4:1];  // 源字模行 (1x直用 / 2x>>1)
    reg  [2:0]  ch_idx;                       // 字符索引 0..4
    reg  [2:0]  cx;                           // 源字模列 0..7
    reg         ch_col;                       // 当前列落在字符内(非间距列)
    reg  [10:0] cx_local;                     // 字符单元内列
    always @* begin
        ch_col   = 1'b0;
        ch_idx   = 3'd0;
        cx       = 3'd0;
        cx_local = 11'd0;
        if      (dx < T1) begin ch_idx = 3'd0; cx_local = dx;      ch_col = (cx_local < TCHW); end
        else if (dx < T2) begin ch_idx = 3'd1; cx_local = dx - T1; ch_col = (cx_local < TCHW); end
        else if (dx < T3) begin ch_idx = 3'd2; cx_local = dx - T2; ch_col = (cx_local < TCHW); end
        else if (dx < T4) begin ch_idx = 3'd3; cx_local = dx - T3; ch_col = (cx_local < TCHW); end
        else if (dx < T5) begin ch_idx = 3'd4; cx_local = dx - T4; ch_col = (cx_local < TCHW); end
        cx = I_quad_mode ? cx_local[2:0] : cx_local[3:1];   // 源字模列 (1x直用 / 2x>>1)
    end
    // 文字命中 = 有效区 && 告警 && 文本区域 && 字符列 && 点阵点亮(2x2块)
    wire [7:0] text_font_byte = font_byte(ch_idx, font_row);
    wire text_hit = de && r_alarm && in_text_x && in_text_y && ch_col &&
                    text_font_byte[7 - cx];

    // 总命中: 警示框(可选) 或 文字, 均在告警有效时
    wire hit = r_alarm && (frame_hit || text_hit);

    // ---- 输出寄存 (1拍流水: 叠加色/alpha 与打1拍的同步信号同拍输出) ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_osd_pixel <= 24'd0;
            O_osd_alpha <= 4'd0;
            O_de_out    <= 1'b0;
            O_hsync_out <= 1'b0;
            O_vsync_out <= 1'b0;
        end else begin
            O_de_out    <= de;          // 内部生成的像素有效, 打1拍
            O_hsync_out <= I_hsync;     // 同步信号打1拍, 流水线对齐
            O_vsync_out <= I_vsync;
            if(hit) begin
                O_osd_pixel <= 24'hFF0000;   // 红色文字
                O_osd_alpha <= ALPHA_OSD;    // 4'hF = 不透明
            end else begin
                O_osd_pixel <= 24'd0;        // 无OSD: 全透明直通
                O_osd_alpha <= 4'd0;
            end
        end
    end

endmodule
