// ============================================================
// osd_param.v - 算法参数 OSD 叠加 (烟头阈值 LUM_TH + ISP时延 LATENCY + 置信度 CF)
// 位置: QUAD 模式 RED MASK 象限 (左下, x 0-639 / y 360-719) 右上角
// 三行文字, 排版与 hdmi_mixer 的黄色 CNT/MIN/MAX OSD 完全一致:
//   TH :123      烟头亮点阈值, 3位十进制
//   LAT:12345    ISP流水线时延, 5位十进制(单位csi拍)
//   CF :123      检测置信度, 3位十进制
// 规格同黄色OSD: 字母 16x16 1:1 (osd_char_lib), 冒号 8 宽横 2x,
//   数字 15x24 (横 16/15, 纵 16/24), 颜色 0xfff200 (COLOR_DYN_TEXT), 不透明。
// 接口与 osd_box/osd_alarm 同构: 输入 de/hsync/vsync, 输出叠加色+alpha+打1拍同步。
// BCD 用组合展开 double-dabble (8bit/16bit 位宽小, 组合路径短, 上屏非关键路径)。
// ============================================================
module osd_param (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_de,
    input  wire        I_hsync,
    input  wire        I_vsync,
    input  wire [7:0]  I_lum_th,     // 烟头亮点阈值 (0-255)
    input  wire [15:0] I_latency,    // ISP流水线时延 (csi拍数)
    input  wire [7:0]  I_confidence, // 检测置信度 (0-99)
    output reg  [23:0] O_osd_pixel,
    output reg  [3:0]  O_osd_alpha,
    output reg         O_de_out,
    output reg         O_hsync_out,
    output reg         O_vsync_out
);

    // ================= 坐标计数器 (与 osd_box 同, 锚定 hsync/vsync) =================
    reg hs_d1, vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin hs_d1 <= 1'b0; vs_d1 <= 1'b0; end
        else         begin hs_d1 <= I_hsync; vs_d1 <= I_vsync; end
    end
    wire hsync_rise = I_hsync && !hs_d1;
    wire vsync_rise = I_vsync && !vs_d1;

    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    reg        locked;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            x_cnt <= 11'd0;
        else if(hsync_rise)
            x_cnt <= 11'd1390 + 11'd1;
        else if(x_cnt == 11'd1649)
            x_cnt <= 11'd0;
        else
            x_cnt <= x_cnt + 11'd1;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            y_cnt  <= 10'd0;
            locked <= 1'b0;
        end else if(vsync_rise) begin
            y_cnt  <= 10'd725;
            locked <= 1'b1;
        end else if(x_cnt == 11'd1649) begin
            if(y_cnt == 10'd749) y_cnt <= 10'd0;
            else                 y_cnt <= y_cnt + 10'd1;
        end
    end

    // ================= 二进制 -> BCD (组合展开 double-dabble) =================
    function [11:0] bin8_bcd;
        input [7:0] bin;
        reg [19:0] sh;
        integer i;
        begin
            sh = {12'b0, bin};
            for(i=0; i<8; i=i+1) begin
                if(sh[19:16] >= 5) sh[19:16] = sh[19:16] + 4'd3;
                if(sh[15:12] >= 5) sh[15:12] = sh[15:12] + 4'd3;
                if(sh[11:8]  >= 5) sh[11:8]  = sh[11:8]  + 4'd3;
                sh = sh << 1;
            end
            bin8_bcd = sh[19:8];
        end
    endfunction

    function [19:0] bin16_bcd;
        input [15:0] bin;
        reg [35:0] sh;
        integer i;
        begin
            sh = {20'b0, bin};
            for(i=0; i<16; i=i+1) begin
                if(sh[35:32] >= 5) sh[35:32] = sh[35:32] + 4'd3;
                if(sh[31:28] >= 5) sh[31:28] = sh[31:28] + 4'd3;
                if(sh[27:24] >= 5) sh[27:24] = sh[27:24] + 4'd3;
                if(sh[23:20] >= 5) sh[23:20] = sh[23:20] + 4'd3;
                if(sh[19:16] >= 5) sh[19:16] = sh[19:16] + 4'd3;
                sh = sh << 1;
            end
            bin16_bcd = sh[35:16];
        end
    endfunction

    wire [11:0] th_bcd  = bin8_bcd(I_lum_th);
    wire [19:0] lat_bcd = bin16_bcd(I_latency);
    wire [11:0] cf_bcd  = bin8_bcd(I_confidence);

    // ================= 显示布局 (与黄色 CNT/MIN/MAX OSD 同款字模与排版) =================
    // 位置: QUAD 模式 RED MASK 象限 (x 0-639 / y 360-719) 右上角
    // 窗口 170x72 @ (468,364), 三行各 24 高:
    //   行0 "TH:xxx"   行1 "LAT:xxxxx"   行2 "CF:xxx"
    // 规格与 hdmi_mixer MIN/MAX 行完全一致:
    //   字母: 16x16 字模 1:1, 槽宽 16 (x0-15/16-31/32-47), 行内 y 4-19 垂直居中
    //   冒号: x48-55, 8 宽横 2x (col=(cx*16)/8), 同行内 y 4-19, CHAR_COLON 同字模
    //   数字: 15x24 (col=(dx*16)/15, row=(dy*16)/24), 自 x80 起 15px 槽, 行内 y 0-23 全高
    //   颜色: 0xfff200 同 COLOR_DYN_TEXT, alpha 不透明
    localparam [10:0] OSD_X = 11'd468;
    localparam [9:0]  OSD_Y = 10'd364;
    localparam [9:0]  OSD_W = 10'd170;
    localparam [9:0]  OSD_H = 10'd72;

    wire in_param = (x_cnt >= OSD_X) && (x_cnt < OSD_X + OSD_W) &&
                    (y_cnt >= OSD_Y) && (y_cnt < OSD_Y + OSD_H);
    wire [6:0] x_local = x_cnt - OSD_X;
    wire [6:0] y_local = y_cnt - OSD_Y;
    wire [1:0] line_sel = (y_local >= 7'd48) ? 2'd2 :
                          (y_local >= 7'd24) ? 2'd1 : 2'd0;  // 行号 = y/24 (0=TH,1=LAT,2=CF)
    wire [6:0] y_l      = y_local - ({5'd0, line_sel} * 7'd24);  // 行内 y 0..23
    wire letters_band = (y_l >= 7'd4) && (y_l < 7'd20);          // 字母/冒号带(居中16px)

    reg [7:0]  char_ascii;
    reg [3:0]  char_row;
    reg [3:0]  char_col;
    reg [4:0]  char_lx;
    reg        char_on;

    // 字符选择 + 行/列换算 (字母1:1 / 冒号横2x / 数字15x24, 与黄色OSD同式)
    always @* begin
        char_on    = 1'b0;
        char_ascii = 8'h20;
        char_row   = 4'd0;
        char_col   = 4'd0;
        char_lx    = 5'd0;

        if(in_param) begin
            // ---- 字母槽 x0-47 (16px/字) ----
            if((x_local < 7'd48) && letters_band) begin
                char_lx   = x_local[4:0];       // 槽内 0-15
                char_col  = char_lx[3:0];
                char_row  = y_l - 7'd4;         // 带内 4-19 -> 字模行 0-15
                case(line_sel)
                2'd0: begin   // TH
                    if      (x_local < 7'd16) char_ascii = 8'h54;  // T
                    else if (x_local < 7'd32) char_ascii = 8'h48;  // H
                    else                      char_ascii = 8'h20;
                end
                2'd1: begin   // LAT
                    if      (x_local < 7'd16) char_ascii = 8'h4c;  // L
                    else if (x_local < 7'd32) char_ascii = 8'h41;  // A
                    else if (x_local < 7'd48) char_ascii = 8'h54;  // T
                    else                      char_ascii = 8'h20;
                end
                default: begin // CF
                    if      (x_local < 7'd16) char_ascii = 8'h43;  // C
                    else if (x_local < 7'd32) char_ascii = 8'h46;  // F
                    else                      char_ascii = 8'h20;
                end
                endcase
                char_on = (char_ascii != 8'h20);
            end
            // ---- 冒号 x48-55 (8宽, 横2x) ----
            else if((x_local >= 7'd48) && (x_local < 7'd56) && letters_band) begin
                char_ascii = 8'h3a;
                char_lx    = x_local - 7'd48;   // 0..7
                char_col   = (char_lx * 10'd16) / 10'd8;
                char_row   = y_l - 7'd4;
                char_on    = 1'b1;
            end
            // ---- 数字槽 x80+ (15x24/字, 行内 y 0-23 全高) ----
            else if((x_local >= 8'd80) && (x_local < 8'd170) && (y_l < 7'd24)) begin
                case(line_sel)
                2'd0: begin   // TH:xxx
                    if      (x_local < 8'd95)  begin char_lx = x_local - 8'd80;  char_ascii = 8'h30 + th_bcd[11:8];  char_on = 1'b1; end
                    else if (x_local < 8'd110) begin char_lx = x_local - 8'd95;  char_ascii = 8'h30 + th_bcd[7:4];   char_on = 1'b1; end
                    else if (x_local < 8'd125) begin char_lx = x_local - 8'd110; char_ascii = 8'h30 + th_bcd[3:0];   char_on = 1'b1; end
                end
                2'd1: begin   // LAT:xxxxx
                    if      (x_local < 8'd95)  begin char_lx = x_local - 8'd80;  char_ascii = 8'h30 + lat_bcd[19:16]; char_on = 1'b1; end
                    else if (x_local < 8'd110) begin char_lx = x_local - 8'd95;  char_ascii = 8'h30 + lat_bcd[15:12]; char_on = 1'b1; end
                    else if (x_local < 8'd125) begin char_lx = x_local - 8'd110; char_ascii = 8'h30 + lat_bcd[11:8];  char_on = 1'b1; end
                    else if (x_local < 8'd140) begin char_lx = x_local - 8'd125; char_ascii = 8'h30 + lat_bcd[7:4];   char_on = 1'b1; end
                    else if (x_local < 8'd155) begin char_lx = x_local - 8'd140; char_ascii = 8'h30 + lat_bcd[3:0];   char_on = 1'b1; end
                end
                default: begin // CF:xxx
                    if      (x_local < 8'd95)  begin char_lx = x_local - 8'd80;  char_ascii = 8'h30 + cf_bcd[11:8];   char_on = 1'b1; end
                    else if (x_local < 8'd110) begin char_lx = x_local - 8'd95;  char_ascii = 8'h30 + cf_bcd[7:4];    char_on = 1'b1; end
                    else if (x_local < 8'd125) begin char_lx = x_local - 8'd110; char_ascii = 8'h30 + cf_bcd[3:0];    char_on = 1'b1; end
                end
                endcase
                if(char_on) begin
                    char_col = (char_lx * 10'd16) / 10'd15;
                    char_row = (y_l * 10'd16) / 10'd24;
                end
            end
        end
    end

    // ================= 字库查表 (ASCII -> 16bit行点阵) =================
    wire [15:0] font_bits;
    osd_char_lib u_font(
        .I_char     (char_ascii),
        .I_row      (char_row),
        .O_row_bits (font_bits)
    );
    wire glyph = char_on && font_bits[15 - char_col];

    // ================= 输出寄存 (1拍流水: 叠加色/alpha 与打1拍同步同拍) =================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_osd_pixel <= 24'd0;
            O_osd_alpha <= 4'd0;
            O_de_out    <= 1'b0;
            O_hsync_out <= 1'b0;
            O_vsync_out <= 1'b0;
        end else begin
            O_de_out    <= I_de;
            O_hsync_out <= I_hsync;
            O_vsync_out <= I_vsync;
            if(glyph) begin
                O_osd_pixel <= 24'hfff200;   // 黄色, 同黄色OSD (COLOR_DYN_TEXT)
                O_osd_alpha <= 4'hF;         // 不透明
            end else begin
                O_osd_pixel <= 24'd0;
                O_osd_alpha <= 4'd0;
            end
        end
    end

endmodule
