// ============================================================
// osd_status.v - 检测链内部状态仪表盘 OSD (仪器版面板)
// 位置: 左下角 (8,568), 256x144。显隐控制 (组合方案):
//   1) 全屏视图常显: I_show = (view_mode!=0), ORIG/EDGE/MASK/DET 均绘制
//   2) 告警自动弹出: I_fire_valid=1 强制显示, 消退后保持3秒再收起;
//      QUAD 四分屏下火情期间同样弹出 (QUAD 显示判定流, 面板自然叠加)
// 面板分层 (优先级 高->低):
//   标题带(黄底黑字 "● LIVE", 心跳点) / 数据字(黄) / 电平表 /
//   状态灯条(绿/红) / 边框(黄, 告警闪红) / 半透明黑底衬 / 切角
// 数据行 (与黄色 CNT/MIN/MAX OSD 同款字模排版):
//   NOISE xxxxxxx  候选像素数 IIR (帧级 EMA 1/8, 在本模块内完成)
//   AREA  xxxxxxx  自适应面积阈值 (feature_fuse.O_min_area, CDC 同步)
//   LAT   xxxxx    ISP 段时延拍数 (csi域, 每帧 CDC 同步)
//   FPS   xxx      输入帧率 (1秒窗口量化)
//   SNAP  xxx      告警快照完成计数
// 字模: 内置等宽 8x16 ASCII (IBM VGA 经典点阵), 字母/数字/冒号同规格,
//   每字符固定 8 宽 x 16 高, 横向步进恒为 8px (与字形无关);
//   数值右对齐定宽 6位, 高位补0; 电平表 8段 x 4px (对数刻度 top4bit)。
// 接口与 osd_box/osd_alarm 同构: 输入 de/hsync/vsync, 输出叠加色+alpha+打1拍同步。
// ============================================================
module osd_status (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_freeze,     // 暂停时冻结状态/闪烁动画
    input  wire        I_de,
    input  wire        I_hsync,
    input  wire        I_vsync,
    input  wire        I_show,        // 全屏页常显; 检测四分屏由告警弹出
    input  wire        I_suppress,    // 处理对比四分屏时隐藏仪表盘
    input  wire        I_frame_tick,  // 输入帧 tick (hdmi域, 每帧一脉冲)
    input  wire [19:0] I_cand_count,  // 候选像素数 (每帧锁存, CDC 已同步)
    input  wire [19:0] I_area_th,     // 自适应面积阈值 (每帧锁存, CDC 已同步)
    input  wire [15:0] I_latency,     // ISP段时延 (csi拍数)
    input  wire [6:0]  I_fps,         // 输入帧率 (整数 fps)
    input  wire [7:0]  I_snap_count,  // 快照完成计数
    input  wire        I_fire_valid,  // 融合火情判定 (hdmi域)
    output reg  [23:0] O_osd_pixel,
    output reg  [3:0]  O_osd_alpha,
    output reg         O_de_out,
    output reg         O_hsync_out,
    output reg         O_vsync_out
);

    localparam [23:0] COLOR_YEL = 24'hfff200;  // 主色, 同 COLOR_DYN_TEXT
    localparam [23:0] COLOR_BLK = 24'h000000;
    localparam [23:0] COLOR_RED = 24'hff0000;  // 告警红
    localparam [23:0] COLOR_GRN = 24'h00c000;  // 状态绿
    localparam [10:0] OSD_X = 11'd8;
    localparam [9:0]  OSD_Y = 10'd568;
    localparam [10:0] OSD_W = 11'd256;
    localparam [9:0]  OSD_H = 10'd144;

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
        if(!I_rst_n)
            y_cnt <= 10'd0;
        else if(vsync_rise)
            y_cnt <= 10'd725;
        else if(x_cnt == 11'd1649) begin
            if(y_cnt == 10'd749) y_cnt <= 10'd0;
            else                 y_cnt <= y_cnt + 10'd1;
        end
    end

    // ================= 候选数 IIR (帧级 EMA 1/8, 与 feature_fuse 同系数) =================
    reg [19:0] noise_iir;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            noise_iir <= 20'd0;
        else if(I_frame_tick && !I_freeze)
            noise_iir <= noise_iir - (noise_iir >> 3) + (I_cand_count >> 3);
    end

    // 心跳/闪烁计数 (74.25MHz 域: [25]≈0.9s 周期, [24]≈0.45s 周期)
    reg [26:0] S_hb_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) S_hb_cnt <= 27'd0;
        else if(!I_freeze) S_hb_cnt <= S_hb_cnt + 27'd1;
    end
    wire hb_blink = S_hb_cnt[25];   // 心跳点 ~1Hz
    wire hb_flash = S_hb_cnt[24];   // 告警边框闪 ~2.2Hz

    // ================= 告警自动弹出 (组合显隐方案) =================
    // I_fire_valid 拉高即强制显示; 消退后保持 POPUP_HOLD 拍 (约3秒) 再收起
    localparam [27:0] POPUP_HOLD = 28'd222750000;  // 3s @74.25MHz (<2^28)
    reg        popup_active;
    reg [27:0] popup_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            popup_active <= 1'b0;
            popup_cnt    <= 28'd0;
        end else if(!I_freeze && I_fire_valid) begin
            popup_active <= 1'b1;       // 告警期间持续刷新
            popup_cnt    <= 28'd0;
        end else if(!I_freeze && popup_active) begin
            if(popup_cnt == POPUP_HOLD) popup_active <= 1'b0;  // 消退3秒后收起
            else                         popup_cnt    <= popup_cnt + 28'd1;
        end
    end
    wire panel_show = !I_suppress && (I_show || popup_active);

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

    function [27:0] bin20_bcd;
        input [19:0] bin;
        reg [47:0] sh;
        integer i;
        begin
            sh = {28'b0, bin};
            for(i=0; i<20; i=i+1) begin
                if(sh[47:44] >= 5) sh[47:44] = sh[47:44] + 4'd3;
                if(sh[43:40] >= 5) sh[43:40] = sh[43:40] + 4'd3;
                if(sh[39:36] >= 5) sh[39:36] = sh[39:36] + 4'd3;
                if(sh[35:32] >= 5) sh[35:32] = sh[35:32] + 4'd3;
                if(sh[31:28] >= 5) sh[31:28] = sh[31:28] + 4'd3;
                if(sh[27:24] >= 5) sh[27:24] = sh[27:24] + 4'd3;
                if(sh[23:20] >= 5) sh[23:20] = sh[23:20] + 4'd3;
                sh = sh << 1;
            end
            bin20_bcd = sh[47:20];
        end
    endfunction

    wire [27:0] noise_bcd = bin20_bcd(noise_iir);
    wire [27:0] area_bcd  = bin20_bcd(I_area_th);
    wire [19:0] lat_bcd   = bin16_bcd(I_latency);
    wire [11:0] fps_bcd   = bin8_bcd({1'b0, I_fps});
    wire [11:0] snap_bcd  = bin8_bcd(I_snap_count);

    // ================= 面板分区 =================
    wire in_panel = panel_show && (x_cnt >= OSD_X) && (x_cnt < OSD_X + OSD_W) &&
                    (y_cnt >= OSD_Y) && (y_cnt < OSD_Y + OSD_H);
    wire [7:0] x_local = x_cnt - OSD_X;   // 0..255
    wire [7:0] y_local = y_cnt - OSD_Y;   // 0..143

    wire       in_title = (y_local < 8'd24);        // 标题带 24px
    wire [7:0] row_y    = y_local - 8'd24;          // 数据区行内 0..119
    wire [2:0] line = (row_y >= 8'd96) ? 3'd4 :
                      (row_y >= 8'd72) ? 3'd3 :
                      (row_y >= 8'd48) ? 3'd2 :
                      (row_y >= 8'd24) ? 3'd1 : 3'd0;  // 0=NOISE 1=AREA 2=LAT 3=FPS 4=SNAP
    wire [7:0] y_l  = row_y - ({5'd0, line} * 8'd24);  // 行内 y 0..23
    wire letters_band = (y_l >= 8'd4) && (y_l < 8'd20);  // 字母/冒号带(居中16px)

    // ================= 字符选择 + 行/列换算 (与黄色OSD同式) =================
    reg [7:0]  char_ascii;
    reg [3:0]  char_row;
    reg [2:0]  char_col;   // 0..7 (8x16字模 1:1, 无缩放)
    reg [7:0]  char_lx;
    reg        char_on;
    reg        is_title_glyph;   // 标题黑字
    reg        is_area_digit;    // 告警红数字 (AREA行)

    always @* begin
        char_on       = 1'b0;
        char_ascii    = 8'h20;
        char_row      = 4'd0;
        char_col      = 3'd0;
        char_lx       = 8'd0;
        is_title_glyph = 1'b0;
        is_area_digit  = 1'b0;

        if(in_panel) begin
            // ---- 标题带: LIVE x28-59 (4字 x 8px), 带内 y 4-19 ----
            if(in_title) begin
                if((x_local >= 8'd28) && (x_local < 8'd60) &&
                   (y_local >= 8'd4)  && (y_local < 8'd20)) begin
                    char_lx  = x_local - 8'd28;   // 0..31
                    char_col = char_lx[2:0];
                    char_row = y_local - 8'd4;
                    case(char_lx[4:3])
                    2'd0: char_ascii = 8'h4c;  // L
                    2'd1: char_ascii = 8'h49;  // I
                    2'd2: char_ascii = 8'h56;  // V
                    2'd3: char_ascii = 8'h45;  // E
                    endcase
                    char_on        = 1'b1;
                    is_title_glyph = 1'b1;
                end
            end
            // ---- 数据区 ----
            else if(row_y < 8'd120) begin
                // 字母槽 x6-45 (5字 x 8px, 等宽); 槽号取char_lx[5:3](0..39共5格, [4:3]会把第5格x32-39误作首格2'd0)
                if((x_local >= 8'd6) && (x_local < 8'd46) && letters_band) begin
                    char_lx  = x_local - 8'd6;   // 0..39
                    char_col = char_lx[2:0];
                    char_row = y_l - 8'd4;
                    case(line)
                    3'd0: case(char_lx[5:3])     // NOISE
                            2'd0: char_ascii = 8'h4e; 2'd1: char_ascii = 8'h4f;
                            2'd2: char_ascii = 8'h49; 2'd3: char_ascii = 8'h53;
                            default: char_ascii = 8'h45;
                          endcase
                    3'd1: case(char_lx[5:3])     // AREA
                            2'd0: char_ascii = 8'h41; 2'd1: char_ascii = 8'h52;
                            2'd2: char_ascii = 8'h45; 2'd3: char_ascii = 8'h41;
                            default: char_ascii = 8'h20;
                          endcase
                    3'd2: case(char_lx[5:3])     // LAT
                            2'd0: char_ascii = 8'h4c; 2'd1: char_ascii = 8'h41;
                            2'd2: char_ascii = 8'h54;
                            default: char_ascii = 8'h20;
                          endcase
                    3'd3: case(char_lx[5:3])     // FPS
                            2'd0: char_ascii = 8'h46; 2'd1: char_ascii = 8'h50;
                            2'd2: char_ascii = 8'h53;
                            default: char_ascii = 8'h20;
                          endcase
                    default: case(char_lx[5:3])  // SNAP
                            2'd0: char_ascii = 8'h53; 2'd1: char_ascii = 8'h4e;
                            2'd2: char_ascii = 8'h41; 2'd3: char_ascii = 8'h50;
                            default: char_ascii = 8'h20;
                          endcase
                    endcase
                    char_on = (char_ascii != 8'h20);
                end
                // 冒号 x46-53 (8x16 1:1)
                else if((x_local >= 8'd46) && (x_local < 8'd54) && letters_band) begin
                    char_ascii = 8'h3a;
                    char_lx    = x_local - 8'd46;   // 0..7
                    char_col   = char_lx[2:0];
                    char_row   = y_l - 8'd4;
                    char_on    = 1'b1;
                end
                // 数字槽 x54-101 (6槽 x 8px, 右对齐, 高位补0)
                else if((x_local >= 8'd54) && (x_local < 8'd102) && letters_band) begin : DIGIT_SLOT
                    reg [2:0] s;
                    reg [3:0] d;
                    reg [2:0] j;
                    reg [23:0] disp_bcd;
                    s = (x_local - 8'd54) / 8'd8;          // 槽号 0..5 (左起)
                    d = 4'd0;
                    j = 3'd5 - s;                          // 右对齐: 槽5=个位
                    char_lx  = x_local - 8'd54 - ({5'd0, s} * 8'd8);  // 槽内 0..7
                    char_col = char_lx[2:0];
                    char_row = y_l - 8'd4;
                    disp_bcd = 24'd0;
                    case(line)
                    3'd0: begin char_on = 1'b1; disp_bcd = (noise_bcd[27:24] != 4'd0) ? 24'h999999 : noise_bcd[23:0]; end  // 候选数≤921600, 6位足; 溢出饱和
                    3'd1: begin char_on = 1'b1; disp_bcd = (area_bcd[27:24]  != 4'd0) ? 24'h999999 : area_bcd[23:0];  is_area_digit = 1'b1; end
                    3'd2: begin char_on = 1'b1; disp_bcd = {4'd0, lat_bcd};  end  // 16位BCD=5位, 高位补0
                    3'd3: begin char_on = 1'b1; disp_bcd = {12'd0, fps_bcd}; end  // 8位BCD=3位, 高位补0
                    default: begin char_on = 1'b1; disp_bcd = {12'd0, snap_bcd}; end
                    endcase
                    d = disp_bcd[(j*4) +: 4];
                    char_ascii = 8'h30 + d;
                end
            end
        end
    end

    // ================= 内置等宽 8x16 ASCII 字模 (IBM VGA 经典点阵) =================
    // 每字符固定 8 宽 x 16 高, bit7=最左像素; 步进恒为 8px, 与字形无关。
    // 含 A-Z / 0-9 / 冒号; 其余 ASCII 输出空行。与 osd_char_lib(旧16x16)互不影响。
    reg [7:0] font_bits;
    always @* begin
        case(char_ascii)
        8'h41: case(char_row)   // 'A'
            4'd0: font_bits = 8'h00; 4'd1: font_bits = 8'h00;
            4'd2: font_bits = 8'h10; 4'd3: font_bits = 8'h38;
            4'd4: font_bits = 8'h6c; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hc6; 4'd7: font_bits = 8'hfe;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'hc6; 4'd11: font_bits = 8'hc6;
            default: font_bits = 8'h00;
        endcase
        8'h42: case(char_row)   // 'B'
            4'd2: font_bits = 8'hfc; 4'd3: font_bits = 8'h66;
            4'd4: font_bits = 8'h66; 4'd5: font_bits = 8'h66;
            4'd6: font_bits = 8'h7c; 4'd7: font_bits = 8'h66;
            4'd8: font_bits = 8'h66; 4'd9: font_bits = 8'h66;
            4'd10: font_bits = 8'hfc;
            default: font_bits = 8'h00;
        endcase
        8'h43: case(char_row)   // 'C'
            4'd2: font_bits = 8'h3c; 4'd3: font_bits = 8'h66;
            4'd4: font_bits = 8'hc2; 4'd5: font_bits = 8'hc0;
            4'd6: font_bits = 8'hc0; 4'd7: font_bits = 8'hc0;
            4'd8: font_bits = 8'hc2; 4'd9: font_bits = 8'h66;
            4'd10: font_bits = 8'h3c;
            default: font_bits = 8'h00;
        endcase
        8'h44: case(char_row)   // 'D'
            4'd2: font_bits = 8'hf8; 4'd3: font_bits = 8'h6c;
            4'd4: font_bits = 8'h66; 4'd5: font_bits = 8'h66;
            4'd6: font_bits = 8'h66; 4'd7: font_bits = 8'h66;
            4'd8: font_bits = 8'h66; 4'd9: font_bits = 8'h6c;
            4'd10: font_bits = 8'hf8;
            default: font_bits = 8'h00;
        endcase
        8'h45: case(char_row)   // 'E'
            4'd2: font_bits = 8'hfe; 4'd3: font_bits = 8'h62;
            4'd4: font_bits = 8'h68; 4'd5: font_bits = 8'h78;
            4'd6: font_bits = 8'h68; 4'd7: font_bits = 8'h60;
            4'd8: font_bits = 8'h62; 4'd9: font_bits = 8'hfe;
            default: font_bits = 8'h00;
        endcase
        8'h46: case(char_row)   // 'F'
            4'd2: font_bits = 8'hfe; 4'd3: font_bits = 8'h62;
            4'd4: font_bits = 8'h68; 4'd5: font_bits = 8'h78;
            4'd6: font_bits = 8'h68; 4'd7: font_bits = 8'h60;
            4'd8: font_bits = 8'h60; 4'd9: font_bits = 8'hf0;
            default: font_bits = 8'h00;
        endcase
        8'h47: case(char_row)   // 'G'
            4'd2: font_bits = 8'h3e; 4'd3: font_bits = 8'h60;
            4'd4: font_bits = 8'hc0; 4'd5: font_bits = 8'hc0;
            4'd6: font_bits = 8'hc0; 4'd7: font_bits = 8'hce;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'h66;
            4'd10: font_bits = 8'h3e;
            default: font_bits = 8'h00;
        endcase
        8'h48: case(char_row)   // 'H'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hfe; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'hc6;
            default: font_bits = 8'h00;
        endcase
        8'h49: case(char_row)   // 'I'
            4'd2: font_bits = 8'h3c; 4'd3: font_bits = 8'h18;
            4'd4: font_bits = 8'h18; 4'd5: font_bits = 8'h18;
            4'd6: font_bits = 8'h18; 4'd7: font_bits = 8'h18;
            4'd8: font_bits = 8'h18; 4'd9: font_bits = 8'h18;
            4'd10: font_bits = 8'h3c;
            default: font_bits = 8'h00;
        endcase
        8'h4a: case(char_row)   // 'J'
            4'd2: font_bits = 8'h1e; 4'd3: font_bits = 8'h0c;
            4'd4: font_bits = 8'h0c; 4'd5: font_bits = 8'h0c;
            4'd6: font_bits = 8'h0c; 4'd7: font_bits = 8'h0c;
            4'd8: font_bits = 8'hcc; 4'd9: font_bits = 8'hcc;
            4'd10: font_bits = 8'h78;
            default: font_bits = 8'h00;
        endcase
        8'h4b: case(char_row)   // 'K'
            4'd2: font_bits = 8'he6; 4'd3: font_bits = 8'h66;
            4'd4: font_bits = 8'h6c; 4'd5: font_bits = 8'h78;
            4'd6: font_bits = 8'h78; 4'd7: font_bits = 8'h6c;
            4'd8: font_bits = 8'h66; 4'd9: font_bits = 8'h66;
            4'd10: font_bits = 8'he6;
            default: font_bits = 8'h00;
        endcase
        8'h4c: case(char_row)   // 'L'
            4'd2: font_bits = 8'hf0; 4'd3: font_bits = 8'h60;
            4'd4: font_bits = 8'h60; 4'd5: font_bits = 8'h60;
            4'd6: font_bits = 8'h60; 4'd7: font_bits = 8'h60;
            4'd8: font_bits = 8'h62; 4'd9: font_bits = 8'hfe;
            default: font_bits = 8'h00;
        endcase
        8'h4d: case(char_row)   // 'M'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'hee;
            4'd4: font_bits = 8'hfe; 4'd5: font_bits = 8'hfe;
            4'd6: font_bits = 8'hd6; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'hc6;
            default: font_bits = 8'h00;
        endcase
        8'h4e: case(char_row)   // 'N'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'he6;
            4'd4: font_bits = 8'hf6; 4'd5: font_bits = 8'hfe;
            4'd6: font_bits = 8'hde; 4'd7: font_bits = 8'hce;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'hc6;
            default: font_bits = 8'h00;
        endcase
        8'h4f: case(char_row)   // 'O'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hc6; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h50: case(char_row)   // 'P'
            4'd2: font_bits = 8'hfc; 4'd3: font_bits = 8'h66;
            4'd4: font_bits = 8'h66; 4'd5: font_bits = 8'h66;
            4'd6: font_bits = 8'h7c; 4'd7: font_bits = 8'h60;
            4'd8: font_bits = 8'h60; 4'd9: font_bits = 8'h60;
            4'd10: font_bits = 8'hf0;
            default: font_bits = 8'h00;
        endcase
        8'h51: case(char_row)   // 'Q'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hc6; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hd6; 4'd9: font_bits = 8'hde;
            4'd10: font_bits = 8'h7c; 4'd11: font_bits = 8'h0c;
            4'd12: font_bits = 8'h0e;
            default: font_bits = 8'h00;
        endcase
        8'h52: case(char_row)   // 'R'
            4'd2: font_bits = 8'hfc; 4'd3: font_bits = 8'h66;
            4'd4: font_bits = 8'h66; 4'd5: font_bits = 8'h66;
            4'd6: font_bits = 8'h7c; 4'd7: font_bits = 8'h6c;
            4'd8: font_bits = 8'h66; 4'd9: font_bits = 8'h66;
            4'd10: font_bits = 8'he6;
            default: font_bits = 8'h00;
        endcase
        8'h53: case(char_row)   // 'S'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'h60;
            4'd6: font_bits = 8'h38; 4'd7: font_bits = 8'h0c;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h54: case(char_row)   // 'T'
            4'd2: font_bits = 8'h7e; 4'd3: font_bits = 8'h7e;
            4'd4: font_bits = 8'h5a; 4'd5: font_bits = 8'h18;
            4'd6: font_bits = 8'h18; 4'd7: font_bits = 8'h18;
            4'd8: font_bits = 8'h18; 4'd9: font_bits = 8'h18;
            4'd10: font_bits = 8'h3c;
            default: font_bits = 8'h00;
        endcase
        8'h55: case(char_row)   // 'U'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hc6; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h56: case(char_row)   // 'V'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hc6; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'h6c;
            4'd10: font_bits = 8'h38;
            default: font_bits = 8'h00;
        endcase
        8'h57: case(char_row)   // 'W'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'hd6; 4'd7: font_bits = 8'hfe;
            4'd8: font_bits = 8'hfe; 4'd9: font_bits = 8'hee;
            4'd10: font_bits = 8'hc6;
            default: font_bits = 8'h00;
        endcase
        8'h58: case(char_row)   // 'X'
            4'd2: font_bits = 8'hc6; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'h6c; 4'd5: font_bits = 8'h7c;
            4'd6: font_bits = 8'h38; 4'd7: font_bits = 8'h38;
            4'd8: font_bits = 8'h7c; 4'd9: font_bits = 8'h6c;
            4'd10: font_bits = 8'hc6;
            default: font_bits = 8'h00;
        endcase
        8'h59: case(char_row)   // 'Y'
            4'd2: font_bits = 8'h66; 4'd3: font_bits = 8'h66;
            4'd4: font_bits = 8'h66; 4'd5: font_bits = 8'h66;
            4'd6: font_bits = 8'h3c; 4'd7: font_bits = 8'h18;
            4'd8: font_bits = 8'h18; 4'd9: font_bits = 8'h18;
            4'd10: font_bits = 8'h3c;
            default: font_bits = 8'h00;
        endcase
        8'h5a: case(char_row)   // 'Z'
            4'd2: font_bits = 8'hfe; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'h8c; 4'd5: font_bits = 8'h18;
            4'd6: font_bits = 8'h30; 4'd7: font_bits = 8'h60;
            4'd8: font_bits = 8'hc2; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'hfe;
            default: font_bits = 8'h00;
        endcase
        8'h30: case(char_row)   // '0'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hce; 4'd5: font_bits = 8'hde;
            4'd6: font_bits = 8'hf6; 4'd7: font_bits = 8'he6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h31: case(char_row)   // '1'
            4'd2: font_bits = 8'h18; 4'd3: font_bits = 8'h38;
            4'd4: font_bits = 8'h78; 4'd5: font_bits = 8'h18;
            4'd6: font_bits = 8'h18; 4'd7: font_bits = 8'h18;
            4'd8: font_bits = 8'h18; 4'd9: font_bits = 8'h18;
            4'd10: font_bits = 8'h7e;
            default: font_bits = 8'h00;
        endcase
        8'h32: case(char_row)   // '2'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'h06; 4'd5: font_bits = 8'h0c;
            4'd6: font_bits = 8'h18; 4'd7: font_bits = 8'h30;
            4'd8: font_bits = 8'h60; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'hfe;
            default: font_bits = 8'h00;
        endcase
        8'h33: case(char_row)   // '3'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'h06; 4'd5: font_bits = 8'h06;
            4'd6: font_bits = 8'h3c; 4'd7: font_bits = 8'h06;
            4'd8: font_bits = 8'h06; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h34: case(char_row)   // '4'
            4'd2: font_bits = 8'h0c; 4'd3: font_bits = 8'h1c;
            4'd4: font_bits = 8'h3c; 4'd5: font_bits = 8'h6c;
            4'd6: font_bits = 8'hcc; 4'd7: font_bits = 8'hfe;
            4'd8: font_bits = 8'h0c; 4'd9: font_bits = 8'h0c;
            4'd10: font_bits = 8'h1e;
            default: font_bits = 8'h00;
        endcase
        8'h35: case(char_row)   // '5'
            4'd2: font_bits = 8'hfe; 4'd3: font_bits = 8'hc0;
            4'd4: font_bits = 8'hc0; 4'd5: font_bits = 8'hc0;
            4'd6: font_bits = 8'hfc; 4'd7: font_bits = 8'h06;
            4'd8: font_bits = 8'h06; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h36: case(char_row)   // '6'
            4'd2: font_bits = 8'h38; 4'd3: font_bits = 8'h60;
            4'd4: font_bits = 8'hc0; 4'd5: font_bits = 8'hc0;
            4'd6: font_bits = 8'hfc; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h37: case(char_row)   // '7'
            4'd2: font_bits = 8'hfe; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'h06; 4'd5: font_bits = 8'h06;
            4'd6: font_bits = 8'h0c; 4'd7: font_bits = 8'h18;
            4'd8: font_bits = 8'h30; 4'd9: font_bits = 8'h30;
            4'd10: font_bits = 8'h30;
            default: font_bits = 8'h00;
        endcase
        8'h38: case(char_row)   // '8'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'h7c; 4'd7: font_bits = 8'hc6;
            4'd8: font_bits = 8'hc6; 4'd9: font_bits = 8'hc6;
            4'd10: font_bits = 8'h7c;
            default: font_bits = 8'h00;
        endcase
        8'h39: case(char_row)   // '9'
            4'd2: font_bits = 8'h7c; 4'd3: font_bits = 8'hc6;
            4'd4: font_bits = 8'hc6; 4'd5: font_bits = 8'hc6;
            4'd6: font_bits = 8'h7e; 4'd7: font_bits = 8'h06;
            4'd8: font_bits = 8'h06; 4'd9: font_bits = 8'h0c;
            4'd10: font_bits = 8'h78;
            default: font_bits = 8'h00;
        endcase
        8'h3a: case(char_row)   // ':'
            4'd4: font_bits = 8'h18; 4'd5: font_bits = 8'h18;
            4'd10: font_bits = 8'h18; 4'd11: font_bits = 8'h18;
            default: font_bits = 8'h00;
        endcase
        default: font_bits = 8'h00;
        endcase
    end
    wire glyph = char_on && font_bits[7 - char_col];

    // ================= 电平表 (NOISE/AREA 行, 8段x4px, 对数刻度 top4bit) =================
    wire        in_meter = ((line == 3'd0) || (line == 3'd1)) &&
                           (x_local >= 8'd208) && (x_local < 8'd247) &&
                           (y_l >= 8'd8) && (y_l < 8'd16);
    wire [8:0]  seg_x    = {1'b0, x_local} - 8'd208;      // 0..38 (208超出6位, 必须8位字面量!)
    wire [2:0]  seg_idx  = seg_x / 6'd5;                   // 段号 0..7
    wire [2:0]  seg_rem  = seg_x % 6'd5;
    wire        seg_ink  = (seg_rem != 3'd4);              // 段间 1px 缝
    wire [3:0]  meter_val = (line == 3'd0) ? noise_iir[19:16] : I_area_th[19:16];
    wire [4:0]  seg_th   = ({2'b0, seg_idx} << 1) + 5'd1;  // 段 i 亮: val >= 2i+1 (1,3..15)
    wire        seg_lit  = meter_val >= seg_th;

    // ================= 像素合成 (分层优先级: 字 > 电平表 > 灯条 > 边框 > 底衬) =================
    reg [23:0] pix_color;
    reg [3:0]  pix_alpha;
    always @* begin
        pix_color = 24'd0;
        pix_alpha = 4'd0;
        if(in_panel) begin
            if(in_title) begin
                // 标题带: 实心黄底
                pix_color = COLOR_YEL;
                pix_alpha = 4'hF;
                // 心跳点 8x8 (黑闪, 告警红常亮)
                if((x_local >= 8'd14) && (x_local < 8'd22) &&
                   (y_local >= 8'd8)  && (y_local < 8'd16) &&
                   (I_fire_valid || hb_blink))
                    pix_color = I_fire_valid ? COLOR_RED : COLOR_BLK;
                // LIVE 黑字
                if(glyph && is_title_glyph)
                    pix_color = COLOR_BLK;
            end else begin
                // 数据区: 半透明黑底衬
                pix_color = COLOR_BLK;
                pix_alpha = 4'd8;
                // 电平表亮段 (暗段即底衬)
                if(in_meter && seg_ink && seg_lit) begin
                    pix_color = COLOR_YEL;
                    pix_alpha = 4'hF;
                end
                // 数据字 (字母/冒号/数字, AREA行告警变红)
                if(glyph && !is_title_glyph) begin
                    pix_color = (is_area_digit && I_fire_valid) ? COLOR_RED : COLOR_YEL;
                    pix_alpha = 4'hF;
                end
                // 状态灯条 2px (绿正常/红告警)
                if((x_local >= 8'd3) && (x_local < 8'd5) &&
                   (y_local >= 8'd28) && (y_local < 8'd136)) begin
                    pix_color = I_fire_valid ? COLOR_RED : COLOR_GRN;
                    pix_alpha = 4'hF;
                end
                // 边框 1px (黄, 告警 ~2Hz 闪红)
                if((x_local == 8'd0) || (x_local == 8'd255) || (y_local == 8'd143)) begin
                    pix_color = (I_fire_valid && hb_flash) ? COLOR_RED : COLOR_YEL;
                    pix_alpha = 4'hF;
                end
            end
            // 切角: 四角 1px 透明
            if(((x_local < 8'd2) && (y_local < 8'd2)) ||
               ((x_local < 8'd2) && (y_local > 8'd141)) ||
               ((x_local > 8'd253) && (y_local < 8'd2)) ||
               ((x_local > 8'd253) && (y_local > 8'd141))) begin
                pix_color = 24'd0;
                pix_alpha = 4'd0;
            end
        end
    end

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
            O_osd_pixel <= pix_color;
            O_osd_alpha <= pix_alpha;
        end
    end

endmodule
