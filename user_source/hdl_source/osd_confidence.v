// ============================================================
// osd_confidence.v — 检测置信度 OSD (TARGET 标签同款字体, 简洁)
//
// 字体: 5x7 点阵白字 + 深色底条, 与 quad_top 的 "TARGET" 标签完全同风格
//   (全屏: 每行重复 4 次 -> 20x28 字形, 字距 24px;
//    QUAD: 与 TARGET 标题相同 5x7 字形, 字距 10px)
//
// 显示位置:
//   1) 第四张图 (DET 判定视图): 底部居中, 正常字号
//   2) 四分屏 (QUAD) 第 4 块区域: (648,390), 与标题同字号并留出间距
//
// 文本: "CONF 092%  CONFIRMED"  (等级 ≥80)
//       "CONF 045%  WARNING"    (40~79)
//       "CONF 012%  NORMAL"     (<40)
// 置信度取 明火四维融合置信度 与 烟头置信度 的较大者; 等级词按等级着色。
// ============================================================
module osd_confidence (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_de,
    input  wire        I_hsync,
    input  wire        I_vsync,
    input  wire        I_show,          // 1 = 第四张图(DET)
    input  wire        I_quad_mode,     // 1 = QUAD 四分屏
    input  wire [7:0]  I_fire_conf,     // 明火四维融合置信度 0~100
    input  wire [7:0]  I_cig_conf,      // 烟头置信度 0~100
    output reg  [23:0] O_osd_pixel,
    output reg  [3:0]  O_osd_alpha,
    output reg         O_de_out,
    output reg         O_hsync_out,
    output reg         O_vsync_out
);

    // ================= 坐标计数器 (与 quad_top/osd_* 同, 锚定 hsync/vsync) =================
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

    // ================= 置信度 / 等级 =================
    wire [7:0] conf  = (I_fire_conf > I_cig_conf) ? I_fire_conf : I_cig_conf;
    wire [1:0] level = (conf >= 8'd80) ? 2'd2 : (conf >= 8'd40) ? 2'd1 : 2'd0;
    wire active = I_quad_mode | I_show;

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
    wire [11:0] conf_bcd = bin8_bcd(conf);

    // ================= 位置/字号 =================
    // QUAD 第4块: 与 TARGET 标签同字号 (5x7, 字距10) @ (648,390)
    // 单画面(DET): 5x7 -> 20x28 (4倍), 字距24, 居中且上移到安全区 (避开底部过扫描)
    wire [10:0] X0  = I_quad_mode ? 11'd648  : 11'd400;
    wire [9:0]  Y0  = I_quad_mode ? 10'd390  : 10'd620;
    wire [10:0] ADV = I_quad_mode ? 11'd10   : 11'd24;
    wire [9:0]  GH  = I_quad_mode ? 10'd14   : 10'd28;
    wire [10:0] TW  = ADV * 11'd20;                    // 20 字符总宽
    wire [10:0] BX0 = X0 - 11'd4;
    wire [9:0]  BY0 = Y0 - 10'd3;
    wire [10:0] BX1 = X0 + TW + 11'd4;
    wire [9:0]  BY1 = Y0 + GH + 10'd3;

    wire in_text = active && (x_cnt >= X0) && (x_cnt < X0 + TW) &&
                              (y_cnt >= Y0) && (y_cnt < Y0 + GH);
    wire in_bar  = active && (x_cnt >= BX0) && (x_cnt < BX1) &&
                             (y_cnt >= BY0) && (y_cnt < BY1);

    // ---- 字符序号: ci = lx / ADV (ADV=10 QUAD, 24 单画面) ----
    wire [10:0] lx  = in_text ? (x_cnt - X0) : 11'd0;
    wire [9:0]  ly  = in_text ? (y_cnt - Y0) : 10'd0;
    // >>12: QUAD 乘410(/10), 单画面乘171(/24), 精确到整字符
    wire [18:0] cimul = lx * (I_quad_mode ? 9'd410 : 9'd171);
    wire [4:0]  ci    = cimul[16:12];

    // 字符内像素偏移 cx = lx - ci*ADV
    wire [10:0] adv_w = I_quad_mode ? 11'd10 : 11'd24;
    wire [15:0] cipos = ci * adv_w;
    wire [10:0] cx    = lx - cipos[10:0];

    // 字形内坐标: QUAD 字高2倍, 单画面字高4倍
    wire [10:0] gcx_v = I_quad_mode ? cx : (cx >> 2);
    wire [9:0]  row_v = I_quad_mode ? (ly >> 1) : (ly >> 2);
    wire [2:0]  gcx   = gcx_v[2:0];
    wire [3:0]  row   = row_v[3:0];

    // ================= 文本内容 =================
    function [7:0] txt;
        input [4:0] i;
        input [1:0] lvl;
        begin
            txt = 8'h20;
            case(i)
                5'd0: txt = 8'h43;                      // C
                5'd1: txt = 8'h4f;                      // O
                5'd2: txt = 8'h4e;                      // N
                5'd3: txt = 8'h46;                      // F
                5'd4: txt = 8'h20;                      // ' '
                5'd5: txt = 8'h30 + conf_bcd[11:8];
                5'd6: txt = 8'h30 + conf_bcd[7:4];
                5'd7: txt = 8'h30 + conf_bcd[3:0];
                5'd8: txt = 8'h25;                      // %
                5'd9: txt = 8'h20;
                5'd10: txt = 8'h20;
                default: begin
                    if(lvl == 2'd2) begin     // CONFIRMED
                        case(i)
                            5'd11: txt = 8'h43;  // C
                            5'd12: txt = 8'h4f;  // O
                            5'd13: txt = 8'h4e;  // N
                            5'd14: txt = 8'h46;  // F
                            5'd15: txt = 8'h49;  // I
                            5'd16: txt = 8'h52;  // R
                            5'd17: txt = 8'h4d;  // M
                            5'd18: txt = 8'h45;  // E
                            5'd19: txt = 8'h44;  // D
                            default: txt = 8'h20;
                        endcase
                    end else if(lvl == 2'd1) begin  // WARNING
                        case(i)
                            5'd11: txt = 8'h57;  // W
                            5'd12: txt = 8'h41;  // A
                            5'd13: txt = 8'h52;  // R
                            5'd14: txt = 8'h4e;  // N
                            5'd15: txt = 8'h49;  // I
                            5'd16: txt = 8'h4e;  // N
                            5'd17: txt = 8'h47;  // G
                            default: txt = 8'h20;
                        endcase
                    end else begin                  // NORMAL
                        case(i)
                            5'd11: txt = 8'h4e;  // N
                            5'd12: txt = 8'h4f;  // O
                            5'd13: txt = 8'h52;  // R
                            5'd14: txt = 8'h4d;  // M
                            5'd15: txt = 8'h41;  // A
                            5'd16: txt = 8'h4c;  // L
                            default: txt = 8'h20;
                        endcase
                    end
                end
            endcase
        end
    endfunction

    wire [7:0] cur_ch = txt(ci, level);

    // ================= 5x7 点阵字库 (与 quad_top TARGET 标签同款) =================
    function [4:0] g5;
        input [7:0] ch;
        input [3:0] r;
        begin
            g5 = 5'd0;
            case(ch)
            8'h43: case(r) // C
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b10000; 3:g5=5'b10000;
                4:g5=5'b10000; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h4f: case(r) // O
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b10001;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h4e: case(r) // N
                0:g5=5'b10001; 1:g5=5'b11001; 2:g5=5'b10101; 3:g5=5'b10011;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b10001; default:g5=5'd0; endcase
            8'h46: case(r) // F
                0:g5=5'b11111; 1:g5=5'b10000; 2:g5=5'b10000; 3:g5=5'b11110;
                4:g5=5'b10000; 5:g5=5'b10000; 6:g5=5'b10000; default:g5=5'd0; endcase
            8'h49: case(r) // I
                0:g5=5'b11111; 1:g5=5'b00100; 2:g5=5'b00100; 3:g5=5'b00100;
                4:g5=5'b00100; 5:g5=5'b00100; 6:g5=5'b11111; default:g5=5'd0; endcase
            8'h44: case(r) // D
                0:g5=5'b11110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b10001;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b11110; default:g5=5'd0; endcase
            8'h45: case(r) // E
                0:g5=5'b11111; 1:g5=5'b10000; 2:g5=5'b10000; 3:g5=5'b11110;
                4:g5=5'b10000; 5:g5=5'b10000; 6:g5=5'b11111; default:g5=5'd0; endcase
            8'h50: case(r) // P
                0:g5=5'b11110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b11110;
                4:g5=5'b10000; 5:g5=5'b10000; 6:g5=5'b10000; default:g5=5'd0; endcase
            8'h54: case(r) // T
                0:g5=5'b11111; 1:g5=5'b00100; 2:g5=5'b00100; 3:g5=5'b00100;
                4:g5=5'b00100; 5:g5=5'b00100; 6:g5=5'b00100; default:g5=5'd0; endcase
            8'h41: case(r) // A
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b11111;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b10001; default:g5=5'd0; endcase
            8'h52: case(r) // R
                0:g5=5'b11110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b11110;
                4:g5=5'b10100; 5:g5=5'b10010; 6:g5=5'b10001; default:g5=5'd0; endcase
            8'h47: case(r) // G
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b10000; 3:g5=5'b10111;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h4d: case(r) // M
                0:g5=5'b10001; 1:g5=5'b11011; 2:g5=5'b10101; 3:g5=5'b10101;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b10001; default:g5=5'd0; endcase
            8'h4c: case(r) // L
                0:g5=5'b10000; 1:g5=5'b10000; 2:g5=5'b10000; 3:g5=5'b10000;
                4:g5=5'b10000; 5:g5=5'b10000; 6:g5=5'b11111; default:g5=5'd0; endcase
            8'h57: case(r) // W
                0:g5=5'b10001; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b10101;
                4:g5=5'b10101; 5:g5=5'b11011; 6:g5=5'b10001; default:g5=5'd0; endcase
            8'h30: case(r) // 0
                0:g5=5'b01110; 1:g5=5'b10011; 2:g5=5'b10101; 3:g5=5'b10101;
                4:g5=5'b11001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h31: case(r) // 1
                0:g5=5'b00100; 1:g5=5'b01100; 2:g5=5'b00100; 3:g5=5'b00100;
                4:g5=5'b00100; 5:g5=5'b00100; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h32: case(r) // 2
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b00001; 3:g5=5'b00010;
                4:g5=5'b00100; 5:g5=5'b01000; 6:g5=5'b11111; default:g5=5'd0; endcase
            8'h33: case(r) // 3
                0:g5=5'b11111; 1:g5=5'b00010; 2:g5=5'b00100; 3:g5=5'b00010;
                4:g5=5'b00001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h34: case(r) // 4
                0:g5=5'b00010; 1:g5=5'b00110; 2:g5=5'b01010; 3:g5=5'b10010;
                4:g5=5'b11111; 5:g5=5'b00010; 6:g5=5'b00010; default:g5=5'd0; endcase
            8'h35: case(r) // 5
                0:g5=5'b11111; 1:g5=5'b10000; 2:g5=5'b11110; 3:g5=5'b00001;
                4:g5=5'b00001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h36: case(r) // 6
                0:g5=5'b00110; 1:g5=5'b01000; 2:g5=5'b10000; 3:g5=5'b11110;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h37: case(r) // 7
                0:g5=5'b11111; 1:g5=5'b00001; 2:g5=5'b00010; 3:g5=5'b00100;
                4:g5=5'b01000; 5:g5=5'b01000; 6:g5=5'b01000; default:g5=5'd0; endcase
            8'h38: case(r) // 8
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b01110;
                4:g5=5'b10001; 5:g5=5'b10001; 6:g5=5'b01110; default:g5=5'd0; endcase
            8'h39: case(r) // 9
                0:g5=5'b01110; 1:g5=5'b10001; 2:g5=5'b10001; 3:g5=5'b01111;
                4:g5=5'b00001; 5:g5=5'b00010; 6:g5=5'b01100; default:g5=5'd0; endcase
            8'h25: case(r) // %
                0:g5=5'b11001; 1:g5=5'b11011; 2:g5=5'b00010; 3:g5=5'b00100;
                4:g5=5'b01000; 5:g5=5'b11011; 6:g5=5'b10011; default:g5=5'd0; endcase
            default: g5 = 5'd0;
            endcase
        end
    endfunction

    wire [4:0] row_bits = g5(cur_ch, row);
    // QUAD 3x -> 字形 15 列; 单画面 4x -> 字形 20 列
    wire       in_glyph = I_quad_mode ? (cx < 5'd5) : (cx < 5'd20);
    wire       glyph    = in_text && (ci <= 5'd19) && in_glyph && row_bits[4 - gcx];

    // ================= 颜色 =================
    wire [23:0] COLOR_LEVEL = (level == 2'd2) ? 24'h00e676 :   // 绿: 确认
                              (level == 2'd1) ? 24'hffb300 :   // 黄: 预警
                                                24'hc0c8d0;    // 灰: 正常

    wire [23:0] pix = glyph ? ((ci >= 5'd11) ? COLOR_LEVEL : 24'hFFFFFF) : 24'h101820;
    wire [3:0]  alp = glyph ? 4'd15 : 4'd8;   // 底条半透明(8/16), 文字不透明
    wire        draw = glyph || in_bar;

    // ================= 输出寄存 (1 拍流水) =================
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
            if(draw) begin
                O_osd_pixel <= pix;
                O_osd_alpha <= alp;
            end else begin
                O_osd_pixel <= 24'd0;
                O_osd_alpha <= 4'd0;
            end
        end
    end

endmodule
