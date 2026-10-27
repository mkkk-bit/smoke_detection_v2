// ============================================================
// osd_box.v - OSD目标矩形框叠加 (纯PL, 像素时钟域, 零丢帧流水)
// 功能:
//   1. 逐像素实时判定: 像素落在目标外接矩形边框上 -> 输出叠加色+alpha
//   2. 明火=红色边框(0xFF0000), 烟头=黄色边框(0xFFFF00), 仅画边框不填充
//   3. 单帧多目标 (OBJ_MAX 可配), 坐标帧头(vsync)锁存, 帧内稳定
//   4. 内部1拍流水, 附带延迟1拍的 de/hsync/vsync 输出, 与视频流同拍对齐
// 混合公式 (接入现有Alpha半透明通路):
//   out_rgb = (video_rgb*(16-a) + osd_rgb*a) >> 4   (纯整数运算)
//   a=0 全透明(视频直通), a=16 不透明, 1~15 半透明
// ============================================================
module osd_box #(
    parameter        OBJ_MAX     = 4,        // 单帧最多目标数
    parameter [10:0] FRAME_W     = 11'd1280, // 帧宽 (计数器回绕用)
    parameter [9:0]  FRAME_H     = 10'd720,  // 帧高 (计数器回绕用)
    parameter [10:0] H_TOTAL     = 11'd1650, // HDMI行总时钟数
    parameter [10:0] H_SYNC_START= 11'd1390, // hsync起始相位
    parameter [9:0]  V_TOTAL     = 10'd750,  // HDMI场总行数
    parameter [9:0]  V_SYNC_START= 10'd725,  // vsync起始行
    parameter [3:0]  BORDER_T    = 4'd3,     // 边框线宽(像素)
    parameter [4:0]  ALPHA_FIRE  = 5'd8,     // 明火框alpha (8=50%半透明, 不遮挡框内目标)
    parameter [4:0]  ALPHA_SMOKE = 5'd8      // 烟头框alpha (8=50%半透明)
)(
    input                      I_clk,
    input                      I_rst_n,
    input                      I_de,         // 像素有效
    input                      I_hsync,      // 行同步
    input                      I_vsync,      // 场同步
    // 目标坐标 (帧级锁存输入: 帧头更新, 帧内稳定)
    // 打包约定: [OBJ_MAX-1:0] 每目标一段, 低位=目标0
    input  [OBJ_MAX*11-1:0]    I_xmin,
    input  [OBJ_MAX*10-1:0]    I_ymin,
    input  [OBJ_MAX*11-1:0]    I_xmax,
    input  [OBJ_MAX*10-1:0]    I_ymax,
    input  [OBJ_MAX-1:0]       I_obj_valid,  // 每目标有效位
    input  [OBJ_MAX*2-1:0]     I_obj_class,  // 每目标2位: 01=明火 10=烟头 其他=不画
    // OSD 叠加输出 (与输入像素流同拍, 同步信号延迟1拍)
    output reg [23:0]          O_osd_pixel,  // RGB888 叠加色
    output reg [4:0]           O_osd_alpha,  // 0=透明 16=不透明
    output reg                 O_de_out,
    output reg                 O_hsync_out,
    output reg                 O_vsync_out
);

// ---- 同步边沿检测 ----
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
wire frame_start = vsync_rise;

// ---- 坐标锁存 (帧头更新, 帧内稳定) ----
reg [10:0] r_xmin [0:OBJ_MAX-1];
reg [9:0]  r_ymin [0:OBJ_MAX-1];
reg [10:0] r_xmax [0:OBJ_MAX-1];
reg [9:0]  r_ymax [0:OBJ_MAX-1];
reg        r_v    [0:OBJ_MAX-1];
reg [1:0]  r_cls  [0:OBJ_MAX-1];

genvar gi;
generate
for(gi=0; gi<OBJ_MAX; gi=gi+1) begin : GEN_LATCH
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_xmin[gi] <= 11'd0; r_ymin[gi] <= 10'd0;
            r_xmax[gi] <= 11'd0; r_ymax[gi] <= 10'd0;
            r_v[gi]    <= 1'b0;  r_cls[gi]  <= 2'd0;
        end else if(frame_start) begin
            r_xmin[gi] <= I_xmin[gi*11 +: 11];
            r_ymin[gi] <= I_ymin[gi*10 +: 10];
            r_xmax[gi] <= I_xmax[gi*11 +: 11];
            r_ymax[gi] <= I_ymax[gi*10 +: 10];
            r_v[gi]    <= I_obj_valid[gi];
            r_cls[gi]  <= I_obj_class[gi*2 +: 2];
        end
    end
end
endgenerate

// ---- HDMI光栅坐标计数 ----
// 与工程中的 1650x750 VTC 相位一致。关键点: y 只在行计数器回绕时加1,
// 不在 hsync 高电平期间反复加; hsync 上升沿负责把 x 锚定到行相位。
reg [10:0] x_cnt;
reg [9:0]  y_cnt;
reg        locked;
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        x_cnt <= 11'd0;
    end else if(hsync_rise) begin
        x_cnt <= H_SYNC_START + 11'd1;
    end else if(x_cnt == H_TOTAL - 11'd1) begin
        x_cnt <= 11'd0;
    end else begin
        x_cnt <= x_cnt + 11'd1;
    end
end
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        y_cnt  <= 10'd0;
        locked <= 1'b0;
    end else if(vsync_rise) begin
        y_cnt  <= V_SYNC_START;
        locked <= 1'b1;
    end else if(x_cnt == H_TOTAL - 11'd1) begin
        if(y_cnt == V_TOTAL - 10'd1)
            y_cnt <= 10'd0;
        else
            y_cnt <= y_cnt + 10'd1;
    end
end

wire pixel_active = locked && (x_cnt < FRAME_W) && (y_cnt < FRAME_H);

// ---- 边框命中判定 (组合逻辑, 整数运算) ----
// 命中 = 目标有效 且 像素在外接矩形内 且 距某条边 < BORDER_T
// (前四项短路保证后续减法均为非负, 无回绕风险)
wire [OBJ_MAX-1:0] hit;
generate
for(gi=0; gi<OBJ_MAX; gi=gi+1) begin : GEN_HIT
    assign hit[gi] = pixel_active && r_v[gi] &&
                     (x_cnt >= r_xmin[gi]) && (x_cnt <= r_xmax[gi]) &&
                     (y_cnt >= r_ymin[gi]) && (y_cnt <= r_ymax[gi]) &&
                     ( ((x_cnt - r_xmin[gi]) < BORDER_T) ||
                       ((r_xmax[gi] - x_cnt) < BORDER_T) ||
                       ((y_cnt - r_ymin[gi]) < BORDER_T) ||
                       ((r_ymax[gi] - y_cnt) < BORDER_T) );
end
endgenerate

// ---- 按类别归并: 明火命中 / 烟头命中 ----
wire [OBJ_MAX-1:0] hit_fire;
wire [OBJ_MAX-1:0] hit_smoke;
generate
for(gi=0; gi<OBJ_MAX; gi=gi+1) begin : GEN_CLS
    assign hit_fire[gi]  = hit[gi] && (r_cls[gi] == 2'b01);
    assign hit_smoke[gi] = hit[gi] && (r_cls[gi] == 2'b10);
end
endgenerate
wire any_fire  = |hit_fire;
wire any_smoke = |hit_smoke;

// ---- 输出寄存 (1拍流水: 叠加色/alpha 与延迟1拍的同步信号同拍输出) ----
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_osd_pixel <= 24'd0;
        O_osd_alpha <= 5'd0;
        O_de_out    <= 1'b0;
        O_hsync_out <= 1'b0;
        O_vsync_out <= 1'b0;
    end else begin
        O_de_out    <= I_de;
        O_hsync_out <= I_hsync;
        O_vsync_out <= I_vsync;
        if(any_fire) begin                     // 明火优先级高 (重叠画红框)
            O_osd_pixel <= 24'hFF0000;         // 红框
            O_osd_alpha <= ALPHA_FIRE;
        end else if(any_smoke) begin
            O_osd_pixel <= 24'hFFFF00;         // 黄框
            O_osd_alpha <= ALPHA_SMOKE;
        end else begin
            O_osd_pixel <= 24'd0;              // 无框: 全透明直通
            O_osd_alpha <= 5'd0;
        end
    end
end

endmodule
