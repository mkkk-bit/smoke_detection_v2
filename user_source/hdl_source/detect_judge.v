// ============================================================
// detect_judge.v - 目标有效判定 + 积分锁定 + 蜂鸣告警 (阶段1)
// 时钟域: S_hdmi_pixel_clk (74.25MHz), 仅用已同步的信号
// I_frame_tick : 每帧一个脉冲 (bbox valid CDC toggle的边沿检测)
// I_fire_valid : 目标检测有效 (bbox valid 帧级标志, 与OSD ALARM同源)
// I_candidate  : 上一帧候选像素数 (帧首锁存, 帧内稳定, L2烟头路径使用)
// 判定规则 (2026-10-18 目标检测落地):
//   L1 告警: I_fire_valid = 最大红色连通域面积>=MIN_AREA (有框必有警,
//            取代原"候选像素计数>=2": 散点噪声不形成连通域, 无法触发)
//   L2 烟头: 逻辑保留, 当前被L1优先覆盖, 行投影小簇检测留待后续阶段
// 积分锁定: 候选成立 +SCORE_INC / 不成立 -SCORE_DEC, 钳位[0,SCORE_CLAMP]
//           >=SCORE_LOCK 锁定告警, <SCORE_UNLOCK 解锁 (抗闪烁/抗误触发)
// 蜂鸣音型: 国标火警双音交替 (GB 26851-2011: 变调周期 0.2~5.0s, 双音须
//           明显可区分; 本实现高音1000Hz/低音800Hz, 每音持续0.5s、
//           交替周期1.0s), 报警期间连续交替无断续门控, 撤警即静音
// 板载蜂鸣器低电平有效 (开发板手册: BUZZER=0 三极管导通响, =1 截止静音),
//   输出取反; 蜂鸣器与FPGA之间有乒乓开关SW5 (拨上断开/拨下接通, 上板需拨下)
// ============================================================
module detect_judge #(
    parameter [19:0] TH_SMALL_MIN = 20'd2,       // L2 小簇下限 (当前被L1优先覆盖)
    parameter [19:0] TH_SMALL_MAX = 20'd30,      // L2 小簇上限
    parameter [8:0]  SCORE_INC    = 9'd2,        // 每帧在带 +2
    parameter [8:0]  SCORE_DEC    = 9'd1,        // 每帧出带 -1
    parameter [15:0] SCORE_LOCK   = 16'd60,      // 锁定阈值 0.5s@60fps (快速响应)
    parameter [15:0] SCORE_UNLOCK = 16'd30,      // 解锁阈值 0.5s@60fps
    parameter [8:0]  SCORE_CLAMP  = 9'd60        // 积分上限 (锁定后撤警约0.5s, 与alarm_module保持窗口对齐)
)(
    input              I_clk,
    input              I_rst_n,
    input              I_frame_tick,   // 每帧一个脉冲
    input       [19:0] I_candidate,    // 帧候选像素数 (保留, L2 旧路径)
    input              I_fire_valid,   // 目标检测有效 (bbox, 与OSD同源)
    input              I_cig_valid,    // 烟头有效 (smoke_bbox, 暗光亮点)
    output reg         O_alarm_fire,   // L1 明火告警 (蜂鸣器音源门控)
    output reg         O_alarm_cig,    // L2 烟头告警
    output reg         O_buzzer        // 蜂鸣方波 (国标火警双音交替, 低电平有效)
);

// ---- 帧判定 (在帧结算沿评估) ----
// L1 触发源 = 目标检测有效 (bbox, 与OSD ALARM同源); L2 = smoke_bbox 暗光亮点烟头
wire fire_cand = I_fire_valid;
wire cig_cand  = I_cig_valid;

// ---- 积分器 (钳位[0, SCORE_CLAMP]) ----
reg [8:0] score_fire;
reg [8:0] score_cig;

always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        score_fire <= 9'd0;
        score_cig  <= 9'd0;
    end else if(I_frame_tick) begin
        if(fire_cand)
            score_fire <= (score_fire + SCORE_INC >= SCORE_CLAMP) ? SCORE_CLAMP :
                          score_fire + SCORE_INC;
        else
            score_fire <= (score_fire <= SCORE_DEC) ? 9'd0 :
                          score_fire - SCORE_DEC;
        if(cig_cand)
            score_cig <= (score_cig + SCORE_INC >= SCORE_CLAMP) ? SCORE_CLAMP :
                         score_cig + SCORE_INC;
        else
            score_cig <= (score_cig <= SCORE_DEC) ? 9'd0 :
                         score_cig - SCORE_DEC;
    end
end

// ---- 锁定/解锁 ----
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_alarm_fire <= 1'b0;
        O_alarm_cig  <= 1'b0;
    end else if(I_frame_tick) begin
        if(score_fire >= SCORE_LOCK)
            O_alarm_fire <= 1'b1;
        else if(score_fire < SCORE_UNLOCK)
            O_alarm_fire <= 1'b0;
        if(score_cig >= SCORE_LOCK)
            O_alarm_cig <= 1'b1;
        else if(score_cig < SCORE_UNLOCK)
            O_alarm_cig <= 1'b0;
    end
end

// ---- 蜂鸣器: 国标火警双音交替 (GB 26851-2011) ----
// 高音 1000Hz / 低音 800Hz, 每个音持续 0.5s (交替周期 1.0s, 变调周期
// 落在国标 0.2~5.0s 范围内, 双音频率差明显可区分); 报警期间连续交替,
// 无断续门控; 换音拍音调计数器相位归零, 避免半周期残留
// 半周期比较值 @74.25MHz:
//   高音: 74.25e6 / (2*1000) = 37125     -> 比较值 37124, 实测 1000.00Hz
//   低音: 74.25e6 / (2*800)  = 46406.25  -> 取 46405 (半周期46406拍), 实测 ~800.00Hz
localparam [15:0] TONE_HI_HP = 16'd37124;
localparam [15:0] TONE_LO_HP = 16'd46405;
localparam [25:0] ALT_PERIOD = 26'd37125000;  // 0.5s @74.25MHz

// 双音交替选择: 每 0.5s 翻转一次 (1=高音 0=低音)
reg [25:0] alt_cnt;
reg        alt_sel;
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        alt_cnt <= 26'd0;
        alt_sel <= 1'b0;
    end else if(alt_cnt == ALT_PERIOD - 1'b1) begin
        alt_cnt <= 26'd0;
        alt_sel <= ~alt_sel;
    end else begin
        alt_cnt <= alt_cnt + 1'b1;
    end
end

// 音调方波: 计数到半周期比较值翻转; 换音拍相位归零
reg        alt_sel_d1;
reg [15:0] tone_cnt;
reg        tone;
wire [15:0] tone_hp   = alt_sel ? TONE_HI_HP : TONE_LO_HP;
wire        tone_half = (tone_cnt == tone_hp);
wire        alt_toggle = (alt_sel != alt_sel_d1);
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        alt_sel_d1 <= 1'b0;
        tone_cnt   <= 16'd0;
        tone       <= 1'b0;
    end else begin
        alt_sel_d1 <= alt_sel;
        if(alt_toggle) begin          // 换音: 相位归零 (优先级高于 tone_half)
            tone_cnt <= 16'd0;
            tone     <= 1'b0;
        end else if(tone_half) begin
            tone_cnt <= 16'd0;
            tone     <= ~tone;
        end else begin
            tone_cnt <= tone_cnt + 1'b1;
        end
    end
end

// 输出: 板载蜂鸣器低电平有效 (BUZZER=0 三极管导通响, =1 截止静音);
//   告警期间输出 tone 取反的连续双音交替, 无告警恒高电平静音
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n)
        O_buzzer <= 1'b1;      // 复位静音 (高电平=三极管截止)
    else
        O_buzzer <= (O_alarm_fire || O_alarm_cig) ? ~tone : 1'b1;
end

endmodule
