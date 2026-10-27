/******************************************************************************
 * feature_fuse.v — 四维特征帧级融合 + 硬件IIR + 自适应面积阈值 + 火情判定
 *
 * 位置: user_source/hdl_source/isp/feature_fuse.v
 * 时钟: 与 isp_top 同源 (axi4s_video_aclk = S_csi_rx_clk)
 *
 * 输入均帧内稳定 (帧头锁存上一帧结果):
 *   I_candidate_count : 颜色候选像素数 (color_mask_96.O_frame_count)
 *   I_cand_edge_sum   : 候选区水平梯度能量 (color_mask_96.O_cand_edge_sum)
 *   I_luma_sum        : 全帧亮度 r+g+b (color_mask_96.O_luma_sum)
 *   I_bbox_valid      : detect_ccl_96 多目标 valid (面积 >= I_min_area)
 *
 * 输出:
 *   O_min_area  : 自适应面积阈值, 回喂 detect_ccl_96.I_min_area
 *   O_fire_valid: 融合火情判定 = bbox_valid[0] AND 边缘门 AND 颜色门
 *
 * 设计 (四维: 色彩/饱和度/亮度/边缘纹理):
 *   - 色彩/饱和度: 由 color_mask_96 完成, 此处体现为候选像素数
 *   - 边缘纹理: 候选区水平梯度能量 (火焰内部/边界高频, 实心红物体内部低频)
 *   - 亮度: 场景均值亮度 -> 自适应面积阈值 (暗光降阈防漏检, 强光升阈防误检)
 *   - IIR: 对候选数/边缘能量/亮度做 1 极点 EMA (移位系数 1/2^IIR_SHIFT, 直流增益1)
 *
 * 已知取舍:
 *   - 边缘能量是候选区"水平"梯度均值, 含物体左右边界贡献; 对大面积均匀红色
 *     假目标(内部梯度≈0)均值趋小、对火焰(内部闪烁高频)均值偏大, 可区分;
 *     小目标边界占比高, 判别力下降 — EDGE_NUM 需上板标定。
 *   - IIR 为帧级 EMA (非逐像素; 逐像素需 ~900Kbit 帧缓冲, 本器件不划算)。
 *   - 融合只做"收紧"门控 (fire_valid ⊆ bbox_valid[0]), 暗光漏检由自适应
 *     MIN_AREA 解决, 保证框/警一致。
 * 全整数运算, 无除法器, 无浮点。所有阈值/权重为 parameter, 供上板标定。
 *****************************************************************************/
module feature_fuse #(
    parameter [2:0]  IIR_SHIFT  = 3'd3,       // IIR 极点: 1/2^K (时间常数 ~2^K 帧)
    parameter [19:0] AREA_MIN   = 20'd16,     // 自适应下限 (暗光)
    parameter [19:0] AREA_MAX   = 20'd65536,  // 自适应上限: 须 >= K3/K4 手动门槛上限(65536), 否则手动调节 >256 被钳死
    parameter [9:0]  DARK_REF   = 10'd128,    // 参考亮度 (sum>>20 单位, 低于此视为暗光)
    parameter [7:0]  DARK_GAIN  = 8'd1,       // 暗光补偿增益
    parameter [3:0]  DARK_SHIFT = 4'd0,       // 暗光补偿移位
    parameter [9:0]  BRIGHT_REF   = 10'd384,  // 强光参考亮度 (高于此视为强光, 升阈防误检)
    parameter [7:0]  BRIGHT_GAIN  = 8'd1,     // 强光补偿增益
    parameter [3:0]  BRIGHT_SHIFT = 4'd2,     // 强光补偿移位 (>>2, 缩放1/4, 待标定)
    parameter [7:0]  EDGE_NUM   = 8'd8,       // 边缘门: 候选平均梯度 >= EDGE_NUM/2^EDGE_SHIFT
    parameter [3:0]  EDGE_SHIFT = 4'd0,       // 边缘门移位 (默认 8/1 = 8.0)
    parameter [7:0]  SAT_NUM    = 8'd40,      // 饱和度门: 候选平均饱和度 >= SAT_NUM/2^SAT_SHIFT (火焰高饱和, 洗白/肤色低饱和)
    parameter [3:0]  SAT_SHIFT  = 4'd0,       // 饱和度门移位 (默认 40/1 = 40, 待标定)
    parameter [19:0] COLOR_MIN  = 20'd16,     // 颜色门: 候选数下限 (必须 <= AREA_MIN, 否则与自适应面积阈值下限冲突; 候选数恒>=最大连通域面积, 故该门是面积门的冗余下界)
    parameter [7:0]  FLICKER_NUM   = 8'd1,    // 闪烁门: 候选数帧间变化率 >= FLICKER_NUM/2^FLICKER_SHIFT
    parameter [3:0]  FLICKER_SHIFT = 4'd4     // 闪烁门移位 (默认 1/16 = 6.25% 帧间变化)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tuser,
    input  wire [19:0] I_candidate_count,
    input  wire [29:0] I_cand_edge_sum,
    input  wire [29:0] I_luma_sum,
    input  wire [29:0] I_cand_sat_sum,   // 候选区饱和度 (color_mask_96.O_cand_sat_sum)
    input  wire [1:0]  I_bbox_valid,
    input  wire [19:0] I_area_base,   // 手动面积门槛基值 (来自按键 K3/K4, 复位默认64)
    output reg  [19:0] O_min_area,
    output reg         O_fire_valid,
    output reg  [7:0]  O_confidence   // 检测置信度 0~99 (4门基础分40 + 面积强度0~59)
);

    // ---- 帧结算触发: tuser 延迟 2 拍 ----
    // color_mask 的帧统计在 tuser 拍锁存, detect_ccl 的 bbox 在 tuser_d1 拍
    // (非阻塞) 锁存、tuser_d2 拍才可见; 用 tuser_d2 触发使二者同为"上一帧"结果。
    reg [1:0] tuser_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) tuser_d <= 2'd0;
        else         tuser_d <= {tuser_d[0], I_tuser};
    end
    wire frame_tick = tuser_d[1];

    // ---- 场景亮度: 均值 ~= I_luma_sum / 921600 ~= I_luma_sum >> 20 ----
    wire [9:0] luma_raw = I_luma_sum[29:20];

    // ---- 帧级 IIR (1 极点 EMA, 直流增益1, 移位系数) ----
    reg [19:0] cand_iir;
    reg [29:0] edge_iir;
    reg [29:0] sat_iir;
    reg [9:0]  luma_iir;
    wire [19:0] cand_iir_next = cand_iir - (cand_iir >> IIR_SHIFT) + (I_candidate_count >> IIR_SHIFT);
    wire [29:0] edge_iir_next = edge_iir - (edge_iir >> IIR_SHIFT) + (I_cand_edge_sum >> IIR_SHIFT);
    wire [29:0] sat_iir_next  = sat_iir  - (sat_iir  >> IIR_SHIFT) + (I_cand_sat_sum >> IIR_SHIFT);
    wire [9:0]  luma_iir_next = luma_iir - (luma_iir >> IIR_SHIFT) + (luma_raw >> IIR_SHIFT);

    // ---- 自适应面积阈值 (暗光降阈防漏检 / 强光升阈防误检) ----
    wire [9:0]  dark_boost   = (luma_iir_next < DARK_REF)   ? (DARK_REF   - luma_iir_next) : 10'd0;
    wire [9:0]  bright_boost = (luma_iir_next > BRIGHT_REF) ? (luma_iir_next - BRIGHT_REF) : 10'd0;
    wire [19:0] area_sub     = ({10'd0, dark_boost}   * DARK_GAIN)   >> DARK_SHIFT;   // 显式加宽防截断
    wire [19:0] area_add     = ({10'd0, bright_boost} * BRIGHT_GAIN) >> BRIGHT_SHIFT;
    wire [19:0] area_raw     = (area_sub >= I_area_base + area_add) ? 20'd0 : (I_area_base + area_add - area_sub);
    wire [19:0] area_clamped = (area_raw < AREA_MIN) ? AREA_MIN :
                               (area_raw > AREA_MAX) ? AREA_MAX : area_raw;

    // ---- 边缘门: 候选平均梯度 >= EDGE_NUM/2^EDGE_SHIFT (无除法) ----
    wire [27:0] cand_scaled = {8'd0, cand_iir_next} * EDGE_NUM;  // 20bit*8bit, 显式加宽到28bit防截断
    wire [27:0] edge_thr    = cand_scaled >> EDGE_SHIFT;         // 阈值 = 候选数 * 平均梯度门限
    wire        edge_gate   = (edge_iir_next >= {2'd0, edge_thr});

    // ---- 颜色门: 候选数足够 ----
    wire color_gate = (cand_iir_next >= COLOR_MIN);

    // ---- 闪烁门: 候选数帧间变化率 (火焰 ~10Hz 闪烁/运动, 静态红物体帧间稳定) ----
    reg [19:0] cand_prev;    // 上一帧候选数
    reg [19:0] flicker_iir;  // 帧间变化量的 IIR
    wire [19:0] delta = (I_candidate_count >= cand_prev) ? (I_candidate_count - cand_prev)
                                                         : (cand_prev - I_candidate_count);
    wire [19:0] flicker_iir_next = flicker_iir - (flicker_iir >> IIR_SHIFT) + (delta >> IIR_SHIFT);
    wire [27:0] flicker_thr = ({8'd0, cand_iir_next} * FLICKER_NUM) >> FLICKER_SHIFT;  // 阈值 = 候选数*变化率门限
    wire        flicker_gate = (flicker_iir_next >= {8'd0, flicker_thr[19:0]});

    // ---- 饱和度门: 候选平均饱和度 >= SAT_NUM/2^SAT_SHIFT (火焰高饱和, 洗白/肤色低饱和) ----
    wire [27:0] sat_scaled = {8'd0, cand_iir_next} * SAT_NUM;
    wire [27:0] sat_thr    = sat_scaled >> SAT_SHIFT;
    wire        sat_gate   = (sat_iir_next >= {2'd0, sat_thr});

    // ---- 火情判定条件 (4门全过) ----
    wire fire_cond = I_bbox_valid[0] && edge_gate && color_gate && flicker_gate && sat_gate;

    // ---- 检测置信度 (0~99): 4门基础分40 + 候选面积强度0~59 ----
    wire [7:0] conf_area = (cand_iir_next[19:8] > 8'd59) ? 8'd59 : cand_iir_next[19:8];
    wire [7:0] conf_next = fire_cond ? (8'd40 + conf_area) : 8'd0;

    // ---- 帧级结算 ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            cand_iir     <= 20'd0;
            edge_iir     <= 30'd0;
            sat_iir      <= 30'd0;
            luma_iir     <= 10'd0;
            cand_prev    <= 20'd0;
            flicker_iir  <= 20'd0;
            O_min_area   <= I_area_base;  // 复位默认基值 (手动门槛), 防 MIN_AREA=0 误触发
            O_fire_valid <= 1'b0;
            O_confidence <= 8'd0;
        end else if(frame_tick) begin
            cand_iir <= cand_iir_next;
            edge_iir <= edge_iir_next;
            sat_iir <= sat_iir_next;
            luma_iir <= luma_iir_next;
            cand_prev <= I_candidate_count;
            flicker_iir <= flicker_iir_next;
            O_min_area <= area_clamped;
            O_fire_valid <= fire_cond;
            O_confidence <= conf_next;
        end
    end

endmodule
