/******************************************************************************
 * threshold_calib.v — 按键阈值标定 (长按 KEY2 循环上调烟头亮点阈值 LUM_TH)
 *
 * 位置: user_source/hdl_source/threshold_calib.v
 * 时钟: S_csi_rx_clk (与 luma_spot/smoke_bbox 同域, 阈值寄存器无需跨域)
 *
 * 功能:
 *   长按板载 KEY2 (>~0.8s) 每次把 LUM_TH 上调 16, 0..255 回绕。
 *   LUM_TH 送 isp_top.I_lum_th -> luma_spot 暗光亮点判定阈值,
 *   标定"暗光烟头亮点"的亮度下限 (暗光下调低、误检上调高)。
 *   短按 KEY2 仍归 hdmi 域的画面暂停功能 (key_remove_shakes) 处理,
 *   本模块只用长按事件 (O_key2_long), 二者复用同一按键、事件互不冲突。
 *   长按瞬间暂停键会翻转一次 (key_remove_shakes 不区分短/长按), 属已知
 *   轻微交互, 不影响主功能。
 *
 * 说明: 阈值以 +16 步进、回绕, 全整数无除法无浮点; 复位默认 8'd150。
 *****************************************************************************/
module threshold_calib(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_key2,     // 板载KEY2 (低有效)
    output reg [7:0]  O_lum_th    // 烟头亮点亮度阈值
);

    wire key2_short, key2_long;
    key_calib u_key_calib(
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_key1      (1'b0),
        .I_key2      (~I_key2),
        .O_key1_short(),
        .O_key1_long (),
        .O_key2_short(key2_short),
        .O_key2_long (key2_long)
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_lum_th <= 8'd150;
        else if(key2_long)
            O_lum_th <= O_lum_th + 8'd16;   // 长按 +16, 0..255 回绕
    end

endmodule
