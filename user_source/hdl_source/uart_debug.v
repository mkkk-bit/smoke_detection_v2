// ============================================================
// uart_debug.v - 检测状态 UART 调试输出 (每秒一行)
// 时钟: hdmi 像素时钟域 (74.25MHz)
// 格式 (45字节, 每秒一次):
//   "F0 C0 T150 P060 L01234 S000 CF092 CG045 LV2\r\n"
//   F<0/1>=明火  C<0/1>=烟头  T<3位>=烟头阈值  P<3位>=帧率
//   L<5位>=ISP段时延  S<3位>=快照计数
//   CF<3位>=明火四维融合置信度(0~100)  CG<3位>=烟头置信度(0~100)
//   LV<1位>=分级告警等级 (0=正常 1=预警 2=确认)
// 复用官方 uart_tx.v (115200, 8N1), 输出接板子 UART TX (H1)。
// 用途: 上位机/串口终端实时监控检测状态与置信度, 便于场景标定与记录。
// ============================================================
module uart_debug (
    input  wire        I_clk,          // 74.25MHz
    input  wire        I_rst_n,
    input  wire        I_frame_tick,   // 每帧一个脉冲
    input  wire        I_fire_valid,   // 明火有效
    input  wire        I_cig_valid,    // 烟头有效
    input  wire [7:0]  I_lum_th,       // 烟头阈值
    input  wire [6:0]  I_fps,          // 帧率 (fps)
    input  wire [15:0] I_latency,      // ISP段时延 (csi域拍数)
    input  wire [7:0]  I_snap_count,   // 快照完成计数 (调试)
    input  wire [7:0]  I_fire_conf,    // 明火四维融合置信度 0~100
    input  wire [7:0]  I_cig_conf,     // 烟头置信度 0~100
    input  wire [1:0]  I_level,        // 分级告警等级 0/1/2
    output wire        O_tx_pin
);

    // ---- 每秒触发一次 (60帧 @60fps) ----
    reg [5:0] frame_cnt;
    reg       send_trig;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            frame_cnt <= 6'd0;
            send_trig <= 1'b0;
        end else begin
            send_trig <= 1'b0;
            if(I_frame_tick) begin
                if(frame_cnt == 6'd59) begin
                    frame_cnt <= 6'd0;
                    send_trig <= 1'b1;
                end else
                    frame_cnt <= frame_cnt + 6'd1;
            end
        end
    end

    // ---- 阈值 BCD (8bit -> 3位十进制, 组合 double-dabble) ----
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
    // ---- 16bit 二进制 -> 5位十进制 BCD (组合 double-dabble) ----
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
    wire [11:0] th_bcd = bin8_bcd(I_lum_th);
    wire [11:0] fps_bcd = bin8_bcd({1'b0, I_fps});   // 7bit -> 3位十进制
    wire [19:0] latency_bcd = bin16_bcd(I_latency);  // 16bit -> 5位十进制
    wire [11:0] snap_bcd = bin8_bcd(I_snap_count);   // 8bit -> 3位十进制
    wire [11:0] firecf_bcd = bin8_bcd(I_fire_conf);  // 明火置信度 0~100
    wire [11:0] cigcf_bcd  = bin8_bcd(I_cig_conf);   // 烟头置信度 0~100

    // ---- 当前发送字节内容 (组合) ----
    reg [5:0] byte_idx;
    reg       sending;
    reg       tx_valid;
    wire      tx_ready;
    reg [7:0] tx_data;
    always @* begin
        tx_data = 8'h00;
        case(byte_idx)
            6'd0:  tx_data = 8'h46;                    // 'F'
            6'd1:  tx_data = 8'h30 + I_fire_valid;     // 明火 0/1
            6'd2:  tx_data = 8'h20;                    // ' '
            6'd3:  tx_data = 8'h43;                    // 'C'
            6'd4:  tx_data = 8'h30 + I_cig_valid;      // 烟头 0/1
            6'd5:  tx_data = 8'h20;                    // ' '
            6'd6:  tx_data = 8'h54;                    // 'T'
            6'd7:  tx_data = 8'h30 + th_bcd[11:8];     // 阈值百位
            6'd8:  tx_data = 8'h30 + th_bcd[7:4];      // 十位
            6'd9:  tx_data = 8'h30 + th_bcd[3:0];      // 个位
            6'd10: tx_data = 8'h20;                    // ' '
            6'd11: tx_data = 8'h50;                    // 'P'
            6'd12: tx_data = 8'h30 + fps_bcd[11:8];    // 帧率百位
            6'd13: tx_data = 8'h30 + fps_bcd[7:4];     // 十位
            6'd14: tx_data = 8'h30 + fps_bcd[3:0];     // 个位
            6'd15: tx_data = 8'h20;                    // ' '
            6'd16: tx_data = 8'h4C;                    // 'L'
            6'd17: tx_data = 8'h30 + latency_bcd[19:16]; // 时延万位
            6'd18: tx_data = 8'h30 + latency_bcd[15:12]; // 千位
            6'd19: tx_data = 8'h30 + latency_bcd[11:8];  // 百位
            6'd20: tx_data = 8'h30 + latency_bcd[7:4];   // 十位
            6'd21: tx_data = 8'h30 + latency_bcd[3:0];   // 个位
            6'd22: tx_data = 8'h20;                    // ' '
            6'd23: tx_data = 8'h53;                    // 'S'
            6'd24: tx_data = 8'h30 + snap_bcd[11:8];   // 快照百位
            6'd25: tx_data = 8'h30 + snap_bcd[7:4];    // 十位
            6'd26: tx_data = 8'h30 + snap_bcd[3:0];    // 个位
            6'd27: tx_data = 8'h20;                    // ' '
            6'd28: tx_data = 8'h43;                    // 'C'
            6'd29: tx_data = 8'h46;                    // 'F'  (明火置信度)
            6'd30: tx_data = 8'h30 + firecf_bcd[11:8];
            6'd31: tx_data = 8'h30 + firecf_bcd[7:4];
            6'd32: tx_data = 8'h30 + firecf_bcd[3:0];
            6'd33: tx_data = 8'h20;                    // ' '
            6'd34: tx_data = 8'h43;                    // 'C'
            6'd35: tx_data = 8'h47;                    // 'G'  (烟头置信度)
            6'd36: tx_data = 8'h30 + cigcf_bcd[11:8];
            6'd37: tx_data = 8'h30 + cigcf_bcd[7:4];
            6'd38: tx_data = 8'h30 + cigcf_bcd[3:0];
            6'd39: tx_data = 8'h20;                    // ' '
            6'd40: tx_data = 8'h4C;                    // 'L'
            6'd41: tx_data = 8'h56;                    // 'V'  (分级告警等级)
            6'd42: tx_data = 8'h30 + {6'd0, I_level};  // 0/1/2
            6'd43: tx_data = 8'h0D;                    // '\r'
            6'd44: tx_data = 8'h0A;                    // '\n'
            default: tx_data = 8'h00;
        endcase
    end

    // ---- 发送状态机 (字节被 uart_tx 接受后推进) ----
    wire byte_accepted = tx_valid && tx_ready;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            byte_idx <= 6'd0;
            sending  <= 1'b0;
            tx_valid <= 1'b0;
        end else begin
            if(send_trig && !sending) begin
                sending  <= 1'b1;
                byte_idx <= 6'd0;
                tx_valid <= 1'b1;
            end else if(sending && byte_accepted) begin
                if(byte_idx == 6'd44) begin
                    sending  <= 1'b0;
                    tx_valid <= 1'b0;
                end else begin
                    byte_idx <= byte_idx + 6'd1;   // tx_valid 保持 1
                end
            end
        end
    end

    // ---- 官方 UART TX (74.25MHz 近似取 74, 误差 0.4% 在 UART 容忍内) ----
    uart_tx #(
        .CLK_FRE   (74),
        .BAUD_RATE (115200)
    ) u_uart_tx(
        .clk          (I_clk),
        .rst_n        (I_rst_n),
        .tx_data      (tx_data),
        .tx_data_valid(tx_valid),
        .tx_data_ready(tx_ready),
        .tx_pin       (O_tx_pin)
    );

endmodule
