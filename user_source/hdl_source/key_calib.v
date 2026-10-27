/******************************************************************************
 * key_calib.v — 双按键消抖 + 短按/长按事件 (P2-2 阈值标定用)
 *
 * 位置: user_source/hdl_source/key_calib.v
 * 时钟: S_csi_rx_clk (与 detect_top 同域, 阈值寄存器无需跨域)
 *
 * 功能:
 *   I_key1/I_key2 输入"按下=1"的电平 (顶层取反板载按键),
 *   每个键独立消抖 (连续稳定 DEBOUNCE_CNT 拍后确认电平),
 *   输出 4 个单拍脉冲:
 *     O_key?_short : 按下后释放, 且按下时长 < LONG_CNT (短按)
 *     O_key?_long  : 按下持续达到 LONG_CNT 的瞬间 (长按, 一次)
 *   按下时长 >= LONG_CNT 后释放不再产生 short (避免长按误判短按)
 *
 * 参数 (按 S_csi_rx_clk = 103.5MHz 折算):
 *   LONG_CNT     = 长按判定阈值 (拍数): 800ms @ 103.5MHz ≈ 82.8M → 28bit
 *   DEBOUNCE_CNT = 消抖稳定拍数: ~10ms @ 103.5MHz ≈ 1.035M → 21bit
 *****************************************************************************/
module key_calib #(
    parameter LONG_CNT     = 28'd82800000,   // 800ms @ 103.5MHz
    parameter DEBOUNCE_CNT = 22'd1035000     // 10ms @ 103.5MHz
)(
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire I_key1,          // 按下=1
    input  wire I_key2,          // 按下=1
    output reg  O_key1_short,
    output reg  O_key1_long,
    output reg  O_key2_short,
    output reg  O_key2_long
);

    localparam LONG_W = 28;   // LONG_CNT=165.6M < 2^28
    localparam DEB_W  = 22;   // DEBOUNCE_CNT=2.07M < 2^22

    // ============ 消抖: 连续稳定 DEBOUNCE_CNT 拍才翻转电平 ============
    reg [DEB_W-1:0] st_cnt1, st_cnt2;
    reg        key1_db, key2_db;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            st_cnt1 <= {DEB_W{1'b0}}; st_cnt2 <= {DEB_W{1'b0}};
            key1_db <= 1'b0; key2_db <= 1'b0;
        end else begin
            if(I_key1 == key1_db) begin
                if(st_cnt1 < DEBOUNCE_CNT)
                    st_cnt1 <= st_cnt1 + 1'b1;
            end else begin
                st_cnt1 <= {DEB_W{1'b0}};
                if(st_cnt1 >= DEBOUNCE_CNT)
                    key1_db <= I_key1;
            end
            if(I_key2 == key2_db) begin
                if(st_cnt2 < DEBOUNCE_CNT)
                    st_cnt2 <= st_cnt2 + 1'b1;
            end else begin
                st_cnt2 <= {DEB_W{1'b0}};
                if(st_cnt2 >= DEBOUNCE_CNT)
                    key2_db <= I_key2;
            end
        end
    end

    // ============ 短按/长按判定 (key1) ============
    reg key1_db_1d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) key1_db_1d <= 1'b0;
        else         key1_db_1d <= key1_db;
    end
    wire key1_press   =  key1_db & ~key1_db_1d;
    wire key1_release = ~key1_db &  key1_db_1d;

    reg [LONG_W-1:0] press_cnt1;
    reg        long_done1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            press_cnt1 <= {LONG_W{1'b0}};
            long_done1 <= 1'b0;
        end else begin
            if(key1_release) begin
                press_cnt1 <= {LONG_W{1'b0}};
                long_done1 <= 1'b0;
            end else if(key1_press) begin
                press_cnt1 <= {{(LONG_W-1){1'b0}}, 1'b1};
                long_done1 <= 1'b0;
            end else if(key1_db && !long_done1) begin
                if(press_cnt1 >= LONG_CNT)
                    long_done1 <= 1'b1;
                else
                    press_cnt1 <= press_cnt1 + 1'b1;
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_key1_short <= 1'b0;
            O_key1_long  <= 1'b0;
        end else begin
            O_key1_short <= key1_release && (press_cnt1 < LONG_CNT);
            O_key1_long  <= key1_db && !long_done1 && (press_cnt1 == LONG_CNT);
        end
    end

    // ============ 短按/长按判定 (key2) ============
    reg key2_db_1d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) key2_db_1d <= 1'b0;
        else         key2_db_1d <= key2_db;
    end
    wire key2_press   =  key2_db & ~key2_db_1d;
    wire key2_release = ~key2_db &  key2_db_1d;

    reg [LONG_W-1:0] press_cnt2;
    reg        long_done2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            press_cnt2 <= {LONG_W{1'b0}};
            long_done2 <= 1'b0;
        end else begin
            if(key2_release) begin
                press_cnt2 <= {LONG_W{1'b0}};
                long_done2 <= 1'b0;
            end else if(key2_press) begin
                press_cnt2 <= {{(LONG_W-1){1'b0}}, 1'b1};
                long_done2 <= 1'b0;
            end else if(key2_db && !long_done2) begin
                if(press_cnt2 >= LONG_CNT)
                    long_done2 <= 1'b1;
                else
                    press_cnt2 <= press_cnt2 + 1'b1;
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_key2_short <= 1'b0;
            O_key2_long  <= 1'b0;
        end else begin
            O_key2_short <= key2_release && (press_cnt2 < LONG_CNT);
            O_key2_long  <= key2_db && !long_done2 && (press_cnt2 == LONG_CNT);
        end
    end

endmodule
