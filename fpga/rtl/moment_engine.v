// ============================================================================
// İstatistiksel Moment Motoru (Statistical Moment Engine)
// TÜBİTAK 2209A - RF Protokol Sınıflandırma Projesi
// ============================================================================
// 1024 adet 12-bit ADC örneğinden 4 istatistiksel moment hesaplar:
//   Feature 0: Mean     (Ortalama, 1. moment)
//   Feature 1: Variance (Varyans, 2. moment)
//   Feature 2: Skewness (Çarpıklık, 3. merkezi moment)
//   Feature 3: Kurtosis (Basıklık, 4. merkezi moment)
//
// Çalışma:
//   1. COLLECT:  1024 örneği BRAM'e kaydet, toplamı biriktir
//   2. MEAN:     Ortalama hesapla (toplam >> 10)
//   3. MOMENTS:  Her örnek için (x - mean)^2, ^3, ^4 hesapla ve biriktir
//   4. FINALIZE: N'e böl, 16-bit çıkışlara yaz
// ============================================================================

module moment_engine #(
    parameter N_SAMPLES = 1024,   // Pencere boyutu (2'nin kuvveti)
    parameter LOG2_N    = 10      // log2(N_SAMPLES)
)(
    input  wire        clk,
    input  wire        rst,
    
    // ADC örnek girişi
    input  wire [11:0] sample_in,
    input  wire        sample_valid,
    
    // 16-bit sonuçlar (SPI ile STM32'ye gönderilecek)
    output reg  [15:0] mean_out,
    output reg  [15:0] variance_out,
    output reg  [15:0] skewness_out,
    output reg  [15:0] kurtosis_out,
    output reg         results_valid,
    
    // Durum sinyalleri
    output wire        busy,
    output reg         collecting
);

    // =========================================================
    // Durum Makinesi
    // =========================================================
    localparam S_IDLE     = 3'd0;
    localparam S_COLLECT  = 3'd1;
    localparam S_MEAN     = 3'd2;
    localparam S_MOMENTS  = 3'd3;
    localparam S_FINALIZE = 3'd4;
    
    reg [2:0] state;
    reg [2:0] sub_state;       // MOMENTS faz alt-durumu (0-4)
    
    assign busy = (state != S_IDLE);

    // =========================================================
    // BRAM: 1024 x 12-bit örnek deposu
    // =========================================================
    reg [11:0] sample_bram [0:N_SAMPLES-1];
    reg [LOG2_N-1:0] wr_addr;
    reg [LOG2_N-1:0] rd_addr;
    reg [11:0] bram_dout;       // Registered BRAM çıkışı
    
    // BRAM okuma (registered - Xilinx BRAM inference için)
    always @(posedge clk) begin
        bram_dout <= sample_bram[rd_addr];
    end

    // =========================================================
    // Sayaçlar ve Biriktirme Registerları
    // =========================================================
    reg [LOG2_N:0]  sample_count;      // 0 - N_SAMPLES
    reg [21:0]      sum_x;             // Σx (22-bit: 12-bit × 1024)
    reg [11:0]      mean_reg;          // Ortalama (12-bit)
    
    // Moment pipeline (DSP48E1 25x18 sinirina gore kuculduldu)
    // dev      : 13-bit signed  -> dev * dev = 26-bit (1 DSP)
    // dev_sq_s : 18-bit signed  (dev_sq'nin ust kismini atarak DSP'ye sigdir)
    // dev_cb   : 31-bit signed  (18x13 = 31 -> 1 DSP)
    // dev_qd   : 36-bit unsigned (18x18 = 36 -> 1 DSP)
    reg signed [12:0] dev;
    reg signed [12:0] dev_d1;
    reg signed [25:0] dev_sq;
    reg signed [17:0] dev_sq_s;        // dev_sq'nin 18-bit olceklenmis kopyasi
    reg signed [30:0] dev_cb;          // 18x13 = 31-bit
    reg        [35:0] dev_qd;          // 18x18 = 36-bit

    // Biriktirme registerlari
    reg [35:0]        acc_dev2;
    reg signed [40:0] acc_dev3;        // 31-bit + log2(1024)=10 -> 41-bit
    reg        [45:0] acc_dev4;        // 36-bit + 10 = 46-bit
    
    // MOMENTS fazı sayacı
    reg [LOG2_N:0] moment_count;
    
    // =========================================================
    // Ana Durum Makinesi
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            results_valid <= 1'b0;
            collecting    <= 1'b0;
            wr_addr       <= 0;
            sample_count  <= 0;
            sum_x         <= 0;
        end
        else begin
            case (state)
            
                // -------------------------------------------------
                // IDLE: İlk örneği bekle
                // -------------------------------------------------
                S_IDLE: begin
                    results_valid <= 1'b0;
                    collecting    <= 1'b0;
                    wr_addr       <= 0;
                    sample_count  <= 0;
                    sum_x         <= 0;
                    
                    if (sample_valid) begin
                        sample_bram[0] <= sample_in;
                        sum_x          <= {10'b0, sample_in};
                        wr_addr        <= 1;
                        sample_count   <= 1;
                        collecting     <= 1'b1;
                        state          <= S_COLLECT;
                    end
                end
                
                // -------------------------------------------------
                // COLLECT: 1024 örneği topla ve BRAM'e yaz
                // -------------------------------------------------
                S_COLLECT: begin
                    if (sample_valid) begin
                        sample_bram[wr_addr] <= sample_in;
                        sum_x        <= sum_x + {10'b0, sample_in};
                        wr_addr      <= wr_addr + 1'b1;
                        sample_count <= sample_count + 1'b1;
                        
                        if (sample_count == N_SAMPLES - 1) begin
                            collecting <= 1'b0;
                            state      <= S_MEAN;
                        end
                    end
                end
                
                // -------------------------------------------------
                // MEAN: Ortalamayı hesapla (sum >> 10)
                // -------------------------------------------------
                S_MEAN: begin
                    mean_reg <= sum_x[LOG2_N+11 : LOG2_N];
                    mean_out <= {4'b0, sum_x[LOG2_N+11 : LOG2_N]};
                    
                    // Biriktirme registerlarını sıfırla
                    acc_dev2     <= 0;
                    acc_dev3     <= 0;
                    acc_dev4     <= 0;
                    rd_addr      <= 0;
                    moment_count <= 0;
                    sub_state    <= 0;
                    
                    state <= S_MOMENTS;
                end
                
                // -------------------------------------------------
                // MOMENTS: Her örnek için sapma ve momentleri hesapla
                // 5 alt-adım/örnek:
                //   0: BRAM adresi ayarla (bram_dout 1 saat sonra hazır)
                //   1: Deviation hesapla
                //   2: dev² hesapla (registered multiply)
                //   3: dev³ ve dev⁴ hesapla (registered multiply)
                //   4: Biriktir, sonraki örneğe geç
                // -------------------------------------------------
                S_MOMENTS: begin
                    case (sub_state)
                        3'd0: begin
                            // BRAM okuma adresi zaten ayarlı
                            // bram_dout bir sonraki saat kenarında hazır olacak
                            sub_state <= 3'd1;
                        end
                        
                        3'd1: begin
                            // Deviation: sample - mean (signed)
                            dev <= $signed({1'b0, bram_dout}) - $signed({1'b0, mean_reg});
                            sub_state <= 3'd2;
                        end
                        
                        3'd2: begin
                            // dev² = deviation × deviation
                            dev_sq <= dev * dev;
                            dev_d1 <= dev;   // Gecikmiş deviation (dev³ için)
                            sub_state <= 3'd3;
                        end
                        
                        3'd3: begin
                            // dev_sq'nin 18-bit olceklenmis kopyasi
                            // (ust bitler kirpilir, kurtosis/skewness dinamik araligi icin yeterli)
                            dev_sq_s <= dev_sq[25:8];  // 18-bit signed
                            sub_state <= 3'd4;
                        end

                        3'd4: begin
                            // dev³ = dev_sq_s (18-bit) × dev_d1 (13-bit) -> 1 DSP
                            dev_cb <= dev_sq_s * dev_d1;
                            // dev⁴ = dev_sq_s × dev_sq_s (18x18) -> 1 DSP
                            dev_qd <= dev_sq_s * dev_sq_s;
                            sub_state <= 3'd5;
                        end
                        
                        3'd5: begin
                            // Biriktir (genisletilmis bit genislikleri)
                            acc_dev2 <= acc_dev2 + {10'd0, dev_sq[25:0]};
                            acc_dev3 <= acc_dev3 + {{10{dev_cb[30]}}, dev_cb};
                            acc_dev4 <= acc_dev4 + {10'd0, dev_qd};

                            moment_count <= moment_count + 1'b1;

                            if (moment_count == N_SAMPLES - 1) begin
                                state <= S_FINALIZE;
                            end else begin
                                rd_addr   <= rd_addr + 1'b1;
                                sub_state <= 3'd0;
                            end
                        end
                        
                        default: sub_state <= 3'd0;
                    endcase
                end
                
                // -------------------------------------------------
                // FINALIZE: N'e böl ve 16-bit çıkışlara yaz
                // -------------------------------------------------
                S_FINALIZE: begin
                    // Variance: acc_dev2 / 1024 -> orta bitler
                    variance_out <= acc_dev2[LOG2_N+15 : LOG2_N];
                    // Skewness: acc_dev3 / 1024 -> signed orta bitler
                    skewness_out <= acc_dev3[LOG2_N+15 : LOG2_N];
                    // Kurtosis: acc_dev4 / 1024 -> orta bitler
                    kurtosis_out <= acc_dev4[LOG2_N+15 : LOG2_N];

                    results_valid <= 1'b1;
                    state         <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule