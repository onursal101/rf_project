// ============================================================================
// Top-Level Module - Basys3 SPI Slave Prototipi
// TÜBİTAK 2209A - RF Protokol Sınıflandırma Projesi
// ============================================================================
// NOT: sample_in/sample_valid portları kaldırıldı.
//      ADC bağlanana kadar LFSR tabanlı test sinyali içeride üretiliyor.
//      Gerçek Pmod AD1 eklendiğinde:
//        1. LFSR bloğunu sil
//        2. sample_in / sample_valid portlarını geri ekle
//        3. XDC'ye Pmod AD1 pin atamaları ekle
// ============================================================================

module spi_slave_top(
    input  wire        CLK100MHZ,
    input  wire        JB2,        // SCLK  (STM32 PA5) - MRCC pin
    input  wire        JA1,        // MOSI  (STM32 PA7)
    output wire        JA2,        // MISO  (STM32 PA6)
    input  wire        JA7,        // SS    (STM32 PA4)
    output wire        JB3_INT,    // INT   (STM32 GPIO) - kesme sinyali
    output wire JC1,   // ~CS
    input  wire JC2,   // D0/MISO_A
    output wire JC4,   // SCLK
    output wire [7:0]  LED
);
    parameter USE_ADC = 0;   // 0 = LFSR test, 1 = Pmod AD1

    // =========================================================
    // Clock Buffer
    // Vivado otomatik BUFG ekler (explicit instantiation placer sorununa yol acar)
    // XDC'de create_clock ile W5 pinine saat tanimlandigi icin
    // sentez aracı clock ağını otomatik yönetir.
    // =========================================================
    wire clk;
    assign clk = CLK100MHZ;


    
    // =========================================================
    // Power-on Reset
    // =========================================================
    reg [3:0] rst_cnt = 4'hF;
    wire rst = rst_cnt[3];
    always @(posedge clk)
        if (rst_cnt != 0) rst_cnt <= rst_cnt - 1'b1;


        // =========================================================
    // ADC SPI Master (Pmod AD1) - USE_ADC=1 iken aktif
    // =========================================================
    wire [11:0] adc_sample;
    wire        adc_valid;
    wire        adc_cs_n_w, adc_sclk_w;

    adc_spi_master u_adc (
        .clk          (clk),
        .rst          (rst),
        .adc_cs_n     (adc_cs_n_w),
        .adc_sclk     (adc_sclk_w),
        .adc_miso_a   (JC2),
        .sample_out   (adc_sample),
        .sample_valid (adc_valid)
    );

    // USE_ADC=1: JC pinleri ADC'ye bağlı, sample_in_r ADC'den
    // USE_ADC=0: JC pinleri idle, sample_in_r LFSR'dan (mevcut davranış)
    assign JC1 = (USE_ADC == 1) ? adc_cs_n_w : 1'b1;
    assign JC4 = (USE_ADC == 1) ? adc_sclk_w : 1'b0;
    // =========================================================
    // LFSR Test Sinyal Üreteci
    // Pmod AD1 bağlanana kadar kullanılır
    // 100MHz / 128 = ~781 kHz örnekleme hızı
    // =========================================================
    reg [15:0] lfsr     = 16'hACE1;
    reg [6:0]  rate_div = 7'd0;
    wire [11:0] sample_in;
    wire        sample_valid;

    reg [11:0] sample_in_r;
    reg        sample_valid_r;
    assign sample_in    = sample_in_r;
    assign sample_valid = sample_valid_r;

    always @(posedge clk) begin
        if (rst) begin
            lfsr          <= 16'hACE1;
            rate_div      <= 7'd0;
            sample_in_r   <= 12'd0;
            sample_valid_r <= 1'b0;
        end else begin
            // LFSR: x^16 + x^15 + x^13 + x^4 + 1
            lfsr     <= {lfsr[14:0], lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3]};


            // USE_ADC=1: ADC'den gelen örneği kullan (adc_valid pulse'unda)
            // USE_ADC=0: LFSR test verisi (mevcut davranış)
            if (USE_ADC == 1) begin
                if (adc_valid) begin
                    sample_in_r    <= adc_sample;
                    sample_valid_r <= 1'b1;
                end else begin
                    sample_valid_r <= 1'b0;
                end
            end else begin
                // LFSR modu: 100MHz / 100 = 1 MSPS
                if (rate_div == 7'd99) begin
                    rate_div       <= 7'd0;
                    sample_in_r    <= lfsr[11:0];
                    sample_valid_r <= 1'b1;
                end else begin
                    rate_div       <= rate_div + 1'b1;
                    sample_valid_r <= 1'b0;
                end
            end
        end
    end

    
    // =========================================================
    // Moment Motoru (Feature 0-3)
    // =========================================================
    wire [15:0] feat_mean, feat_variance, feat_skewness, feat_kurtosis;
    wire        moment_valid, moment_busy, moment_collecting;

    moment_engine #(.N_SAMPLES(1024), .LOG2_N(10)) u_moment (
        .clk          (clk),          .rst          (rst),
        .sample_in    (sample_in),    .sample_valid (sample_valid),
        .mean_out     (feat_mean),    .variance_out (feat_variance),
        .skewness_out (feat_skewness),.kurtosis_out (feat_kurtosis),
        .results_valid(moment_valid), .busy         (moment_busy),
        .collecting   (moment_collecting)
    );

    reg [15:0] feat0_mean, feat1_variance, feat2_skewness, feat3_kurtosis;
    always @(posedge clk) begin
        if (rst) begin
            feat0_mean     <= 0; feat1_variance <= 0;
            feat2_skewness <= 0; feat3_kurtosis <= 0;
        end else if (moment_valid) begin
            feat0_mean     <= feat_mean;
            feat1_variance <= feat_variance;
            feat2_skewness <= feat_skewness;
            feat3_kurtosis <= feat_kurtosis;
        end
    end

    // =========================================================
    // FFT Wrapper
    // =========================================================
    wire [15:0] fft_re, fft_im;
    wire [10:0] fft_index;
    wire        fft_out_valid, fft_frame_done, fft_busy;

    fft_wrapper u_fft (
        .clk           (clk),            .rst           (rst),
        .sample_in     (sample_in),      .sample_valid  (sample_valid),
        .fft_re        (fft_re),         .fft_im        (fft_im),
        .fft_index     (fft_index),      .fft_out_valid (fft_out_valid),
        .fft_frame_done(fft_frame_done), .fft_busy      (fft_busy)
    );

    // =========================================================
    // Spectral Analyzer (Feature 4-13)
    // =========================================================
    wire [15:0] feat4_peak_freq, feat5_peak_mag,  feat6_bandwidth,
                feat7_centroid,  feat8_flatness,   feat9_energy,
                feat10_harm2,    feat11_harm3,     feat12_snr, feat13_duty;
    wire        spectral_valid;

    spectral_analyzer u_spectral (
        .clk           (clk),            .rst           (rst),
        .fft_re        (fft_re),         .fft_im        (fft_im),
        .fft_index     (fft_index),      .fft_out_valid (fft_out_valid),
        .fft_frame_done(fft_frame_done),
        .feat_peak_freq(feat4_peak_freq),.feat_peak_mag (feat5_peak_mag),
        .feat_bandwidth(feat6_bandwidth),.feat_centroid (feat7_centroid),
        .feat_flatness (feat8_flatness), .feat_energy   (feat9_energy),
        .feat_harm2    (feat10_harm2),   .feat_harm3    (feat11_harm3),
        .feat_snr      (feat12_snr),     .feat_duty     (feat13_duty),
        .spectral_valid(spectral_valid)
    );

    // =========================================================
    // Feature Latch + Kesme Üreteci
    // =========================================================
    reg [15:0] latch_peak_freq, latch_peak_mag,  latch_bandwidth,
               latch_centroid,  latch_flatness,   latch_energy,
               latch_harm2,     latch_harm3,      latch_snr, latch_duty;

    reg moment_done, spectral_done, features_ready;
    reg [4:0] int_cnt;
    reg       int_active;
    assign JB3_INT = int_active;

    always @(posedge clk) begin
        if (rst) begin
            moment_done     <= 0; spectral_done   <= 0;
            features_ready  <= 0; int_active      <= 0; int_cnt <= 0;
            latch_peak_freq <= 0; latch_peak_mag  <= 0;
            latch_bandwidth <= 0; latch_centroid  <= 0;
            latch_flatness  <= 0; latch_energy    <= 0;
            latch_harm2     <= 0; latch_harm3     <= 0;
            latch_snr       <= 0; latch_duty      <= 0;
        end
        else begin
            if (moment_valid)
                moment_done <= 1'b1;

            if (spectral_valid) begin
                spectral_done   <= 1'b1;
                latch_peak_freq <= feat4_peak_freq;
                latch_peak_mag  <= feat5_peak_mag;
                latch_bandwidth <= feat6_bandwidth;
                latch_centroid  <= feat7_centroid;
                latch_flatness  <= feat8_flatness;
                latch_energy    <= feat9_energy;
                latch_harm2     <= feat10_harm2;
                latch_harm3     <= feat11_harm3;
                latch_snr       <= feat12_snr;
                latch_duty      <= feat13_duty;
            end

            // Her iki motor hazır → features_ready + INT
            // Her pencere sonu yeni INT pulse atılır (STM32'ye haber)
            if (moment_done && spectral_done) begin
                features_ready <= 1'b1;
                int_active     <= 1'b1;
                int_cnt        <= 5'd0;
                moment_done    <= 1'b0;
                spectral_done  <= 1'b0;
            end

            // INT pulse: 16 saat HIGH
            if (int_active) begin
                if (int_cnt == 5'd15) begin
                    int_active <= 1'b0;
                    int_cnt    <= 5'd0;
                end else
                    int_cnt <= int_cnt + 1'b1;
            end
        end
    end

    // =========================================================
    // Feature Vektörü ve SPI
    // =========================================================
    wire [223:0] feature_vector = {
        latch_duty,      latch_snr,      latch_harm3,    latch_harm2,
        latch_energy,    latch_flatness, latch_centroid, latch_bandwidth,
        latch_peak_mag,  latch_peak_freq,feat3_kurtosis, feat2_skewness,
        feat1_variance,  feat0_mean
    };

    // bit2=fft_busy, bit1=moment_busy, bit0=features_ready
    wire [7:0] status_byte = {5'b0, fft_busy, moment_busy, features_ready};

    wire [7:0] cmd_reg;
    wire       cmd_valid;
    wire [4:0] byte_count;

    spi_slave u_spi_slave (
        .SCLK(JB2),       .MOSI(JA1),
        .MISO(JA2),       .SS(JA7),
        .features(feature_vector),
        .status_byte(status_byte),
        .cmd_reg(cmd_reg),.cmd_valid(cmd_valid),
        .byte_count(byte_count)
    );

    // =========================================================
    // LED
    // LED[7]: fft_busy
    // LED[6]: moment_collecting
    // LED[5]: features_ready
    // LED[4]: spectral_valid (anlık pulse)
    // LED[3]: int_active
    // LED[2:0]: peak_freq[2:0]
    // =========================================================
    assign LED[7]   = fft_busy;
    assign LED[6]   = moment_collecting;
    assign LED[5]   = features_ready;
    assign LED[4]   = spectral_valid;
    assign LED[3]   = int_active;
    assign LED[2:0] = latch_peak_freq[2:0];

endmodule