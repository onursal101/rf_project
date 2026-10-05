// ============================================================================
// Spectral Analyzer - Pipeline Mimarisi (LUT Optimize)
// TÜBİTAK 2209A - RF Protokol Sınıflandırma Projesi
// ============================================================================
// Önceki versiyonda bandwidth for döngüsü 512 paralel komparatör
// üretiyordu → 10,000+ LUT. Bu versiyonda sequential pipeline ile
// her clock'ta 1 bin işleniyor → ~200 LUT.
//
// Pipeline Aşamaları:
//   ACCUMULATE : FFT binleri gelirken akümüle et (fft_out_valid süresince)
//   LATCH      : fft_frame_done sonrası basit öznitelikleri kaydet (1 saat)
//   BW_SCAN    : Bandwidth için mag_bram'i sıralı tara (512 saat)
//   HARM       : Harmonik binleri oku (2 saat)
//   DONE       : spectral_valid pulse (1 saat)
// ============================================================================

module spectral_analyzer (
    input  wire        clk,
    input  wire        rst,

    input  wire [15:0] fft_re,
    input  wire [15:0] fft_im,
    input  wire [10:0] fft_index,
    input  wire        fft_out_valid,
    input  wire        fft_frame_done,

    output reg  [15:0] feat_peak_freq,
    output reg  [15:0] feat_peak_mag,
    output reg  [15:0] feat_bandwidth,
    output reg  [15:0] feat_centroid,
    output reg  [15:0] feat_flatness,
    output reg  [15:0] feat_energy,
    output reg  [15:0] feat_harm2,
    output reg  [15:0] feat_harm3,
    output reg  [15:0] feat_snr,
    output reg  [15:0] feat_duty,
    output reg         spectral_valid
);

    // =========================================================
    // FSM
    // =========================================================
    localparam S_ACCUMULATE = 3'd0;
    localparam S_LATCH      = 3'd1;
    localparam S_BW_SCAN    = 3'd2;
    localparam S_HARM       = 3'd3;
    localparam S_DONE       = 3'd4;

    reg [2:0] state;

    // =========================================================
    // Magnitude² (kombinasyonel, tek komparatör)
    // =========================================================
    wire signed [15:0] re_s   = fft_re;
    wire signed [15:0] im_s   = fft_im;
    wire        [31:0] mag_sq = (re_s * re_s) + (im_s * im_s);

    // =========================================================
    // Akümülatörler
    // =========================================================
    reg [47:0] acc_energy;
    reg [47:0] acc_centroid_num;
    reg [31:0] max_mag_sq;
    reg [10:0] peak_bin;
    reg [15:0] duty_count;

    // =========================================================
    // mag² BRAM: 512 × 32-bit (tek taraflı spektrum)
    // Sentezde Block RAM veya Distributed RAM olarak çıkar
    // =========================================================
    reg [31:0] mag_bram [0:511];

    // =========================================================
    // Pipeline Stage 1: mag_sq ve centroid çarpımını kaydet
    // Kritik path: fft_index × mag_sq + acc_centroid_num → 15ns
    // Çözüm: çarpım ayrı clock'ta, biriktime bir sonraki clock'ta
    // =========================================================
    reg [31:0] mag_sq_r;           // mag_sq pipeline register (clock 1)
    reg [47:0] centroid_prod_r;    // fft_index × mag_sq (clock 1)
    reg        fft_valid_r;        // fft_out_valid gecikmeli (clock 1)
    reg [10:0] fft_index_r;        // fft_index gecikmeli (clock 1)

    // =========================================================
    // Pipeline Geçici Değişkenler (modül seviyesi)
    // =========================================================
    reg [31:0] max_mag_sq_latch;   // BW_SCAN için kayıtlı kopya
    reg [31:0] bw_half;            // max_mag_sq / 2
    reg [15:0] bw_count;           // Bandwidth sayacı
    reg [8:0]  scan_addr;          // BW_SCAN adresi (0..511)
    reg [9:0]  h2_idx;             // 2. harmonik indeksi
    reg [10:0] h3_idx;             // 3. harmonik indeksi
    reg        harm_stage;         // 0=h2 oku, 1=h3 oku

    // =========================================================
    // Pipeline Stage 1 (FSM'den bağımsız - her saat çalışır)
    // Clock T:   fft_out_valid + fft_index + mag_sq gelir
    // Clock T+1: pipeline register'lardan okunur, akümülatöre yazılır
    // Bu sayede kritik path: sadece çarpım (~5ns) veya toplama (~4ns)
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            mag_sq_r        <= 32'd0;
            centroid_prod_r <= 48'd0;
            fft_valid_r     <= 1'b0;
            fft_index_r     <= 11'd0;
        end else begin
            fft_valid_r     <= fft_out_valid;
            fft_index_r     <= fft_index;
            mag_sq_r        <= mag_sq;
            // Stage 1 çarpım: fft_index (11-bit) × mag_sq (32-bit) = 43-bit
            // Vivado bunu 2 DSP48 ile yapar (~4ns)
            centroid_prod_r <= {37'd0, fft_index} * {5'd0, mag_sq};
        end
    end

    // =========================================================
    // Ana FSM
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            state            <= S_ACCUMULATE;
            acc_energy       <= 0;
            acc_centroid_num <= 0;
            max_mag_sq       <= 0;
            max_mag_sq_latch <= 0;
            peak_bin         <= 0;
            duty_count       <= 0;
            bw_count         <= 0;
            scan_addr        <= 0;
            harm_stage       <= 0;
            spectral_valid   <= 0;
            feat_peak_freq   <= 0; feat_peak_mag  <= 0;
            feat_bandwidth   <= 0; feat_centroid  <= 0;
            feat_flatness    <= 0; feat_energy    <= 0;
            feat_harm2       <= 0; feat_harm3     <= 0;
            feat_snr         <= 0; feat_duty      <= 0;
            // Pipeline stage 1 reset
            fft_valid_r     <= 0;
            fft_index_r     <= 0;
            mag_sq_r        <= 0;
            centroid_prod_r <= 0;
        end
        else begin
            spectral_valid <= 1'b0;

            case (state)

                // -------------------------------------------------
                // S_ACCUMULATE: Pipeline register'lardan biriktir
                // fft_valid_r / mag_sq_r: 1 saat gecikmeli (pipeline stage 1)
                // Kritik path bölündü:
                //   Stage 1: centroid_prod_r = fft_index × mag_sq  (~5ns)
                //   Stage 2: acc_centroid_num += centroid_prod_r    (~4ns)
                // -------------------------------------------------
                S_ACCUMULATE: begin
                    // Stage 2: pipeline register'dan biriktir
                    if (fft_valid_r) begin
                        if (fft_index_r < 11'd512)
                            mag_bram[fft_index_r[8:0]] <= mag_sq_r;

                        acc_energy       <= acc_energy + {16'd0, mag_sq_r};
                        // Çarpım zaten stage 1'de hesaplandı → sadece toplama
                        acc_centroid_num <= acc_centroid_num + centroid_prod_r;

                        if (mag_sq_r > max_mag_sq) begin
                            max_mag_sq <= mag_sq_r;
                            peak_bin   <= fft_index_r;
                        end

                        if (mag_sq_r > (max_mag_sq >> 3))
                            duty_count <= duty_count + 16'd1;
                    end

                    // fft_frame_done 1 saat sonra geldiğinde son bin
                    // pipeline'dan geçmiş olacak (fft_valid_r ile yakalanır)
                    if (fft_frame_done)
                        state <= S_LATCH;
                end

                // -------------------------------------------------
                // S_LATCH: Basit öznitelikleri kaydet (1 saat)
                // -------------------------------------------------
                S_LATCH: begin
                    feat_peak_freq   <= {5'd0, peak_bin};
                    feat_peak_mag    <= max_mag_sq[31:16];
                    feat_energy      <= acc_energy[47:32];
                    feat_centroid    <= (acc_energy != 0)
                                       ? acc_centroid_num[47:32] : 16'd0;
                    feat_flatness    <= (max_mag_sq != 0)
                                       ? acc_energy[40:25] : 16'd0;
                    feat_snr         <= max_mag_sq[31:16];
                    feat_duty        <= {duty_count[8:0], 7'd0};

                    // BW_SCAN için hazırlık
                    max_mag_sq_latch <= max_mag_sq;
                    bw_half          <= max_mag_sq >> 1;
                    bw_count         <= 0;
                    scan_addr        <= 0;

                    // Akümülatörleri sıfırla
                    acc_energy       <= 0;
                    acc_centroid_num <= 0;
                    max_mag_sq       <= 0;
                    peak_bin         <= 0;
                    duty_count       <= 0;

                    state <= S_BW_SCAN;
                end

                // -------------------------------------------------
                // S_BW_SCAN: 512 bin sıralı tara - her saat 1 bin
                // 512 paralel komparatör yerine TEK komparatör
                // 512 saat sürer ama LUT kullanımı ~200'e düşer
                // -------------------------------------------------
                S_BW_SCAN: begin
                    // mag_bram registered output: scan_addr'dan 1 saat gecikmeli
                    // Bu yüzden scan_addr+1 adresini önceden yüklüyoruz
                    if (mag_bram[scan_addr] > bw_half)
                        bw_count <= bw_count + 16'd1;

                    if (scan_addr == 9'd511) begin
                        feat_bandwidth <= bw_count;
                        // Harmonik okuma için hazırlık
                        h2_idx     <= {1'b0, peak_bin[8:0]} << 1;
                        h3_idx     <= {2'b0, peak_bin[8:0]}
                                    + ({2'b0, peak_bin[8:0]} << 1);
                        harm_stage <= 1'b0;
                        state      <= S_HARM;
                    end else begin
                        scan_addr <= scan_addr + 1'b1;
                    end
                end

                // -------------------------------------------------
                // S_HARM: İki harmonik binini oku (2 saat)
                // harm_stage=0: h2 adresini ver → 1 saat bekle
                // harm_stage=1: h2 çıktısını al, h3'ü ver
                // S_DONE'da h3 çıktısını al
                // -------------------------------------------------
                S_HARM: begin
                    if (!harm_stage) begin
                        // h2 okunuyor (mag_bram[h2_idx] 1 saat gecikmeli)
                        feat_harm2 <= (h2_idx < 10'd512)
                                      ? mag_bram[h2_idx[8:0]][31:16]
                                      : 16'd0;
                        harm_stage <= 1'b1;
                    end else begin
                        feat_harm3 <= (h3_idx < 11'd512)
                                      ? mag_bram[h3_idx[8:0]][31:16]
                                      : 16'd0;
                        state <= S_DONE;
                    end
                end

                // -------------------------------------------------
                // S_DONE: spectral_valid pulse, sonra başa dön
                // -------------------------------------------------
                S_DONE: begin
                    spectral_valid <= 1'b1;
                    state          <= S_ACCUMULATE;
                end

                default: state <= S_ACCUMULATE;
            endcase
        end
    end

endmodule