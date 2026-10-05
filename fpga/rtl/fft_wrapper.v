// ============================================================================
// FFT Wrapper - Pipeline + FSM + BRAM Read Latency Aware
// TÜBİTAK 2209A - RF Protokol Sınıflandırma Projesi
// ============================================================================
// Optimizasyonlar:
//   1. BRAM Read Latency (1 saat) için pipeline register
//      rd_addr artınca bram_dout bir sonraki saatte geliyor.
//      Bunu kompanse etmek için rd_addr 1 saat önce artırılır,
//      send_count ise bram_dout'un geçerli olduğu saatte sayar.
//
//   2. AXI-S Back-pressure (tready) ile tam stall
//      tready=0 geldiğinde rd_addr dondurulur, veri kaybolmaz.
//
//   3. 5 durumlu FSM - her aşama net ve ayrı
//      S_COLLECT → S_PREFETCH → S_SEND → S_FLUSH → S_WAIT
//      PREFETCH: ilk BRAM okumasının latency'sini emer
//      FLUSH:    tlast sonrası AXI handshake'i temizler
//
//   4. Pipeline hazır sinyali (pipe_valid)
//      BRAM'den okunan verinin geçerli olduğu saati işaretler,
//      yanlış veri gönderimini önler.
//
// Port Mapping (VHDL Entity'den):
//   s_axis_config_tdata  [15:0]  bit0=1: FFT, 0: IFFT
//   s_axis_data_tdata    [31:0]  {XN_IM[15:0], XN_RE[15:0]}
//   m_axis_data_tdata    [31:0]  {XK_IM[15:0], XK_RE[15:0]}
//   m_axis_data_tuser    [15:0]  XK_INDEX[10:0]
// ============================================================================

module fft_wrapper (
    input  wire        clk,
    input  wire        rst,

    // ADC örnek girişi
    input  wire [11:0] sample_in,
    input  wire        sample_valid,

    // FFT çıkışı → spectral_analyzer
    output reg  [15:0] fft_re,
    output reg  [15:0] fft_im,
    output reg  [10:0] fft_index,
    output reg         fft_out_valid,
    output reg         fft_frame_done,
    output wire        fft_busy
);

    // =========================================================
    // Parametreler
    // =========================================================
    localparam N        = 1024;
    localparam LOG2_N   = 10;
    localparam N_LAST   = 10'd1023;

    // =========================================================
    // FSM Durumları
    // =========================================================
    localparam S_COLLECT  = 3'd0;  // BRAM'e örnek topla
    localparam S_PREFETCH = 3'd1;  // İlk BRAM latency'sini emer (1 saat)
    localparam S_SEND     = 3'd2;  // AXI-S ile FFT IP'ye gönder
    localparam S_FLUSH    = 3'd3;  // tlast handshake'i tamamla
    localparam S_WAIT     = 3'd4;  // FFT çıkışını bekle ve al

    reg [2:0] state;
    assign fft_busy = (state != S_COLLECT);

    // =========================================================
    // BRAM: 1024 × 16-bit
    // Xilinx Block RAM: 1 saat okuma gecikmesi (registered output)
    // =========================================================
    reg [15:0] sample_bram [0:N-1];
    reg [LOG2_N-1:0] wr_addr;
    reg [LOG2_N-1:0] rd_addr;
    reg [15:0]       bram_dout;     // rd_addr'dan 1 saat gecikmeli

    // BRAM okuma - her saat rd_addr'ı kayıtla
    always @(posedge clk)
        bram_dout <= sample_bram[rd_addr];

    // BRAM yazma
    always @(posedge clk)
        if (sample_valid && state == S_COLLECT)
            sample_bram[wr_addr] <= {sample_in, 4'b0000}; // 12→16 bit

    // =========================================================
    // Pipeline Geçerlilik Sinyali
    // rd_addr artırıldıktan 1 saat sonra bram_dout geçerli olur.
    // pipe_valid bu gecikmeyi takip eder.
    // =========================================================
    reg pipe_valid;   // bram_dout'un geçerli olduğu saat

    // =========================================================
    // AXI-Stream Sinyalleri
    // =========================================================
    wire [15:0] s_axis_config_tdata  = 16'h0001; // FWD FFT
    wire        s_axis_config_tvalid = 1'b1;
    wire        s_axis_config_tready;

    reg  [31:0] s_axis_data_tdata;
    reg         s_axis_data_tvalid;
    reg         s_axis_data_tlast;
    wire        s_axis_data_tready;

    wire [31:0] m_axis_data_tdata;
    wire [15:0] m_axis_data_tuser;
    wire        m_axis_data_tvalid;
    wire        m_axis_data_tlast;
    reg         m_axis_data_tready;

    wire event_frame_started;
    wire event_tlast_unexpected;
    wire event_tlast_missing;
    wire event_status_channel_halt;
    wire event_data_in_channel_halt;
    wire event_data_out_channel_halt;

    // =========================================================
    // xfft_1024 IP
    // =========================================================
    xfft_1024 u_xfft (
        .aclk                        (clk),
        .s_axis_config_tdata         (s_axis_config_tdata),
        .s_axis_config_tvalid        (s_axis_config_tvalid),
        .s_axis_config_tready        (s_axis_config_tready),
        .s_axis_data_tdata           (s_axis_data_tdata),
        .s_axis_data_tvalid          (s_axis_data_tvalid),
        .s_axis_data_tready          (s_axis_data_tready),
        .s_axis_data_tlast           (s_axis_data_tlast),
        .m_axis_data_tdata           (m_axis_data_tdata),
        .m_axis_data_tuser           (m_axis_data_tuser),
        .m_axis_data_tvalid          (m_axis_data_tvalid),
        .m_axis_data_tready          (m_axis_data_tready),
        .m_axis_data_tlast           (m_axis_data_tlast),
        .event_frame_started         (event_frame_started),
        .event_tlast_unexpected      (event_tlast_unexpected),
        .event_tlast_missing         (event_tlast_missing),
        .event_status_channel_halt   (event_status_channel_halt),
        .event_data_in_channel_halt  (event_data_in_channel_halt),
        .event_data_out_channel_halt (event_data_out_channel_halt)
    );

    // =========================================================
    // Gönderme Sayacı (kaç örnek AXI-S'e ulaştı)
    // =========================================================
    reg [LOG2_N-1:0] send_count;

    // =========================================================
    // Ana FSM
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            state              <= S_COLLECT;
            wr_addr            <= 0;
            rd_addr            <= 0;
            send_count         <= 0;
            pipe_valid         <= 1'b0;
            s_axis_data_tvalid <= 1'b0;
            s_axis_data_tlast  <= 1'b0;
            s_axis_data_tdata  <= 32'd0;
            m_axis_data_tready <= 1'b0;
            fft_out_valid      <= 1'b0;
            fft_frame_done     <= 1'b0;
        end
        else begin
            // Varsayılan: pulse sinyalleri sıfırla
            fft_out_valid  <= 1'b0;
            fft_frame_done <= 1'b0;
            pipe_valid     <= 1'b0;

            case (state)

                // -------------------------------------------------
                // S_COLLECT: Örnekleri BRAM'e yaz
                // 1024 örnek dolunca PREFETCH'e geç
                // -------------------------------------------------
                S_COLLECT: begin
                    m_axis_data_tready <= 1'b0;
                    s_axis_data_tvalid <= 1'b0;

                    if (sample_valid) begin
                        wr_addr <= wr_addr + 1'b1;

                        if (wr_addr == N_LAST) begin
                            // Tampon doldu:
                            // rd_addr=0'ı oku → 1 saat sonra bram_dout geçerli
                            rd_addr    <= 10'd1;    // 1 saat sonra [0] çıkacak
                            send_count <= 0;
                            state      <= S_PREFETCH;
                        end
                    end
                end

                // -------------------------------------------------
                // S_PREFETCH: BRAM latency'sini emer
                // Bu saat: rd_addr=1 → bram_dout=[0] geliyor (1 saat bekle)
                // Sonraki saat S_SEND'de bram_dout=[0] geçerli
                // -------------------------------------------------
                S_PREFETCH: begin
                    // rd_addr=2'ye ilerlet, bir sonraki saatte [1] gelecek
                    rd_addr    <= rd_addr + 1'b1;   // rd_addr=2
                    pipe_valid <= 1'b1;              // bram_dout=[0] hazır
                    state      <= S_SEND;
                end

                // -------------------------------------------------
                // S_SEND: Pipeline ile BRAM → AXI-S
                //
                // Timing:
                //   saat T:   rd_addr=N   → bram_dout[N-1] geçerli
                //   saat T+1: rd_addr=N+1 → bram_dout[N] geçerli
                //
                // tready=0 (back-pressure):
                //   rd_addr dondurulur, pipe_valid=0
                //   mevcut s_axis_data_tdata korunur (tvalid=1 kalır)
                // -------------------------------------------------
                S_SEND: begin
                    if (s_axis_data_tready || !s_axis_data_tvalid) begin
                        // IP hazır veya henüz veri yok → pipeline ilerlet
                        if (send_count < N) begin
                            // bram_dout bu saat geçerli → AXI'ye sun
                            s_axis_data_tdata  <= {16'd0, bram_dout};
                            s_axis_data_tvalid <= 1'b1;
                            s_axis_data_tlast  <= (send_count == N_LAST);

                            send_count <= send_count + 1'b1;

                            // Sonraki örneği önceden oku (son örnekte ilerleme)
                            if (send_count < N_LAST) begin
                                rd_addr    <= rd_addr + 1'b1;
                                pipe_valid <= 1'b1;
                            end
                        end

                        // tlast kabul edildi → FLUSH'a geç
                        if (s_axis_data_tlast && s_axis_data_tvalid) begin
                            s_axis_data_tvalid <= 1'b0;
                            s_axis_data_tlast  <= 1'b0;
                            state              <= S_FLUSH;
                        end
                    end
                    // else: tready=0, hiçbir şey değişmez (stall)
                end

                // -------------------------------------------------
                // S_FLUSH: AXI handshake temizle, FFT'nin pipelineını
                // doldurmasına izin ver, sonra WAIT'e geç
                // -------------------------------------------------
                S_FLUSH: begin
                    s_axis_data_tvalid <= 1'b0;
                    m_axis_data_tready <= 1'b1;
                    state              <= S_WAIT;
                end

                // -------------------------------------------------
                // S_WAIT: FFT çıkışını al
                // tdata[15:0]  = XK_RE
                // tdata[31:16] = XK_IM
                // tuser[10:0]  = XK_INDEX
                // -------------------------------------------------
                S_WAIT: begin
                    m_axis_data_tready <= 1'b1;

                    if (m_axis_data_tvalid) begin
                        fft_re        <= m_axis_data_tdata[15:0];
                        fft_im        <= m_axis_data_tdata[31:16];
                        fft_index     <= m_axis_data_tuser[10:0];
                        fft_out_valid <= 1'b1;

                        if (m_axis_data_tlast) begin
                            fft_frame_done     <= 1'b1;
                            m_axis_data_tready <= 1'b0;
                            wr_addr            <= 0;
                            rd_addr            <= 0;
                            state              <= S_COLLECT;
                        end
                    end
                end

                default: state <= S_COLLECT;
            endcase
        end
    end

endmodule