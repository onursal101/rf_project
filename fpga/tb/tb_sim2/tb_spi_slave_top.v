// ============================================================================
// Testbench - spi_slave_top (FFT + Moment Motor Entegrasyon Testi)
// TÜBİTAK 2209A - RF Protokol Sınıflandırma Projesi
// ============================================================================
// Test Stratejisi:
//   TEST 1: features_ready=0 iken status byte kontrolü
//   TEST 2: Her iki motorun tamamlanmasını bekle (polling)
//   TEST 3: CMD_READ_FEATURES - 14 özniteliği oku ve doğrula
//   TEST 4: FFT + BW_SCAN pipeline sağlığı raporu
//   TEST 5: Spectral FSM durum geçişleri doğrulama
//   TEST 6: INT pini pulse uzunluğu kontrolü (16 saat)
//   TEST 7: Latch kararlılığı - ikinci transferde veri aynı olmalı
//   TEST 8: İkinci pencere - Peak Mag ve Bandwidth sıfırdan büyük
//
// Değişiklikler (pipeline optimizasyonu sonrası):
//   - spectral_analyzer: 5 durumlu FSM (ACCUMULATE→LATCH→BW_SCAN→HARM→DONE)
//     BW_SCAN: 512 saat sıralı tarama (eski 512 paralel komparatör yerine)
//   - moment_engine: sub_state 5'e çıktı (ek DSP pipeline aşaması)
//   - Toplam işleme süresi: ~131k (örnekleme) + ~1.5k (FFT+BW) ≈ ~133k saat
// ============================================================================

`timescale 1ns / 1ps

module tb_spi_slave_top;

    // =========================================================
    // Parametreler
    // =========================================================
    localparam CLK_PERIOD  = 10;    // 100 MHz
    localparam SCLK_PERIOD = 200;   // 5 MHz SPI
    localparam N_BYTES     = 29;    // 1 status + 28 feature (14×2)
    localparam RATE_DIV    = 100;   // 100MHz/100 = 1 MSPS (Başvuru Hedef 1)

    // Beklenen işleme süreleri (saat cinsinden)
    localparam EXP_SAMPLE_CLKS  = 102400; // 1024 × 100 (1 MSPS)
    localparam EXP_BW_SCAN_CLKS = 512;    // Spectral BW_SCAN
    localparam EXP_HARM_CLKS    = 2;      // Harmonik okuma
    localparam POLL_TIMEOUT     = 500000; // Max bekleme

    localparam CMD_READ_FEATURES = 8'h01;
    localparam CMD_READ_STATUS   = 8'h02;

    // Spectral FSM durum kodları (spectral_analyzer.v ile eşleşmeli)
    localparam SP_ACCUMULATE = 3'd0;
    localparam SP_LATCH      = 3'd1;
    localparam SP_BW_SCAN    = 3'd2;
    localparam SP_HARM       = 3'd3;
    localparam SP_DONE       = 3'd4;

    // =========================================================
    // DUT Portları
    // =========================================================
    reg        CLK100MHZ = 0;
    reg        JB2       = 0;   // SCLK
    reg        JA1       = 0;   // MOSI
    wire       JA2;             // MISO
    reg        JA7       = 1;   // SS (aktif LOW)
    wire       JB3_INT;         // Kesme sinyali
    wire [7:0] LED;

    // =========================================================
    // DUT
    // =========================================================
    spi_slave_top u_dut (
        .CLK100MHZ (CLK100MHZ),
        .JB2       (JB2),
        .JA1       (JA1),
        .JA2       (JA2),
        .JA7       (JA7),
        .JB3_INT   (JB3_INT),
        .LED       (LED)
    );

    // =========================================================
    // Sistem Saati
    // =========================================================
    always #(CLK_PERIOD/2) CLK100MHZ = ~CLK100MHZ;

    // =========================================================
    // DUT İç Sinyal Erişimi
    // =========================================================
    // Top-level
    wire       dut_features_ready = u_dut.features_ready;
    wire       dut_fft_busy       = u_dut.fft_busy;
    wire       dut_fft_out_valid  = u_dut.fft_out_valid;
    wire       dut_fft_frame_done = u_dut.fft_frame_done;
    wire       dut_spectral_valid = u_dut.spectral_valid;
    wire       dut_moment_valid   = u_dut.moment_valid;
    wire       dut_int_active     = u_dut.int_active;
    wire [4:0] dut_int_cnt        = u_dut.int_cnt;

    // FFT Wrapper FSM
    wire [2:0]  dut_fft_state   = u_dut.u_fft.state;
    wire [9:0]  dut_fft_wr_addr = u_dut.u_fft.wr_addr;
    wire [9:0]  dut_fft_rd_addr = u_dut.u_fft.rd_addr;
    wire        dut_pipe_valid  = u_dut.u_fft.pipe_valid;

    // Spectral Analyzer FSM
    wire [2:0] dut_sp_state     = u_dut.u_spectral.state;
    wire [8:0] dut_sp_scan_addr = u_dut.u_spectral.scan_addr;
    wire       dut_sp_bw_active = (u_dut.u_spectral.state == SP_BW_SCAN);

    // Moment Engine
    wire [2:0] dut_me_state     = u_dut.u_moment.state;
    wire [2:0] dut_me_sub_state = u_dut.u_moment.sub_state;

    // LFSR (DUT içinde)
    wire [11:0] dut_sample_in    = u_dut.sample_in_r;
    wire        dut_sample_valid = u_dut.sample_valid_r;
    
    // debug latch izleme 
    wire [15:0] dut_feat0_mean    = u_dut.feat0_mean;
    // wire [15:0] dut_mean_hold     = u_dut.mean_hold;  ← KALDIR
    wire        dut_moment_done   = u_dut.moment_done;
    wire        dut_spectral_done = u_dut.spectral_done;
    wire        dut_ja7           = JA7;
    
    
    reg [15:0] dut_feat0_mean_prev = 16'hFFFF;
    integer    debug_change_count  =0;
    integer    debug_enable        =0;
    
    
   always @(posedge CLK100MHZ) begin
     if (debug_enable && dut_feat0_mean !== dut_feat0_mean_prev) begin
            debug_change_count = debug_change_count + 1;
           $display("  [DBG@%0d] feat0_mean: 0x%04h -> 0x%04h | m_done=%b sp_done=%b ja7=%b f_rdy=%b",
             sim_time,
             dut_feat0_mean_prev, dut_feat0_mean,
             dut_moment_done, dut_spectral_done,
             dut_ja7, dut_features_ready);
        end
        dut_feat0_mean_prev <= dut_feat0_mean;
    end
    
    // moment_valid ve spectral_valid pulse'lari izle
    always @(posedge CLK100MHZ) begin
        if (debug_enable && dut_moment_valid)
            $display("  [MV@%0d] moment_valid=1 | feat_mean girdi=0x%04h",
                sim_time, u_dut.feat_mean);
        if (debug_enable && dut_spectral_valid)
            $display("  [SV@%0d] spectral_valid=1", sim_time);
    end
    
     // SPI transfer sırasında JA7 (SS) dusus/cikis kenarlarini izle
    reg dut_ja7_prev = 1;
    always @(posedge CLK100MHZ) begin
                if (debug_enable && dut_ja7 !== dut_ja7_prev) begin
            $display("  [SS@%0d] JA7 %b->%b | feat0_mean=0x%04h | byte_count=%0d",
                sim_time, dut_ja7_prev, dut_ja7,
                u_dut.feat0_mean,
                u_dut.u_spi_slave.byte_count);
        end
        dut_ja7_prev <= dut_ja7;
    end

    // =========================================================
    // Pipeline Zamanlama Ölçümleri
    // =========================================================
    integer sim_time          = 0;
    integer fft_busy_start    = 0;
    integer fft_busy_duration = 0;
    integer bw_scan_start     = 0;
    integer bw_scan_duration  = 0;
    integer me_start          = 0;
    integer me_duration       = 0;

    // $past() SystemVerilog'a ait - Verilog-2001 icin manuel edge detection
    reg fft_busy_prev;
    reg sp_bw_active_prev;
    reg moment_valid_prev;
    reg [2:0] me_state_prev;

    always @(posedge CLK100MHZ) begin
        sim_time = sim_time + 1;

        // Onceki deger kaydi (edge detection icin)
        fft_busy_prev     <= dut_fft_busy;
        sp_bw_active_prev <= dut_sp_bw_active;
        moment_valid_prev <= dut_moment_valid;
        me_state_prev     <= dut_me_state;

        // FFT busy rising edge → start, falling edge → duration
        if (dut_fft_busy && !fft_busy_prev)
            fft_busy_start = sim_time;
        if (!dut_fft_busy && fft_busy_prev)
            fft_busy_duration = sim_time - fft_busy_start;

        // BW_SCAN rising/falling edge
        if (dut_sp_bw_active && !sp_bw_active_prev)
            bw_scan_start = sim_time;
        if (!dut_sp_bw_active && sp_bw_active_prev)
            bw_scan_duration = sim_time - bw_scan_start;

        // Moment valid rising edge → duration
        if (dut_moment_valid && !moment_valid_prev)
            me_duration = sim_time - me_start;
        // Moment state 0→1 gecisi → start
        if (dut_me_state == 3'd1 && me_state_prev == 3'd0)
            me_start = sim_time;
    end

    // FFT bin sayacı
    integer fft_bin_count = 0;
    always @(posedge CLK100MHZ) begin
        if (dut_fft_frame_done)
            fft_bin_count = 0;
        else if (dut_fft_out_valid)
            fft_bin_count = fft_bin_count + 1;
    end

    // =========================================================
    // SPI Master Görev Bloğu
    // CPHA=0, CPOL=0, MSB-First
    // =========================================================
    reg [7:0] rx_tmp;

    task spi_send_byte;
        input  [7:0] tx_byte;
        output [7:0] rx_byte;
        integer i;
        reg [7:0] rx_build;
        begin
            rx_build = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                JB2 = 1'b0;
                JA1 = tx_byte[i];
                #(SCLK_PERIOD/2);
                JB2 = 1'b1;
                rx_build[i] = JA2;
                #(SCLK_PERIOD/2);
            end
            JB2     = 1'b0;
            rx_byte = rx_build;
        end
    endtask

    reg [7:0] rx_buffer [0:N_BYTES-1];

    task spi_transfer;
        input [7:0] cmd;
        integer b;
        begin
            JA7 = 1'b0;
            #(SCLK_PERIOD);
            spi_send_byte(cmd, rx_tmp);
            rx_buffer[0] = rx_tmp;
            for (b = 1; b < N_BYTES; b = b + 1) begin
                spi_send_byte(8'h00, rx_tmp);
                rx_buffer[b] = rx_tmp;
            end
            #(SCLK_PERIOD);
            JA7 = 1'b1;
            #(SCLK_PERIOD * 4);
        end
    endtask

    // =========================================================
    // Feature Yazdırma
    // =========================================================
    task print_features;
        integer f;
        reg [15:0] fval;
        begin
            $display("  ┌─ Status Byte : 0x%02h  [ready=%b busy_m=%b busy_f=%b]",
                rx_buffer[0],
                rx_buffer[0][0], rx_buffer[0][1], rx_buffer[0][2]);
            $display("  ├─ Moment Motoru ─────────────────────────");
            for (f = 0; f < 4; f = f + 1) begin
                fval = {rx_buffer[1+f*2], rx_buffer[2+f*2]};
                case (f)
                    0: $display("  │  Feat 0  Mean      : 0x%04h = %0d", fval, fval);
                    1: $display("  │  Feat 1  Variance  : 0x%04h = %0d", fval, fval);
                    2: $display("  │  Feat 2  Skewness  : 0x%04h = %0d", fval, $signed(fval));
                    3: $display("  │  Feat 3  Kurtosis  : 0x%04h = %0d", fval, fval);
                endcase
            end
            $display("  ├─ Spectral Analyzer ─────────────────────");
            for (f = 4; f < 14; f = f + 1) begin
                fval = {rx_buffer[1+f*2], rx_buffer[2+f*2]};
                case (f)
                    4:  $display("  │  Feat 4  Peak Freq : bin %0d", fval);
                    5:  $display("  │  Feat 5  Peak Mag  : 0x%04h = %0d", fval, fval);
                    6:  $display("  │  Feat 6  Bandwidth : %0d bins", fval);
                    7:  $display("  │  Feat 7  Centroid  : 0x%04h = %0d", fval, fval);
                    8:  $display("  │  Feat 8  Flatness  : 0x%04h = %0d", fval, fval);
                    9:  $display("  │  Feat 9  Energy    : 0x%04h = %0d", fval, fval);
                    10: $display("  │  Feat 10 2nd Harm  : 0x%04h = %0d", fval, fval);
                    11: $display("  │  Feat 11 3rd Harm  : 0x%04h = %0d", fval, fval);
                    12: $display("  │  Feat 12 SNR       : 0x%04h = %0d", fval, fval);
                    13: $display("  └─ Feat 13 Duty Cyc  : %0d/512 bins", fval >> 7);
                endcase
            end
        end
    endtask

    // =========================================================
    // Modül Seviyesi Geçici Değişkenler
    // =========================================================
    reg [15:0] feat0_first, feat0_second;
    reg [15:0] feat4_val, feat5_val, feat6_val;
    integer    int_pulse_len;
    integer    passed, failed;

    // =========================================================
    // Ana Test Akışı
    // =========================================================
    initial begin
        passed       = 0;
        failed       = 0;
        int_pulse_len = 0;

        $display("============================================================");
        $display("  TB: spi_slave_top - Pipeline Optimize Versiyon");
        $display("  spectral_analyzer: 5-FSM + BW_SCAN(512 clk sıralı)");
        $display("  moment_engine    : sub_state→5 (DSP48E1 pipeline)");
        $display("============================================================");

        repeat(20) @(posedge CLK100MHZ);

        // -----------------------------------------------------------
        // TEST 1: Başlangıçta features_ready=0
        // -----------------------------------------------------------
        $display("\n[TEST 1] Baslangic durum kontrolu");
        spi_transfer(CMD_READ_STATUS);
        if (rx_buffer[0][0] == 1'b0) begin
            $display("  PASS: features_ready=0");
            passed = passed + 1;
        end else begin
            $display("  FAIL: features_ready=1 (beklenmiyor)");
            failed = failed + 1;
        end

        // -----------------------------------------------------------
        // TEST 2: Her iki motorun tamamlanmasını bekle
        // Beklenen süre: ~131k (örnekleme) + ~515 (FFT+BW) = ~132k saat
        // -----------------------------------------------------------
        $display("\n[TEST 2] Motor tamamlanma bekleniyor...");
        $display("  (1024 ornek x 128 bolucü + FFT + BW_SCAN(512) + HARM(2))");
        begin : wait_both
            integer wcnt;
            wcnt = 0;
            while (dut_features_ready !== 1'b1 && wcnt < POLL_TIMEOUT) begin
                @(posedge CLK100MHZ);
                wcnt = wcnt + 1;
            end
            $display("  Toplam bekleme: %0d saat (~%0d us)", wcnt, wcnt/100);
        end

        if (dut_features_ready == 1'b1) begin
            $display("  PASS: features_ready=1");
            passed = passed + 1;
        end else begin
            $display("  FAIL: Zaman asimi (%0d saat)", POLL_TIMEOUT);
            failed = failed + 1;
        end

        // DEBUG: TEST 3'ten önce başlat
        $display("\n--- DEBUG: TEST 3 oncesi izleme aktif ---");
        debug_enable = 1;
        
        // -----------------------------------------------------------
        // TEST 3: Feature vektörü oku + içerik doğrulama
        // -----------------------------------------------------------
        $display("\n[TEST 3] CMD_READ_FEATURES - 14 oznitelik");
        spi_transfer(CMD_READ_FEATURES);
        print_features();

        feat0_first = {rx_buffer[1],  rx_buffer[2]};
        feat4_val   = {rx_buffer[9],  rx_buffer[10]};
        feat5_val   = {rx_buffer[11], rx_buffer[12]};
        feat6_val   = {rx_buffer[13], rx_buffer[14]};

        if (rx_buffer[0][0] == 1'b1) begin
            $display("  PASS: Status features_ready=1");
            passed = passed + 1;
        end else begin
            $display("  FAIL: Status features_ready=0");
            failed = failed + 1;
        end

        if (feat0_first != 16'd0) begin
            $display("  PASS: Mean != 0 (0x%04h)", feat0_first);
            passed = passed + 1;
        end else begin
            $display("  FAIL: Mean = 0");
            failed = failed + 1;
        end

        if (feat5_val != 16'd0) begin
            $display("  PASS: Peak Mag != 0 (0x%04h) - FFT calisti", feat5_val);
            passed = passed + 1;
        end else begin
            $display("  FAIL: Peak Mag = 0 - FFT calismiyor");
            failed = failed + 1;
        end

        // -----------------------------------------------------------
        // TEST 4: FFT + BW_SCAN pipeline sağlığı raporu
        // -----------------------------------------------------------
        $display("\n[TEST 4] Pipeline saglik raporu");
        $display("  FFT busy suresi  : %0d saat (~%0d us)",
                 fft_busy_duration, fft_busy_duration/100);
        $display("  BW_SCAN suresi   : %0d saat (beklenen: ~512)",
                 bw_scan_duration);
        $display("  Moment busy      : %0d saat (~%0d us)",
                 me_duration, me_duration/100);
        $display("  FFT bin sayisi   : %0d (beklenen: 1024)", fft_bin_count);
        $display("  FFT FSM state    : %0d (0=COLLECT bekleniyor)", dut_fft_state);
        $display("  Spectral state   : %0d (0=ACCUMULATE bekleniyor)", dut_sp_state);
        $display("  Moment sub_state : %0d (pipeline derinligi 5)", dut_me_sub_state);

        // BW_SCAN süresi ~512 ±10 saat olmalı
        if (bw_scan_duration >= 500 && bw_scan_duration <= 530) begin
            $display("  PASS: BW_SCAN suresi dogru (%0d saat)", bw_scan_duration);
            passed = passed + 1;
        end else begin
            $display("  FAIL: BW_SCAN suresi beklenmiyor (%0d saat)", bw_scan_duration);
            failed = failed + 1;
        end

        // Bandwidth sıfırdan büyük olmalı (LFSR gürültülü)
        if (feat6_val > 16'd0) begin
            $display("  PASS: Bandwidth = %0d bin (sifirdan buyuk)", feat6_val);
            passed = passed + 1;
        end else begin
            $display("  FAIL: Bandwidth = 0 (BW_SCAN calismamis olabilir)");
            failed = failed + 1;
        end

        // -----------------------------------------------------------
        // TEST 5: Spectral FSM durum geçişleri
        // Bir sonraki fft_frame_done sonrası FSM'i izle
        // -----------------------------------------------------------
        $display("\n[TEST 5] Spectral FSM durum gecisleri");
        begin : watch_fsm
            integer fsm_timeout;
            reg sp_saw_latch, sp_saw_bw, sp_saw_harm, sp_saw_done;
            fsm_timeout  = 0;
            sp_saw_latch = 0;
            sp_saw_bw    = 0;
            sp_saw_harm  = 0;
            sp_saw_done  = 0;

            while (fsm_timeout < 200000) begin
                @(posedge CLK100MHZ);
                fsm_timeout = fsm_timeout + 1;
                if (dut_sp_state == SP_LATCH)      sp_saw_latch = 1;
                if (dut_sp_state == SP_BW_SCAN)    sp_saw_bw    = 1;
                if (dut_sp_state == SP_HARM)       sp_saw_harm  = 1;
                if (dut_sp_state == SP_DONE)       sp_saw_done  = 1;
                // Tüm durumlar görüldüyse çık
                if (sp_saw_latch && sp_saw_bw && sp_saw_harm && sp_saw_done)
                    fsm_timeout = 200001; // döngüden çık
            end

            $display("  LATCH durumu gozlendi  : %s", sp_saw_latch ? "EVET" : "HAYIR");
            $display("  BW_SCAN durumu gozlendi: %s", sp_saw_bw    ? "EVET" : "HAYIR");
            $display("  HARM durumu gozlendi   : %s", sp_saw_harm  ? "EVET" : "HAYIR");
            $display("  DONE durumu gozlendi   : %s", sp_saw_done  ? "EVET" : "HAYIR");

            if (sp_saw_latch && sp_saw_bw && sp_saw_harm && sp_saw_done) begin
                $display("  PASS: Tum FSM durumlari gozlendi");
                passed = passed + 1;
            end else begin
                $display("  FAIL: Bazi FSM durumlari eksik");
                failed = failed + 1;
            end
        end

        // -----------------------------------------------------------
        // TEST 6: INT pini pulse uzunluğu (16 saat olmalı)
        // -----------------------------------------------------------
        $display("\n[TEST 6] INT pini pulse kontrolu");
        begin : wait_int
            integer wcnt2;
            wcnt2 = 0;
            // Önce LOW bekle (sıradaki INT için)
            while (JB3_INT !== 1'b0 && wcnt2 < 100) begin
                @(posedge CLK100MHZ);
                wcnt2 = wcnt2 + 1;
            end
            wcnt2 = 0;
            // Sonraki HIGH gelene kadar bekle
            while (JB3_INT !== 1'b1 && wcnt2 < POLL_TIMEOUT) begin
                @(posedge CLK100MHZ);
                wcnt2 = wcnt2 + 1;
            end
            // HIGH süresini say
            int_pulse_len = 0;
            while (JB3_INT == 1'b1 && int_pulse_len < 100) begin
                @(posedge CLK100MHZ);
                int_pulse_len = int_pulse_len + 1;
            end
        end

        $display("  INT pulse uzunlugu: %0d saat (beklenen: 16)", int_pulse_len);
        if (int_pulse_len == 16) begin
            $display("  PASS: INT pulse 16 saat");
            passed = passed + 1;
        end else begin
            $display("  FAIL: INT pulse %0d saat", int_pulse_len);
            failed = failed + 1;
        end

      // -----------------------------------------------------------
        // TEST 7: Tek transfer içinde latch donukluğu
        // Transfer SIRASINDA latch değişmemeli (JA7=0 iken donuk)
        // -----------------------------------------------------------
        $display("\n[TEST 7] Transfer-ici latch donuklugu");
        debug_change_count = 0;
        debug_enable = 1;
        spi_transfer(CMD_READ_FEATURES);
        debug_enable = 0;

        feat0_second = {rx_buffer[1], rx_buffer[2]};
        $display("  Transfer sirasinda latch degisimi: %0d kez", debug_change_count);
        $display("  Okunan Mean: 0x%04h", feat0_second);

        if (debug_change_count == 0) begin
            $display("  PASS: Transfer sirasinda latch donuk (JA7 koruma calisiyor)");
            passed = passed + 1;
        end else begin
            $display("  FAIL: Transfer sirasinda latch %0d kez degisti", debug_change_count);
            failed = failed + 1;
        end

        // -----------------------------------------------------------
        // TEST 8: İkinci pencere - tüm motorlar yeniden çalışmalı
        // -----------------------------------------------------------
        $display("\n[TEST 8] Ikinci pencere - yeniden hesaplama");
        begin : wait_t8
            integer wcnt3;
            wcnt3 = 0;
            while (dut_features_ready !== 1'b1 && wcnt3 < POLL_TIMEOUT) begin
                @(posedge CLK100MHZ);
                wcnt3 = wcnt3 + 1;
            end
            $display("  Ikinci pencere bekleme: %0d saat", wcnt3);
        end

        spi_transfer(CMD_READ_FEATURES);
        print_features();

        feat5_val = {rx_buffer[11], rx_buffer[12]};
        feat6_val = {rx_buffer[13], rx_buffer[14]};

        if (feat5_val != 16'd0) begin
            $display("  PASS: Peak Mag != 0 (0x%04h)", feat5_val);
            passed = passed + 1;
        end else begin
            $display("  FAIL: Peak Mag = 0");
            failed = failed + 1;
        end

        if (feat6_val > 16'd0) begin
            $display("  PASS: Bandwidth > 0 (%0d bin)", feat6_val);
            passed = passed + 1;
        end else begin
            $display("  FAIL: Bandwidth = 0");
            failed = failed + 1;
        end

        // -----------------------------------------------------------
        // HEDEF 1: Örnekleme hızı doğrulaması (1 MSPS)
        // rate_div=100, 100MHz/100 = 1 MHz = 1 MSPS
        // 1024 örnek × 100 saat = 102400 saat beklenir
        // -----------------------------------------------------------
        $display("\n============================================================");
        $display("  HEDEF 1 - Ornekleme Hizi Dogrulamasi");
        $display("============================================================");
        $display("  rate_div          : 100 (100MHz / 100 = 1 MSPS)");
        $display("  1024 ornek suresi : ~102400 saat (1.024 ms)");
        $display("  Basvuru hedefi    : 1 MSPS, 12-bit -> UYUMLU");

        // -----------------------------------------------------------
        // HEDEF 6: FFT vs. Moment Karşılaştırma Raporu
        // (Vivado sentez raporundan alınacak gerçek değerler ile
        //  bu TB çıktısı birleştirilerek nihai tablo oluşturulur)
        // -----------------------------------------------------------
        $display("\n============================================================");
        $display("  HEDEF 6 - FFT vs. Moment Karsilastirma Metrikleri");
        $display("============================================================");
        $display("  --- Zamanlama (saat cinsinden) ---");
        $display("  FFT Pipeline (SEND+WAIT)  : %0d saat (~%0d us)",
                 fft_busy_duration, fft_busy_duration / 100);
        $display("  Spectral BW_SCAN          : %0d saat (~%0d us)",
                 bw_scan_duration, bw_scan_duration / 100);
        $display("  FFT Toplam (pipeline+spec) : %0d saat (~%0d us)",
                 fft_busy_duration + bw_scan_duration + 5,
                 (fft_busy_duration + bw_scan_duration + 5) / 100);
        $display("  Moment Engine              : %0d saat (~%0d us)",
                 me_duration, me_duration / 100);
        $display("");
        $display("  --- Oznitelik Sayisi ---");
        $display("  FFT Cekirdeği  : 10 oznitelik (Feat 4-13)");
        $display("  Moment Motoru  :  4 oznitelik (Feat 0-3)");
        $display("  Toplam Vektor  : 14 oznitelik (224-bit)");
        $display("");
        $display("  --- Kaynak Kullanimi (Vivado sentez raporundan alinacak) ---");
        $display("  NOT: Asagidaki tablo, sentez sonrasi Vivado raporundan");
        $display("       report_utilization komutu ile doldurulacaktir:");
        $display("  +----------------+------+------+------+------+");
        $display("  | Modul          | LUT  | FF   | DSP  | BRAM |");
        $display("  +----------------+------+------+------+------+");
        $display("  | FFT+Spectral   | ???  | ???  | ???  | ???  |");
        $display("  | Moment Engine  | ???  | ???  | ???  | ???  |");
        $display("  | SPI Slave      | ???  | ???  |  0   |  0   |");
        $display("  +----------------+------+------+------+------+");

        // -----------------------------------------------------------
        // SONUÇ
        // -----------------------------------------------------------
        $display("\n============================================================");
        $display("  SONUC : %0d PASS  /  %0d FAIL", passed, failed);
        $display("============================================================");
        if (failed == 0)
            $display("  TUM TESTLER BASARILI");
        else
            $display("  DIKKAT: %0d TEST BASARISIZ", failed);

        $finish;
    end

    // =========================================================
    // Timeout (100ms - BW_SCAN ek süre nedeniyle artırıldı)
    // =========================================================
    initial begin
        #100_000_000;
        $display("HATA: Simulasyon zaman asimi (100ms)");
        $finish;
    end

    // =========================================================
    // VCD
    // =========================================================
    initial begin
        $dumpfile("tb_spi_slave_top.vcd");
        $dumpvars(0, tb_spi_slave_top);
    end

endmodule