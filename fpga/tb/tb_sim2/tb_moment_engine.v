// ============================================================================
// Testbench: tb_moment_engine.v
// Test edilen modül: moment_engine.v
// ============================================================================
// Test Senaryoları:
//   TC1: Sabit değer (DC sinyal) - Mean = sabit, Variance ≈ 0
//   TC2: İki nokta arası kare dalga - Mean = (A+B)/2
//   TC3: Rampa sinyali           - Bilinen mean hesabı
//   TC4: RST davranışı           - Sıfırlama kontrolü
//   TC5: Art arda pencere        - Otomatik yeniden başlatma
// ============================================================================

`timescale 1ns / 1ps

module tb_moment_engine;

    // =========================================================
    // Parametre ve Port Tanımları
    // =========================================================
    parameter N_SAMPLES = 1024;
    parameter LOG2_N    = 10;

    reg         clk;
    reg         rst;
    reg  [11:0] sample_in;
    reg         sample_valid;

    wire [15:0] mean_out;
    wire [15:0] variance_out;
    wire [15:0] skewness_out;
    wire [15:0] kurtosis_out;
    wire        results_valid;
    wire        busy;
    wire        collecting;

    // =========================================================
    // DUT Örneği
    // =========================================================
    moment_engine #(
        .N_SAMPLES(N_SAMPLES),
        .LOG2_N   (LOG2_N)
    ) uut (
        .clk          (clk),
        .rst          (rst),
        .sample_in    (sample_in),
        .sample_valid (sample_valid),
        .mean_out     (mean_out),
        .variance_out (variance_out),
        .skewness_out (skewness_out),
        .kurtosis_out (kurtosis_out),
        .results_valid(results_valid),
        .busy         (busy),
        .collecting   (collecting)
    );

    // =========================================================
    // 100 MHz Sistem Saati
    // =========================================================
    parameter CLK_PERIOD = 10; // 10 ns
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================
    // Test Değişkenleri
    // =========================================================
    integer errors;
    integer i;
    reg [15:0] captured_mean;
    reg [15:0] captured_variance;
    reg [15:0] captured_skewness;
    reg [15:0] captured_kurtosis;

    // Hesaplama için geçici
    real ref_mean;
    real ref_variance;
    integer sum_val;

    // =========================================================
    // RST Sinyali
    // =========================================================
    task do_reset;
        integer k;
        begin
            rst          = 1'b1;
            sample_valid = 1'b0;
            sample_in    = 12'd0;
            repeat(5) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
        end
    endtask

    // =========================================================
    // Görev: N adet örnek gönder (her 5 clock'ta bir)
    // =========================================================
    task send_samples;
        input [11:0] fixed_val;   // Sabit değer (rampa için kullanılmaz)
        input        use_ramp;    // 1 = rampa, 0 = sabit
        input        use_square;  // 1 = kare dalga (A=512, B=1536)
        integer idx;
        begin
            for (idx = 0; idx < N_SAMPLES; idx = idx + 1) begin
                @(posedge clk);
                if (use_ramp)
                    sample_in = idx[11:0]; // 0 → 1023
                else if (use_square)
                    sample_in = (idx < N_SAMPLES/2) ? 12'd512 : 12'd1536;
                else
                    sample_in = fixed_val;

                sample_valid = 1'b1;
                @(posedge clk);
                sample_valid = 1'b0;

                // Örnek hızı simülasyonu: 5 clock bekleme
                repeat(4) @(posedge clk);
            end
        end
    endtask

    // =========================================================
    // Görev: results_valid gelene kadar bekle (timeout ile)
    // =========================================================
    task wait_for_result;
        output [15:0] out_mean;
        output [15:0] out_variance;
        output [15:0] out_skewness;
        output [15:0] out_kurtosis;
        integer timeout;
        begin
            timeout = 0;
            // results_valid pulse'ını yakala
            while (!results_valid && timeout < 500000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end

            if (timeout >= 500000) begin
                $display("  [TIMEOUT] results_valid gelmedi!");
                errors = errors + 1;
                out_mean     = 16'hFFFF;
                out_variance = 16'hFFFF;
                out_skewness = 16'hFFFF;
                out_kurtosis = 16'hFFFF;
            end
            else begin
                out_mean     = mean_out;
                out_variance = variance_out;
                out_skewness = skewness_out;
                out_kurtosis = kurtosis_out;
                @(posedge clk); // pulse bitti
            end
        end
    endtask

    // =========================================================
    // Yardımcı: Toleranslı eşitlik kontrolü
    // =========================================================
    task check_near;
        input [15:0] actual;
        input [15:0] expected;
        input [15:0] tolerance;
        input [127:0] label;
        integer diff;
        begin
            if (actual >= expected)
                diff = actual - expected;
            else
                diff = expected - actual;

            if (diff <= tolerance) begin
                $display("  [PASS] %s : 0x%04X (%0d)  ±%0d'e göre beklenen 0x%04X (%0d)",
                         label, actual, actual, tolerance, expected, expected);
            end else begin
                $display("  [FAIL] %s : 0x%04X (%0d)  beklenen 0x%04X (%0d)  |fark=%0d|",
                         label, actual, actual, expected, expected, diff);
                errors = errors + 1;
            end
        end
    endtask

    task test_pass;
        input [127:0] msg;
        begin $display("  [PASS] %s", msg); end
    endtask

    task test_fail;
        input [127:0] msg;
        begin
            $display("  [FAIL] %s", msg);
            errors = errors + 1;
        end
    endtask

    // =========================================================
    // Ana Test Akışı
    // =========================================================
    initial begin
        errors       = 0;
        sample_valid = 1'b0;
        sample_in    = 12'd0;

        $display("==============================================");
        $display("  Moment Engine Testbench Başlıyor");
        $display("==============================================");

        // --------------------------------------------------
        // TC4: RST Davranışı
        // --------------------------------------------------
        $display("\n[TC4] RST davranışı testi...");
        rst = 1'b1;
        @(posedge clk);
        @(posedge clk);
        if (busy !== 1'b0)
            test_fail("RST sonrası busy=0 olmalı");
        else
            test_pass("RST sonrası busy=0");

        if (collecting !== 1'b0)
            test_fail("RST sonrası collecting=0 olmalı");
        else
            test_pass("RST sonrası collecting=0");

        if (results_valid !== 1'b0)
            test_fail("RST sonrası results_valid=0 olmalı");
        else
            test_pass("RST sonrası results_valid=0");

        rst = 1'b0;

        // --------------------------------------------------
        // TC1: Sabit Değer (512 - DC sinyal)
        //   Beklenen: mean = 512, variance ≈ 0
        // --------------------------------------------------
        $display("\n[TC1] Sabit değer testi (sample=512)...");
        do_reset();

        // Örnek toplama başladığında collecting=1 olmalı
        @(posedge clk);
        sample_in    = 12'd512;
        sample_valid = 1'b1;
        @(posedge clk);
        sample_valid = 1'b0;
        repeat(3) @(posedge clk);

        if (collecting !== 1'b1)
            test_fail("Toplama başında collecting=1 olmalı");
        else
            test_pass("collecting=1 (toplama devam ediyor)");

        if (busy !== 1'b1)
            test_fail("Toplama başında busy=1 olmalı");
        else
            test_pass("busy=1");

        // Kalan örnekleri gönder
        for (i = 1; i < N_SAMPLES; i = i + 1) begin
            @(posedge clk);
            sample_in    = 12'd512;
            sample_valid = 1'b1;
            @(posedge clk);
            sample_valid = 1'b0;
            repeat(4) @(posedge clk);
        end

        wait_for_result(
            captured_mean,
            captured_variance,
            captured_skewness,
            captured_kurtosis
        );

        // Mean = 512 = 0x0200
        check_near(captured_mean,     16'h0200, 16'd1,  "TC1 Mean    ");
        // Variance ≈ 0 (sabit sinyal)
        check_near(captured_variance, 16'h0000, 16'd1,  "TC1 Variance");

        // --------------------------------------------------
        // TC2: Kare Dalga (512 / 1536 - eşit sayıda)
        //   Beklenen: mean = (512+1536)/2 = 1024 = 0x0400
        //   Variance = ((512-1024)^2 + (1536-1024)^2) / 2 = 512^2 = 262144 → 16-bit:
        //   acc_dev2 = 1024 * 512^2 = 268435456 → >> 10 = 262144 → 0x40000
        //   Çıkış [25:10] = 262144 >> 10 = 256 = 0x0100
        // --------------------------------------------------
        $display("\n[TC2] Kare dalga testi (512/1536)...");
        do_reset();

        for (i = 0; i < N_SAMPLES; i = i + 1) begin
            @(posedge clk);
            sample_in    = (i < N_SAMPLES/2) ? 12'd512 : 12'd1536;
            sample_valid = 1'b1;
            @(posedge clk);
            sample_valid = 1'b0;
            repeat(4) @(posedge clk);
        end

        wait_for_result(
            captured_mean,
            captured_variance,
            captured_skewness,
            captured_kurtosis
        );

        // Mean = 1024 = 0x0400
        check_near(captured_mean,     16'h0400, 16'd2,   "TC2 Mean    ");
        // Variance beklenen: 262144 → 16-bit çıkışta 0x0100 (256)
        check_near(captured_variance, 16'h0100, 16'd4,   "TC2 Variance");
        // Skewness: simetrik dağılım → ≈ 0
        check_near(captured_skewness, 16'h0000, 16'd10,  "TC2 Skewness");

        // --------------------------------------------------
        // TC3: Rampa Sinyali (0 → 1023)
        //   Beklenen: mean = (0+1023)/2 ≈ 511 = 0x01FF
        // --------------------------------------------------
        $display("\n[TC3] Rampa sinyali testi (0 → 1023)...");
        do_reset();

        for (i = 0; i < N_SAMPLES; i = i + 1) begin
            @(posedge clk);
            sample_in    = i[11:0];
            sample_valid = 1'b1;
            @(posedge clk);
            sample_valid = 1'b0;
            repeat(4) @(posedge clk);
        end

        wait_for_result(
            captured_mean,
            captured_variance,
            captured_skewness,
            captured_kurtosis
        );

        // Rampa mean: (0+1023)/2 = 511 = 0x01FF
        // N=1024, sum = 1023*1024/2 = 523776, mean = 523776 >> 10 = 511
        check_near(captured_mean, 16'h01FF, 16'd2, "TC3 Mean    ");
        $display("  [INFO] Variance = 0x%04X (%0d)", captured_variance, captured_variance);
        $display("  [INFO] Skewness = 0x%04X (%0d)", captured_skewness, captured_skewness);
        $display("  [INFO] Kurtosis = 0x%04X (%0d)", captured_kurtosis, captured_kurtosis);

        // --------------------------------------------------
        // TC5: Art Arda Pencere (Otomatik Yeniden Başlatma)
        //   İlk pencere: 512, ikinci pencere: 1000
        // --------------------------------------------------
        $display("\n[TC5] Art arda pencere testi...");
        do_reset();

        // Birinci pencere: sabit 512
        for (i = 0; i < N_SAMPLES; i = i + 1) begin
            @(posedge clk);
            sample_in    = 12'd512;
            sample_valid = 1'b1;
            @(posedge clk);
            sample_valid = 1'b0;
            repeat(4) @(posedge clk);
        end
        wait_for_result(captured_mean, captured_variance, captured_skewness, captured_kurtosis);
        check_near(captured_mean, 16'h0200, 16'd1, "TC5a Mean(512) ");

        // İkinci pencere: sabit 1000
        for (i = 0; i < N_SAMPLES; i = i + 1) begin
            @(posedge clk);
            sample_in    = 12'd1000;
            sample_valid = 1'b1;
            @(posedge clk);
            sample_valid = 1'b0;
            repeat(4) @(posedge clk);
        end
        wait_for_result(captured_mean, captured_variance, captured_skewness, captured_kurtosis);
        check_near(captured_mean, 16'h03E8, 16'd1, "TC5b Mean(1000)");

        // --------------------------------------------------
        // Özet
        // --------------------------------------------------
        $display("\n==============================================");
        if (errors == 0)
            $display("  TÜM TESTLER BAŞARILI!");
        else
            $display("  %0d TEST BAŞARISIZ!", errors);
        $display("==============================================");

        $finish;
    end

    // =========================================================
    // Durum izleme (simülasyon süresince)
    // =========================================================
    initial begin
        $monitor("[t=%0t] state=%b busy=%b collecting=%b valid=%b | mean=%04X var=%04X",
                 $time, uut.state, busy, collecting, results_valid,
                 mean_out, variance_out);
    end

    // =========================================================
    // VCD Dalga Dosyası
    // =========================================================
    initial begin
        $dumpfile("tb_moment_engine.vcd");
        $dumpvars(0, tb_moment_engine);
    end

    // =========================================================
    // Güvenlik timeout (sonsuz döngü koruması)
    // =========================================================
    initial begin
        #100_000_000; // 100 ms
        $display("[TIMEOUT] Simülasyon zaman aşımına uğradı!");
        $finish;
    end

endmodule
