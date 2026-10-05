// ============================================================================
// Testbench: tb_spi_slave.v
// Test edilen modül: spi_slave.v
// SPI Mode 0 (CPOL=0, CPHA=0), MSB First, 29-byte transfer
// ============================================================================
// Test Senaryoları:
//   TC1: CMD_READ_FEATURES (0x01) - Tüm 14 feature okunuyor
//   TC2: CMD_READ_STATUS   (0x02) - Status byte okunuyor
//   TC3: SS yüksek iken MISO idle (1) kontrolü
//   TC4: Birden fazla art arda transfer
// ============================================================================

`timescale 1ns / 1ps

module tb_spi_slave;

    // =========================================================
    // DUT Portları
    // =========================================================
    reg         SCLK;
    reg         MOSI;
    wire        MISO;
    reg         SS;
    reg  [223:0] features;
    reg  [7:0]  status_byte;
    wire [7:0]  cmd_reg;
    wire        cmd_valid;
    wire [4:0]  byte_count;

    // =========================================================
    // DUT Örneği
    // =========================================================
    spi_slave uut (
        .SCLK       (SCLK),
        .MOSI       (MOSI),
        .MISO       (MISO),
        .SS         (SS),
        .features   (features),
        .status_byte(status_byte),
        .cmd_reg    (cmd_reg),
        .cmd_valid  (cmd_valid),
        .byte_count (byte_count)
    );

    // =========================================================
    // SPI Parametreleri
    // =========================================================
    parameter SPI_CLK_PERIOD = 100; // 100 ns → 10 MHz SPI clock
    parameter SPI_HALF       = SPI_CLK_PERIOD / 2;

    // Alınan veriler
    reg [7:0]  rx_bytes [0:28]; // Master'ın MISO'dan okuduğu 29 byte
    integer i, j;
    integer errors;

    // =========================================================
    // Başlangıç Değerleri
    // =========================================================
    initial begin
        SCLK        = 1'b0;  // CPOL=0 → idle low
        MOSI        = 1'b0;
        SS          = 1'b1;  // aktif LOW → başlangıçta deaktif
        errors      = 0;

        // Bilinen test vektörü:
        // Feature 0: Mean     = 0x1234
        // Feature 1: Variance = 0x5678
        // Feature 2: Skewness = 0x9ABC
        // Feature 3: Kurtosis = 0xDEF0
        // Feature 4-13: placeholder 0x1111 ... 0xAAAA
        features = {
            16'hAAAA, // F13
            16'h9999, // F12
            16'h8888, // F11
            16'h7777, // F10
            16'h6666, // F9
            16'h5555, // F8
            16'h4444, // F7
            16'h3333, // F6
            16'h2222, // F5
            16'h1111, // F4
            16'hDEF0, // F3 Kurtosis
            16'h9ABC, // F2 Skewness
            16'h5678, // F1 Variance
            16'h1234  // F0 Mean
        };
        status_byte = 8'h03; // features_ready=1, moment_busy=1

        // Başlangıç sinyallerini yerleştir
        #50;

        $display("==============================================");
        $display("  SPI Slave Testbench Başlıyor");
        $display("==============================================");

        // --------------------------------------------------
        // TC3: SS = HIGH iken MISO = 1 (idle) kontrolü
        // --------------------------------------------------
        $display("\n[TC3] SS HIGH iken MISO idle kontrolü...");
        #20;
        check_miso_idle();

        // --------------------------------------------------
        // TC1: CMD_READ_FEATURES (0x01)
        // --------------------------------------------------
        $display("\n[TC1] CMD_READ_FEATURES (0x01) - 29 byte transfer...");
        spi_transfer_29(8'h01);

        // Status byte kontrolü
        $display("  [STATUS ] RX[0] = 0x%02X (beklenen: 0x%02X)", rx_bytes[0], status_byte);
        if (rx_bytes[0] !== status_byte)
            test_fail("STATUS BYTE HATASI");
        else
            test_pass("Status byte doğru");

        // Feature 0: Mean = 0x1234 → byte[1]=0x12, byte[2]=0x34
        check_feature(0, 16'h1234, 1, 2);
        check_feature(1, 16'h5678, 3, 4);
        check_feature(2, 16'h9ABC, 5, 6);
        check_feature(3, 16'hDEF0, 7, 8);
        check_feature(4, 16'h1111, 9, 10);
        check_feature(5, 16'h2222, 11, 12);
        check_feature(6, 16'h3333, 13, 14);
        check_feature(7, 16'h4444, 15, 16);
        check_feature(8, 16'h5555, 17, 18);
        check_feature(9, 16'h6666, 19, 20);
        check_feature(10, 16'h7777, 21, 22);
        check_feature(11, 16'h8888, 23, 24);
        check_feature(12, 16'h9999, 25, 26);
        check_feature(13, 16'hAAAA, 27, 28);

        // --------------------------------------------------
        // TC2: CMD_READ_STATUS (0x02)
        // --------------------------------------------------
        $display("\n[TC2] CMD_READ_STATUS (0x02)...");
        #200;
        spi_transfer_29(8'h02);
        $display("  [STATUS ] RX[0] = 0x%02X (beklenen: 0x%02X)", rx_bytes[0], status_byte);
        if (rx_bytes[0] !== status_byte)
            test_fail("STATUS BYTE (TC2) HATASI");
        else
            test_pass("Status byte (TC2) doğru");

        // --------------------------------------------------
        // TC4: Art arda iki transfer
        // --------------------------------------------------
        $display("\n[TC4] Art arda transfer testi...");
        #500;
        spi_transfer_29(8'h01);
        #200;
        spi_transfer_29(8'h01);
        check_feature(0, 16'h1234, 1, 2);
        test_pass("Art arda iki transfer başarılı");

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
    // Görev: 29 Byte SPI Transfer
    //   master_cmd: İlk byte'da gönderilecek komut
    //   Slave'in MISO'dan gönderdiği 29 byte rx_bytes[] içine kaydedilir
    // =========================================================
    task spi_transfer_29;
        input [7:0] master_cmd;
        reg [7:0] tx_byte;
        integer b, bit_idx;
        begin
            // SS → LOW (seçim başlat)
            SS = 1'b0;
            #(SPI_HALF);

            for (b = 0; b < 29; b = b + 1) begin
                // İlk byte'da komut, geri kalanında 0x00 gönder
                if (b == 0)
                    tx_byte = master_cmd;
                else
                    tx_byte = 8'h00;

                rx_bytes[b] = 8'h00;

                for (bit_idx = 7; bit_idx >= 0; bit_idx = bit_idx - 1) begin
                    // MOSI: Rising edge öncesinde kur (CPHA=0)
                    MOSI = tx_byte[bit_idx];
                    #(SPI_HALF);

                    // Rising edge → slave MOSI'yi örnekler, biz MISO'yu okuruz
                    SCLK = 1'b1;
                    rx_bytes[b][bit_idx] = MISO;
                    #(SPI_HALF);

                    // Falling edge
                    SCLK = 1'b0;
                end
            end

            // SS → HIGH (transfer bitti)
            MOSI = 1'b0;
            SS   = 1'b1;
            #(SPI_HALF * 4);
        end
    endtask

    // =========================================================
    // Görev: MISO idle (=1) kontrolü
    // =========================================================
    task check_miso_idle;
        begin
            if (MISO !== 1'b1)
                test_fail("SS=HIGH iken MISO idle değil!");
            else
                test_pass("SS=HIGH, MISO=1 (idle OK)");
        end
    endtask

    // =========================================================
    // Görev: Feature kontrolü
    // =========================================================
    task check_feature;
        input integer feat_no;
        input [15:0] expected_val;
        input integer hi_idx;   // rx_bytes yüksek byte indeksi
        input integer lo_idx;   // rx_bytes düşük byte indeksi
        reg [15:0] received;
        begin
            received = {rx_bytes[hi_idx], rx_bytes[lo_idx]};
            $display("  [F%-2d   ] RX = 0x%04X  (beklenen: 0x%04X)", feat_no, received, expected_val);
            if (received !== expected_val)
                test_fail("Feature değeri hatalı!");
        end
    endtask

    // =========================================================
    // Görev: Test geçti / başarısız mesajları
    // =========================================================
    task test_pass;
        input [127:0] msg;
        begin
            $display("  [PASS] %s", msg);
        end
    endtask

    task test_fail;
        input [127:0] msg;
        begin
            $display("  [FAIL] %s", msg);
            errors = errors + 1;
        end
    endtask

    // =========================================================
    // VCD Dalga Dosyası
    // =========================================================
    initial begin
        $dumpfile("tb_spi_slave.vcd");
        $dumpvars(0, tb_spi_slave);
    end

endmodule
