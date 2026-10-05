// ============================================================================
// SPI Slave - Multi-Byte Protocol (Mode 0: CPOL=0, CPHA=0, MSB First)
// ============================================================================
// SPI Transfer Formatı:
//   Master TX: [CMD] [0x00] [0x00] ... [0x00]    (29 byte)
//   Slave  TX: [STATUS] [F0_H] [F0_L] ... [F13_H] [F13_L] (29 byte)
//
// Komutlar:
//   0x01 = CMD_READ_FEATURES  (14 öznitelik oku)
//   0x02 = CMD_READ_STATUS    (durum bilgisi oku)
//
// Feature Vektörü (14 x 16-bit = 28 byte):
//   Feature 0:  Mean              (İstatistiksel Moment)
//   Feature 1:  Variance          (İstatistiksel Moment)
//   Feature 2:  Skewness          (İstatistiksel Moment)
//   Feature 3:  Kurtosis          (İstatistiksel Moment)
//   Feature 4:  Peak Frequency    (FFT)
//   Feature 5:  Peak Magnitude    (FFT)
//   Feature 6:  Bandwidth         (FFT)
//   Feature 7:  Spectral Centroid (FFT)
//   Feature 8:  Spectral Flatness (FFT)
//   Feature 9:  Spectral Energy   (FFT)
//   Feature 10: 2nd Harmonic      (FFT)
//   Feature 11: 3rd Harmonic      (FFT)
//   Feature 12: SNR               (FFT)
//   Feature 13: Duty Cycle        (FFT)
// ============================================================================

module spi_slave(
    input  wire        SCLK,            // SPI Clock
    input  wire        MOSI,            // Master Out Slave In
    output wire        MISO,            // Master In Slave Out
    input  wire        SS,              // Slave Select (aktif LOW)
    
    // Feature vektörü girişi (14 x 16-bit = 224 bit)
    // Paketleme: {feature_13, feature_12, ..., feature_1, feature_0}
    input  wire [223:0] features,
    
    // Status byte
    input  wire [7:0]  status_byte,
    
    // Master'dan gelen komut
    output reg  [7:0]  cmd_reg,
    output reg         cmd_valid,
    
    // Transfer bilgisi
    output reg  [4:0]  byte_count       // Mevcut byte pozisyonu
);

    reg [2:0] bit_count;                // Byte içi bit sayacı (0-7)
    reg [7:0] rx_shift;                 // RX shift register

    // =========================================================
    // TX Byte Seçici (kombinasyonel)
    // byte_count'a göre gönderilecek byte'ı seçer
    // =========================================================
    reg [7:0] tx_byte_sel;
    always @(*) begin
        case (byte_count)
            5'd0:  tx_byte_sel = status_byte;
            // Feature 0: Mean
            5'd1:  tx_byte_sel = features[15:8];
            5'd2:  tx_byte_sel = features[7:0];
            // Feature 1: Variance
            5'd3:  tx_byte_sel = features[31:24];
            5'd4:  tx_byte_sel = features[23:16];
            // Feature 2: Skewness
            5'd5:  tx_byte_sel = features[47:40];
            5'd6:  tx_byte_sel = features[39:32];
            // Feature 3: Kurtosis
            5'd7:  tx_byte_sel = features[63:56];
            5'd8:  tx_byte_sel = features[55:48];
            // Feature 4: Peak Frequency
            5'd9:  tx_byte_sel = features[79:72];
            5'd10: tx_byte_sel = features[71:64];
            // Feature 5: Peak Magnitude
            5'd11: tx_byte_sel = features[95:88];
            5'd12: tx_byte_sel = features[87:80];
            // Feature 6: Bandwidth
            5'd13: tx_byte_sel = features[111:104];
            5'd14: tx_byte_sel = features[103:96];
            // Feature 7: Spectral Centroid
            5'd15: tx_byte_sel = features[127:120];
            5'd16: tx_byte_sel = features[119:112];
            // Feature 8: Spectral Flatness
            5'd17: tx_byte_sel = features[143:136];
            5'd18: tx_byte_sel = features[135:128];
            // Feature 9: Spectral Energy
            5'd19: tx_byte_sel = features[159:152];
            5'd20: tx_byte_sel = features[151:144];
            // Feature 10: 2nd Harmonic
            5'd21: tx_byte_sel = features[175:168];
            5'd22: tx_byte_sel = features[167:160];
            // Feature 11: 3rd Harmonic
            5'd23: tx_byte_sel = features[191:184];
            5'd24: tx_byte_sel = features[183:176];
            // Feature 12: SNR
            5'd25: tx_byte_sel = features[207:200];
            5'd26: tx_byte_sel = features[199:192];
            // Feature 13: Duty Cycle
            5'd27: tx_byte_sel = features[223:216];
            5'd28: tx_byte_sel = features[215:208];
            default: tx_byte_sel = 8'h00;
        endcase
    end

    // =========================================================
    // MISO: Kombinasyonel çıkış (CPHA=0)
    // SS HIGH → idle (1), SS LOW → seçilen byte'ın ilgili biti
    // bit_count: 0..7 => MSB..LSB
    // =========================================================
    assign MISO = SS ? 1'b1 : tx_byte_sel[7 - bit_count];

    // =========================================================
    // RX: Rising edge'de MOSI'den veri al (CPHA=0)
    // posedge SS = async reset
    // =========================================================
    always @(posedge SCLK or posedge SS) begin
        if (SS) begin
            bit_count  <= 3'd0;
            byte_count <= 5'd0;
            rx_shift   <= 8'd0;
            cmd_reg    <= 8'd0;
            cmd_valid  <= 1'b0;
        end
        else begin
            // MOSI'den bit al
            rx_shift <= {rx_shift[6:0], MOSI};
            
            if (bit_count == 3'd7) begin
                // Tam byte alındı
                if (byte_count == 5'd0) begin
                    // İlk byte = komut
                    cmd_reg   <= {rx_shift[6:0], MOSI};
                    cmd_valid <= 1'b1;
                end else begin
                    cmd_valid <= 1'b0;
                end
                byte_count <= byte_count + 1'b1;
            end
            else begin
                cmd_valid <= 1'b0;
            end
            
            bit_count <= bit_count + 1'b1;
        end
    end

endmodule
