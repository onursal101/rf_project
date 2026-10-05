// ============================================================================
// ADC SPI Master - Pmod AD1 (ADC121S101) Sürücüsü
// ============================================================================
// SPI Mode: CPOL=1, CPHA=1 (ADC121S101 datasheet)
//
// Zamanlama:
//   sys_clk = 100 MHz
//   SCLK    = 100/8 = 12.5 MHz (16 SCLK = 1.28 us frame)
//   Frame   = 16 SCLK çevrimi (3 leading + 12 veri + 1 trailing)
//   Sample period = 100 saat (1 us = 1 MSPS hedefi)
//
// Çıkış formatı:
//   sample_out[11:0] = 12-bit ADC değeri (0..4095, 0V..3.3V)
//   sample_valid     = 1 saat HIGH pulse
// ============================================================================

module adc_spi_master(
    input  wire        clk,         // 100 MHz
    input  wire        rst,         // Senkron reset

    // Pmod AD1 SPI
    output reg         adc_cs_n,    // ~CS aktif LOW
    output reg         adc_sclk,    // SCLK
    input  wire        adc_miso_a,  // D0 (kanal A)

    // Çıkış
    output reg  [11:0] sample_out,
    output reg         sample_valid
);

    // =========================================================
    // SCLK divider: 100MHz / 8 = 12.5 MHz
    // Her 4 sys_clk = SCLK yarı periyodu (4 LOW, 4 HIGH)
    // =========================================================
    reg [2:0] clk_div;
    wire      sclk_edge = (clk_div == 3'd3);

    // =========================================================
    // FSM
    // =========================================================
    localparam S_IDLE   = 2'd0;
    localparam S_ACTIVE = 2'd1;
    localparam S_WAIT   = 2'd2;

    reg [1:0]  state;
    reg [4:0]  bit_count;    // 0..15 (16 SCLK)
    reg [11:0] shift_reg;
    reg [6:0]  wait_count;   // 1 MSPS için bekleme

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            adc_cs_n     <= 1'b1;
            adc_sclk     <= 1'b1;       // CPOL=1
            clk_div      <= 3'd0;
            bit_count    <= 5'd0;
            shift_reg    <= 12'd0;
            wait_count   <= 7'd0;
            sample_out   <= 12'd0;
            sample_valid <= 1'b0;
        end
        else begin
            sample_valid <= 1'b0;

            // SCLK divider
            if (clk_div == 3'd3)
                clk_div <= 3'd0;
            else
                clk_div <= clk_div + 1'b1;

            case (state)
                S_IDLE: begin
                    adc_cs_n  <= 1'b0;       // ~CS LOW = frame başlat
                    adc_sclk  <= 1'b1;
                    bit_count <= 5'd0;
                    clk_div   <= 3'd0;
                    state     <= S_ACTIVE;
                end

                S_ACTIVE: begin
                    if (sclk_edge) begin
                        adc_sclk <= ~adc_sclk;
                        // Falling edge'de MISO örnekle (CPHA=1)
                        if (adc_sclk == 1'b1) begin
                            // bit 3..14 = 12-bit veri (MSB first)
                            if (bit_count >= 5'd3 && bit_count <= 5'd14) begin
                                shift_reg <= {shift_reg[10:0], adc_miso_a};
                            end
                            bit_count <= bit_count + 1'b1;

                            // 16. çevrim tamamlandı
                            if (bit_count == 5'd15) begin
                                sample_out   <= {shift_reg[10:0], adc_miso_a};
                                sample_valid <= 1'b1;
                                adc_cs_n     <= 1'b1;
                                adc_sclk     <= 1'b1;
                                state        <= S_WAIT;
                                wait_count   <= 7'd0;
                            end
                        end
                    end
                end

                S_WAIT: begin
                    // Frame ~128 saat sürdü, ~100 saat hedef için zaten yeterli
                    // Birkaç saat ekstra bekle (~10 saat)
                    if (wait_count == 7'd9)
                        state <= S_IDLE;
                    else
                        wait_count <= wait_count + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule