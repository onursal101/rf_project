`timescale 1ns / 1ps

module tb_spectral_analyzer;

    reg         clk;
    reg         rst;
    reg  [15:0] fft_re;
    reg  [15:0] fft_im;
    reg  [10:0] fft_index;
    reg         fft_out_valid;
    reg         fft_frame_done;

    wire [15:0] feat_peak_freq;
    wire [15:0] feat_peak_mag;
    wire [15:0] feat_bandwidth;
    wire [15:0] feat_centroid;
    wire [15:0] feat_flatness;
    wire [15:0] feat_energy;
    wire [15:0] feat_harm2;
    wire [15:0] feat_harm3;
    wire [15:0] feat_snr;
    wire [15:0] feat_duty;
    wire        spectral_valid;

    integer errors;
    integer i;
    integer timeout;

    localparam CLK_PERIOD = 10;
    localparam integer PEAK_BIN = 100;
    localparam integer H2_BIN   = 200;
    localparam integer H3_BIN   = 300;

    spectral_analyzer uut (
        .clk           (clk),
        .rst           (rst),
        .fft_re        (fft_re),
        .fft_im        (fft_im),
        .fft_index     (fft_index),
        .fft_out_valid (fft_out_valid),
        .fft_frame_done(fft_frame_done),
        .feat_peak_freq(feat_peak_freq),
        .feat_peak_mag (feat_peak_mag),
        .feat_bandwidth(feat_bandwidth),
        .feat_centroid (feat_centroid),
        .feat_flatness (feat_flatness),
        .feat_energy   (feat_energy),
        .feat_harm2    (feat_harm2),
        .feat_harm3    (feat_harm3),
        .feat_snr      (feat_snr),
        .feat_duty     (feat_duty),
        .spectral_valid(spectral_valid)
    );

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    task do_reset;
        begin
            rst = 1'b1;
            fft_re = 16'd0;
            fft_im = 16'd0;
            fft_index = 11'd0;
            fft_out_valid = 1'b0;
            fft_frame_done = 1'b0;
            repeat (5) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
        end
    endtask

    task drive_bin;
        input [10:0] idx;
        input [15:0] re_val;
        input        last_bin;
        begin
            @(posedge clk);
            fft_index     <= idx;
            fft_re        <= re_val;
            fft_im        <= 16'd0;
            fft_out_valid <= 1'b1;
            fft_frame_done<= last_bin;
        end
    endtask

    task pass;
        input [160*8:1] msg;
        begin
            $display("  [PASS] %0s", msg);
        end
    endtask

    task fail;
        input [160*8:1] msg;
        begin
            $display("  [FAIL] %0s", msg);
            errors = errors + 1;
        end
    endtask

    initial begin
        errors = 0;
        do_reset();

        $display("==============================================");
        $display("  Spectral Analyzer Testbench Basladi");
        $display("==============================================");

        // 1024 binlik tek frame gonder:
        // - peak bin 100 (re=1000 -> mag_sq=1,000,000 -> high16=15)
        // - 2. harmonik bin 200 (re=700 -> high16=7)
        // - 3. harmonik bin 300 (re=600 -> high16=5)
        // - diger tek tarafli binler (0..511) re=100
        // - ikinci yarisı (512..1023) re=0
        for (i = 0; i < 1024; i = i + 1) begin
            if (i == PEAK_BIN)
                drive_bin(i[10:0], 16'd1000, (i == 1023));
            else if (i == H2_BIN)
                drive_bin(i[10:0], 16'd700, (i == 1023));
            else if (i == H3_BIN)
                drive_bin(i[10:0], 16'd600, (i == 1023));
            else if (i < 512)
                drive_bin(i[10:0], 16'd100, (i == 1023));
            else
                drive_bin(i[10:0], 16'd0, (i == 1023));
        end

        // Girdiyi kes
        @(posedge clk);
        fft_out_valid  <= 1'b0;
        fft_frame_done <= 1'b0;
        fft_index      <= 11'd0;
        fft_re         <= 16'd0;
        fft_im         <= 16'd0;

        // spectral_valid bekle
        timeout = 0;
        while (!spectral_valid && timeout < 5000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        if (timeout >= 5000)
            fail("spectral_valid timeout");
        else
            pass("spectral_valid pulse geldi");

        // Beklenenler
        if (feat_peak_freq == 16'd100) pass("peak_freq dogru");
        else fail("peak_freq hatali");

        if (feat_bandwidth == 16'd1) pass("bandwidth dogru (yalniz peak > max/2)");
        else fail("bandwidth hatali");

        if (feat_harm2 == 16'd7) pass("harm2 dogru");
        else fail("harm2 hatali");

        if (feat_harm3 == 16'd5) pass("harm3 dogru");
        else fail("harm3 hatali");

        if (feat_peak_mag == 16'd15) pass("peak_mag dogru");
        else fail("peak_mag hatali");

        $display("INFO: peak_freq=%0d peak_mag=%0d bw=%0d h2=%0d h3=%0d centroid=%0d energy=%0d duty=%0d",
                 feat_peak_freq, feat_peak_mag, feat_bandwidth, feat_harm2, feat_harm3,
                 feat_centroid, feat_energy, feat_duty);

        $display("==============================================");
        if (errors == 0)
            $display("  TUM TESTLER BASARILI");
        else
            $display("  %0d TEST BASARISIZ", errors);
        $display("==============================================");

        $finish;
    end

    initial begin
        $dumpfile("tb_spectral_analyzer.vcd");
        $dumpvars(0, tb_spectral_analyzer);
    end

    initial begin
        #100_000_000;
        $display("[TIMEOUT] Simulasyon zaman asimina ugradi");
        $finish;
    end

endmodule
