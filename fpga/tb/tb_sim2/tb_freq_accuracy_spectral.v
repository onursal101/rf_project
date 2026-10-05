`timescale 1ns / 1ps

module tb_freq_accuracy_spectral;

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
    integer t;
    integer det_bin;
    integer err_bin;
    integer acc_percent;
    integer avg_acc;
    integer sum_acc;
    integer timeout;

    localparam CLK_PERIOD = 10;
    localparam N_CASES = 9;
    integer target_bins [0:N_CASES-1];
    integer case_mode   [0:N_CASES-1];

    spectral_analyzer dut (
        .clk(clk),
        .rst(rst),
        .fft_re(fft_re),
        .fft_im(fft_im),
        .fft_index(fft_index),
        .fft_out_valid(fft_out_valid),
        .fft_frame_done(fft_frame_done),
        .feat_peak_freq(feat_peak_freq),
        .feat_peak_mag(feat_peak_mag),
        .feat_bandwidth(feat_bandwidth),
        .feat_centroid(feat_centroid),
        .feat_flatness(feat_flatness),
        .feat_energy(feat_energy),
        .feat_harm2(feat_harm2),
        .feat_harm3(feat_harm3),
        .feat_snr(feat_snr),
        .feat_duty(feat_duty),
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
            repeat (6) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
        end
    endtask

    // mode:
    //   0 = nominal
    //   1 = düşük SNR (tepe zayıf, taban yüksek)
    //   2 = güçlü leakage (komşu binler yüksek)
    //   3 = ayna tarafı güçlendirilmiş (tek-taraf peak seçimi testi)
    task drive_frame_with_peak;
        input integer peak_bin;
        input integer mode;
        integer b;
        reg [15:0] amp;
        reg [15:0] peak_amp;
        reg [15:0] nb_amp;
        reg [15:0] mirror_amp;
        reg [15:0] noise_lo;
        reg [15:0] noise_hi;
        begin
            case (mode)
                1: begin
                    peak_amp   = 16'd900;
                    nb_amp     = 16'd400;
                    mirror_amp = 16'd380;
                    noise_lo   = 16'd200;
                    noise_hi   = 16'd140;
                end
                2: begin
                    peak_amp   = 16'd1400;
                    nb_amp     = 16'd1200;
                    mirror_amp = 16'd500;
                    noise_lo   = 16'd120;
                    noise_hi   = 16'd80;
                end
                3: begin
                    peak_amp   = 16'd1600;
                    nb_amp     = 16'd700;
                    mirror_amp = 16'd1500;
                    noise_lo   = 16'd90;
                    noise_hi   = 16'd60;
                end
                default: begin
                    peak_amp   = 16'd2000;
                    nb_amp     = 16'd600;
                    mirror_amp = 16'd500;
                    noise_lo   = 16'd80;
                    noise_hi   = 16'd40;
                end
            endcase

            for (b = 0; b < 1024; b = b + 1) begin
                // Basit leakage modeli: tepe bin + komsu binler
                if (b == peak_bin)
                    amp = peak_amp;
                else if ((b == (peak_bin-1)) || (b == (peak_bin+1)))
                    amp = nb_amp;
                else if ((b == (1024-peak_bin)) || (b == (1024-peak_bin-1)) || (b == (1024-peak_bin+1)))
                    amp = mirror_amp;
                else if (b < 512)
                    amp = noise_lo;
                else
                    amp = noise_hi;

                @(posedge clk);
                fft_index      <= b[10:0];
                fft_re         <= amp;
                fft_im         <= 16'd0;
                fft_out_valid  <= 1'b1;
                fft_frame_done <= (b == 1023);
            end

            @(posedge clk);
            fft_out_valid  <= 1'b0;
            fft_frame_done <= 1'b0;
            fft_re         <= 16'd0;
            fft_im         <= 16'd0;
            fft_index      <= 11'd0;
        end
    endtask

    initial begin
        errors = 0;
        sum_acc = 0;
        target_bins[0] = 16;
        target_bins[1] = 64;
        target_bins[2] = 128;
        target_bins[3] = 220;
        target_bins[4] = 300;
        target_bins[5] = 460;
        target_bins[6] = 96;
        target_bins[7] = 250;
        target_bins[8] = 384;

        case_mode[0] = 0;
        case_mode[1] = 0;
        case_mode[2] = 0;
        case_mode[3] = 0;
        case_mode[4] = 0;
        case_mode[5] = 0;
        case_mode[6] = 1; // düşük SNR
        case_mode[7] = 2; // güçlü leakage
        case_mode[8] = 3; // ayna tarafı baskın test

        do_reset();
        $display("==============================================");
        $display("  Spectral Frequency Accuracy Test");
        $display("==============================================");

        for (t = 0; t < N_CASES; t = t + 1) begin
            drive_frame_with_peak(target_bins[t], case_mode[t]);

            timeout = 0;
            while (!spectral_valid && timeout < 6000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end

            if (timeout >= 6000) begin
                $display("  [FAIL] Case %0d timeout", t);
                errors = errors + 1;
            end else begin
                det_bin = feat_peak_freq;
                err_bin = (det_bin >= target_bins[t]) ? (det_bin - target_bins[t]) : (target_bins[t] - det_bin);
                // Accuracy = 100 * (1 - |err|/target), alt sinir 0
                if (target_bins[t] != 0)
                    acc_percent = 100 - ((err_bin * 100) / target_bins[t]);
                else
                    acc_percent = (err_bin == 0) ? 100 : 0;
                if (acc_percent < 0) acc_percent = 0;

                sum_acc = sum_acc + acc_percent;
                $display("  Case%0d mode=%0d target=%0d detected=%0d err=%0d acc=%0d%%",
                         t, case_mode[t], target_bins[t], det_bin, err_bin, acc_percent);

                if (acc_percent >= 90)
                    $display("    PASS: >=90%%");
                else begin
                    $display("    FAIL: <90%%");
                    errors = errors + 1;
                end
            end
        end

        avg_acc = sum_acc / N_CASES;
        $display("----------------------------------------------");
        $display("  Ortalama Frekans Dogrulugu: %0d%%", avg_acc);
        if (avg_acc >= 90)
            $display("  PASS: Sistem hedefi saglandi (>=90%%)");
        else begin
            $display("  FAIL: Sistem hedefi saglanmadi (<90%%)");
            errors = errors + 1;
        end
        $display("==============================================");
        if (errors == 0)
            $display("  TUM TESTLER BASARILI");
        else
            $display("  %0d HATA", errors);
        $display("==============================================");

        $finish;
    end

    initial begin
        $dumpfile("tb_freq_accuracy_spectral.vcd");
        $dumpvars(0, tb_freq_accuracy_spectral);
    end

endmodule
