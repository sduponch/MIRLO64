// Bench of the JVS host (rtl/analogizer/an_jvs.sv) on an RS485 bus with one board (jvs_dev.sv): the frames on the wire byte for byte
// (sync, length, escapes, sum), DIR's lead and tail, the answers in the buffer, the init sequence, polling by itself, and the
// boards that answer late, not at all, broken, too long, with noise, or at another bit time. Prints ALL PASS or SOME FAILED.
// Questa (run_tests.sh) or Verilator --binary.
`timescale 1ns/1ps
`default_nettype none
module tb_jvs;
localparam int DIV = 543;
reg          clk = 0, rst = 1;
always #8 clk = ~clk;                           // 62.5 MHz

reg          sel_wr = 0, sel_rd = 0, own = 1;
reg  [3:0]   a = 0;
reg  [31:0]  wd = 0;
wire [31:0]  rd;
wire         en, tx, dir, rx, sense;
an_jvs host (.clk, .rst, .sel_wr, .sel_rd, .a, .wd, .rd, .own, .en, .tx, .dir, .rx, .sense);

// the bus: the host's driver while DIR, else the board's, else the fail-safe bias (idle, 1); the host's receiver is off while DIR
wire         de, dtx;
wire         bus = dir ? tx : de ? dtx : 1'b1;
assign rx = dir ? 1'b1 : bus;
reg  [15:0]  d_div = DIV, d_delay = 40;
reg  [7:0]   d_mode = 0;
reg  [63:0]  d_sw = 64'h0;
reg  [127:0] d_an = 128'h0;
wire [31:0]  d_frames, d_bad, d_ans;
wire [7:0]   d_addr, d_nout;
wire [7:0]   d_out [0:7];
jvs_dev dev (.clk, .rst, .bus, .de, .txd(dtx), .sense, .div(d_div), .delay_us(d_delay), .mode(d_mode), .sw(d_sw), .an(d_an),
             .n_frames(d_frames), .n_bad(d_bad), .n_ans(d_ans), .addr(d_addr), .last_out(d_out), .n_out(d_nout));

integer      fails = 0, checks = 0;
task automatic check(input bit c, input string what);
    checks = checks + 1;
    if (!c) begin fails = fails + 1; $display("FAIL: %s", what); end
endtask

// ---------------------------------------------------------------- the wire, as the host drives it: bytes, DIR's lead and tail
reg  [7:0]   wbuf [0:511];
integer      wn = 0, w_act = 0, w_cnt = 0, w_nb = 0, n_dir = 0, conflicts = 0, bad_stop = 0;
reg  [7:0]   w_sh;
time         t_rise, t_first, t_stop_end, t_last_rise = 0, lead_min = 1e9, lead_max = 0, tail_min = 1e9, tail_max = 0, per_last = 0;
reg          dir_d = 0, w_firstseen = 0;
always @(posedge clk) begin
    dir_d <= dir;
    if (dir && de) conflicts = conflicts + 1;
    if (dir && !dir_d) begin
        wn = 0; w_act = 0; w_firstseen = 0; n_dir = n_dir + 1; t_rise = $time;
        if (t_last_rise != 0) per_last = t_rise - t_last_rise;
        t_last_rise = t_rise;
    end
    if (dir) begin
        if (w_act) begin
            w_cnt = w_cnt - 1;
            if (w_cnt == 0) begin
                w_cnt = DIV; w_nb = w_nb + 1;
                if (w_nb <= 8) w_sh = {tx, w_sh[7:1]};
                else begin
                    w_act = 0; if (!tx) bad_stop = bad_stop + 1;
                    wbuf[wn] = w_sh; wn = wn + 1; t_stop_end = $time + DIV / 2 * 16;
                end
            end
        end else if (!tx) begin
            w_act = 1; w_cnt = DIV / 2 + DIV; w_nb = 0;
            if (!w_firstseen) begin w_firstseen = 1; t_first = $time; end
        end
    end
    if (!dir && dir_d && w_firstseen) begin
        if (t_first - t_rise < lead_min) lead_min = t_first - t_rise;
        if (t_first - t_rise > lead_max) lead_max = t_first - t_rise;
        if ($time - t_stop_end < tail_min) tail_min = $time - t_stop_end;
        if ($time - t_stop_end > tail_max) tail_max = $time - t_stop_end;
    end
end

// ---------------------------------------------------------------- the registers
localparam int R_ID = 0, R_CTRL = 1, R_STAT = 2, R_XFER = 3, R_PTR = 4, R_DATA = 5;
task automatic wr(input [3:0] ad, input [31:0] v);
    @(negedge clk); a = ad; wd = v; sel_wr = 1;
    @(negedge clk); sel_wr = 0;
endtask
task automatic rdr(input [3:0] ad, output [31:0] v);
    @(negedge clk); a = ad; #1 v = rd; if (ad == 4'(R_DATA)) sel_rd = 1;
    @(negedge clk); sel_rd = 0;
endtask

reg  [7:0]   fr [0:127];                        // the frame to send: node, data
reg  [7:0]   ans [0:127];                       // the last good answer, read back
reg  [7:0]   ex [0:511];                        // the frame expected on the wire
integer      exn;
reg  [31:0]  xs;                                // XFER after the transaction
time         x_time;

task automatic load(input integer n);
    wr(R_PTR, 0);
    for (int i = 0; i < n; i++) wr(R_DATA, {24'd0, fr[i]});
endtask
task automatic read_ans(input integer n);
    reg [31:0] v;
    wr(R_PTR, 128);
    for (int i = 0; i < n; i++) begin rdr(R_DATA, v); ans[i] = v[7:0]; end
endtask
task automatic xfer(input integer n);
    reg [31:0] v; time t0;
    load(n);
    t0 = $time;
    wr(R_XFER, n);
    v = 32'h8000_0000;
    for (int i = 0; i < 2000000 && v[31]; i++) rdr(R_XFER, v);
    xs = v; x_time = $time - t0;
    if (!v[31]) read_ans(v[7:0]);
endtask
// the frame fr[0 .. n-1] as it should be on the wire
task automatic expect_wire(input integer n);
    reg [7:0] s, b;
    s = 0; exn = 1;
    ex[0] = 8'hE0;
    for (int i = 0; i <= n + 1; i++) begin
        b = i == 0 ? fr[0] : i == 1 ? 8'(n) : i == n + 1 ? s : fr[i - 1];
        s = s + b;
        if (b == 8'hE0 || b == 8'hD0) begin ex[exn] = 8'hD0; ex[exn + 1] = b - 8'd1; exn = exn + 2; end
        else begin ex[exn] = b; exn = exn + 1; end
    end
endtask
function automatic bit wire_is_expected;
    if (wn != exn) return 0;
    for (int i = 0; i < exn; i++) if (wbuf[i] != ex[i]) return 0;
    return 1;
endfunction
function automatic bit ans_sum_ok(input integer n);
    reg [7:0] s; s = 0;
    for (int i = 0; i < n - 1; i++) s = s + ans[i];
    return s == ans[n - 1] && ans[1] == 8'(n - 2);
endfunction
task automatic set_fr(input integer n, input [8*16-1:0] b);     // fr[0 .. n-1] from b's top bytes, first first
    for (int i = 0; i < n; i++) fr[i] = b[8 * (15 - i) +: 8];
endtask

localparam string ID = "MIRLO64;JVS TEST BOARD;837-00000 ;Ver9.99;26/11";
reg  [31:0]  v, st0, st1;
integer      g0, e0, ok;
time         t0;
reg  [7:0]   keep [0:127];

initial begin
    repeat (10) @(negedge clk); rst = 0; repeat (10) @(negedge clk);

    // ---- registers
    rdr(R_ID, v);   check(v == 32'h4A56_0001, "ID");
    rdr(R_CTRL, v); check(v == 32'h0000_1400, $sformatf("CTRL resets to PERIOD 20, off (%08x)", v));
    rdr(R_XFER, v); check(v == 0, "XFER at reset");

    // ---- not on, or the adapter not owning the port: a timeout at once, and DIR never on
    set_fr(3, {8'hFF, 8'hF0, 8'hD9, 104'd0});
    xfer(3);
    check(!xs[31] && xs[30] && !xs[29] && x_time < 2000 && n_dir == 0, $sformatf("off: XFER %08x in %0t ns, DIR %0d times", xs, x_time, n_dir));
    own = 0; wr(R_CTRL, 32'h0000_1401);                         // ON, PERIOD 20 (the init's answers are up to 9 ms long)
    xfer(3);
    check(xs[30] && x_time < 2000 && n_dir == 0, $sformatf("not the adapter's port: XFER %08x, DIR %0d times", xs, n_dir));
    check(en == 0 && tx == 1 && dir == 0, "not the adapter's port: the pins idle");
    own = 1; #1;
    check(en == 1, "on and owned: en");

    // ---- reset twice (broadcast, no answer): the frame on the wire, a timeout after PERIOD, the board unaddressed
    rdr(R_STAT, st0);
    check(st0[0] == 1, "SENSE high before the board is addressed");
    for (int r = 0; r < 2; r++) begin
        xfer(3);
        expect_wire(3);
        check(wire_is_expected(), $sformatf("reset %0d: the wire (%0d bytes, want %0d: E0 FF 03 F0 D9 CB)", r, wn, exn));
        check(xs[30] && !xs[29], $sformatf("reset %0d: TIMEOUT (XFER %08x)", r, xs));
        check(x_time > 20.0e6 && x_time < 20.1e6, $sformatf("reset %0d: BUSY for PERIOD (20 ms) from the XFER: %0t", r, x_time));
    end
    check(d_frames == 2 && d_bad == 0 && d_addr == 0, $sformatf("the board took the resets: %0d frames, %0d bad", d_frames, d_bad));
    rdr(R_STAT, v);
    check(v[23:16] == st0[23:16] + 2 && v[15:8] == st0[15:8], "the timeouts count as errors");

    // ---- set the address
    set_fr(3, {8'hFF, 8'hF1, 8'h01, 104'd0});
    xfer(3);
    check(!xs[30] && !xs[29] && xs[7:0] == 5, $sformatf("SETADDR: XFER %08x", xs));
    check(ans[0] == 8'h00 && ans[1] == 8'h03 && ans[2] == 8'h01 && ans[3] == 8'h01 && ans[4] == 8'h05, "SETADDR: answer 00 03 01 01 05");
    check(d_addr == 1, "the board has address 1");
    check(x_time < 1.5e6, $sformatf("SETADDR: the answer ends the transaction (%0t ns)", x_time));
    repeat (10) @(negedge clk);
    rdr(R_STAT, v);
    check(v[0] == 0 && v[15:8] == st0[15:8] + 1, $sformatf("SENSE low once addressed, GOOD + 1 (STAT %08x)", v));
    // the board's address taken: another SETADDR is not answered
    xfer(3);
    check(xs[30], "a second SETADDR: no answer");
    read_ans(5);
    check(ans[0] == 8'h00 && ans[4] == 8'h05, "a timeout leaves the last good answer");

    // ---- identity: a long answer (52 bytes)
    set_fr(2, {8'h01, 8'h10, 112'd0});
    xfer(2);
    ok = 1;
    for (int i = 0; i < ID.len(); i++) if (ans[4 + i] != ID[i]) ok = 0;
    check(!xs[30] && !xs[29] && xs[7:0] == 6 + ID.len() && ok && ans[4 + ID.len()] == 0 && ans_sum_ok(xs[7:0]),
          $sformatf("IOIDENT: XFER %08x, the string %0s", xs, ok ? "as sent" : "wrong"));
    // ---- the revisions and the features, five commands in one frame
    set_fr(5, {8'h01, 8'h11, 8'h12, 8'h13, 8'h14, 88'd0});
    xfer(5);
    check(!xs[29] && !xs[30] && xs[7:0] == 24 && ans_sum_ok(24) && ans[3] == 1 && ans[4] == 8'h13 && ans[5] == 1 && ans[6] == 8'h30
          && ans[7] == 1 && ans[8] == 8'h10 && ans[9] == 1 && ans[10] == 1 && ans[11] == 2 && ans[12] == 12 && ans[14] == 3,
          $sformatf("CMDREV JVSREV COMVER FEATCHK: XFER %08x", xs));

    // ---- escapes out: data bytes E0 and D0, and a frame whose sum is E0 (01 + 04 + 32 + 01 + A8)
    set_fr(5, {8'h01, 8'h32, 8'h02, 8'hE0, 8'hD0, 88'd0});
    xfer(5);
    expect_wire(5);
    check(wire_is_expected(), $sformatf("escapes in the data: the wire (%0d bytes, want %0d)", wn, exn));
    check(d_bad == 0 && d_nout == 2 && d_out[0] == 8'hE0 && d_out[1] == 8'hD0, "escapes in the data: the board took E0 D0");
    check(!xs[29] && !xs[30], "escapes in the data: answered");
    set_fr(4, {8'h01, 8'h32, 8'h01, 8'hA8, 96'd0});
    xfer(4);
    expect_wire(4);
    check(ex[exn - 2] == 8'hD0 && ex[exn - 1] == 8'hDF && wire_is_expected(), "a sum of E0: sent as D0 DF");
    check(d_bad == 0 && d_out[0] == 8'hA8 && !xs[29] && !xs[30], "a sum of E0: the board took it");

    // ---- DIR so far
    check(lead_min >= 9.9e3 && lead_max <= 10.3e3, $sformatf("DIR's lead %0t .. %0t ns (10 us)", lead_min, lead_max));
    check(tail_min >= 29.5e3 && tail_max <= 30.5e3, $sformatf("DIR's tail %0t .. %0t ns (30 us)", tail_min, tail_max));
    check(conflicts == 0 && bad_stop == 0, $sformatf("the host and the board never drive together (%0d), stop bits (%0d)", conflicts, bad_stop));

    // ---- polling by itself: switches and 8 analog channels (a 3.5 ms transaction), every 5 ms; escapes in the answer (E0, D0, DF, CF)
    d_sw = 64'h0000_0000_C3A5_0F80;                             // system 80, P1 0F A5, P2 C3 00
    d_an = {16'h1234, 16'hFFFF, 16'h0000, 16'hCFDF, 16'hD000, 16'h00E0, 16'hE0D0, 16'h8000};
    set_fr(6, {8'h01, 8'h20, 8'h02, 8'h02, 8'h22, 8'h08, 80'd0});
    load(6);
    wr(R_XFER, 6);
    wr(R_CTRL, 32'h0000_0503);                                  // ON, POLL, PERIOD 5
    rdr(R_STAT, st0);
    t0 = $time;
    #(31ms);
    rdr(R_STAT, st1);
    check(st1[15:8] - st0[15:8] >= 6 && st1[15:8] - st0[15:8] <= 7 && st1[23:16] == st0[23:16],
          $sformatf("polling 31 ms at 5 ms: GOOD + %0d (6 or 7), ERRORS + %0d", st1[15:8] - st0[15:8], st1[23:16] - st0[23:16]));
    check(per_last > 4.999e6 && per_last < 5.001e6, $sformatf("polling: a frame every %0t", per_last));
    rdr(R_XFER, v);
    check(!v[31] && v[7:0] == 3 + 1 + 5 + 1 + 16 + 1, $sformatf("polling: the answer's bytes (XFER %08x)", v));
    read_ans(v[7:0]);
    check(ans_sum_ok(27) && ans[2] == 1 && ans[3] == 1 && ans[4] == 8'h80 && ans[5] == 8'h0F && ans[6] == 8'hA5 && ans[7] == 8'hC3 && ans[8] == 0
          && ans[9] == 1 && ans[10] == 8'h80 && ans[11] == 8'h00 && ans[12] == 8'hE0 && ans[13] == 8'hD0 && ans[14] == 8'h00 && ans[15] == 8'hE0
          && ans[16] == 8'hD0 && ans[18] == 8'hCF && ans[19] == 8'hDF && ans[22] == 8'hFF && ans[23] == 8'hFF && ans[24] == 8'h12 && ans[25] == 8'h34,
          "polling: switches and channels, unescaped");
    // the board's inputs change: the answer follows within two periods
    d_sw[15:8] = 8'h01; d_an[15:0] = 16'h4321;
    #(10.1ms);
    read_ans(27);
    check(ans[5] == 8'h01 && ans[10] == 8'h43 && ans[11] == 8'h21, "polling: new inputs within two periods");

    // ---- broken answers, while polling: never taken, the last good one kept, each an error
    for (int m = 0; m < 5; m++) begin
        reg [7:0] md; string nm;
        md = m == 0 ? 8'h01 : m == 1 ? 8'h02 : m == 2 ? 8'h04 : m == 3 ? 8'h10 : 8'h08;
        nm = m == 0 ? "no answer" : m == 1 ? "a wrong sum" : m == 2 ? "an answer that stops" : m == 3 ? "a bad stop bit" : "noise before the sync";
        d_sw[15:8] = 8'h40 + 8'(m);
        d_mode = md;
        #(5ms);
        rdr(R_STAT, st0);
        #(15ms);
        rdr(R_STAT, st1);
        rdr(R_XFER, v);
        read_ans(27);
        if (m < 4) begin
            check(st1[15:8] == st0[15:8] && st1[23:16] - st0[23:16] == 3, $sformatf("%0s: GOOD + %0d, ERRORS + %0d (3)", nm, st1[15:8] - st0[15:8], st1[23:16] - st0[23:16]));
            check(m == 0 ? v[30] && !v[29] : v[29] && !v[30], $sformatf("%0s: XFER %08x", nm, v));
            check(ans[5] != 8'h40 + 8'(m) && ans_sum_ok(27), $sformatf("%0s: the last good answer kept", nm));
        end else begin
            check(st1[15:8] - st0[15:8] == 3 && st1[23:16] == st0[23:16], $sformatf("%0s: GOOD + %0d (3), ERRORS + %0d", nm, st1[15:8] - st0[15:8], st1[23:16] - st0[23:16]));
            check(ans[5] == 8'h44 && ans_sum_ok(27), "noise before the sync: taken");
        end
    end
    d_mode = 0;
    // other bit times: 2 % slow and fast
    for (int b = 0; b < 2; b++) begin
        d_div = b == 0 ? 16'd532 : 16'd554; d_sw[15:8] = 8'h50 + 8'(b);
        #(5ms);
        rdr(R_STAT, st0);
        #(15ms);
        rdr(R_STAT, st1); read_ans(27);
        check(st1[15:8] - st0[15:8] == 3 && st1[23:16] == st0[23:16] && ans[5] == 8'h50 + 8'(b),
              $sformatf("the board at %0d clocks a bit: GOOD + %0d, ERRORS + %0d", d_div, st1[15:8] - st0[15:8], st1[23:16] - st0[23:16]));
    end
    d_div = DIV;
    // a late board: 1 ms of delay still fits the 5 ms period; 2.5 ms in a 2 ms one does not
    d_delay = 1000; d_sw[15:8] = 8'h60;
    #(5ms); rdr(R_STAT, st0); #(15ms); rdr(R_STAT, st1); read_ans(27);
    check(st1[15:8] - st0[15:8] == 3 && st1[23:16] == st0[23:16] && ans[5] == 8'h60, $sformatf("1 ms late: GOOD + %0d", st1[15:8] - st0[15:8]));
    wr(R_CTRL, 32'h0000_0401);                                  // POLL off (PERIOD 4)
    #(5ms);
    d_delay = 2500;
    set_fr(2, {8'h01, 8'h10, 112'd0});
    wr(R_CTRL, 32'h0000_0201);                                  // PERIOD 2
    xfer(2);
    check(xs[30], $sformatf("2.5 ms late in a 2 ms period: TIMEOUT (XFER %08x)", xs));
    d_delay = 40;
    #(8ms);                                                     // (the late answer goes by)

    // ---- an answer longer than the buffer's half: BAD, the last good one kept
    d_mode = 8'h20;
    wr(R_CTRL, 32'h0000_1401);                                  // PERIOD 20
    xfer(2);
    check(xs[29] && !xs[30], $sformatf("a 150-byte identity: BAD (XFER %08x)", xs));
    d_mode = 0;
    #(15ms);
    xfer(2);
    check(!xs[29] && !xs[30] && xs[7:0] == 6 + ID.len(), $sformatf("then the identity again: XFER %08x", xs));

    // ---- the runtime's half of the buffer: writes to the answer's half do nothing, PTR wraps
    wr(R_PTR, 130); wr(R_DATA, 32'h55); read_ans(4);
    check(ans[2] == 8'h01, "a write to the answer's half does nothing");
    wr(R_PTR, 255); rdr(R_DATA, v); rdr(R_PTR, v);
    check(v == 0, "PTR wraps at 256");

    check(conflicts == 0 && bad_stop == 0, $sformatf("at the end: no conflicts (%0d), stop bits (%0d)", conflicts, bad_stop));
    check(tail_min >= 29.5e3 && lead_min >= 9.9e3, "at the end: DIR's lead and tail");
    $display("%0d checks, %0d failed", checks, fails);
    if (fails == 0) $display("ALL PASS"); else $display("SOME FAILED");
    $finish;
end
initial begin #(600ms); $display("FAIL: the bench timed out"); $display("SOME FAILED"); $finish; end
endmodule
`default_nettype wire
