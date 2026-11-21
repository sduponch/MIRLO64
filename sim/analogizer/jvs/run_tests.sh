#!/bin/bash
# Bench of the JVS host (rtl/analogizer/an_jvs.sv) against a behavioural I/O board (jvs_dev.sv): tb_jvs.sv, in SystemVerilog alone.
#   ./run_tests.sh              Verilator (--binary) if there is one, else Questa
#   SIM=questa ./run_tests.sh   (Questa: QUESTA names its bin directory, SALT_LICENSE_SERVER its licence)
# Builds into $OBJ (default /tmp/an_jvs_$USER). Exit status 0 if all pass.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"; R="$(cd "$HERE/../../.." && pwd)"; A="$R/rtl/analogizer"
OBJ="${OBJ:-/tmp/an_jvs_${USER:-u}}"; mkdir -p "$OBJ"
SRC="$A/an_jvs.sv $HERE/jvs_dev.sv $HERE/tb_jvs.sv"
export VERILATOR_ROOT="${VERILATOR_ROOT:-$HOME/local/usr/share/verilator}"
VL="${VERILATOR:-$HOME/local/usr/bin/verilator}"
command -v "$VL" >/dev/null 2>&1 || VL="$(command -v verilator || true)"
SIM="${SIM:-$([ -n "$VL" ] && echo verilator || echo questa)}"
if [ "$SIM" = verilator ]; then
    "$VL" --binary -j 8 -O2 -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-BLKSEQ \
        -Wno-INITIALDLY -Wno-MULTIDRIVEN -Wno-UNOPTFLAT -Wno-REALCVT --top-module tb_jvs -Mdir "$OBJ" $SRC -o tb > "$OBJ.log" 2>&1 \
        || { grep -E "%Error" "$OBJ.log" | head -20; tail -5 "$OBJ.log"; echo "build failed"; exit 1; }
    "$OBJ/tb" | tee "$OBJ/run.log"
else
    Q="${QUESTA:-$HOME/altera_lite/25.1std/questa_fse/bin}"
    export SALT_LICENSE_SERVER="${SALT_LICENSE_SERVER:-$HOME/.altera.quartus/questa_lic.dat}"
    cd "$OBJ"
    "$Q/vlib" work > /dev/null
    "$Q/vlog" -sv -quiet $SRC > "$OBJ.log" 2>&1 || { grep -E "Error" "$OBJ.log" | head -20; echo "build failed"; exit 1; }
    "$Q/vsim" -c -quiet tb_jvs -do "run -all; quit -f" | sed -n 's/^# //p' | grep -vE "^(Loading|//|\*\*|End time|Errors:|run -all|quit)" | tee "$OBJ/run.log"
fi
grep -q "^ALL PASS" "$OBJ/run.log"
