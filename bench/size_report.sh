#!/bin/sh
#
# size_report.sh - Flash/RAM footprint of PIDX, per compile-time profile.
#
# Not part of the library.
#
# Reports .text + .rodata (Flash) and .data + .bss (static RAM) for every
# source file, for each of the four profiles, so the cost of enabling a
# feature is visible rather than asserted.
#
# WHY `size -A` AND NOT `size -t`:
#   `size -t` prints a "total" line that double-counts on some binutils
#   versions when given multiple objects, which silently inflates every
#   figure. `size -A` prints the real per-section table and the sums here are
#   computed from it. This was a measured mistake, not a stylistic choice.
#
# WHY THE HOST COMPILER:
#   There is no arm-none-eabi-gcc in this environment, so these are x86-64
#   numbers. They are a RELATIVE guide - the ordering and the deltas between
#   profiles port to Cortex-M, the absolute byte counts do not. Thumb-2 code
#   is typically appreciably smaller. No Cortex-M number is invented here.
#
# Usage:  sh bench/size_report.sh  [CC]      (from the project root)

set -e

CC=${1:-gcc}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

CFLAGS="-std=c99 -Os -Wall -Wextra -Werror -I$ROOT/include -ffunction-sections -fdata-sections"

PROFILES="MINIMAL MOTION PROCESS FULL"

printf '=== PIDX size report (%s, -Os) ===\n\n' "$(basename "$CC")"
printf 'Host x86-64 figures. Relative comparison is the deliverable;\n'
printf 'absolute bytes differ on Cortex-M (Thumb-2 is usually smaller).\n\n'
printf '  %-10s %10s %10s %10s %10s\n' profile text rodata data+bss "flash"
printf '  %-10s %10s %10s %10s %10s\n' '--------' '------' '------' '--------' '-----'

for prof in $PROFILES; do
    text=0; rodata=0; ram=0

    extra=""
    if [ "$prof" = "MINIMAL" ]; then
        # Diagnostics off also requires telemetry off; otherwise the ring
        # buffer is still compiled in and the "minimal" figure is a lie.
        extra="-DPIDX_ENABLE_TELEMETRY=0"
    fi

    for src in "$ROOT"/src/*.c; do
        obj="$TMP/$(basename "$src" .c).o"
        # One file at a time: `gcc -c a.c b.c -o out` is a hard error and
        # would make this loop report failures that are the loop's fault.
        $CC $CFLAGS -DPIDX_PROFILE_"$prof" $extra -c "$src" -o "$obj" 2>/dev/null || {
            printf '  %-10s  (compile failed for %s)\n' "$prof" "$(basename "$src")"
            continue
        }

        # -ffunction-sections splits .text into .text.<symbol>, so these
        # must match on PREFIX. Matching the exact name ".text" silently
        # reports 0 for every module - which is exactly what an earlier
        # version of this script did.
        vals=$(size -A "$obj" | awk '
            index($1, ".text")   == 1 { t += $2 }
            index($1, ".rodata") == 1 { r += $2 }
            index($1, ".data")   == 1 { d += $2 }
            index($1, ".bss")    == 1 { d += $2 }
            END { printf "%d %d %d", t+0, r+0, d+0 }')
        text=$((text + $(echo "$vals" | cut -d' ' -f1)))
        rodata=$((rodata + $(echo "$vals" | cut -d' ' -f2)))
        ram=$((ram + $(echo "$vals" | cut -d' ' -f3)))
    done

    printf '  %-10s %10d %10d %10d %10d\n' \
           "$prof" "$text" "$rodata" "$ram" "$((text + rodata))"
done

printf '\n=== Per-module .text, FULL profile ===\n\n'
for src in "$ROOT"/src/*.c; do
    obj="$TMP/m_$(basename "$src" .c).o"
    $CC $CFLAGS -DPIDX_PROFILE_FULL -c "$src" -o "$obj" 2>/dev/null || continue
    t=$(size -A "$obj" | awk 'index($1, ".text") == 1 { s += $2 } END { print s+0 }')
    printf '  %-28s %8s B\n' "$(basename "$src")" "${t:-0}"
done

printf '\n=== Struct sizes (FULL) ===\n\n'
cat > "$TMP/sz.c" <<'EOF'
#include <stdio.h>
#include "pidx/pid.h"
#include "pidx/pid_autotune.h"
#include "pidx/pid_fixed.h"
int main(void){
    printf("  %-22s %5zu B\n", "PID_Handle",   sizeof(PID_Handle));
    printf("  %-22s %5zu B\n", "PID_Config",   sizeof(PID_Config));
    printf("  %-22s %5zu B\n", "PID_Status",   sizeof(PID_Status));
#if PIDX_ENABLE_AUTOTUNE
    printf("  %-22s %5zu B\n", "PID_AutoTune", sizeof(PID_AutoTune));
#endif
#if PIDX_ENABLE_FIXED_POINT
    printf("  %-22s %5zu B\n", "PIDq_Handle",  sizeof(PIDq_Handle));
#endif
    return 0;
}
EOF
$CC $CFLAGS -DPIDX_PROFILE_FULL "$TMP/sz.c" "$ROOT"/src/*.c -o "$TMP/sz" -lm 2>/dev/null \
    && "$TMP/sz" \
    || printf '  (could not build the struct-size probe)\n'

printf '\nNote: a disabled feature must cost 0 Flash and 0 RAM. The MINIMAL\n'
printf 'row is the evidence for that claim; compare it against FULL.\n'
