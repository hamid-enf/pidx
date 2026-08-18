# PIDX - top-level convenience Makefile.
#
# The library itself needs no build system: add src/*.c to your project and
# put include/ on the include path. This Makefile only drives the host-side
# artifacts (tests, simulations, benchmarks, examples).
#
#   make test      run the unit test suites
#   make examples  build and run the ten examples
#   make sim       run the simulation studies
#   make bench     run the host benchmark and the size report
#   make ports     cross-language conformance: C vs Python vs Octave vs Mono
#   make gate      the full warning gate: every file, every profile, -Os/-O2
#   make all       test + examples + sim + bench + ports
#   make clean     remove every build artifact
#
# There is no `install` target on purpose: an embedded library that installs
# itself into /usr/local is a library that will be the wrong version on
# somebody's board.

.PHONY: all test examples sim bench ports gate clean distclean help

help:
	@echo "PIDX - targets: test examples sim bench ports gate all clean"
	@echo "The library needs no build system; this drives host artifacts."

all: test examples sim bench ports

test:
	@$(MAKE) -C tests run

examples:
	@$(MAKE) -C examples run

sim:
	@$(MAKE) -C sim run

bench:
	@$(MAKE) -C bench all-report

# Runs every port against the C reference and diffs the numbers. A port whose
# toolchain is absent is named as skipped rather than passed over in silence.
ports:
	@$(MAKE) -C ports compare

# The warning gate. Compiles every source file on its own, under every
# compile-time profile, at both -Os and -O2, with warnings as errors.
#
# One file at a time is not a stylistic choice: `gcc -c a.c b.c -o out` is a
# fatal error, and a loop that ignores it reports failures that are the
# loop's own fault.
gate:
	@echo "=== PIDX warning gate ==="
	@fail=0; n=0; \
	flags="-std=c99 -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow -Wcast-qual -pedantic -Werror"; \
	incs="-Iinclude -Iexamples/common -Iplatform/posix -Iplatform/stm32 -Itests/stm32_stub -D_POSIX_C_SOURCE=199309L"; \
	espincs="-Iplatform/esp32 -Itests/esp32_stub"; \
	for src in src/*.c platform/posix/*.c platform/stm32/*.c platform/esp32/*.c; do \
	  for prof in MINIMAL MOTION PROCESS FULL; do \
	    for opt in -Os -O2; do \
	      extra=""; \
	      if [ "$$prof" = "MINIMAL" ]; then extra="-DPIDX_ENABLE_TELEMETRY=0"; fi; \
	      n=$$((n+1)); \
	      case "$$src" in platform/esp32/*) extra="$$extra $$espincs";; esac; \
	      out=$$(gcc $$flags $$incs -DPIDX_PROFILE_$$prof $$extra $$opt -c $$src -o /tmp/pidx_gate.o 2>&1) || fail=$$((fail+1)); \
	      if [ -n "$$out" ]; then echo "$$src [$$prof $$opt]"; echo "$$out"; fail=$$((fail+1)); fi; \
	    done; \
	  done; \
	done; \
	rm -f /tmp/pidx_gate.o; \
	echo "compilations: $$n   failures/warnings: $$fail"; \
	test $$fail -eq 0

clean:
	@$(MAKE) -C tests clean 2>/dev/null || true
	@$(MAKE) -C examples clean 2>/dev/null || true
	@$(MAKE) -C sim clean 2>/dev/null || true
	@$(MAKE) -C bench clean 2>/dev/null || true
	@$(MAKE) -C ports clean 2>/dev/null || true

distclean: clean
	@$(MAKE) -C sim distclean 2>/dev/null || true
