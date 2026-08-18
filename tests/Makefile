# PIDX unit-test harness (PHASE 17)
#
#   make            build + run every suite, print a summary
#   make build      build only
#   make <name>     build+run one suite, e.g. `make test_filter`
#   make gate       rebuild every suite under the full -Werror wall
#   make clean
#
# Two compile profiles are used on purpose:
#
#   STRICT  the same wall the library itself must survive. Every suite that
#           can live inside C99 is built with it, so a warning introduced by a
#           test is as fatal as one in src/.
#   RELAXED for test_autotune_accuracy only. That suite needs M_PI (hidden by
#           -std=c99 -pedantic) and is written in double throughout because it
#           compares the tuner against closed-form frequency-response maths, so
#           -Wdouble-promotion and -Wconversion would fire on every line of the
#           reference model rather than on anything real. -Wall -Wextra -Werror
#           still apply, and the LIBRARY sources it links are compiled by the
#           same rule, so the relaxation never reaches src/ in the gate below.

CC      ?= gcc
ROOT    := ..
INC     := -I$(ROOT)/include -I$(ROOT)/platform/posix -I$(ROOT)/platform/stm32 \
           -I$(ROOT)/tests/stm32_stub
WALL    := -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow -Wcast-qual \
           -pedantic -Werror
STRICT  := -std=c99 $(WALL) -D_POSIX_C_SOURCE=199309L
# No -D_POSIX_C_SOURCE here: defining it in isolation puts glibc into strict
# POSIX mode, which HIDES M_PI (an X/Open name). This suite needs no POSIX
# interfaces, so the default feature set is the right one.
RELAXED := -Wall -Wextra -Wshadow -Wcast-qual -Werror
OPT     ?= -O2

LIBSRC  := $(wildcard $(ROOT)/src/*.c)
POSIX   := $(ROOT)/platform/posix/pid_posix.c
# The stub provides the CMSIS-shaped registers (pidx_stub_dwt, pidx_stub_primask,
# ...) that pid_stm32.c writes to, so it is part of the STM32 link, not just an
# include path.
STM32   := $(ROOT)/platform/stm32/pid_stm32.c $(ROOT)/tests/stm32_stub/stm32_stub.c
# The ESP32 layer needs no companion .c: its stub is header-only, because
# ESP-IDF's timebase is a function call rather than a memory-mapped register
# block. The test itself defines the two simulated clocks.
ESP32   := $(ROOT)/platform/esp32/pid_esp32.c

BIN     := bin

# ---- suite inventory ------------------------------------------------------
# core library suites: link src/ only
CORE_SUITES := smoke_core test_antiwindup test_bumpless test_fastpath \
               test_autotune_safety test_diag test_safety test_fixed \
               test_filter test_shaper test_gainsched test_cascade \
               test_core_contract
POSIX_SUITES := test_posix
STM32_SUITES := test_stm32_host
ESP32_SUITES := test_esp32_host
LOOSE_SUITES := test_autotune_accuracy

ALL := $(CORE_SUITES) $(POSIX_SUITES) $(STM32_SUITES) $(ESP32_SUITES) \
       $(LOOSE_SUITES)

.PHONY: all build run clean gate $(ALL)

all: run

build: $(addprefix $(BIN)/,$(ALL))

$(BIN):
	@mkdir -p $(BIN)

$(addprefix $(BIN)/,$(CORE_SUITES)): $(BIN)/%: %.c $(LIBSRC) | $(BIN)
	@$(CC) $(STRICT) $(OPT) $(INC) $< $(LIBSRC) -lm -o $@

$(addprefix $(BIN)/,$(POSIX_SUITES)): $(BIN)/%: %.c $(LIBSRC) $(POSIX) | $(BIN)
	@$(CC) $(STRICT) $(OPT) $(INC) $< $(LIBSRC) $(POSIX) -lm -o $@

# The STM32 suite compiles the platform layer against the CMSIS look-alike in
# tests/stm32_stub, so it must see that directory FIRST.
$(addprefix $(BIN)/,$(STM32_SUITES)): $(BIN)/%: %.c $(LIBSRC) $(STM32) | $(BIN)
	@$(CC) $(STRICT) $(OPT) -I$(ROOT)/tests/stm32_stub $(INC) $< $(LIBSRC) $(STM32) -lm -o $@

# Likewise the ESP32 suite must see its own stub directory FIRST, so that the
# pid_esp32_conf.h it picks up is the host one and not a user's board config.
$(addprefix $(BIN)/,$(ESP32_SUITES)): $(BIN)/%: %.c $(LIBSRC) $(ESP32) | $(BIN)
	@$(CC) $(STRICT) $(OPT) -I$(ROOT)/tests/esp32_stub -I$(ROOT)/platform/esp32 \
	  $(INC) $< $(LIBSRC) $(ESP32) -lm -o $@

$(addprefix $(BIN)/,$(LOOSE_SUITES)): $(BIN)/%: %.c $(LIBSRC) | $(BIN)
	@$(CC) $(RELAXED) $(OPT) $(INC) $< $(LIBSRC) -lm -o $@

# `make test_filter` builds and runs just that one.
$(ALL): %: $(BIN)/%
	@./$(BIN)/$@

run: build
	@# Restore the execute bit before running. Some environments (workspace
	@# snapshot/restore, archive extraction, a FAT/exFAT or noexec-remounted
	@# share, or a copy from Windows) drop it from files that were built
	@# earlier, and the suites then fail with "Permission denied" - which
	@# looks exactly like a code regression and is not one. Re-setting it is
	@# free when it is already correct.
	@chmod +x $(BIN)/* 2>/dev/null || true
	@fail=0; \
	for t in $(ALL); do \
	  printf '\n===== %s =====\n' "$$t"; \
	  ./$(BIN)/$$t || { fail=1; echo "*** $$t FAILED ***"; }; \
	done; \
	echo ""; \
	if [ $$fail -eq 0 ]; then echo "ALL SUITES PASSED"; else echo "*** SUITE FAILURES ***"; fi; \
	exit $$fail

# Rebuild everything at -O0 and -Os too: optimisation level changes floating
# point scheduling on x86 (x87 vs SSE spills), and a test that only passes at
# one -O level is a test with a hidden tolerance bug.
gate:
	@for o in -O0 -Os -O2; do \
	  echo "--- gate $$o ---"; \
	  $(MAKE) --no-print-directory clean build OPT=$$o || exit 1; \
	done
	@echo "gate: all suites compile clean at -O0/-Os/-O2"

clean:
	@rm -rf $(BIN)
