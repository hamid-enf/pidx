# Builds and runs all ten PIDX examples.
#
#   make            build everything
#   make run        build and run everything in order
#   make ex03       build one example
#   make run-ex03   build and run one
#   make clean
#
# The warning set is the library's own build gate: every example is held to the
# same standard as the library, because an example that only compiles with the
# warnings off is not a usable template.

CC      ?= cc
ROOT    := ..
CFLAGS  := -std=c99 -O2 \
           -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow \
           -Wcast-qual -pedantic -Werror \
           -I$(ROOT)/include -Icommon -I$(ROOT)/platform/posix
LDFLAGS := -lm

# Examples 05 and 08 use clock_gettime / the POSIX platform layer.
POSIX_FLAGS := -D_POSIX_C_SOURCE=199309L

CORE    := $(wildcard $(ROOT)/src/*.c)
COMMON  := $(wildcard common/*.c)
PLAT    := $(ROOT)/platform/posix/pid_posix.c

BIN     := bin
EXAMPLES := ex01 ex02 ex03 ex04 ex05 ex06 ex07 ex08 ex09 ex10

DIR_ex01 := 01_minimal
DIR_ex02 := 02_temperature_pwm
DIR_ex03 := 03_motor_speed
DIR_ex04 := 04_motor_position
DIR_ex05 := 05_current_control
DIR_ex06 := 06_cascade_pos_vel_cur
DIR_ex07 := 07_autotune_relay
DIR_ex08 := 08_freertos_task
DIR_ex09 := 09_tim_isr
DIR_ex10 := 10_full_featured

.PHONY: all run clean $(EXAMPLES) $(addprefix run-,$(EXAMPLES))

all: $(addprefix $(BIN)/,$(EXAMPLES))

$(BIN):
	@mkdir -p $(BIN)

# Examples 05 and 08 need the POSIX feature macro; 08 also links the platform
# layer. Everything else builds from the core plus the shared harness.
$(BIN)/ex05: $(DIR_ex05)/main.c $(COMMON) $(CORE) | $(BIN)
	$(CC) $(CFLAGS) $(POSIX_FLAGS) $^ $(LDFLAGS) -o $@

$(BIN)/ex08: $(DIR_ex08)/main.c $(COMMON) $(PLAT) $(CORE) | $(BIN)
	$(CC) $(CFLAGS) $(POSIX_FLAGS) $^ $(LDFLAGS) -o $@

$(BIN)/ex%: $(wildcard */main.c) $(COMMON) $(CORE) | $(BIN)
	$(CC) $(CFLAGS) $(DIR_ex$*)/main.c $(COMMON) $(CORE) $(LDFLAGS) -o $@

$(EXAMPLES): %: $(BIN)/%

$(addprefix run-,$(EXAMPLES)): run-%: $(BIN)/%
	@echo "======================================================================"
	@./$<

run: all
	@# See tests/Makefile: a restored workspace can lose the execute bit on
	@# binaries built in an earlier session.
	@chmod +x $(BIN)/* 2>/dev/null || true
	@for e in $(EXAMPLES); do \
	  echo "======================================================================"; \
	  ./$(BIN)/$$e || exit 1; \
	  echo; \
	done

clean:
	@rm -rf $(BIN)
