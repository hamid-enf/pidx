# PIDX benchmarks.
#
#   make            build bench_host
#   make run        build and run it
#   make size       Flash/RAM report per profile
#   make all-report run both, for pasting into docs/17_performance.md
#
# bench_dwt.c is deliberately NOT built here: it only compiles for a Cortex-M
# target and is meant to be dropped into the user's firmware project.

CC      ?= gcc
OPT     ?= -O2
PROFILE ?= FULL

ROOT    := ..
BIN     := bin

# Same gate as the rest of the project. A benchmark that needs relaxed
# warnings is a benchmark that is measuring something other than the library.
CFLAGS  := -std=c99 $(OPT) \
           -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow \
           -Wcast-qual -pedantic -Werror \
           -I$(ROOT)/include \
           -DPIDX_PROFILE_$(PROFILE) \
           -D_POSIX_C_SOURCE=199309L

LDLIBS  := -lm
SRC     := $(wildcard $(ROOT)/src/*.c)

.PHONY: all run size all-report clean

all: $(BIN)/bench_host

$(BIN)/bench_host: bench_host.c $(SRC) | $(BIN)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

$(BIN):
	@mkdir -p $(BIN)

run: all
	@# A restored workspace can lose the execute bit on a binary built in an
	@# earlier session; re-setting it is free when it is already correct.
	@chmod +x $(BIN)/* 2>/dev/null || true
	@./$(BIN)/bench_host

size:
	@sh size_report.sh $(CC)

all-report: run size

clean:
	@rm -rf $(BIN)
