# Builds and runs the PIDX simulation studies.
#
#   make            build every simulation
#   make run        build and run every simulation in order
#   make rules      build one study
#   make run-rules  build and run one
#   make plots      regenerate the figures from the CSVs (needs matplotlib)
#   make clean      remove binaries
#   make distclean  also remove results/
#
# Same warning set as the library build gate: a simulation that only compiles
# with the warnings off is not evidence of anything.
#
# The studies reuse examples/common/ex_plant.c for the plant physics rather
# than carrying a second copy of the model, so a change to the plant cannot
# silently make the simulations disagree with the examples.

CC      ?= cc
ROOT    := ..
OPT     ?= -O2
CFLAGS  := -std=c99 $(OPT) \
           -Wall -Wextra -Wconversion -Wdouble-promotion -Wshadow \
           -Wcast-qual -pedantic -Werror \
           -I$(ROOT)/include -I$(ROOT)/examples/common -I.
LDFLAGS := -lm

CORE    := $(wildcard $(ROOT)/src/*.c)
PLANT   := $(ROOT)/examples/common/ex_plant.c
SUPPORT := sim_common.c $(PLANT) $(CORE)

BIN     := bin
RESULTS := results
SIMS    := rules robust autotune

.PHONY: all run plots clean distclean $(SIMS) $(addprefix run-,$(SIMS))

all: $(addprefix $(BIN)/,$(SIMS))

$(BIN):
	@mkdir -p $(BIN)

$(BIN)/%: sim_%.c sim_common.c sim_common.h $(PLANT) $(CORE) | $(BIN)
	$(CC) $(CFLAGS) $< $(SUPPORT) $(LDFLAGS) -o $@

$(SIMS): %: $(BIN)/%

$(addprefix run-,$(SIMS)): run-%: $(BIN)/%
	./$<

run: all
	@# See tests/Makefile: a restored workspace can lose the execute bit on
	@# binaries built in an earlier session, which fails as "Permission
	@# denied" and mimics a real regression.
	@chmod +x $(BIN)/* 2>/dev/null || true
	@for s in $(SIMS); do \
	  echo "=== $$s ==="; \
	  ./$(BIN)/$$s || exit 1; \
	done

plots: $(RESULTS)/rules.csv
	python3 plot.py

clean:
	@rm -rf $(BIN)

distclean: clean
	@rm -rf $(RESULTS)
