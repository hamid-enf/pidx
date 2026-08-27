# Reference numbers for the MATLAB simulation tool.
#
#   make          build matlab_ref against the PIDX C sources
#   make run      build and write ref.csv
#   make clean
#
# matlab_ref runs the same scenarios the MATLAB tests assert on, through the
# C library. It is the oracle: if the MATLAB port and this disagree, the
# MATLAB port is wrong, because the C library is what the STM32 runs.
#
# Built in DOUBLE precision, like ports/c_ref/conform_c.c, so a disagreement
# is a logic difference rather than float rounding.

ROOT   := ../..
CC     ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wconversion -Wshadow -pedantic \
          -DPIDX_PROFILE_FULL -DPIDX_USE_DOUBLE=1 -I$(ROOT)/include
LIBSRC := $(wildcard $(ROOT)/src/*.c)

.PHONY: all run clean

all: matlab_ref

matlab_ref: matlab_ref.c $(LIBSRC)
	@echo "  CC    matlab_ref (double precision)"
	@$(CC) $(CFLAGS) matlab_ref.c $(LIBSRC) -lm -o matlab_ref

REF_COPY := $(ROOT)/ports/matlab/simlab_tests/c_reference.csv

run: matlab_ref
	@echo "  RUN   matlab_ref -> ref.csv"
	@./matlab_ref > ref.csv
	@cp ref.csv $(REF_COPY)
	@echo "  wrote ref.csv ($$(grep -vc '^#' ref.csv) values)"
	@echo "  copied to ports/matlab/simlab_tests/c_reference.csv"

# The copy is what the MATLAB tests read. Regenerating it is how the MATLAB
# port is re-verified after any change to either side.
$(REF_COPY): run

clean:
	@rm -f matlab_ref ref.csv
