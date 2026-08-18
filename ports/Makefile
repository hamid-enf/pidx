# PIDX cross-language conformance harness.
#
#   make            build every available port, run them, compare
#   make compare    compare existing CSVs only
#   make ref        build + run the C reference alone
#   make clean
#
# A port whose toolchain is missing is SKIPPED WITH A MESSAGE, never silently.
# A comparison that quietly checked three ports because the fourth would not
# build is worse than no comparison at all.

ROOT     := ..
BUILD    := build
RESULTS  := results
SCEN     := compare/scenarios.txt
REF      := $(RESULTS)/ref_c.csv

# The reference is built in DOUBLE precision on purpose: Python, Octave and C#
# are all natively double, so a float reference would report rounding as if it
# were disagreement and hide the real thing. See ports/SPEC_conformance.md.
CC       ?= gcc
CFLAGS   := -std=c99 -Wall -Wextra -Wconversion -Wshadow -pedantic -Werror \
            -O2 -DPIDX_PROFILE_FULL -DPIDX_USE_DOUBLE=1 -I$(ROOT)/include
LIBSRC   := $(wildcard $(ROOT)/src/*.c)

PYTHON   ?= python3
OCTAVE   ?= octave-cli
MCS      ?= mcs
MONO     ?= mono

# Which optional toolchains are actually present.
HAVE_PY  := $(shell command -v $(PYTHON) 2>/dev/null)
HAVE_OCT := $(shell command -v $(OCTAVE) 2>/dev/null)
HAVE_MCS := $(shell command -v $(MCS) 2>/dev/null)
HAVE_MONO:= $(shell command -v $(MONO) 2>/dev/null)

CANDIDATES :=

.PHONY: all ref py m cs compare clean dirs

all: compare

dirs:
	@mkdir -p $(BUILD) $(RESULTS)

# ---- C reference ----------------------------------------------------------

$(BUILD)/conform_c: c_ref/conform_c.c $(LIBSRC) | dirs
	@echo "  CC    conform_c (double precision reference)"
	@$(CC) $(CFLAGS) $< $(LIBSRC) -lm -o $@

$(REF): $(BUILD)/conform_c $(SCEN)
	@echo "  RUN   C reference"
	@chmod +x $(BUILD)/conform_c 2>/dev/null || true
	@./$(BUILD)/conform_c $(SCEN) > $@

ref: $(REF)

# ---- Python ---------------------------------------------------------------

$(RESULTS)/py.csv: python/conform_py.py $(SCEN) $(wildcard python/pidx/*.py) | dirs
ifeq ($(HAVE_PY),)
	@echo "  SKIP  python  ($(PYTHON) not found)"
else
	@echo "  RUN   python"
	@$(PYTHON) python/conform_py.py $(SCEN) > $@
endif

# ---- MATLAB / Octave ------------------------------------------------------

$(RESULTS)/m.csv: matlab/conform_m.m $(SCEN) $(wildcard matlab/+pidx/*.m) | dirs
ifeq ($(HAVE_OCT),)
	@echo "  SKIP  matlab  ($(OCTAVE) not found; MathWorks MATLAB untested)"
else
	@echo "  RUN   octave"
	@cd matlab && $(OCTAVE) --quiet \
	   --eval "conform_m('../$(SCEN)')" > ../$@
endif

# ---- C# / Mono ------------------------------------------------------------

$(BUILD)/conform_cs.exe: $(wildcard csharp/*.cs) | dirs
ifeq ($(HAVE_MCS),)
	@echo "  SKIP  csharp  ($(MCS) not found; .NET SDK is not in Debian)"
else
	@echo "  CS    conform_cs"
	@$(MCS) -out:$@ csharp/*.cs
endif

$(RESULTS)/cs.csv: $(BUILD)/conform_cs.exe $(SCEN) | dirs
ifeq ($(HAVE_MONO),)
	@echo "  SKIP  csharp  ($(MONO) not found)"
else
	@echo "  RUN   mono"
	@$(MONO) $(BUILD)/conform_cs.exe $(SCEN) > $@
endif

py: $(RESULTS)/py.csv
m:  $(RESULTS)/m.csv
cs: $(RESULTS)/cs.csv

# ---- comparison -----------------------------------------------------------

ifneq ($(HAVE_PY),)
CANDIDATES += $(RESULTS)/py.csv
endif
ifneq ($(HAVE_OCT),)
CANDIDATES += $(RESULTS)/m.csv
endif
ifneq ($(HAVE_MONO),)
CANDIDATES += $(RESULTS)/cs.csv
endif

compare: $(REF) $(CANDIDATES)
	@echo ""
	@# Name the ports that are NOT being compared. A run that silently
	@# checked three of four because one toolchain was missing would read
	@# exactly like a full pass.
ifeq ($(HAVE_PY),)
	@echo "  NOT COMPARED: python  ($(PYTHON) not found)"
endif
ifeq ($(HAVE_OCT),)
	@echo "  NOT COMPARED: matlab  ($(OCTAVE) not found)"
endif
ifeq ($(HAVE_MONO),)
	@echo "  NOT COMPARED: csharp  ($(MONO) not found)"
endif
ifeq ($(HAVE_PY),)
	@echo "*** compare.py itself needs python3; nothing to do."
else
	@$(PYTHON) compare/compare.py $(REF) $(CANDIDATES)
endif

clean:
	@rm -rf $(BUILD) $(RESULTS)
