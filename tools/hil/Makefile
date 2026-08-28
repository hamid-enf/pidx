# Hardware-in-the-loop firmware.
#
#   make host TUNING=/path/to/pidx_tuning_myLoop.h SYMBOL=myLoop
#       build hil_board_host, which speaks the protocol on stdin/stdout.
#       Run this FIRST: it proves the exported tuning compiles, links and
#       answers the protocol before you spend an afternoon finding out on a
#       board that the symbol name was wrong.
#
#   smoke TUNING=... SYMBOL=...
#       build and drive it through one full session from a canned script.
#
# The STM32 build is yours: hil_board.c needs hil_putc/hil_getc and a main()
# for your UART. Everything else is portable C99.

ROOT   := ../..
CC     ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wshadow -pedantic \
          -DPIDX_PROFILE_FULL -DHIL_HOST -I$(ROOT)/include
LIBSRC := $(wildcard $(ROOT)/src/*.c)

SYMBOL ?= hilLoop
TUNING ?= $(CURDIR)/pidx_tuning_$(SYMBOL).h

# The generated .c sits next to the generated .h. Both are needed: the header
# declares the loop, the source defines it, and linking without the source is
# the failure that shows up as five undefined references rather than one clear
# message.
ifneq (,$(wildcard $(TUNING)))
TUNDIR  := $(patsubst %/,%,$(dir $(TUNING)))
TUNSRC  := $(TUNDIR)/$(patsubst %.h,%.c,$(notdir $(TUNING)))
else
TUNDIR  :=
TUNSRC  :=
endif

.PHONY: host smoke clean

host: hil_board_host

hil_board_host: hil_board.c $(LIBSRC) $(TUNING) $(TUNSRC)
	@if [ -z "$(TUNSRC)" ] || [ ! -f "$(TUNSRC)" ]; then \
	    echo "  ERROR: no tuning source found."; \
	    echo "         TUNING=$(TUNING)"; \
	    echo "         Expected $(TUNSRC) next to it."; \
	    echo "         Generate both with simlab.exportSTM32."; \
	    exit 1; \
	fi
	@echo "  CC    hil_board_host (symbol=$(SYMBOL))"
	@echo "        tuning: $(TUNSRC)"
	@$(CC) $(CFLAGS) -DHIL_SYMBOL=$(SYMBOL) \
	    -I$(TUNDIR) \
	    hil_board.c $(TUNSRC) $(LIBSRC) -lm -o hil_board_host

# A canned session: identify, configure, three samples, disconnect. Any
# unexpected answer fails the target, so this is usable as a CI check that the
# exported file and the firmware still agree.
smoke: hil_board_host
	@echo "  SMOKE hil_board_host"
	@printf 'ID\nPING\nRESET 0.01\nS 0.0 1.0\nS 0.5 1.0\nGAINS 1 1 0 0\nBYE\n' \
	    | ./hil_board_host > smoke.out
	@cat smoke.out
	@grep -q '^PIDX_HIL $(SYMBOL) lib' smoke.out || { echo "FAIL: no ID"; exit 1; }
	@grep -q '^PONG$$' smoke.out                 || { echo "FAIL: no PONG"; exit 1; }
	@grep -q '^OK' smoke.out                     || { echo "FAIL: no OK"; exit 1; }
	@grep -q '^REFUSED' smoke.out                || { echo "FAIL: GAINS should be refused"; exit 1; }
	@n=$$(grep -cE '^-?[0-9.e+-]+ -?[0-9.e+-]+ [0-9]+$$' smoke.out); \
	 if [ "$$n" -lt 2 ]; then echo "FAIL: expected 2 sample replies, got $$n"; exit 1; fi
	@echo "  smoke OK"

clean:
	@rm -f hil_board_host smoke.out
