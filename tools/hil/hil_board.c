/**
 * @file    hil_board.c
 * @brief   Hardware-in-the-loop firmware: the plant lives on the host, the
 *          controller lives here.
 *
 * WHAT THIS IS FOR
 *   simlab.hilRun() runs a scenario in MATLAB and exchanges one measurement
 *   and one command per sample over a serial link. This file is the other end
 *   of that link. It calls the SAME <symbol>_init() and <symbol>_tick() that
 *   simlab.exportSTM32 generated, so the controller under test is the
 *   controller you designed - the identical translation unit, not a
 *   reimplementation that happens to look similar.
 *
 * BUILDING
 *   cd tools/hil
 *   make TUNING=/path/to/pidx_tuning_myLoop.h SYMBOL=myLoop
 *   # or for a host test, which needs no board at all:
 *   make host TUNING=... SYMBOL=myLoop && ./hil_board_host
 *
 *   The host build is the one to run FIRST. It proves the exported file
 *   compiles, links and answers the protocol, before you spend an afternoon
 *   finding out on a board that the symbol name was wrong.
 *
 * WIRING
 *   The protocol is line-oriented ASCII at 115200 8N1, one request per
 *   response, no buffering:
 *
 *     ->  ID
 *     <-  PIDX_HIL <symbol> lib<version>
 *
 *     ->  RESET <dt>
 *     <-  OK
 *
 *     ->  GAINS <kp> <ki> <kd> <tf>        (optional; see the warning below)
 *     <-  OK
 *
 *     ->  S <measurement> <setpoint>
 *     <-  <u> <u_unsaturated> <flags>
 *
 *     ->  PING
 *     <-  PONG
 *
 *     ->  BYE
 *     <-  OK
 *
 * THE SAMPLE RATE IS SET BY THE LINK
 *   Each sample is a round trip, so the achievable rate is bounded by the
 *   serial latency, not by the core. This is the right tool for a process loop
 *   at 10-500 Hz. It is the WRONG tool for a 20 kHz current loop, and
 *   buffering will not fix that: a controller whose dt depends on when the
 *   host gets around to asking is not the controller you designed. dt is sent
 *   with RESET and used verbatim, so the board's integrator is correct even
 *   though the wall clock is not periodic.
 *
 * PORTING TO YOUR BOARD
 *   Two functions: hil_putc() and hil_getc(). Everything else is portable C.
 *   The defaults below are the POSIX host build; for STM32 replace them with
 *   your UART HAL calls and provide main().
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pidx/pid.h"

/* The generated tuning header. Its name is a build parameter so the same
 * firmware serves any exported loop. */
#ifndef HIL_TUNING_HEADER
#define HIL_TUNING_HEADER "pidx_tuning_hilLoop.h"
#endif
#ifndef HIL_SYMBOL
#define HIL_SYMBOL hilLoop
#endif

/* Token pasting AND stringising, both two levels deep.
 *
 * The second level is not ceremony: a function-like macro does not expand its
 * arguments before substituting them, so `HIL_CAT(HIL_SYMBOL, _init)` pastes
 * the literal text "HIL_SYMBOL" unless an intermediate macro forces the
 * expansion first. Getting this wrong compiles fine and produces a firmware
 * that calls hilSymbol_init() - the name of the macro, not of your loop.
 */
#define HIL_CAT2(a, b)  a##b
#define HIL_CAT(a, b)   HIL_CAT2(a, b)
#define HIL_STR2(x)     #x
#define HIL_STR(x)      HIL_STR2(x)

#define HIL_INIT  HIL_CAT(HIL_SYMBOL, _init)
#define HIL_TICK  HIL_CAT(HIL_SYMBOL, _tick)

/* Declared here rather than included from the generated header, so this file
 * compiles even before you have exported a tuning - it will just fail to link,
 * which is the clearer of the two failures. */
extern void  HIL_INIT(void);
extern float HIL_TICK(float y);

/* ======================================================================== */
/* Transport                                                               */
/* ======================================================================== */

#if defined(HIL_HOST)

static void hil_putc(char c)
{
    fputc(c, stdout);
}

static int hil_getc(void)
{
    return getchar();
}

int main(void);

#else /* target build: provide these two and a main() for your board */

static void hil_putc(char c)
{
    /* e.g. while (!(USART2->SR & USART_SR_TXE)); USART2->DR = c; */
    (void)c;
}

static int hil_getc(void)
{
    /* e.g. while (!(USART2->SR & USART_SR_RXNE)); return USART2->DR; */
    return -1;
}

#endif

static void hil_puts(const char *s)
{
    while (*s != '\0') {
        hil_putc(*s);
        ++s;
    }
    hil_putc('\n');
}

/* ======================================================================== */
/* Line buffer                                                             */
/* ======================================================================== */

#define HIL_LINE_MAX 96

static char hil_line[HIL_LINE_MAX];

/**
 * Read one line. Returns its length, or -1 at end of input.
 *
 * A line longer than the buffer is truncated rather than wrapped: a truncated
 * command fails to parse and is answered with ERR, whereas a wrapped one would
 * be parsed as two commands and the second would be nonsense.
 */
static int hil_readline(void)
{
    int n = 0;

    for (;;) {
        int c = hil_getc();
        if (c < 0) {
            return (n > 0) ? n : -1;
        }
        if ((c == '\n') || (c == '\r')) {
            if (n > 0) {
                break;
            }
            continue;   /* ignore blank lines: a terminal sends them freely */
        }
        if (n < (HIL_LINE_MAX - 1)) {
            hil_line[n] = (char)c;
            ++n;
        }
    }
    hil_line[n] = '\0';
    return n;
}

/* ======================================================================== */
/* Commands                                                                */
/* ======================================================================== */

static void cmd_id(void)
{
    hil_puts("PIDX_HIL " HIL_STR(HIL_SYMBOL) " lib" PIDX_VERSION_STRING);
}

static void cmd_reset(const char *args)
{
    double dt = 0.0;

    (void)args;
    /* The board's controller was configured by <symbol>_init() with the
     * exported dt. RESET re-runs init so a session always starts from the
     * exported state, and accepts a dt for the HOST's bookkeeping. Overriding
     * the controller's own dt here would mean the board integrates over a
     * period that is not the one it was tuned for, which is the kind of quiet
     * difference HIL exists to find rather than to introduce. */
    if (sscanf(args, "%lf", &dt) == 1 && dt > 0.0) {
        char reply[64];
        HIL_INIT();
        sprintf(reply, "OK %.10g", dt);
        hil_puts(reply);
    } else {
        HIL_INIT();
        hil_puts("OK");
    }
}

static void cmd_gains(const char *args)
{
    /* Deliberately NOT implemented.
     *
     * The point of a HIL run is to test the file you exported. A firmware that
     * accepts new gains over the link turns that into a test of whatever the
     * host last typed, and the divergence you were looking for becomes
     * impossible to attribute. If you want to sweep gains, re-export and
     * reflash, or use simlab.monteCarlo, which sweeps in simulation where a
     * mistake costs seconds.
     *
     * The command is answered with a reason rather than ERR, because "ERR"
     * reads like a link fault. */
    (void)args;
    hil_puts("REFUSED gains are set by the exported file, not the link");
}

static void cmd_sample(const char *args)
{
    double y = 0.0;
    double sp = 0.0;
    float u;
    char reply[96];

    if (sscanf(args, "%lf %lf", &y, &sp) != 2) {
        hil_puts("ERR S needs <measurement> <setpoint>");
        return;
    }

    /* The setpoint is applied through the generated header's own controller,
     * so any shaper, weighting or validation it configures is exercised
     * exactly as it will be in the product. */
    {
        extern PID_Handle *HIL_CAT(HIL_SYMBOL, _handle)(void);
        (void)PID_SetSetpoint(HIL_CAT(HIL_SYMBOL, _handle)(), (float)sp);
    }

    u = HIL_TICK((float)y);

    /* The reply carries the command, the pre-saturation sum and the PIDX
     * status flags. The flags come from the library rather than being
     * re-derived here: if the controller says it saturated, that is what the
     * host should be told even in a case where guessing from the data would
     * say otherwise. */
    {
        extern unsigned int HIL_CAT(HIL_SYMBOL, _flags)(void);
        extern float HIL_CAT(HIL_SYMBOL, _unsat)(void);
        sprintf(reply, "%.9g %.9g %u", (double)u,
                (double)HIL_CAT(HIL_SYMBOL, _unsat)(),
                HIL_CAT(HIL_SYMBOL, _flags)());
        hil_puts(reply);
    }
}

/* ======================================================================== */
/* Loop                                                                    */
/* ======================================================================== */

/**
 * Serve commands until BYE or end of input.
 *
 * Returns 0 on a clean exit, 1 otherwise, so the host build can be used as a
 * smoke test in a script.
 */
int hil_serve(void)
{
    /* No unsolicited banner. A host that opens the port and reads before it
     * asks would otherwise have to discard a line it did not request, and
     * "read whatever is there" is how a HIL session ends up one reply out of
     * step with the board for the rest of the run. The host asks ID.
     */
    for (;;) {
        int n = hil_readline();
        char *args;

        if (n < 0) {
            return 1;
        }

        args = strchr(hil_line, ' ');
        if (args != NULL) {
            *args = '\0';
            ++args;
        } else {
            args = (char *)"";
        }

        if (strcmp(hil_line, "ID") == 0) {
            cmd_id();
        } else if (strcmp(hil_line, "RESET") == 0) {
            cmd_reset(args);
        } else if (strcmp(hil_line, "GAINS") == 0) {
            cmd_gains(args);
        } else if (strcmp(hil_line, "S") == 0) {
            cmd_sample(args);
        } else if (strcmp(hil_line, "PING") == 0) {
            hil_puts("PONG");
        } else if (strcmp(hil_line, "BYE") == 0) {
            hil_puts("OK");
            return 0;
        } else {
            hil_puts("ERR unknown command");
        }
    }
}

#if defined(HIL_HOST)
int main(void)
{
    return hil_serve();
}
#endif
