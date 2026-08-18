#!/usr/bin/env python3
"""Conformance runner for the Python port.

Reads the scenario file described in ports/SPEC_conformance.md and emits the
CSV that is compared against the C reference. See ports/c_ref/conform_c.c for
the oracle implementation - this file must mirror its dispatch exactly.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pidx
from pidx.types import Status

NAN = float("nan")


def num(tok):
    """Parse a scenario number, including the non-finite spellings."""
    if tok == "nan":
        return NAN
    if tok == "inf":
        return math.inf
    if tok == "-inf":
        return -math.inf
    return float(tok)


def f(t, i):
    return num(t[i]) if i < len(t) else 0.0


def n(t, i):
    return int(t[i]) if i < len(t) else 0


def fmt(v):
    """Render a float exactly as the C runner's %.17g does.

    repr() on a Python float gives the shortest string that round-trips, which
    is not the same text as %.17g but denotes the same binary64 value. The
    comparison script parses both back to floats, so what matters is only that
    no precision is lost on the way out - and '%.17g' guarantees that.
    """
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        if math.isinf(v):
            return "inf" if v > 0 else "-inf"
    return "%.17g" % v


class Runner:
    def __init__(self):
        self.cfg = pidx.Config()
        self.pid = pidx.PID()
        self.sched_points = []
        self.scenario = "none"
        self.row = 0
        self.inited = False

    def emit(self, cmd, rc, out, sp, err, p, i, d, ff, unsat, flags, last):
        print("%s,%d,%s,%d,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d" % (
            self.scenario, self.row, cmd, int(rc),
            fmt(out), fmt(sp), fmt(err), fmt(p), fmt(i), fmt(d),
            fmt(ff), fmt(unsat), int(flags), int(last)))
        self.row += 1

    def emit_update(self, cmd, rc, out):
        s = self.pid.get_status()
        if s is None:
            self.emit(cmd, rc, out, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
                      0, self.pid.peek_last_error())
            return
        self.emit(cmd, rc, out, s.setpoint_shaped, s.error, s.p_term,
                  s.i_term, s.d_term, s.ff_term, s.output_unsat,
                  s.flags, self.pid.peek_last_error())

    # -- configuration commands -----------------------------------------

    def do_config(self, t):
        c = t[0]
        cfg = self.cfg
        if c == "gains":
            cfg.core.kp, cfg.core.ki, cfg.core.kd = f(t, 1), f(t, 2), f(t, 3)
        elif c == "dt":
            cfg.core.sample_time = f(t, 1)
        elif c == "direction":
            cfg.core.direction = n(t, 1)
        elif c == "mode":
            cfg.core.mode = n(t, 1)
        elif c == "integration":
            cfg.core.integration = n(t, 1)
        elif c == "outlim":
            cfg.limits.use_output_limits = True
            cfg.limits.output_min, cfg.limits.output_max = f(t, 1), f(t, 2)
        elif c == "intlim":
            cfg.limits.use_integral_limits = True
            cfg.limits.integral_min, cfg.limits.integral_max = f(t, 1), f(t, 2)
        elif c == "dtlim":
            cfg.limits.dt_min, cfg.limits.dt_max = f(t, 1), f(t, 2)
        elif c == "aw":
            cfg.integral.mode, cfg.integral.kt = n(t, 1), f(t, 2)
        elif c == "separation":
            cfg.integral.separation_threshold = f(t, 1)
        elif c == "deadband":
            cfg.integral.deadband = f(t, 1)
        elif c == "ienable":
            cfg.integral.enabled = (n(t, 1) != 0)
        elif c == "dmode":
            cfg.filter.derivative_mode = n(t, 1)
        elif c == "tf":
            cfg.filter.tf = f(t, 1)
        elif c == "nfilter":
            cfg.filter.n_filter = f(t, 1)
        elif c == "inlpf":
            cfg.filter.input_lpf_tau = f(t, 1)
        elif c == "weights":
            cfg.weight.beta, cfg.weight.gamma = f(t, 1), f(t, 2)
        elif c == "ff":
            cfg.feedforward.enabled = (n(t, 1) != 0)
            cfg.feedforward.value, cfg.feedforward.gain = f(t, 2), f(t, 3)
        elif c == "shaper":
            cfg.shaper.sp_rate_max = f(t, 1)
            cfg.shaper.sp_accel = f(t, 2)
            cfg.shaper.sp_decel = f(t, 3)
            cfg.shaper.out_slew_max = f(t, 4)
        elif c == "safety":
            cfg.safety.enabled = (n(t, 1) != 0)
            cfg.safety.meas_min, cfg.safety.meas_max = f(t, 2), f(t, 3)
            cfg.safety.meas_rate_max = f(t, 4)
            cfg.safety.failsafe_output = f(t, 5)
            cfg.safety.fault_persist_n = n(t, 6)
            cfg.safety.auto_recover = (n(t, 7) != 0)
        else:
            sys.stderr.write("unknown config cmd: %s\n" % c)
            sys.exit(2)

    # -- runtime commands ------------------------------------------------

    def do_run(self, t):
        c = t[0]
        p = self.pid

        if c == "u":
            out = p.update_dt(f(t, 1), f(t, 2))
            self.emit_update("u", p.peek_last_error(), out)
        elif c == "un":
            out = p.update(f(t, 1))
            self.emit_update("un", p.peek_last_error(), out)
        elif c == "ufast":
            out = p.update_fast(f(t, 1))
            self.emit("ufast", p.peek_last_error(), out, p.get_setpoint(),
                      NAN, NAN, p.get_integrator(), NAN, NAN, NAN,
                      0, p.peek_last_error())
        elif c == "uex":
            inp = pidx.Input()
            inp.measurement = f(t, 1)
            inp.dt = f(t, 2)
            inp.setpoint = f(t, 3)
            inp.feedforward = f(t, 4)
            inp.tracking = f(t, 5)
            inp.schedule_var = f(t, 6)
            out, rc = p.update_ex(inp)
            self.emit_update("uex", rc, out)
        elif c == "sp":
            p.set_setpoint(f(t, 1))
        elif c == "spimm":
            p.set_setpoint_immediate(f(t, 1))
        elif c == "setmode":
            p.set_mode(n(t, 1))
        elif c == "manual":
            p.set_manual_output(f(t, 1))
        elif c == "setgains":
            p.set_gains(f(t, 1), f(t, 2), f(t, 3))
        elif c == "rescale":
            p.set_gains_rescale_integral(f(t, 1), f(t, 2), f(t, 3))
        elif c == "setaw":
            p.set_anti_windup(n(t, 1), f(t, 2))
        elif c == "setoutlim":
            p.set_output_limits(f(t, 1), f(t, 2))
        elif c == "clroutlim":
            p.clear_output_limits()
        elif c == "setintlim":
            p.set_integral_limits(f(t, 1), f(t, 2))
        elif c == "setint":
            p.set_integrator(f(t, 1))
        elif c == "track":
            p.set_tracking_input(f(t, 1))
        elif c == "setdmode":
            p.set_derivative_mode(n(t, 1))
        elif c == "settf":
            p.set_derivative_filter(f(t, 1))
        elif c == "setn":
            p.set_derivative_filter_n(f(t, 1))
        elif c == "setdir":
            p.set_direction(n(t, 1))
        elif c == "setweights":
            p.set_weights(f(t, 1), f(t, 2))
        elif c == "setff":
            p.set_feedforward(f(t, 1))
        elif c == "setramp":
            p.set_setpoint_ramp(f(t, 1), f(t, 2), f(t, 3))
        elif c == "setslew":
            p.set_output_slew_rate(f(t, 1))
        elif c == "setinlpf":
            p.set_input_filter(f(t, 1))
        elif c == "setsep":
            p.set_integral_separation(f(t, 1))
        elif c == "setdb":
            p.set_integral_deadband(f(t, 1))
        elif c == "setienable":
            p.enable_integral(n(t, 1) != 0)
        elif c == "setdtnom":
            p.set_sample_time(f(t, 1))
        elif c == "reset":
            p.reset()
        elif c == "clearfault":
            p.clear_fault()
        elif c == "schedpoints":
            cnt = n(t, 1)
            self.sched_points = [
                pidx.GainPoint(f(t, 2 + i * 4), f(t, 3 + i * 4),
                               f(t, 4 + i * 4), f(t, 5 + i * 4))
                for i in range(cnt)]
        elif c == "schedcfg":
            if self.sched_points:
                sch = pidx.GainSchedule()
                sch.init(self.sched_points, n(t, 1), n(t, 2))
                sch.set_hysteresis(f(t, 3))
                pidx.attach(p, sch)
        elif c == "schedvar":
            pidx.set_var(p, f(t, 1))
        elif c == "rule":
            mdl = pidx.PlantModel()
            mdl.kind = n(t, 3)
            if mdl.kind == pidx.ModelKind.FREQ:
                mdl.ku, mdl.pu = f(t, 4), f(t, 5)
            else:
                mdl.k, mdl.t, mdl.l = f(t, 4), f(t, 5), f(t, 6)
            rc, g = pidx.rule_apply(n(t, 1), mdl, n(t, 2), f(t, 7))
            self.emit("rule", rc, g.kp, g.ki, g.kd, g.ti, g.td, g.tf,
                      NAN, NAN, 0, 0)
        else:
            sys.stderr.write("unknown run cmd: %s\n" % c)
            sys.exit(2)

    # -- driver ----------------------------------------------------------

    def run(self, path):
        print("scenario,k,cmd,rc,output,setpoint,error,p,i,d,ff,unsat,"
              "flags,last_error")
        with open(path, "r") as fh:
            for line in fh:
                t = line.split()
                if not t or t[0].startswith("#"):
                    continue
                if t[0] == "scenario":
                    self.cfg = pidx.Config()
                    self.pid = pidx.PID()
                    self.sched_points = []
                    self.inited = False
                    self.row = 0
                    self.scenario = t[1] if len(t) > 1 else "?"
                elif t[0] == "init":
                    rc = self.pid.init(self.cfg)
                    self.inited = (rc == Status.OK)
                    self.emit("init", rc, 0.0, NAN, NAN, NAN, NAN, NAN, NAN,
                              NAN, 0, 0)
                elif t[0] == "end":
                    self.inited = False
                elif not self.inited:
                    self.do_config(t)
                else:
                    self.do_run(t)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.stderr.write("usage: %s <scenario-file>\n" % sys.argv[0])
        sys.exit(1)
    Runner().run(sys.argv[1])
