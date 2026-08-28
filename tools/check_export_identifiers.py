#!/usr/bin/env python3
"""
Check that every PIDX identifier the MATLAB exporter can emit really exists
in the C headers.

WHY THIS EXISTS
    simlab.exportSTM32 writes C code as text. A typo in an enum name, or a
    struct field renamed in pid_types.h without the exporter following,
    produces a file that does not compile - and the person who finds out is
    the one who just spent an hour tuning. This script catches that class of
    error without needing MATLAB, by reading the identifier names out of the
    MATLAB source and looking each one up in include/pidx/.

    It also checks the reverse direction: the compile-time profile table
    hard-coded in the exporter against the PIDX_DEF_* macros in pid_conf.h.

USAGE
    python3 tools/check_export_identifiers.py
"""

import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INCLUDE = os.path.join(REPO, "include", "pidx")
EXPORTER = os.path.join(REPO, "ports", "matlab", "+simlab", "exportSTM32.m")
CONF = os.path.join(INCLUDE, "pid_conf.h")


def header_text():
    out = []
    for fn in sorted(os.listdir(INCLUDE)):
        if fn.endswith(".h"):
            with open(os.path.join(INCLUDE, fn), encoding="utf-8") as fh:
                out.append(fh.read())
    return "\n".join(out)


def strip_mstrings(line):
    """Blank out single-quoted MATLAB string bodies, like matlab_lint does."""
    out = []
    in_str = False
    i = 0
    while i < len(line):
        c = line[i]
        if c == "'":
            prev = line[i - 1] if i > 0 else ""
            if in_str:
                if i + 1 < len(line) and line[i + 1] == "'":
                    out.append("  ")
                    i += 2
                    continue
                in_str = False
                out.append("'")
                i += 1
                continue
            if prev.isalnum() or prev in ")]}'_.":
                out.append(c)
                i += 1
                continue
            in_str = True
            out.append("'")
            i += 1
            continue
        if c == "%" and not in_str:
            break
        out.append(" " if in_str else c)
        i += 1
    return "".join(out)


def emitted_identifiers():
    """Every PID_* name that appears in the exporter's string literals.

    Only strings are of interest: those are what end up in the generated
    file. Identifier names used by the MATLAB code itself are checked by
    matlab_lint.py instead.
    """
    names = set()
    pat = re.compile(r"'([^']*)'")
    with open(EXPORTER, encoding="utf-8") as fh:
        for line in fh:
            for lit in pat.findall(line):
                for m in re.findall(r"\bPID[A-Za-z_]\w*\b", lit):
                    # `PIDX_PROFILE_` and `PIDX_TUNING_` are prefixes the
                    # exporter concatenates a name onto at generation time;
                    # they are not identifiers and must not be looked up.
                    if m.endswith("_"):
                        continue
                    names.add(m)
    return names


def profile_table():
    """The (profile, feature) -> enabled mapping hard-coded in the exporter."""
    src = open(EXPORTER, encoding="utf-8").read()
    body = src[src.index("function ok = profileHas"):]
    body = body[:body.index("\nend")]
    table = {}
    for feature, expr in re.findall(
            r"case '(\w+)'\s*\n\s*ok = ([^\n;]+);", body):
        table[feature] = expr.strip()
    return table


def conf_defaults():
    """PIDX_DEF_<FEATURE> per profile, straight from pid_conf.h."""
    src = open(CONF, encoding="utf-8").read()
    profiles = {}
    # Each #if / #elif / #else branch of the profile block defines a full set
    # of PIDX_DEF_* macros. Split on the branch keyword itself so the last
    # branch (`#else /* PIDX_PROFILE_FULL */`) is found too - a regex that
    # only matches `#else` followed by a comment silently drops it and then
    # the FULL profile is never checked.
    current = "PIDX_PROFILE_MINIMAL"
    for line in src.split("\n"):
        m = re.match(r"#\s*elif\s+defined\((\w+)\)", line)
        if m:
            current = m.group(1)
            continue
        if re.match(r"#\s*else\b", line):
            current = "PIDX_PROFILE_FULL"
            continue
        if re.match(r"#\s*endif", line):
            if profiles:
                break
            continue
        m = re.match(r"#\s*define\s+PIDX_DEF_(\w+)\s+(\d)", line)
        if m:
            profiles.setdefault(current, {})[m.group(1)] = m.group(2)
    return profiles


def main():
    problems = []
    hdr = header_text()

    ids = sorted(emitted_identifiers())
    print("check_export_identifiers: %d PID* names emitted by %s"
          % (len(ids), os.path.relpath(EXPORTER, REPO)))
    for name in ids:
        # Struct fields are emitted as `cfg.<struct>.<field>`; the struct tag
        # and the field both have to exist. Field names alone (kp, ki) are
        # too common to look up usefully, so only PID* names are checked.
        if not re.search(r"\b%s\b" % re.escape(name), hdr):
            problems.append("emitted identifier %s is not in include/pidx/" % name)

    # ---- the profile table against pid_conf.h ----
    table = profile_table()
    conf = conf_defaults()
    if not conf:
        problems.append("could not read any PIDX_DEF_* macros out of pid_conf.h")
    print("  profile table: %d feature(s), pid_conf.h: %d profile(s)"
          % (len(table), len(conf)))
    for feature, expr in sorted(table.items()):
        # `~strcmpi(profile, 'MINIMAL')` means "enabled unless MINIMAL".
        m = re.search(r"strcmpi\(profile,\s*'(\w+)'\)", expr)
        if not m:
            problems.append("profileHas: cannot parse the rule for %s" % feature)
            continue
        excluded = m.group(1).upper()
        for prof, defs in conf.items():
            key = feature
            if key not in defs:
                problems.append("pid_conf.h has no PIDX_DEF_%s for %s"
                                % (key, prof))
                continue
            enabled_in_conf = defs[key] == "1"
            enabled_in_exporter = prof != "PIDX_PROFILE_" + excluded
            if enabled_in_conf != enabled_in_exporter:
                problems.append(
                    "profile table disagrees with pid_conf.h: %s under %s "
                    "is %s in the header but %s in the exporter"
                    % (key, prof,
                       "enabled" if enabled_in_conf else "disabled",
                       "enabled" if enabled_in_exporter else "disabled"))

    if problems:
        print()
        for p in problems:
            print("  " + p)
        print("\n%d problem(s)" % len(problems))
        return 1
    print("  every emitted identifier exists, and the profile table matches pid_conf.h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
