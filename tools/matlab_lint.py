#!/usr/bin/env python3
"""
Structural linter for the MATLAB sources in ports/matlab.

WHY THIS EXISTS
    No MATLAB or Octave interpreter was available where the simlab tool was
    written, so the code could not be executed. This script cannot execute it
    either - but a large class of MATLAB failures are structural rather than
    semantic, and those it can find:

      * an `end` that closes nothing, or a block that is never closed;
      * a function defined inside a file whose first statement is not that
        function (MATLAB requires the primary function to come first);
      * unbalanced brackets, braces or parentheses on a logical line;
      * an unterminated string literal;
      * a call to a `pidx.*` or `simlab.*` function that does not exist in
        the tree - the typo class that only shows up at runtime;
      * `methods` blocks inside a classdef that are not closed.

    What it cannot find: wrong variable names, wrong argument order, wrong
    semantics. Those need the interpreter, which is what
    simlab_tests/test_suite.m is for. Run that before trusting any number.

USAGE
    python3 tools/matlab_lint.py [path ...]     default: ports/matlab
"""

import os
import re
import sys

BLOCK_OPENERS = {
    "function", "if", "for", "while", "switch", "try", "parfor",
    "classdef", "methods", "properties", "events", "arguments", "spmd",
}
# `end` used as an index (x(end)) does not close a block. Heuristic: if the
# token before `end` is one of these, or `end` is immediately preceded by
# `(`/`{`/`,`/`:` with no whitespace, it is an index.
INDEX_BEFORE = {"(", "{", "[", ",", ":", "+", "-", "*", "/", "=", "~", "&", "|", "<", ">"}


def strip_comment(line):
    """Remove a %-comment and blank out single-quoted string bodies.

    String BODIES are replaced by spaces rather than dropped, so that
    character positions survive: an `end` written inside a string such as
    `case 'end'` must not be counted as closing a block, and a transpose
    operator `x'` must not be mistaken for the start of a string.
    """
    out = []
    in_str = False
    i = 0
    while i < len(line):
        c = line[i]
        if c == "'":
            prev = line[i - 1] if i > 0 else ""
            if in_str:
                if i + 1 < len(line) and line[i + 1] == "'":
                    out.append("  ")          # an escaped quote inside a string
                    i += 2
                    continue
                in_str = False
                out.append("'")
                i += 1
                continue
            if prev.isalnum() or prev in ")]}'_.":
                out.append(c)                 # transpose, not a string
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
    return "".join(out), in_str


def logical_lines(lines):
    """Yield (lineno, text) with continuations joined."""
    buf = ""
    start = 0
    for n, raw in enumerate(lines, 1):
        code, unterminated = strip_comment(raw)
        if unterminated:
            raise SyntaxError("%d: unterminated string literal: %s" % (n, raw.strip()))
        if not buf:
            start = n
        buf = (buf + " " + code) if buf else code
        if not buf.rstrip().endswith("..."):
            yield start, buf
            buf = ""
        else:
            buf = buf.rstrip()[:-3]
    if buf.strip():
        yield start, buf


def count_ends(text):
    """Count `end` tokens that close a block, not index an array.

    `text` has had string bodies blanked out, so `case 'end'` contributes
    nothing. An `end` used as an index is recognised by the character that
    precedes it: `x(end)`, `x(1:end)`, `x{end}` all have an operator or an
    opening bracket immediately before, whereas a block-closing `end` is
    preceded by whitespace or by the start of the line.
    """
    n = 0
    for m in re.finditer(r"\bend\b", text):
        before = text[:m.start()].rstrip()
        if before and before[-1] in INDEX_BEFORE:
            continue
        n += 1
    return n


CLASSDEF_KEYWORDS = {"classdef", "methods", "properties", "events", "arguments"}


def count_openers(text):
    """Count block-opening keywords.

    Only the statement-initial keyword of a line is a block opener for the
    classdef family: `properties` and `methods` appear perfectly legally as
    struct field names and function arguments, and counting those would
    report a phantom unclosed block. `if`, `for`, `while`, `switch`, `try`
    and `function` are unambiguous wherever they appear.
    """
    n = 0
    stripped = text.strip()
    head = stripped.split()[0] if stripped.split() else ""
    if head in CLASSDEF_KEYWORDS:
        # A block header is the keyword alone, optionally followed by an
        # attribute list (`methods (Static)`), an inheritance clause
        # (`classdef Foo < handle`) or a name (`classdef Foo`). An assignment
        # such as `events = getOpt(...)` starts with the same word and is NOT
        # a block - counting it reports a phantom unclosed block.
        rest = stripped[len(head):].lstrip()
        if rest == "" or rest[0] in "(<;" or re.match(r"^[A-Za-z_]\w*", rest):
            n += 1
        # Counted here rather than in the generic loop below: this is the only
        # place that can tell a block header from an assignment to a variable
        # of the same name, and counting it twice would report every classdef
        # file as having an unclosed block.
        return n
    for tok in re.findall(r"\b[A-Za-z_]\w*\b", stripped):
        if tok in BLOCK_OPENERS and tok not in CLASSDEF_KEYWORDS:
            n += 1
    # `elseif` contains `if` as a substring but \b prevents that match.
    # A one-line `if x, y = 1; end` opens and closes on the same line, which
    # count_ends() also counts, so the balance works out.
    return n


def lint_file(path):
    problems = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        lines = fh.read().split("\n")

    # An empty or stub file is a structural problem too, and the most
    # dangerous one: it parses perfectly, lints clean, and silently does
    # nothing. A demo emptied by a bad edit passed every other check in this
    # script, which is why this check exists.
    code_lines = [ln for ln in lines if strip_comment(ln)[0].strip()]
    if len(code_lines) == 0:
        return ["%s: file contains no code at all" % path]
    if len(code_lines) < 3:
        problems.append("%s: only %d line(s) of code - a stub?"
                        % (path, len(code_lines)))

    try:
        lls = list(logical_lines(lines))
    except SyntaxError as exc:
        return ["%s: %s" % (path, exc)]

    depth = 0
    first_stmt = None
    is_classdef = False
    primary = None
    fname = os.path.splitext(os.path.basename(path))[0]

    for lineno, text in lls:
        s = text.strip()
        if not s:
            continue
        if first_stmt is None:
            first_stmt = s
        if s.startswith("classdef"):
            is_classdef = True

        m = re.match(r"function\s+(?:\[[^\]]*\]|[A-Za-z_]\w*)\s*=\s*([A-Za-z_]\w*)", s)
        if m and primary is None:
            primary = m.group(1)
        m2 = re.match(r"function\s+([A-Za-z_]\w*)\s*\(", s)
        if m2 and primary is None and "=" not in s.split("(")[0]:
            primary = m2.group(1)

        depth += count_openers(s)
        depth -= count_ends(s)
        if depth < 0:
            problems.append("%s:%d: 'end' with no open block" % (path, lineno))
            depth = 0

    if depth != 0:
        problems.append("%s: %d unclosed block(s) at end of file" % (path, depth))

    if first_stmt is not None:
        if is_classdef:
            if not first_stmt.startswith("classdef"):
                problems.append("%s: a classdef file must start with 'classdef'" % path)
        elif primary is not None and primary != fname:
            problems.append(
                "%s: primary function '%s' does not match the file name '%s'"
                % (path, primary, fname))
        elif first_stmt.startswith("function") is False and primary is not None:
            problems.append("%s: first statement is not the primary function" % path)

    # bracket balance across the whole file
    code_all = " ".join(t for _, t in lls)
    joined = code_all

    # The fillOpt anti-pattern: `o = fillOpt(opt, ...)` REBUILDS o from opt on
    # every call, so a chain of them keeps only the last option. The first
    # call may build from opt; every later one must merge into o. This exact
    # bug shipped once and cost compareRules its seed field under real
    # MATLAB (R2025b), which is how it was found.
    if len(re.findall(r"\bo = fillOpt\(opt,", code_all)) > 1:
        problems.append(
            "%s: fillOpt called on `opt` more than once - every call after "
            "the first wipes the previous options; pass `o` instead" % path)
    for op, cl, name in (("(", ")", "parentheses"), ("[", "]", "brackets"),
                         ("{", "}", "braces")):
        # a bare `(` inside a string would skew this; strings were stripped
        if joined.count(op) != joined.count(cl):
            problems.append("%s: unbalanced %s (%d vs %d)"
                            % (path, name, joined.count(op), joined.count(cl)))
    return problems


def collect_package_functions(root):
    """Map 'pkg.name' -> path for every +pkg/function.m under root."""
    funcs = {}
    for dirpath, dirnames, filenames in os.walk(root):
        for fn in filenames:
            if not fn.endswith(".m"):
                continue
            base = os.path.basename(dirpath)
            full = os.path.join(dirpath, fn)
            if base.startswith("+"):
                funcs["%s.%s" % (base[1:], fn[:-2])] = full
            else:
                funcs[fn[:-2]] = full
    return funcs


def package_root(path):
    """The nearest ancestor of PATH that contains +pidx.

    Package directories are children of a directory on the MATLAB path, so a
    call to pidx.X resolves from anywhere inside the tree - not just from the
    root the user happened to lint. Walking up is what makes
    `matlab_lint.py ports/matlab/+simlab` report real problems rather than
    flagging every cross-package call as missing.
    """
    d = os.path.dirname(os.path.abspath(path))
    for _ in range(8):
        if os.path.isdir(os.path.join(d, "+pidx")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def check_cross_references(paths, known):
    """Flag calls to pidx.X / simlab.X that have no file behind them."""
    problems = []
    pat = re.compile(r"\b(pidx|simlab|simlab_tests)\.([A-Za-z_]\w*)")
    for path in paths:
        root = package_root(path)
        local = known
        if root is not None:
            local = collect_package_functions(root)
        elif known is None:
            # No package tree in scope at all: nothing to resolve against, so
            # say nothing rather than flag every call.
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            for n, raw in enumerate(fh.read().split("\n"), 1):
                code, _ = strip_comment(raw)
                for pkg, name in pat.findall(code):
                    key = "%s.%s" % (pkg, name)
                    if key in local:
                        continue
                    # property/method access on an object, or a Constant block
                    if name[0].isupper() and key.split(".")[1] in (
                            "Const", "PID", "Plant", "Sim", "Scenario",
                            "AutoTune", "Cascade", "GainSchedule", "Shaper"):
                        continue
                    problems.append(
                        "%s:%d: call to %s but no %s/+%s/%s.m exists"
                        % (path, n, key,
                           root if root else os.path.dirname(os.path.dirname(path)),
                           pkg, name))
    return problems


def main(argv):
    roots = argv[1:] or ["ports/matlab"]
    paths = []
    for root in roots:
        for dirpath, _, filenames in os.walk(root):
            for fn in sorted(filenames):
                if fn.endswith(".m"):
                    paths.append(os.path.join(dirpath, fn))

    print("matlab_lint: %d files under %s" % (len(paths), ", ".join(roots)))
    all_problems = []
    for path in paths:
        all_problems.extend(lint_file(path))

    known = collect_package_functions(roots[0])
    all_problems.extend(check_cross_references(paths, known))

    if all_problems:
        print()
        for p in all_problems:
            print("  " + p)
        print("\n%d problem(s)" % len(all_problems))
        return 1
    print("  no structural problems found")
    print("\n  NOTE: this checks structure, not semantics. Run")
    print("        ports/matlab/simlab_tests/test_suite.m in MATLAB/Octave")
    print("        before trusting any number the tool produces.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
