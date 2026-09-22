---
allowed-tools: Read, Glob, Grep, Edit, Bash
description: >
  Audit source files for line-length violations and reflow
  them to the 70-80 character target band.
argument-hint: "[file, directory, or glob; default src/]"
---

# Lint: Line-Length Audit and Reflow

Audit source files for line-length violations and reflow them into
the 70-80 character band. `$ARGUMENTS` may be a specific file, a
directory, a glob (`**/*.py`), or nothing — in which case default to
`src/`.

Two deterministic helpers do the mechanical work. Use them first and
fall back to `Edit` only for what they cannot handle. They are
idempotent, they are far cheaper than a hand pass, and unlike a hand
pass they cannot silently reword prose.

## Helper scripts

Both live in `.claude/commands/scripts/`.

- **`rewrap_prose.py`** — reflows comment-block paragraphs and Python
  docstring paragraphs to fill the 70-80 band. Handles Fortran `!`
  comments and Python `#` comments, plus prose paragraphs inside
  multi-line triple-quoted docstrings. Run with `--check` for a dry
  run.

  ```
  python3 .claude/commands/scripts/rewrap_prose.py FILE \
      [--check] [--lines S-E]
  ```

  Notes:
  - `## code` lines are treated as protected content and are never
    touched. Same for Fortran `!!` and for `!` with no following
    space. See "Structured Comment Blocks" in `CLAUDE.md`.
  - A triple-quoted string counts as a docstring only when its
    opening quote is the first non-whitespace token on its line
    (detection uses `tokenize`), so `help_text = """..."""` is
    correctly left alone — argparse text stays a manual exercise.
  - Structure inside docstrings (bullet and numbered lists, doctest
    blocks, section headers, RST directives, numpy-style
    `name : type` field lines, and code or data blocks with no
    sentence punctuation) is detected and left untouched.
  - Double-space-after-period typography survives a reflow pass.

- **`rewrap_code.py`** — auto-detects over-wrapped code blocks (two
  or more lines that would fit on fewer) and compacts them. Tries a
  single-line join first, then a comma split. Scan mode is the
  default:

  ```
  python3 .claude/commands/scripts/rewrap_code.py SOURCE [--check]
  ```

  For manual control, use directive mode:

  ```
  python3 .claude/commands/scripts/rewrap_code.py SOURCE \
      --noscan DIRECTIVES [--check]
  ```

  Directive format, one per line, `#` comments allowed:
  - `unwrap START END` — join lines into one, strip gratuitous
    continuation parentheses.
  - `rewrap START END` — join, then re-split at top-level commas.
  - `merge-strings START END` — concatenate adjacent string literal
    contents and re-split to fill 70-80 per line.

Follow these steps precisely.

**Step 1: Identify target files**

Resolve `$ARGUMENTS` into a list of source files. Use Glob to expand
directories and patterns. Include only source files (`.f90`, `.F90`,
`.py`, `.pl`, `.pm`). Skip binary files, swap files, generated
directories, and build artifacts.

If the target resolves to more than 20 files, report the count and
ask whether to proceed or narrow the scope.

**Step 2: Scan for violations**

For each target file, identify:

1. **Over-length** (hard violation): lines exceeding 80 characters.
   These MUST be fixed.

   ```bash
   awk 'length > 80 {print FILENAME": "FNR" ("length")"}' FILE
   ```

2. **Under-filled** (soft violation): lines shorter than 70
   characters whose content could reasonably fill more. This covers
   comment blocks whose consecutive short lines could merge into
   fewer, fuller ones; string literals or argument lists broken
   early; and code split across three or four lines that would fit
   on one or two.

   Do NOT flag as under-filled:
   - Lines naturally short (`implicit none`, `end if`, `return`,
     blank lines, a lone closing bracket).
   - Lines where filling further would hurt logic or readability —
     one-item-per-line formatting that aids clarity, for instance.
   - The last line of a reflowed paragraph; a short remainder is
     expected and correct.

For Fortran and Python files, also run `rewrap_prose.py FILE --check`
and `rewrap_code.py FILE --check`, and fold their output into the
report.

**Step 3: Report findings**

Group by file:

```
Line-length audit: N file(s) scanned, M violation(s).

--- src/project/dos.py ---
  Over-length (2):
    L42  (87 chars): <truncated preview>
    L108 (83 chars): <truncated preview>
  Comment reflow (rewrap_prose --check):
    L55-L58: 4 -> 3 lines
  Over-wrapped code (rewrap_code --check):
    L236-239: sys.exit() split across 4 lines (fits 1)

--- src/project/kpoints.py ---
  No violations found.
```

If nothing is found anywhere, say so and stop.

**Step 4: Ask for approval**

Ask: "Shall I reflow these files to fix the violations? You can
approve all, select specific files, or skip."

Do NOT proceed without explicit approval.

**Step 5: Apply fixes**

*5a. Comment and docstring prose* — run on Fortran and Python files:

```bash
python3 .claude/commands/scripts/rewrap_prose.py FILE
```

*5b. Over-wrapped code* — review the `--check` output, then apply:

```bash
python3 .claude/commands/scripts/rewrap_code.py FILE
```

Run it repeatedly until it reports 0 changes; it is idempotent. For
blocks the scanner cannot handle automatically (`merge-strings`, for
example), use directive mode.

*5c. Remaining violations* — use `Edit` for what the scripts cannot
do: over-length code needing language-aware wrapping (Fortran `&`,
Python implicit continuation inside brackets, Perl breaking after an
operator or comma), and under-filled lines needing judgment.

Preserve meaning throughout. Never alter logic, variable names, or
executable behavior; only whitespace and line breaks may change.

**Step 6: Verify and summarize**

Re-scan every modified file to confirm no new violations were
introduced, then report:

```
Reflow complete:
  src/project/dos.py:
    rewrap_prose: 3 comment blocks reflowed
    rewrap_code:  5 blocks compacted
    manual edit:  2 over-length lines wrapped
  src/project/kpoints.py: skipped (no approval)
```
