---
allowed-tools: Read, Glob, Grep, Bash, Edit
description: Full commit -- doc propagation check, quality pass, commit message.
---

# Full Commit

Perform a thorough, documented commit. Follow each step in order.

**Step 1: Review all changes**

Run `git diff HEAD` and `git status` to see every uncommitted change,
including untracked files. List the files, and for each name the
chain level it belongs to: VISION, ARCHITECTURE, DESIGN, PSEUDOCODE,
CODE, or non-chain (notes, figures, tooling, ledgers).

**Step 2: Chain gate check**

For every changed file under `src/`, name the `dev/pseudocode/`
section that governs it, found via the "Governs" column of
`dev/PSEUDOCODE.md`. If a source change has no governing section, or
the section does not actually describe the change, stop and report
it: this is the gate described in `CLAUDE.md`, and a commit is
exactly the wrong moment to quietly back-fill the level.

**Step 3: Index integrity**

If any file under `dev/design/` or `dev/pseudocode/` was added,
removed, or renumbered, verify that `dev/DESIGN.md` and
`dev/PSEUDOCODE.md` were updated in the same change set. A new
section file with no index row is the split layout's characteristic
failure, and it is invisible until someone cannot find the section.

**Step 4: Document propagation check**

For each changed source file or document:
- Check that `dev/VISION.md`, `dev/ARCHITECTURE.md`, and the affected
  `dev/design/` sections still accurately describe the system.
- Check that completed TODO items are marked `- [x]` and moved to
  ARCHIVE in `dev/TODO.md`.
- Check that new design decisions are recorded in the appropriate
  document rather than only in the commit message.

Report every inconsistency found. If any are found, pause and ask the
programmer to resolve them before proceeding.

**Step 5: Code quality check**

Run the deterministic checks rather than reading for them by eye:

```bash
awk 'length > 80 {print FILENAME": "FNR" ("length")"}' <changed files>
python3 .claude/commands/scripts/rewrap_prose.py FILE --check
python3 .claude/commands/scripts/rewrap_code.py FILE --check
```

Then read for what the scripts cannot see:
- No unused imports or dead code remains.
- Variable names are expressive, not abbreviated (`CLAUDE.md`).
- Non-obvious logic carries an explanatory comment.
- Existing documentation was preserved through any refactor, not
  summarized away.
- Structured comment blocks use the protected form (`##` in Python,
  `!!` in Fortran) so a later reflow will not destroy them.

Fix mechanical issues inline. For substantive issues, stop and ask.

**Step 6: Confirm**

Present a brief summary of what will be committed and ask the
programmer to confirm.

**Step 7: Commit**

Stage and commit with a comprehensive message:
- First line: brief summary, imperative mood, <= 72 characters
- Blank line
- Body: what changed, why, and which document sections are affected
  (cite them by number: `D3.2`, `P1.4`)
- Blank line
- Include this trailer:
  `Co-Authored-By: Claude <noreply@anthropic.com>`

**Step 8: Report**

Report the commit hash and list any remaining open TODO items related
to the committed work.
