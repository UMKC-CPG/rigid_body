---
allowed-tools: Read, Glob, Grep
description: >
  Focus the session: summarize TODO.md and load only the
  context the chosen work actually needs.
argument-hint: "[topic, section number, or file]"
---

# Session Focus

You are starting a focused development session. The point of this
command is to load a *little* context deliberately rather than a lot
accidentally, so at every step prefer reading an index or a heading
list over reading a whole document.

`$ARGUMENTS` may name a topic, a chain section number (`D3`, `P1.4`),
a source file, or nothing at all. Follow these steps precisely.

**Step 1: Read TODO.md**

Read `dev/TODO.md`. Identify all unchecked items (`- [ ]`) in each
section: VISION, ARCHITECTURE, DESIGN, PSEUDOCODE, CODE, and any
campaign sections the project has added.

**Step 2: Summarize**

Present the pending items grouped by level, with the count per level
and each item's citation tag:

```
Pending items by level:

VISION (N):
  - (V2) <item>

ARCHITECTURE (N):
  - (A4) <item>

DESIGN (N):
  - (D3.2) <item>

PSEUDOCODE (N):
  - (P1.4) <item>

CODE (N):
  - (src/project/thing.py) <item>
```

Write "(none)" for a level with no pending items. If the project has
campaign sections, list them under their own headings after CODE.

**Step 3: Choose the work**

If `$ARGUMENTS` was given, treat it as the answer: state which
pending items it corresponds to and move to Step 4. If it matches
nothing in the list, say so plainly and ask rather than guessing.

If `$ARGUMENTS` was empty, ask: "What would you like to work on
today?"

**Step 4: Load focused context**

Read only what the chosen work needs.

- **VISION work:** read `dev/VISION.md` (it is short; read it whole).
- **ARCHITECTURE work:** read `dev/ARCHITECTURE.md`.
- **DESIGN work:** read the index `dev/DESIGN.md`, then read only
  the `dev/design/NN-*.md` files the work touches. Never read the
  whole `dev/design/` directory. (In a project whose `dev/DESIGN.md`
  is a single file rather than an index, read only the numbered
  section the work touches, found by its heading.)
- **PSEUDOCODE work:** read the index `dev/PSEUDOCODE.md`, then only
  the relevant `dev/pseudocode/NN-*.md` files, plus the design
  section each one implements. (Single-file layout: the section by
  its heading, as above.)
- **CODE work:** read the relevant files in `src/`, the pseudocode
  section that governs them (find it via the "Governs" column of the
  index), and its design section. If the index names no section for
  those files, say so — that is the chain gate in `CLAUDE.md`, and
  the pseudocode must be written before the code is touched.

**Step 5: Confirm**

State in two or three lines: which items are in scope, which chain
sections you loaded, and — for code work — which pseudocode section
governs the change. Then confirm you are ready to begin.
