---
allowed-tools: Read, Glob, Grep
description: Check consistency across the document chain and flag drift.
---

# Refine: Design Chain Consistency Check

Walk the VISION → ARCHITECTURE → DESIGN → PSEUDOCODE → Code chain,
checking that each level faithfully implements the one above it and
that no level has drifted out of sync.

If `$ARGUMENTS` names a subsystem, a section number, or a path,
restrict the whole check to the part of the chain that covers it. A
full-chain pass on a mature project is expensive; a scoped pass is
the normal case.

**Step 1: Check the indexes first**

`dev/DESIGN.md` and `dev/PSEUDOCODE.md` are indexes over
`dev/design/` and `dev/pseudocode/`. (In a project where they are
single files holding the numbered sections themselves, there is no
index to check: skip this step, and treat each section's "Governs"
line as the index's "Governs" column below.) Before anything else,
verify:

- Every file in each directory has a row in its index.
- Every index row points at a file that exists.
- Section numbers in filenames match the numbers in the rows.
- The PSEUDOCODE index's "Governs" column names source files that
  still exist at those paths.

Index drift is the failure mode the split layout introduces, it is
cheap to check, and everything below depends on the indexes being
trustworthy. Report any drift before continuing.

**Step 2: Read the chain**

Read `dev/VISION.md` and `dev/ARCHITECTURE.md` in full. For DESIGN
and PSEUDOCODE, read the index tables, then read in full only the
section files in scope. Read the source files those sections govern.

**Step 3: Check each boundary**

Work top-down, one boundary at a time. At each, ask: "Does the lower
level faithfully implement what the upper level claims?"

```
VISION → ARCHITECTURE
  Does the architecture serve the stated goals and principles? Are
  there architectural choices that contradict a vision principle, or
  that quietly re-admit something listed as a non-goal?

ARCHITECTURE → DESIGN
  Does the design reference the correct modules and layouts? Are
  there design sections describing structures not in the
  architecture, or architectural components with no design coverage?

DESIGN → PSEUDOCODE
  Does the pseudocode match the algorithms the design describes? Are
  there design decisions the pseudocode silently ignores or
  contradicts? Does every design section have a pseudocode section,
  and does each pseudocode section still cite a live design section?

PSEUDOCODE → Code
  Does the source implement the pseudocode? Are there source files
  under src/ that no pseudocode section claims to govern? Are there
  pseudocode steps absent from the implementation?
```

Also check upward: does the code reveal something that should be
documented at a higher level but is not?

**Step 4: Check the notation and naming conventions**

Verify that the symbols fixed in the DESIGN index are used
consistently across the design sections, and that pseudocode variable
names match the names actually used in `src/`. Divergence here is
what makes a chain unreviewable by eye.

**Step 5: Check TODO.md alignment**

Read `dev/TODO.md`. Verify that every unchecked item carries a
citation tag, that no completed work is still listed as pending, that
no pending work is missing, and — importantly — that no entry has
grown into a de facto specification that belongs in `pseudocode/`.

**Step 6: Report findings**

Present each inconsistency as a numbered item:

```
1. [BOUNDARY] DESIGN → PSEUDOCODE
   [FILE] dev/pseudocode/03-scattering.md, §3.2
   [ISSUE] design/03-scattering.md §3.6 specifies the susceptibility
   as S = sigma * (1 - cos(theta)), but the pseudocode omits sigma.
   [SUGGESTION] Add the full formula to pseudocode §3.2.
```

State explicitly where a boundary is clean: "VISION → ARCHITECTURE:
consistent."

**Step 7: Propose resolution**

For each finding, ask the programmer whether to fix the lower level
(bring it into line with the level above), fix the upper level
(update the doc to reflect what is actually built), or defer (add a
TODO item at the appropriate level).

Note the one asymmetry: fixing *upward* is legitimate only where the
code has first been verified to implement DESIGN faithfully.
Everywhere else, a disagreement between pseudocode and code means the
code is wrong. Do not offer the upward fix as if it were symmetric.

Do NOT make changes without the programmer's decision on direction.
