# Task List

> **Document hierarchy:** Tasks are organized by the level of the design
> chain they affect. Each item should cite the relevant document section.

---

## VISION

<!-- Tasks related to goals and principles. -->

---

## ARCHITECTURE

<!-- Tasks related to layout, modules, build. -->

---

## DESIGN

<!-- Tasks related to algorithms and data structures, mathematical
foundations, interaction rules. -->

`DESIGN.md` is complete: all thirteen sections are drafted, and its
Contents table marks every one "written". This list now records only the
milestone work that follows.

**All DESIGN sections are written.** §1–§12 are committed; §13 (scene
description and palettes) is drafted and awaiting commit.

**Next milestone — close out DESIGN and open PSEUDOCODE:**

- [ ] Commit §13. **This is the next step.**
- [ ] Run `/refine` to check consistency across VISION → ARCHITECTURE →
      DESIGN now that the middle level is whole, before locking it.
- [ ] Tag `v0.3-design` (ARCHITECTURE §10) once §13 is committed and the
      chain is consistent.
- [ ] Begin `PSEUDOCODE.md`, the fourth level of the chain.

**Open items carried forward:**

- The §8.4 elliptic solution is numerically verified
  (`dev/spikes/free_top_elliptic.py`, passing). PSEUDOCODE and code for
  `analytic_solutions.py` must reproduce the forms that spike certifies.
- Two DESIGN forward-obligations for the code level: `numerical_inertia.py`
  must reproduce §3.2 to a stated tolerance (§3.7), and the format chosen
  for the scenario (§11.7) must satisfy all three constraints there.

---

## PSEUDOCODE

<!-- Tasks related to algorithm specifications. -->

---

## CODE

<!-- Tasks related to implementation. -->

---

## ARCHIVE

<!-- Resolved items go here. Keep them for reference; use stable numbering
for cross-references. -->
