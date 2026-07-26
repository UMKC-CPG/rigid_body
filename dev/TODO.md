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

`DESIGN.md` is being drafted section by section. Status is tracked in its
own Contents table; this list only records what remains and any open
questions worth carrying forward.

**Sections still to write** (see the Contents table in `DESIGN.md`):

- [ ] §12 Trajectory retention — the history/replay buffer policy
      deferred from ARCHITECTURE §6.5 (ring buffer, downsampling, or
      spilling to the sink). §11.6 records the resolved retention limit;
      this section decides what that limit means and what overflow does.
      **This is the next section.**
- [ ] §13 Scene description and palettes — renderer-agnostic list of
      what to draw, and the labeled scale choices §9.2 and §12 defer here
      (VISION Principles 6 and 12).

**Open items carried forward:**

- Sections §1–§11 are drafted; §1–§10 are committed. The §8.4 elliptic
  solution is numerically verified (`dev/spikes/free_top_elliptic.py`,
  passing).
- After DESIGN is complete: tag `v0.3-design` (ARCHITECTURE §10), then
  begin `PSEUDOCODE.md`.

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
