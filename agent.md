# Agent Documentation Standard

This repository uses a strict bilingual documentation workflow.

## Canonical language (tracked in git)
- English is the canonical project documentation language.
- Canonical files keep original names (for example: `README.md`).
- Canonical docs MUST be committed and kept up to date.

## Local language copy (not tracked)
- Polish copies are local-only helper files for the owner.
- Polish docs MUST use `_PL` suffix (for example: `README_PL.md`).
- Files matching `*_PL.md` are ignored by git.

## Update policy for future agents
When changing documentation:
1. Update English canonical file first (original file name).
2. If a Polish local mirror exists, update the `_PL` version too.
3. Never rename canonical English docs to include language suffix.
4. Never commit `_PL` files.

## Minimum required documentation files
- `README.md` (English, tracked)
- Optional local mirror: `README_PL.md` (Polish, ignored)

## Quick examples
- New doc: create `docs/setup.md` in English (tracked)
- Optional local copy: `docs/setup_PL.md` in Polish (ignored)

## Public GitHub README checklist
Before public release, ensure:
- `README.md` is English and up to date.
- README includes: purpose, features, setup, configuration, run steps, and logging behavior.
- README includes relevant project keywords only (no unrelated SEO terms).
- Secrets are not committed (`include/secrets.h` stays ignored).
- License status is explicit in README, and a `LICENSE` file is added before public release.

## Adaptive deep sleep note
Adaptive deep sleep v1 is implemented and validated on hardware.

Operational summary:
- Cadence learning is based on mapped environmental updates.
- Planning uses median + MAD with confidence/fallback paths.
- Safety guards protect against fast empty-cycle sleep loops.
- Runtime behavior and tuning notes are maintained in `README.md`.

Detailed implementation spec (v1):
- `docs/adaptive-deepsleep-spec-v1.md`
