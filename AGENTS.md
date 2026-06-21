# Agent Instructions

## Core
- ignore symlinked `CLAUDE.md` and `GEMINI.md`. use `AGENTS.md` only.
- Skills are canonical for tool workflows. Keep this file to hard rules only.
- Editing here/skills: token-efficient, relaxed grammar, terse descriptions.
- Build/deploy commands: `skills/build_deploy.md`. Design/architecture/implementation: `docs/design_notes.md`.
- no git `commit, rm, reset --hard, push, clean, restore` unless asked.

## Honesty
Say **"I do not know"** when uncertain — no guessing, no speculation. Partial info OK; label it clearly.

## Code Comments
Why, not what. 
comments on public types, variants, fields — describe semantics (format, range, source). 
1–2 lines max. Skip obvious code.

## outside files
May read but **NEVER** change files here:
- **esp-idf/** - external ESP SDK
