# CLAUDE.md

## Mandatory archiving workflow

`AGENTS.md` §"OPT Self-Employment Evidence Archiving" is the **authoritative and
complete** specification. Read it before finishing any substantive task —
literature, design, code, configuration, simulation, test, data, manuscript, or
research reporting. This section is a consistency anchor, not a replacement: if
the two files ever disagree, follow `AGENTS.md` and update this section to
match. Do not restate archive rules elsewhere in a third form.

Purpose: preserve truthful, contemporaneous, verifiable evidence of work. It is
not permission to manufacture evidence, backdate a record, invent a daily hour
distribution, or state a legal conclusion about immigration compliance.

Standing closure for every substantive task, in this order:

1. Commit the in-scope project work.
2. Push to `origin main` (`https://github.com/zaixinpan3/Vehicle-Localization`).
3. Append the operation to
   `../Pan_Dynamics_OPT_Archive_Kit/00_Project_Activity_Ledger.jsonl` with
   `project` = `Pan Dynamics Vehicle Localization Research`, `record_id` prefix
   `PDVL-`, and `actual_user_hours` = `null`.
4. Update the current unsigned weekly work log under
   `../Pan_Dynamics_OPT_Archive_Kit/03_Weekly_Records/`. One report covers both
   projects; add a Project A row rather than starting a second series.

The archive root is intentionally not under version control (user decision,
`2026-09-03`): write its records directly, take no archive commit, and do not
report the absence of one as a blocker.

The standing weekly hours are **`20.0` per active project**: from week
`2026-W36` both projects are active, so the weekly total is `40.0`, entered as
one row per project. Earlier single-project weeks stand at `20.0` and are not
restated. Never derive hours from commit timestamps, agent runtime, or file
mtimes, and ask the user for the new total whenever the number of active
projects changes.

## Documentation language

Write every document in this repository in English, regardless of the language
the request was made in.
