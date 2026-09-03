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
   `../Pan_Dynamics_OPT_Archive_Kit/03_Weekly_Records/`.
5. Commit the archive changes as a separate archive commit.

The standing `20.0` weekly hours are **per person per week across both
projects**, never per project. Never derive hours from commit timestamps,
agent runtime, or file mtimes.

## Documentation language

Write every document in this repository in English, regardless of the language
the request was made in.
