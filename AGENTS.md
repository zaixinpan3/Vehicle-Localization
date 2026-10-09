# Repository Guidelines

## Project Structure

MATLAB implementation of *Robust Vehicle Localization Fusing Road Curb,
Pole-like Feature, and Building Facade*. Three modules mirror the paper:

- `perception/` — one frame in, feature masks out (`perceiveFrame.m`):
  voxelization, ground segmentation, ground features (curbs, road surface,
  markings), off-ground features (poles, facades, traffic signs), and the
  semantic voxel product.
- `mapping/` — semantic NDT grid map, temporal-stability GMM map, sliding
  window map, frame-pose matching, and global registration.
- `localization/` — cascaded lateral-velocity and seven-state global observer
  design, delayed-pose replay, and simulation.
- `config/` — one parameter entry point per stage.
- `tests/` — unit tests plus `pipelineRegressionTest.m` against the stored
  reference `tests/reference/pipelineReference.mat`.
- `scripts/` — bag extraction and map-building entry points.

## Literature Sources

The local Zotero library is a valuable source of papers for this project.
When looking for papers or related research, it can be searched using the
available Zotero tools and skills for relevant bibliographic records, attached
PDFs, and notes.

## Default Vehicle Configuration

All new simulations and experiment scripts default to the adopted MnCAV
parameters in `config/mncavVehicleParameters.json`, loaded through
`mncavVehicleConfig` / `lateralObserverConfig`. Replay steering uses
`config/mncavReplayInterface.json` via `mncavReplayConfig` (Python:
`scripts/mncavParameters.py`). Do not copy nominal constants or use historical
output snapshots as live defaults. Re-synthesize gains for changed vehicles;
reject stale replay metadata. The old vehicle profiles and saved lateral gains
are removed; tests synthesize from current configuration. Sensitivity models
require explicit overrides. See `config/MNCAV_CONFIGURATION.md`.

## Documentation language

Write every document in this repository in English — Markdown, LaTeX, archive
records, code comments, and help headers alike. This holds regardless of the
language the request was made in.

## OPT Self-Employment Evidence Archiving

These rules apply only to substantive engineering or academic research progress
in this repository or its related research materials. Do not archive every
conversation, task, command, or file edit.

### Archiving Eligibility

Archive work only when it produces an identifiable implementation, technical
decision, research finding, or experimental result, for example:

- Actual code implementation, fixes, refactoring, or configuration changes that
  advance project behavior, correctness, performance, or maintainability.
- Algorithm, estimator, controller, or model design and improvements, including
  concrete derivations, assumptions, comparisons, and design decisions.
- Academic research such as literature analysis with project-relevant findings,
  theoretical analysis, or substantive manuscript, figure, and research-report
  development.
- Experiments, simulations, tests, and data processing or analysis that produce
  meaningful validation, measurements, findings, or reusable research outputs.

Do not trigger archiving for ordinary questions and explanations, status
updates, navigation, file searches, routine inspections or repeated checks,
general brainstorming or scheduling without a concrete technical result,
environment or tool administration, agent-instruction edits, cosmetic-only
changes, or archive housekeeping. A command run, file changed, or commit made
does not by itself establish substantive project progress. Research reading or
discussion qualifies when it yields an identifiable analysis, derivation,
finding, or design decision; a source-code change is not required.

Partial or failed engineering and research attempts qualify when they produce
meaningful implementation progress, diagnostic evidence, or a research finding.
Record their actual outcome without claiming success. For mixed tasks, archive
only the qualifying work and combine closely related steps into one record.

This eligibility gate governs all ledger, weekly/monthly report, Evidence Index,
and archive-related commit/push/completion requirements below. If no work
qualifies, skip that workflow and its final-response checklist. Explicitly
requested archive maintenance or required integrity corrections may still be
performed, but must not be presented as new engineering or research progress
or recursively generate records merely for maintaining records.

The purpose of the archive is to preserve truthful, contemporaneous, and
verifiable evidence of work. It is not permission to manufacture evidence,
backdate a record, invent a daily distribution of hours, or make a legal
conclusion about immigration compliance.

### Project Identity

- This repository is the **Vehicle Localization** project, filed in the archive
  as Project A (LiDAR).
- Ledger `project` field: `Pan Dynamics Vehicle Localization Research`.
- Ledger `record_id` prefix: `PDVL-YYYYMMDD-NNN`, numbered per activity date.
- The sibling collision-avoidance repository uses
  `Pan Dynamics Collision Avoidance Research` and the `PDCA-` prefix. Both
  projects share one archive root, one ledger, one Evidence Index, and one
  weekly work-log series. Never fork a parallel archive for this project.

### Fixed Context

- The SEVP Portal identifies `Zaixin Pan` as self-employed.
- `Pan Dynamics Research LLC` is the legal entity through which the documented
  internal research and development project is conducted.
- The documented self-employment start date is `2026-07-12`.
- The user has provided standing confirmation that each **active project**
  work week is exactly `20.0` actual hours. The weekly total is therefore
  `20.0` multiplied by the number of projects active that week. Project B was
  already active, and Project A joined it in week `2026-W36`
  (`08/30/2026`--`09/05/2026`); both projects are active together from that
  week. The standing weekly total from `2026-W36` is therefore `40.0`:
  `20.0` for Project A and `20.0` for Project B, entered as separate rows of
  the same weekly report. Weeks before `2026-W36` had one active project and
  stand at `20.0`; do not restate them.
- The signed `2026-07-27` Weekly Hours and Work Pattern Statement describes a
  `20`-hour regular schedule and certifies Week 1 and Week 2 only. It is a
  signed record: do not edit it. A later change to the schedule is recorded as
  a new dated fact, and a superseding statement is written only when the user
  asks for one.
- This standing confirmation does not authorize an invented daily distribution,
  an invented split of a project's `20.0` hours among its own operations, or a
  signature.
- The degree field is Mechanical Engineering.
- This is ordinary post-completion OPT, not a STEM OPT extension. Do not create
  Form I-983 records or apply STEM OPT employer rules unless the user's status
  actually changes and the user provides supporting documentation.
- Do not describe Pan Dynamics Research LLC work as occurring before
  `2026-07-12`, and do not silently change the SEVP employer identity. If an
  official record changes, record the new fact and its source without rewriting
  earlier records.

### Canonical Archive

Use `../Pan_Dynamics_OPT_Archive_Kit/` as the archive root.

- The archive root is deliberately **not** under version control. The user
  decided on `2026-09-03` that archive records are written directly in place.
  Do not initialize a repository there, do not create an archive commit, and do
  not record the absence of one as a blocker or a `partial` outcome.
- Maintain
  `../Pan_Dynamics_OPT_Archive_Kit/00_Project_Activity_Ledger.jsonl` as the
  append-only, machine-readable activity ledger. Append-only discipline is
  enforced by the writing procedure, not by Git: never rewrite or delete an
  existing line, and correct an earlier record only with a new entry carrying a
  `correction_of` reference.
- Use the existing current weekly and monthly report templates. Do not create a
  competing report series or evidence index when a current one already exists.
- Treat
  `../Pan_Dynamics_OPT_Archive_Kit/05_Evidence_Response_Packet_Index.csv` as the
  canonical EV-number index unless a later archive status file expressly names
  a replacement.
- Keep operational code and configurations in their normal repository
  locations. Put evidentiary manifests, report exports, and archive copies
  under the archive root. Archive copies hold text, tables, figures, PDFs and
  videos only; generated numerical data is never copied into the archive root
  (see Research Data Retention).
- Preserve original evidence. An export or copy must be identified as an export
  or copy and must not replace the original.
- All editable archive documents use standalone LaTeX (`.tex`) as the canonical
  source format. Do not create or update DOCX files.

### Git Commit, Push, and Commit-Record Archiving

The user has given standing instruction that completion of the archive workflow
includes committing the current in-scope project work, **pushing it to the
GitHub remote**, and archiving the resulting Git commit record. A substantive
task is not complete until this commit-push-and-record closure has been
performed.

Git closure applies to this repository only. The archive root is not versioned
and takes no commit of its own.

Git closure applies to **code** only: production source, configuration, tests
and entry-point scripts. Research output is not code and is not committed or
pushed (user decision, `2026-10-01`). Research output means a dated study
folder under `research/` with its report, tables, figures and diagnostic
scripts. Keep such a study as local files. A task that produces only research
output takes no project commit and no push: skip steps 1--3 below, and in
step 4 identify the work by its study path and by hashes of its inputs and
artifacts instead of a commit SHA. A task that changes code and also produces
a study commits and pushes the code and leaves the study untracked. This rule
does not by itself remove study folders that are already tracked.

The GitHub remote is `origin`
(`https://github.com/zaixinpan3/Vehicle-Localization`), default branch `main`.

1. Inspect the final diff and run the checks appropriate to the change. Commit
   the in-scope project artifacts with a concise imperative subject. Stage
   deliberately: preserve unrelated user changes, and exclude external
   dependencies, vendored trees, generated binaries, recorded datasets, and
   nested repositories unless the user explicitly places them in scope.
2. Push the commit to `origin main`. Standing instruction of `2026-09-03`
   supersedes the earlier "do not push unless the user asks" rule for this
   repository. Confirm the push landed rather than assuming it.
3. Capture the full commit SHA, subject, committed file scope or diffstat, the
   push result, and the checks actually run and their outcomes.
4. Append the technical operation to the activity ledger and update the current
   weekly log, writing both directly in the unversioned archive root. Include
   the full project commit SHA in the ledger's `versions_or_hashes` field and
   describe its scope, validation, and deliberate exclusions. The weekly log
   may cite that Git commit SHA, or hashes of other independent technical
   artifacts, when they materially support the reported work. Never calculate,
   cite, or record a hash of the weekly report source or PDF itself.

If a substantive task produces no project artifact, do not invent one merely to
create a project commit; write only the truthful archive update.

Edits limited to `AGENTS.md`, `CLAUDE.md`, or other agent-operation instruction
files are administrative agent configuration, not project research activity.
Commit those instruction changes when requested, but do not add them to the OPT
activity ledger, weekly or monthly work records, or Evidence Index unless the
user explicitly identifies a separate underlying project operation that must be
archived.

### Research Data Retention

Generated numerical data is not retained (user decision, `2026-10-08`). It
has no evidentiary value for the archive and dominates disk use: on that date
the research tree held 9.1 GB of which under 0.7 GB was text, and 9.8 GB of
data files and code snapshots were deleted from the repository and the archive
copies (ledger record `PDVL-20261008-015`).

- Generated data means replay and simulation outputs, cached or extracted
  features, point-cloud and frame dumps, intermediate arrays and similar
  machine products, whatever the format: `.mat`, `.npz`, `.npy`, `.bin`,
  `.pkl`, `.h5`, and `.json` or `.csv` files that serve only as caches.
- A study folder under `research/` keeps only what a reader needs to follow
  the finding: the report, small summary tables, figures, the scripts and
  configuration that produced them, and a manifest listing those files. Write
  bulk data to `output/` or a temporary location and delete it when the task
  ends; do not leave it in the study folder for later cleanup.
- Do not copy source code into a study folder (`source_snapshot`,
  `baseline_source`, `development_cache` or similar). Record the executed Git
  commit SHA in the report and the ledger instead; the code is recoverable
  from the repository.
- Do not copy generated data into the archive root. Export copies under
  `04_Project_A_LiDAR/` contain text, tables, figures, PDFs and videos only.
- Ledger and weekly entries record the metrics, the study path, the commit SHA
  and hashes of retained text artifacts. Do not hash or cite by path a data
  file that will not be kept, and list only retained files in a study's
  manifest.
- Data files absent from studies dated before `2026-10-08` were removed under
  the decision above. Their absence is explained by `PDVL-20261008-015` and is
  not a blocker, an integrity failure, or a reason to recreate them.
- Raw recordings, reference datasets and the regression reference
  `tests/reference/pipelineReference.mat` are inputs, not generated data, and
  are unaffected by this rule.

### Record Qualifying Work

For each coherent unit of work that meets Archiving Eligibility, append one
JSON object on one line to the activity ledger. Group implementation,
investigation, and validation steps serving the same engineering or research
purpose into one record; do not create a record per command or minor edit.
Describe the concrete progress or finding and its supporting artifacts or
methods. Do not append a duplicate record at task completion for work already
recorded.

Each ledger object must contain:

```json
{
  "record_id": "PDVL-YYYYMMDD-NNN",
  "recorded_at": "ISO-8601 timestamp with America/Chicago offset",
  "activity_date": "YYYY-MM-DD",
  "project": "Pan Dynamics Vehicle Localization Research",
  "operation_type": "literature|design|code|simulation|test|data|manuscript|archive",
  "summary": "specific factual description",
  "affected_files": ["repository-relative path"],
  "commands_or_methods": ["command or method actually used"],
  "outputs": ["repository-relative path"],
  "evidence_ids": ["EV-####"],
  "versions_or_hashes": ["Git commit, file version, or SHA-256 if available"],
  "outcome": "completed|partial|failed",
  "actual_user_hours": null,
  "notes": ""
}
```

- Record the technical operation as project activity without assigning or
  distinguishing authorship between Zaixin Pan and the coding agent.
- Never convert agent runtime, terminal elapsed time, file timestamps, commit
  timestamps, or an estimated task duration into the user's OPT work hours.
- Leave per-operation `actual_user_hours` as `null` unless the user supplies
  operation-specific time. Do not divide or allocate a project's standing
  weekly `20.0` hours among its individual operations.
- Earlier ledger entries are append-only. Correct an error with a new entry
  containing a `correction_of` reference; do not silently rewrite history.

### Technical Detail Required

Use concrete descriptions that another person can verify:

- Literature work: record the paper title, authors or DOI when available,
  sections or equations examined, and the resulting design decision or open
  question.
- Design work: record the estimator, map representation, feature extractor,
  interface, assumptions, equations, and design decision affected.
- Code work: record the files and functions changed, implemented behavior, and
  full Git commit SHA and file hash when available. Follow the mandatory
  commit-push-and-record workflow above.
- Simulation or test work: record the script or test, configuration, scenario,
  frame or sequence identifiers, parameters and random seed, command actually
  run, result location, key metrics, and whether it passed, failed, or was
  inconclusive.
- Data work: record the input bag or sequence, processing method, units, and
  metrics such as map size, point count, registration residual, or computation
  time when applicable. Name output data files only if they are retained
  (see Research Data Retention); otherwise record the metrics and the script
  that regenerates them.
- Manuscript work: record the section, equation, figure, table, or claim
  changed and the source or PDF version produced.
- Failed and partial work must be recorded honestly. Do not describe an
  unexecuted simulation or test as completed.

### Weekly and Monthly Records

- Write every weekly report entirely in English as a standalone LaTeX source
  and compile a matching PDF with the same base name in the same weekly
  directory. A weekly report is not complete as an archive artifact until both
  the `.tex` source and readable `.pdf` exist.
- One weekly report covers the person, not a project. Both active projects are
  recorded in the same report, each as its own row of the hours-reconciliation
  table and its own rows of the work table. Do not open a second weekly series
  for Project A. Name new weekly reports without a single-project label, and
  keep the hours token in the file name equal to the weekly total the report
  actually states. Rename an existing report only to correct a name that has
  become factually wrong, and record any such rename in the activity ledger.
- Add substantive activity to the current unsigned weekly work log as the work
  occurs. The entry must identify the specific work, actual output, and related
  ledger record.
- Weekly and monthly records are internal activity summaries, not evidence
  items. Do not assign them Evidence IDs, add them to an evidence index or
  evidence manifest, or calculate, store, refresh, or list hashes for their
  source or PDF files. A report must never cite its own hash as evidence.
- A weekly report may cite Git commit SHAs and hashes of independent source
  files, datasets, tests, papers, videos, and other technical artifacts. Keep
  the distinction explicit: those hashes identify supporting artifacts, not
  the report itself and not the user's hours.
- If a weekly report is written after the covered week, label it explicitly as
  a retrospective late entry. State the actual preparation date, the factual
  reason for the delay, the evidence limitations, and whether hours remain
  pending. Never backdate it. For Week 07, the recorded reason is that after
  moving, the desktop computer storing the weekly-report archive was
  temporarily unavailable for work; work continued from a laptop without that
  archive, preventing timely recordkeeping until archive access was restored.
  Week 07 is not a format exception: it requires the same English standalone
  `.tex` source and matching readable PDF as every other weekly report.
- Enter the standing confirmed weekly total automatically: `20.0` hours for
  each project active that week, so `40.0` for a week in which both projects
  are active. Record it as one row per project plus a total. Flexible daily
  distribution is acceptable; leave the daily split unspecified unless the user
  supplies the actual figures.
- Repository history and technical artifacts may support the nature and
  continuity of the work, but they do not independently prove how many hours
  the user worked.
- At weekly close, reconcile the weekly report with the activity ledger and
  actual artifacts. The standing per-project `20.0`-hour confirmation does not
  need to be requested again. If the number of active projects changes, ask the
  user for the new weekly total rather than deriving it.
- Build monthly summaries from the completed weekly records. Do not introduce
  new hours or activities during monthly aggregation.
- On `2026-07-27`, the user granted standing authorization to electronically
  sign completed internal OPT archive records using `/s/ Zaixin Pan`. This
  authorization covers weekly work logs, monthly summaries, internal project
  reports, evidence manifests, and internal archive certifications generated
  from the factual activity ledger and actual artifacts.
- Use the actual America/Chicago calendar date on which the signature is
  applied. Never backdate a signature.
- Before applying the authorized signature, the document must contain no blank
  substantive fields, placeholders, contradictory hours, pending-certification
  language, unsupported activities, or unresolved Evidence IDs.
- Do not apply this standing signature authorization to government forms,
  submissions to SEVP, USCIS, a DSO, or another agency, tax filings, contracts,
  banking documents, immigration filings, intellectual-property assignments,
  notarized documents, sworn statements, statements made under penalty of
  perjury, or third-party documents. Those require separate document-specific
  authorization.
- The user may revoke or narrow this standing authorization at any time.

### Evidence IDs and Integrity

- Assign an Evidence ID only when an actual file or defined evidence bundle is
  intentionally admitted into the archive. Minor edits may remain referenced
  by path in the ledger.
- Before using an EV number, consult the canonical index. Never guess or reuse
  an Evidence ID.
- Every EV entry must identify the file name, evidence date, archive location,
  concise description, project, applicable week, whether it is an original or
  export copy, and a Git hash, version, or SHA-256 when applicable.
- Weekly and monthly reports are excluded from EV admission and hashing. If a
  legacy index still contains a report row, treat its hash field as not
  applicable and do not refresh, cite, or propagate the historical report
  hash. Correct current indexes without rewriting append-only ledger history.
- An EV number must resolve to an existing file. A blank template is not
  evidence.
- Do not alter file metadata or Git history to make evidence appear older.
- Do not fabricate customers, revenue, bank activity, subscriptions, expenses,
  purchases, licenses, contracts, proposals, invoices, correspondence,
  simulation runs, measurements, or third-party validation.
- If no financial transaction or applicable license exists, record no such
  evidence.

### Privacy and Task Completion

- Never archive passwords, API keys, authentication cookies, private keys, or
  unrelated personal data. Use a redacted reference when an evidentiary record
  contains sensitive identifiers.
- Do not copy EAD, passport, I-20, SEVIS, tax, or banking documents into this
  repository. This repository has a public GitHub remote; immigration and
  personal documents belong only in the archive root, which is not published.
- At the end of a task with qualifying engineering or research progress, ensure
  that the activity ledger and current unsigned weekly log capture that work.
  Update the evidence index only if new evidence was admitted. Tasks that do
  not meet Archiving Eligibility require no archive update.
- Complete the closure in this order: project commit, push to `origin`, then
  the archival record of that commit. Research-only work has no commit or push
  and goes straight to the archival record. Do not leave substantive in-scope
  code changes uncommitted unless a concrete blocker is recorded and reported.
- In the final response, state which archive records were updated, whether the
  applicable standing total was applied (`20.0` per active project and `40.0`
  when both projects are active), and whether any document still requires
  review or signature.
- Report facts only. Do not state that the archive guarantees OPT compliance or
  that DHS, USCIS, SEVP, a DSO, or a court will accept a particular document.
