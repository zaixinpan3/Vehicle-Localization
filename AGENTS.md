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
- `localization/` — replay high-gain observer design and simulation.
- `config/` — one parameter entry point per stage.
- `tests/` — unit tests plus `pipelineRegressionTest.m` against the stored
  reference `tests/reference/pipelineReference.mat`.
- `scripts/` — bag extraction and map-building entry points.

## Documentation language

Write every document in this repository in English — Markdown, LaTeX, archive
records, code comments, and help headers alike. This holds regardless of the
language the request was made in.

## OPT Self-Employment Evidence Archiving

These rules are mandatory for every substantive project task performed in this
repository or on its related research materials. They cover literature review,
algorithm and estimator design, source-code or configuration changes,
simulations and tests, data generation or analysis, manuscript work, and
research reporting.

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
  `20.0` multiplied by the number of projects active that week. Both projects
  became active together in week `2026-W36` (`08/30/2026`--`09/05/2026`), so
  from that week the standing weekly total is `40.0`: `20.0` for Project A and
  `20.0` for Project B, entered as separate rows of the same weekly report.
  Weeks before `2026-W36` had one active project and stand at `20.0`; do not
  restate them.
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
  `Pan_Dynamics_OPT_Archive_Kit/00_Project_Activity_Ledger.jsonl` as the
  append-only, machine-readable activity ledger. Append-only discipline is
  enforced by the writing procedure, not by Git: never rewrite or delete an
  existing line, and correct an earlier record only with a new entry carrying a
  `correction_of` reference.
- Use the existing current weekly and monthly report templates. Do not create a
  competing report series or evidence index when a current one already exists.
- Treat
  `Pan_Dynamics_OPT_Archive_Kit/05_Evidence_Response_Packet_Index.csv` as the
  canonical EV-number index unless a later archive status file expressly names
  a replacement.
- Keep operational code, configurations, and research data in their normal
  repository locations. Put evidentiary manifests, report exports, and archive
  copies under the archive root.
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
   describe its scope, validation, and deliberate exclusions. Do not add that
   hash to the weekly log.

If a substantive task produces no project artifact, do not invent one merely to
create a project commit; write only the truthful archive update.

Edits limited to `AGENTS.md`, `CLAUDE.md`, or other agent-operation instruction
files are administrative agent configuration, not project research activity.
Commit those instruction changes when requested, but do not add them to the OPT
activity ledger, weekly or monthly work records, or Evidence Index unless the
user explicitly identifies a separate underlying project operation that must be
archived.

### Record Every Substantive Operation

After each substantive operation, append one JSON object on one line to the
activity ledger. Closely coupled edits made for one engineering purpose may be
one operation. Routine navigation, file discovery, and no-op commands do not
need separate records unless they themselves constitute literature review or
produce a relevant research finding.

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
- Data work: record the input bag or sequence, processing method, output files,
  units, and metrics such as map size, point count, registration residual, or
  computation time when applicable.
- Manuscript work: record the section, equation, figure, table, or claim
  changed and the source or PDF version produced.
- Failed and partial work must be recorded honestly. Do not describe an
  unexecuted simulation or test as completed.

### Weekly and Monthly Records

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
  evidence manifest, or calculate or list hashes solely for those records.
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
- At the end of every substantive task, update the activity ledger and current
  unsigned weekly log. Update the evidence index only if new evidence was
  admitted.
- Complete the closure in this order: project commit, push to `origin`, then
  the archival record of that commit. Do not leave substantive in-scope changes
  uncommitted unless a concrete blocker is recorded and reported.
- In the final response, state which archive records were updated, whether the
  standing `20.0` weekly hours were applied, and whether any document still
  requires review or signature.
- Report facts only. Do not state that the archive guarantees OPT compliance or
  that DHS, USCIS, SEVP, a DSO, or a court will accept a particular document.
