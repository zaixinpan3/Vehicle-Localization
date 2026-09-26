# Targeted literature search, 2026-09-25

Question: Which identification approaches can use the recorded MnCAV steering,
wheel/inertial and GNSS/INS channels while separating physical parameters from
sensor alignment and reference errors?

Include primary research on passenger-vehicle lateral model identification,
sideslip-free or joint parameter/state estimation, excitation/identifiability,
and sensor bias/bank handling. Retain seminal papers regardless of age and
recent methods where applicable. Exclude general road tests, Wikipedia,
unverified model-parameter tables, unrelated control papers and algorithms
requiring unavailable force sensors as immediate implementations.

This is a targeted search, without exhaustive database coverage, fixed hit-count
sampling, PRISMA screening totals or a claim of a complete literature census.

## Search paths

Web queries included:

- vehicle cornering stiffness identification yaw rate lateral acceleration unknown sideslip instrumental variable grey box
- Chrysler Pacifica vehicle dynamics parameter identification cornering stiffness yaw inertia MnCAV
- "MnCAV" "identification"
- "Chrysler Pacifica" "cornering" stiffness
- vehicle lateral dynamics identification instrumental variable yaw rate accelerometer bias relaxation length
- "Identification of physical parameters of a passenger car" author pdf
- "vehicle" "instrumental variable" "cornering stiffness" identification
- "A Beta-Less Approach" cornering stiffness Wittmer Henning Sawodny
- "New adaptive approaches to real-time estimation" Kim authors
- "Set-membership LPV model identification" pdf Cerone
- "A Beta-less Approach for Vehicle Cornering Stiffness Estimation" "sciencedirect"
- "Identification of vehicle parameters using stationary driving maneuvers" "authors"
- "10.1016/j.conengprac.2009.07.002" Kim

The initial author hint "Kim" was incorrect; DOI verification establishes
Seung-Han You, Jin-Oh Hahn and Hyeongcheol Lee. Search query hints are not
bibliographic evidence.

Local Zotero searches used `cornering` (limit 6) and `New adaptive approaches`
(limit 3). The first located item FJSAV2LP, Lu et al.; its metadata and abstract
were read. The second produced no exact match and unrelated semantic fallback
results, which were excluded. No items were imported, retagged or modified.

## Verified sources and reading depth

1. You, S.-H., Hahn, J.-O., & Lee, H. (2009). *New adaptive approaches to
   real-time estimation of vehicle sideslip angle*. Control Engineering
   Practice, 17(12), 1367–1379. DOI
   [10.1016/j.conengprac.2009.07.002](https://doi.org/10.1016/j.conengprac.2009.07.002).
   Read publisher-crawled abstract, introduction and basic-idea excerpts;
   direct publisher fetch returned an error. No full-paper reproduction.
2. Wesemeier, D., & Isermann, R. (2009). *Identification of vehicle parameters
   using stationary driving maneuvers*. Control Engineering Practice, 17(12),
   1426–1431. DOI [10.1016/j.conengprac.2008.10.008](https://doi.org/10.1016/j.conengprac.2008.10.008).
   Abstract-level method screening; direct publisher fetch returned 403.
3. Cerone, V., Piga, D., & Regruto, D. (2011). *Set-membership LPV model
   identification of vehicle lateral dynamics*. Automatica, 47(8), 1794–1799.
   DOI [10.1016/j.automatica.2011.04.016](https://doi.org/10.1016/j.automatica.2011.04.016).
   Publisher and author-repository abstract screening. The IMT Lucca PDF
   download failed DNS resolution; detailed optimization procedures not read.
4. Liao, Y.-W., & Borrelli, F. (2019). *An adaptive approach to real-time
   estimation of vehicle sideslip, road bank angles and sensor bias*.
   [arXiv:1905.08881](https://arxiv.org/abs/1905.08881), v1; PDF is marked an
   accepted IEEE Transactions on Vehicular Technology paper. Downloaded the
   13-page PDF; examined Sections II–III, equations 8–22 and Algorithms 1–2
   selectively, including bias/bank modeling and adaptation conditions.
5. Wittmer, K., Henning, K.-U., & Sawodny, O. (2023). *A Beta-less Approach for
   Vehicle Cornering Stiffness Estimation*. IFAC-PapersOnLine, 56(2), 11477–11482.
   DOI [10.1016/j.ifacol.2023.10.437](https://doi.org/10.1016/j.ifacol.2023.10.437).
   Publisher-crawled abstract; verified title/authors against the authors'
   [institution page](https://www.isys.uni-stuttgart.de/forschung/automotive/).
   Do not conflate this with the separate SMC 2023 paper on varying friction,
   DOI 10.1109/SMC53992.2023.10394015, which was discovered but not implemented.
6. Lu, J., Pan, W., Li, B., Cao, T., Qian, Z., Zhang, L., & Chen, H. (2026).
   *Dynamic Uncertainty-Adaptive Vehicle Lateral Velocity Estimation for
   Near-Limit Cornering With Tire Stiffness Compensation*. IEEE Transactions
   on Industrial Electronics, 73(2), 3269–3279. DOI
   [10.1109/TIE.2025.3610738](https://doi.org/10.1109/TIE.2025.3610738).
   Zotero item FJSAV2LP metadata/abstract read; IEEE direct page required browser
   verification. Full text and training data were not inspected.

All five DOI records were retrieved from the public Crossref API on the search
date and saved in `verified_bibliography.json`. The sixth source is verified
directly on arXiv. This verifies bibliographic identity, not every technical
claim or a separate retraction audit. No Semantic Scholar or OpenAlex check was
performed; no negative-match claim is made for those services.

The author-uploaded *Identification of physical parameters of a passenger car*
was discovered, but the search snippet's 2015 upload date conflicted with an
ECC'99 proceedings listing. It was not used as an independently verified main
bibliographic entry. Informal car reviews and unrelated semantic search
fallbacks were excluded.

## Limits and synthesis boundary

Most selected methods concern different vehicles. Four of the six sources
predate 2021; they are retained for foundational estimation structure, while
the 2023 and 2026 methods cover newer alternatives. Full-text access was uneven.
The method comparisons therefore distinguish abstracts from detailed reading.
The reduced-equation derivation, parameter-family result and MnCAV performance
tables are this project's own analysis and experiment, not findings copied
from the cited vehicles. No literature result establishes MnCAV parameter truth.
