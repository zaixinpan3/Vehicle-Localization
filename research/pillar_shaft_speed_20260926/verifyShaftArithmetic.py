#!/usr/bin/env python3
"""Compare optimized shaft arithmetic with the committed pre-optimization kernel.

The baseline headers are read with git show, never vendored. Only the pure
arithmetic definitions preceding each MEX entry point are compiled; mwSize is
replaced by std::size_t. The generated translation unit and executable live in
a temporary directory. This check does not exercise MEX validation or threads.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


BASELINE = "35cdb88388300ba1b8bb215905435bde670dff03"
HEADER = "perception/native/pillarShaftKernel.hpp"
HELPERS = "perception/native/poleSubsetKernel.hpp"
PRELUDE = """#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>
using mwSize=std::size_t;
"""


def arithmetic_prefix(source):
    """Exclude MEX argument handling and close the arithmetic namespace."""
    marker = "void run("
    if source.count(marker) != 1:
        raise ValueError("Expected one MEX entry point in the kernel header")
    return source.split(marker, 1)[0] + "}\n"


def main():
    here = Path(__file__).resolve().parent
    root = here.parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--seed", type=int, default=82609126)
    parser.add_argument("--scenes", type=int, default=5000)
    parser.add_argument("--output", type=Path, default=here / "arithmetic_validation.json")
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64 or args.scenes < 1:
        parser.error("seed must fit uint64 and scenes must be positive")

    def historical(path):
        return subprocess.run(
            ["git", "show", f"{BASELINE}:{path}"], cwd=root,
            check=True, text=True, capture_output=True,
        ).stdout

    baseline = historical(HEADER)
    helpers = historical(HELPERS)
    if (root / HELPERS).read_text() != helpers:
        raise ValueError("Shared helpers changed; this fixture requires separate helper namespaces")
    current = (root / HEADER).read_text()
    fixture = (here / "shaftArithmeticFixture.cpp").read_text()
    old_prefix = arithmetic_prefix(baseline).replace(
        "namespace pillar_shaft {", "namespace old_shaft {", 1,
    )
    source = (PRELUDE + arithmetic_prefix(helpers) + old_prefix
              + arithmetic_prefix(current) + fixture)
    compiler_version = subprocess.run(
        [args.compiler, "--version"], check=True, text=True, capture_output=True,
    ).stdout.splitlines()[0]
    with tempfile.TemporaryDirectory(prefix="shaft-arithmetic-") as temporary:
        folder = Path(temporary)
        cpp = folder / "verify.cpp"
        executable = folder / "verify"
        cpp.write_text(source)
        subprocess.run(
            [args.compiler, "-O2", "-std=c++17", str(cpp), "-o", str(executable)],
            check=True,
        )
        completed = subprocess.run(
            [str(executable), str(args.seed), str(args.scenes)],
            check=True, text=True, stdout=subprocess.PIPE,
        )
    result = json.loads(completed.stdout)
    result.update({
        "baseline_commit": BASELINE,
        "compiler": compiler_version,
        "compiler_options": ["-O2", "-std=c++17"],
        "comparison": "Exact numeric equality with corresponding NaNs accepted",
        "coverage": ["variable point counts", "repeated heights", "constant heights",
                     "unowned interval endpoints", "unsorted radii"],
        "baseline_header_sha256": hashlib.sha256(baseline.encode()).hexdigest(),
        "baseline_helpers_sha256": hashlib.sha256(helpers.encode()).hexdigest(),
        "optimized_header_sha256": hashlib.sha256(current.encode()).hexdigest(),
        "generated_source_sha256": hashlib.sha256(source.encode()).hexdigest(),
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
