"""Machine geometry taken from the STL files (docs/PLAN.md, section 2).

The meshes are read straight from `rpm_printables/` instead of a copy, so the hardware team's
files stay the single source and their licence attribution stays in one place.
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STL_DIR = REPO_ROOT / "rpm_printables"
