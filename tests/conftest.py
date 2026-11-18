import sys
from pathlib import Path

# Make the repository root importable (for `import acsl_pychrono`)
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))
