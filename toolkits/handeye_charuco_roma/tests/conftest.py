from pathlib import Path
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT.parent / "handeye_charuco"
for p in (ROOT, CORE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

_helpers = CORE / "tests" / "helpers.py"
_spec = importlib.util.spec_from_file_location("handeye_charuco_test_helpers", _helpers)
helpers = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules["handeye_charuco_test_helpers"] = helpers
_spec.loader.exec_module(helpers)
