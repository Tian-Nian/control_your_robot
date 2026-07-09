"""Re-export example/deploy/real_env_client for XPolicyLab robodojo imports."""

from __future__ import annotations

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEPLOY = os.path.join(_ROOT, "example", "deploy")
for path in (_DEPLOY, os.path.join(_ROOT, "src"), _ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from real_env_client import *  # noqa: F403
