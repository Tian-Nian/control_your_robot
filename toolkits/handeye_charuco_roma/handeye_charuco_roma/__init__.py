"""roma 工位 Charuco 手眼。核在 ../handeye_charuco；本包只 Observe + 离线 FK。"""

from .fk import MarvinOfflineFk
from .observe import FakeObserve, grab_pair, rgb_from_sample

__all__ = ["MarvinOfflineFk", "FakeObserve", "grab_pair", "rgb_from_sample"]
