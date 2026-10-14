"""故意在 set_up 里失败，用来确认启动日志能留下 traceback。"""


class BrokenController:
    def __init__(self, **_kwargs):
        return None

    def set_up(self):
        raise RuntimeError("boom-setup")

    def set_collect_info(self, _info):
        return None

    def get(self):
        return {}
