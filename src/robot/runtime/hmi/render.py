"""把 HmiModel 画成一帧文本。"""

from __future__ import annotations

from robot.runtime.hmi.model import HmiModel


def frame(model: HmiModel, width: int, height: int) -> list[str]:
    width = max(width, 20)
    height = max(height, 8)
    alive, errors = model.counts()
    if model.connected:
        header = f" cyr  pid={model.pid}  shm={model.shm_name}  up={alive}/{len(model.names)}  err={errors}"
    else:
        header = f" cyr  disconnected  shm={model.shm_name or '-'}  up={alive}/{len(model.names)}  err={errors}"
    if model.notice:
        header = f" cyr  {model.notice}  shm={model.shm_name or '-'}  pid={model.pid}"
    if model.confirming_stop:
        footer = " 关闭服务?  y 确认   n/Esc 取消"
    else:
        footer = " ↑↓/jk 选择  x 关闭服务  q 离开"
    columns = _fit(f"{'name':<16} {'hz':>7} {'age':>8} {'seq':>8} {'drop':>6}  error", width)
    body = height - 4
    list_height = max(1, body // 2)
    detail_height = max(1, body - list_height)
    rows = model.visible_names()
    start = 0
    if rows and model.selected >= list_height:
        start = model.selected - list_height + 1
    visible = rows[start : start + list_height]
    lines = [_fit(header, width), columns]
    if not visible:
        lines.append(_fit(" (无组件)", width))
    for offset, name in enumerate(visible):
        stat = model.stats.get(name)
        hz = 0.0 if stat is None else stat.hz
        age = "-" if stat is None or stat.age_ms is None else f"{stat.age_ms:.1f}"
        seq = 0 if stat is None else stat.state_seq
        drops = 0 if stat is None else stat.drops
        error = "" if stat is None else stat.error
        marker = ">" if start + offset == model.selected else " "
        lines.append(_fit(f"{marker}{name:<16} {hz:7.1f} {age:>8} {seq:8d} {drops:6d}  {error}", width))
    while len(lines) < 2 + list_height:
        lines.append("")
    selected = model.selected_name() or "-"
    stat = model.selected_stat()
    detail = [] if stat is None else stat.lines
    detail_rows = [_fit(f"── {selected}", width)]
    content_room = max(0, detail_height - 1)
    if not detail and content_room:
        detail_rows.append(_fit(" (无数据)", width))
    else:
        detail_rows.extend(_fit(" " + line, width) for line in detail[:content_room])
    while len(detail_rows) < detail_height:
        detail_rows.append("")
    lines.extend(detail_rows[:detail_height])
    while len(lines) < height - 1:
        lines.append("")
    lines.append(_fit(footer, width))
    return lines[:height]


def _fit(text: str, width: int) -> str:
    if len(text) <= width:
        return text
    if width <= 1:
        return text[:width]
    return text[: width - 1] + "…"
