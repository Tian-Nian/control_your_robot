import uuid
from pathlib import Path

from robot.runtime.log import log_path
from robot.runtime.service import start, stop


def test_setup_failure_is_written_to_the_log(tmp_path, capsys):
    shm = f"cyr_{uuid.uuid4().hex[:12]}"
    config = tmp_path / "runtime.yml"
    config.write_text(
        f"""
name: broken
shm_name: {shm}
cmd_capacity: 8
components:
  - name: broken_arm
    class_path: robot.runtime.tests.broken_device
    class_name: BrokenController
    args: {{}}
    setup: {{}}
    publish: [joint]
    rate_hz: 10
    max_bytes: 256
""",
        encoding="utf-8",
    )
    try:
        assert start(config) == 1
    finally:
        stop()
    text = capsys.readouterr().err
    assert "启动失败" in text
    assert str(log_path()) in text
    log_text = Path(log_path()).read_text(encoding="utf-8")
    assert "boom-setup" in log_text
    assert "Traceback" in log_text
    assert "broken_arm" in log_text
