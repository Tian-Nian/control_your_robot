"""Persistent request outcomes. Pending after restart means unknown, never retry."""
from contextlib import contextmanager
import json
from pathlib import Path
import sqlite3


class RequestStore:
    def __init__(self, path):
        self.path = str(Path(path).resolve())
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as db:
            db.execute("CREATE TABLE IF NOT EXISTS requests (scope TEXT, id TEXT, signature TEXT NOT NULL, result TEXT, acknowledged INTEGER NOT NULL DEFAULT 0, PRIMARY KEY(scope,id))")

    @contextmanager
    def _connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        try:
            with db:
                yield db
        finally:
            db.close()

    def lookup(self, scope, request_id, signature):
        with self._connect() as db:
            row = db.execute("SELECT signature,result FROM requests WHERE scope=? AND id=?", (scope, request_id)).fetchone()
        if row is None:
            return None
        if row[0] != signature:
            raise ValueError("request_id conflict")
        return json.loads(row[1]) if row[1] else {"request_id": request_id, "status": "unknown", "reason": "interrupted persistent request; inspect and recover, do not replay"}

    def begin(self, scope, request_id, signature):
        with self._connect() as db:
            db.execute("INSERT INTO requests(scope,id,signature) VALUES (?,?,?)", (scope, request_id, signature))

    def finish(self, scope, request_id, result):
        serialized = json.dumps(result, allow_nan=False)
        with self._connect() as db:
            db.execute("UPDATE requests SET result=?, acknowledged=? WHERE scope=? AND id=?",
                       (serialized, int(result.get("status") not in ("unknown", "failed", "cancelled")), scope, request_id))

    def unresolved(self, scope):
        with self._connect() as db:
            return [row[0] for row in db.execute("SELECT id FROM requests WHERE scope=? AND acknowledged=0", (scope,))]

    def acknowledge(self, scope):
        with self._connect() as db:
            db.execute("UPDATE requests SET acknowledged=1 WHERE scope=?", (scope,))
