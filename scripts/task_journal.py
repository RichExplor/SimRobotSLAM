"""Robot-side task ledger, events and measured trajectory, independent of the browser."""

from __future__ import annotations

import json
import sqlite3
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

TERMINAL = {"completed", "cancelled", "failed", "interrupted"}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class TaskJournal:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / "tasks.sqlite3"
        with self.connect():
            pass

    @contextmanager
    def connect(self):
        connection = sqlite3.connect(self.path, timeout=10)
        connection.row_factory = sqlite3.Row
        try:
            connection.execute("CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, document TEXT NOT NULL)")
            connection.execute("CREATE TABLE IF NOT EXISTS deleted_tasks (id TEXT PRIMARY KEY)")
            connection.execute("CREATE TABLE IF NOT EXISTS events (task_id TEXT, seq INTEGER, document TEXT, "
                               "PRIMARY KEY(task_id, seq))")
            connection.execute("CREATE TABLE IF NOT EXISTS trace (task_id TEXT, seq INTEGER, document TEXT, "
                               "PRIMARY KEY(task_id, seq))")
            connection.execute("CREATE TABLE IF NOT EXISTS commands (task_id TEXT, id TEXT, action TEXT, "
                               "document TEXT, PRIMARY KEY(task_id, id))")
            with connection:
                yield connection
        finally:
            connection.close()

    def get(self, task_id, details=True):
        with self.connect() as connection:
            row = connection.execute("SELECT document FROM tasks WHERE id = ?", (task_id,)).fetchone()
            if row is None:
                raise KeyError(task_id)
            document = json.loads(row[0])
            if details:
                events = connection.execute("SELECT document FROM events WHERE task_id = ? "
                                            "ORDER BY json_extract(document, '$.time'), seq",
                                            (task_id,)).fetchall()
                document["timeline"] = [json.loads(event[0]) for event in events]
                trace = connection.execute("SELECT document FROM trace WHERE task_id = ? "
                                           "ORDER BY seq DESC LIMIT 4000", (task_id,)).fetchall()
                document["trace"] = [json.loads(point[0]) for point in reversed(trace)]
                document["trace_truncated"] = bool(document["trace"] and
                                                    document["trace"][0]["seq"] > 1)
        return document

    def list(self, pending_only=False):
        with self.connect() as connection:
            query = "SELECT document FROM tasks"
            if pending_only:
                query += " WHERE json_extract(document, '$.restored') = 0"
            rows = connection.execute(query).fetchall()
        return sorted((json.loads(row[0]) for row in rows),
                      key=lambda item: item["started_at"], reverse=True)

    def create(self, document):
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            if connection.execute("SELECT 1 FROM deleted_tasks WHERE id = ?", (document["id"],)).fetchone():
                raise ValueError("此任务 ID 已删除，不能重复执行")
            row = connection.execute("SELECT document FROM tasks WHERE id = ?", (document["id"],)).fetchone()
            if row:
                old = json.loads(row[0])
                if old["route"] != document["route"]:
                    raise ValueError("任务 ID 已用于其他配置")
                return old, False
            active = connection.execute(
                "SELECT 1 FROM tasks WHERE json_extract(document, '$.status') NOT IN "
                "('completed', 'cancelled', 'failed', 'interrupted') "
                "OR json_extract(document, '$.restored') = 0 LIMIT 1").fetchone()
            if active:
                raise ValueError("机器人已有任务或路线图尚未恢复")
            value = {**document, "gateway_revision": 1, "restored": False,
                     "updated_at": utc_now(), "timeline": [], "trace": []}
            connection.execute("INSERT INTO tasks VALUES (?, ?)",
                               (value["id"], json.dumps(value, ensure_ascii=False, allow_nan=False)))
        return value, True

    def update(self, task_id, values, events=(), pose=None):
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute("SELECT document FROM tasks WHERE id = ?", (task_id,)).fetchone()
            if row is None:
                raise KeyError(task_id)
            document = json.loads(row[0])
            if values.get("event_seq", document.get("event_seq", 0)) < document.get("event_seq", 0):
                return document
            now = utc_now()
            for event in events:
                enriched = {**event, "time": event.get("time", now),
                            "title": event.get("detail", event.get("kind", "任务事件"))}
                connection.execute("INSERT OR IGNORE INTO events VALUES (?, ?, ?)",
                                   (task_id, event["seq"], json.dumps(enriched, ensure_ascii=False)))
            if pose is not None:
                seq = document.get("trace_seq", 0) + 1
                point = {**pose, "seq": seq, "time": now}
                connection.execute("INSERT INTO trace VALUES (?, ?, ?)",
                                   (task_id, seq, json.dumps(point, allow_nan=False)))
                document["trace_seq"] = seq
            document.update(values)
            if document.get("recovery_interrupted"):
                document["status"] = "interrupted"
            document["gateway_revision"] += 1
            document["updated_at"] = now
            document["duration_seconds"] = max(0, (datetime.fromisoformat(
                document.get("finished_at", now)) - datetime.fromisoformat(document["started_at"])).total_seconds())
            if document["status"] in TERMINAL:
                document.setdefault("finished_at", now)
            connection.execute("UPDATE tasks SET document = ? WHERE id = ?",
                               (json.dumps(document, ensure_ascii=False, allow_nan=False), task_id))
        return document

    def reserve_cancel(self, document):
        """Prevent a delayed start request from executing an unconfirmed dispatch."""
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            task_id = document["id"]
            if connection.execute("SELECT 1 FROM deleted_tasks WHERE id = ?", (task_id,)).fetchone():
                raise ValueError("任务已删除")
            row = connection.execute("SELECT document FROM tasks WHERE id = ?", (task_id,)).fetchone()
            if row:
                return json.loads(row[0])
            now = utc_now()
            value = {**document, "status": "cancelled", "restored": True,
                     "gateway_revision": 1, "updated_at": now, "finished_at": now,
                     "timeline": [], "trace": [], "detail": "已撤销未确认的下发请求"}
            connection.execute("INSERT INTO tasks VALUES (?, ?)",
                               (task_id, json.dumps(value, ensure_ascii=False, allow_nan=False)))
        return value

    def command(self, task_id, command_id, action):
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute("SELECT action, document FROM commands WHERE task_id = ? AND id = ?",
                                     (task_id, command_id)).fetchone()
            if row:
                if row["action"] != action:
                    raise ValueError("命令 ID 已用于其他操作")
                return json.loads(row["document"]), False
            value = {"pending": True, "command_id": command_id}
            connection.execute("INSERT INTO commands VALUES (?, ?, ?, ?)",
                               (task_id, command_id, action, json.dumps(value)))
        return value, True

    def finish_command(self, task_id, command_id, value):
        with self.connect() as connection:
            connection.execute("UPDATE commands SET document = ? WHERE task_id = ? AND id = ?",
                               (json.dumps(value, ensure_ascii=False), task_id, command_id))

    def delete(self, task_id):
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute("SELECT document FROM tasks WHERE id = ?", (task_id,)).fetchone()
            if row is None:
                return
            document = json.loads(row[0])
            if document["status"] not in TERMINAL or not document.get("restored"):
                raise ValueError("未结束或路线图未恢复的任务不能删除")
            for table in ("tasks", "events", "trace", "commands"):
                key = "id" if table == "tasks" else "task_id"
                connection.execute(f"DELETE FROM {table} WHERE {key} = ?", (task_id,))
            connection.execute("INSERT OR IGNORE INTO deleted_tasks VALUES (?)", (task_id,))
