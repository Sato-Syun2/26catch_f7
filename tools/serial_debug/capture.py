#!/usr/bin/env python3
"""STM32 のシリアルログを収集・要約し、ST-LINK 操作を補助する CLI。

保存形式は JSON Lines (JSONL) とし、1 行を 1 イベントとして扱う。標準出力も
JSON を選択できるため、AI エージェントや CI から安全に取り込める。
"""

from __future__ import annotations

import argparse
import collections
import contextlib
import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any, Iterable


DEFAULT_BAUDRATE = 115200
DEFAULT_ADDRESS = "0x08000000"
LEVEL_PATTERN = re.compile(r"\b(DEBUG|INFO|WARN(?:ING)?|ERROR|ERR|FATAL|PANIC)\b", re.I)
DEFAULT_PROGRAMMER = "STM32_Programmer_CLI"
DEFAULT_RULES = Path(__file__).with_name("events.yaml")


def utc_now() -> str:
    """UTC の ISO 8601 時刻をミリ秒精度で返す。"""
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


def emit(value: Any, as_json: bool) -> None:
    if as_json:
        print(json.dumps(value, ensure_ascii=False, separators=(",", ":")))
    elif isinstance(value, str):
        print(value)
    else:
        print(json.dumps(value, ensure_ascii=False, indent=2))


def normalized_level(message: str) -> str | None:
    match = LEVEL_PATTERN.search(message)
    if not match:
        return None
    level = match.group(1).upper()
    return {"WARNING": "WARN", "ERR": "ERROR"}.get(level, level)


def make_event(raw: bytes, port: str, baudrate: int, sequence: int) -> dict[str, Any]:
    text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
    return {
        "schema": "stm32.serial_log.v1",
        "timestamp": utc_now(),
        "monotonic_s": round(time.monotonic(), 6),
        "sequence": sequence,
        "source": {"port": port, "baudrate": baudrate},
        "message": text,
        "level": normalized_level(text),
        "raw_hex": raw.hex(),
    }


def load_detection_rules(path: Path) -> list[dict[str, Any]]:
    """events.yaml を読み、コンパイル済み正規表現を持つルールへ変換する。"""
    try:
        import yaml  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("PyYAML がありません。pip install -r tools/serial_debug/requirements.txt を実行してください。") from exc
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"異常検出ルールを読めません: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("events"), list):
        raise RuntimeError(f"{path}: events の配列が必要です")
    rules: list[dict[str, Any]] = []
    for index, item in enumerate(data["events"], start=1):
        if not isinstance(item, dict) or not isinstance(item.get("name"), str):
            raise RuntimeError(f"{path}: events[{index}] に name が必要です")
        patterns = item.get("patterns")
        if not isinstance(patterns, list) or not all(isinstance(pattern, str) for pattern in patterns):
            raise RuntimeError(f"{path}: events[{index}] に patterns の文字列配列が必要です")
        try:
            compiled = [re.compile(pattern, re.IGNORECASE) for pattern in patterns]
        except re.error as exc:
            raise RuntimeError(f"{path}: events[{index}] の正規表現が不正です: {exc}") from exc
        rules.append({"name": item["name"], "severity": item.get("severity", "error"), "patterns": compiled})
    return rules


def detect_anomalies(event: dict[str, Any], rules: list[dict[str, Any]]) -> list[dict[str, str]]:
    """ログレベルと YAML ルールから、AI が判断可能な異常ラベルを生成する。"""
    detections: list[dict[str, str]] = []
    if event.get("level") in {"WARN", "ERROR", "FATAL", "PANIC"}:
        detections.append({"name": f"log_{event['level'].lower()}", "severity": event["level"].lower()})
    for rule in rules:
        if any(pattern.search(event["message"]) for pattern in rule["patterns"]):
            detections.append({"name": rule["name"], "severity": str(rule["severity"])})
    return detections


def open_serial(port: str, baudrate: int, timeout: float):
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial がありません。pip install -r tools/serial_debug/requirements.txt を実行してください。") from exc
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def command_capture(args: argparse.Namespace) -> int:
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    captured = 0
    deadline = time.monotonic() + args.duration if args.duration else None

    try:
        with open_serial(args.port, args.baudrate, args.timeout) as device, destination.open("a", encoding="utf-8") as log:
            while deadline is None or time.monotonic() < deadline:
                raw = device.readline()
                if not raw:
                    continue
                captured += 1
                event = make_event(raw, args.port, args.baudrate, captured)
                log.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")
                log.flush()
                if args.echo:
                    emit(event if args.json else event["message"], args.json)
                if args.lines and captured >= args.lines:
                    break
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError) as exc:
        emit({"ok": False, "operation": "capture", "error": str(exc)}, True)
        return 2

    emit({"ok": True, "operation": "capture", "captured": captured, "output": str(destination)}, args.json)
    return 0


def reset_stlink(programmer: str) -> dict[str, Any]:
    """診断中に使う、標準出力を汚さない ST-LINK リセット。"""
    cli = programmer_path(programmer)
    completed = subprocess.run([cli, "-c", "port=SWD", "-rst"], check=False, capture_output=True, text=True, timeout=30)
    return {"requested": True, "ok": completed.returncode == 0, "returncode": completed.returncode, "stderr_tail": completed.stderr[-500:]}


def compact_event(event: dict[str, Any]) -> dict[str, Any]:
    """要約用に元バイト列などを省いた小さなイベント表現。"""
    return {key: event.get(key) for key in ("sequence", "timestamp", "level", "message", "detections") if event.get(key) is not None}


def command_diagnose(args: argparse.Namespace) -> int:
    """ポートを先に開き、必要ならリセット後に時間制限付きで診断する。"""
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        rules = load_detection_rules(Path(args.rules))
        device = open_serial(args.port, args.baudrate, args.timeout)
    except (OSError, RuntimeError) as exc:
        emit({"ok": False, "operation": "diagnose", "error": str(exc)}, True)
        return 2

    counts: collections.Counter[str] = collections.Counter()
    detection_counts: collections.Counter[str] = collections.Counter()
    contexts: list[dict[str, Any]] = []
    prior: collections.deque[dict[str, Any]] = collections.deque(maxlen=args.context_before)
    pending: list[dict[str, Any]] = []
    captured = stored = omitted = sequence = written_bytes = 0
    reset_result: dict[str, Any] = {"requested": False}
    started = utc_now()
    deadline = time.monotonic() + args.duration

    try:
        with contextlib.closing(device), destination.open("w", encoding="utf-8") as log:
            if args.reset:
                try:
                    reset_result = reset_stlink(args.programmer)
                except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                    reset_result = {"requested": True, "ok": False, "error": str(exc)}
            while time.monotonic() < deadline:
                raw = device.readline()
                if not raw:
                    continue
                sequence += 1
                captured += 1
                event = make_event(raw, args.port, args.baudrate, sequence)
                detections = detect_anomalies(event, rules)
                if detections:
                    event["detections"] = detections
                    for detection in detections:
                        detection_counts[detection["name"]] += 1
                counts[event.get("level") or "UNCLASSIFIED"] += 1
                encoded = json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n"
                encoded_bytes = len(encoded.encode("utf-8"))
                if stored < args.max_events and written_bytes + encoded_bytes <= args.max_bytes:
                    log.write(encoded)
                    stored += 1
                    written_bytes += encoded_bytes
                else:
                    omitted += 1

                compact = compact_event(event)
                for context in pending[:]:
                    context["after"].append(compact)
                    context["remaining"] -= 1
                    if context["remaining"] == 0:
                        del context["remaining"]
                        pending.remove(context)
                if detections and len(contexts) < args.max_anomalies:
                    context = {"event": compact, "before": list(prior), "after": [], "remaining": args.context_after}
                    contexts.append(context)
                    if args.context_after:
                        pending.append(context)
                prior.append(compact)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        emit({"ok": False, "operation": "diagnose", "error": str(exc)}, True)
        return 2

    for context in contexts:
        context.pop("remaining", None)
    has_abnormality = bool(contexts)
    summary = {
        "schema": "stm32.serial_diagnosis.v1",
        "ok": not has_abnormality and reset_result.get("ok", True),
        "operation": "diagnose",
        "time_range": {"first": started, "last": utc_now(), "duration_s": args.duration},
        "source": {"port": args.port, "baudrate": args.baudrate},
        "reset": reset_result,
        "log": {"path": str(destination), "captured": captured, "stored": stored, "omitted": omitted, "bytes": written_bytes},
        "levels": dict(sorted(counts.items())),
        "detection_counts": dict(sorted(detection_counts.items())),
        "anomalies": contexts,
    }
    emit(summary, True)
    return 1 if has_abnormality or not reset_result.get("ok", True) else 0


def read_events(path: Path) -> Iterable[dict[str, Any]]:
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: JSONL として読めません: {exc.msg}") from exc
            if isinstance(item, dict):
                yield item


def command_summarize(args: argparse.Namespace) -> int:
    path = Path(args.input)
    try:
        events = list(read_events(path))
    except (OSError, ValueError) as exc:
        emit({"ok": False, "operation": "summarize", "error": str(exc)}, True)
        return 2

    if args.tail:
        events = events[-args.tail :]
    levels = collections.Counter(event.get("level") or "UNCLASSIFIED" for event in events)
    failures = [event for event in events if event.get("level") in {"ERROR", "FATAL", "PANIC"}]
    summary = {
        "schema": "stm32.serial_summary.v1",
        "ok": not failures,
        "operation": "summarize",
        "input": str(path),
        "event_count": len(events),
        "time_range": {
            "first": events[0].get("timestamp") if events else None,
            "last": events[-1].get("timestamp") if events else None,
        },
        "levels": dict(sorted(levels.items())),
        "error_events": failures[-args.max_errors :],
        "last_events": events[-args.tail_events :],
    }
    emit(summary, args.json)
    return 0 if summary["ok"] else 1


def programmer_path(value: str) -> str:
    found = shutil.which(value)
    if found:
        return found
    candidate = Path(value)
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return str(candidate)
    raise RuntimeError(f"STM32CubeProgrammer CLI が見つかりません: {value}")


def run_programmer(args: list[str], as_json: bool, operation: str) -> int:
    try:
        completed = subprocess.run(args, check=False, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired, RuntimeError) as exc:
        emit({"ok": False, "operation": operation, "error": str(exc)}, True)
        return 2
    result = {
        "ok": completed.returncode == 0,
        "operation": operation,
        "command": args,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    emit(result, as_json)
    return completed.returncode


def command_flash(args: argparse.Namespace) -> int:
    firmware = Path(args.firmware)
    if not firmware.is_file():
        emit({"ok": False, "operation": "flash", "error": f"ファームウェアがありません: {firmware}"}, True)
        return 2
    try:
        cli = programmer_path(args.programmer)
    except RuntimeError as exc:
        emit({"ok": False, "operation": "flash", "error": str(exc)}, True)
        return 2
    command = [cli, "-c", "port=SWD", "-w", str(firmware), args.address, "-v"]
    if args.reset:
        command.append("-rst")
    return run_programmer(command, args.json, "flash")


def command_reset(args: argparse.Namespace) -> int:
    if args.method == "stlink":
        try:
            cli = programmer_path(args.programmer)
        except RuntimeError as exc:
            emit({"ok": False, "operation": "reset", "error": str(exc)}, True)
            return 2
        return run_programmer([cli, "-c", "port=SWD", "-rst"], args.json, "reset")

    try:
        with open_serial(args.port, args.baudrate, 1.0) as device:
            device.dtr = False
            device.rts = False
            time.sleep(args.pulse)
            device.dtr = True
            device.rts = True
    except (OSError, RuntimeError) as exc:
        emit({"ok": False, "operation": "reset", "error": str(exc)}, True)
        return 2
    emit({"ok": True, "operation": "reset", "method": "dtr-rts", "port": args.port}, args.json)
    return 0


def command_list_ports(args: argparse.Namespace) -> int:
    try:
        from serial.tools import list_ports  # type: ignore[import-not-found]
    except ImportError as exc:
        emit({"ok": False, "operation": "list-ports", "error": "pyserial がありません"}, True)
        return 2
    ports = [{"device": port.device, "description": port.description, "hwid": port.hwid} for port in list_ports.comports()]
    emit({"ok": True, "operation": "list-ports", "ports": ports}, args.json)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="機械可読 JSON を標準出力へ出力する")
    subparsers = parser.add_subparsers(dest="subcommand", required=True)

    capture = subparsers.add_parser("capture", help="シリアル出力を JSONL として記録する")
    capture.add_argument("--port", required=True)
    capture.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    capture.add_argument("--output", default="tools/serial_debug/logs/serial.jsonl")
    capture.add_argument("--duration", type=float, help="記録時間（秒）。省略時は Ctrl-C まで継続")
    capture.add_argument("--lines", type=int, help="最大記録行数")
    capture.add_argument("--timeout", type=float, default=0.5)
    capture.add_argument("--echo", action="store_true", help="受信行を標準出力にも表示する")
    capture.set_defaults(handler=command_capture)

    diagnose = subparsers.add_parser("diagnose", help="時間制限付きの収集・異常検出・要約を一度に実行する")
    diagnose.add_argument("--port", required=True)
    diagnose.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    diagnose.add_argument("--duration", type=float, default=20.0, help="収集時間（秒）")
    diagnose.add_argument("--output", default="tools/serial_debug/logs/diagnose.jsonl")
    diagnose.add_argument("--rules", default=str(DEFAULT_RULES), help="異常検出 YAML")
    diagnose.add_argument("--reset", action="store_true", help="ポートを開いた後に ST-LINK 経由でリセットする")
    diagnose.add_argument("--programmer", default=DEFAULT_PROGRAMMER)
    diagnose.add_argument("--timeout", type=float, default=0.2)
    diagnose.add_argument("--max-events", type=int, default=2000, help="JSONL に保存する最大イベント数")
    diagnose.add_argument("--max-bytes", type=int, default=2 * 1024 * 1024, help="JSONL の最大バイト数")
    diagnose.add_argument("--context-before", type=int, default=3, help="異常前の要約イベント数")
    diagnose.add_argument("--context-after", type=int, default=3, help="異常後の要約イベント数")
    diagnose.add_argument("--max-anomalies", type=int, default=5, help="要約する異常の最大件数")
    diagnose.set_defaults(handler=command_diagnose)

    summarize = subparsers.add_parser("summarize", help="JSONL を AI 向けに要約する")
    summarize.add_argument("input")
    summarize.add_argument("--tail", type=int, help="末尾 N 件だけを集計する")
    summarize.add_argument("--max-errors", type=int, default=20)
    summarize.add_argument("--tail-events", type=int, default=20)
    summarize.set_defaults(handler=command_summarize)

    flash = subparsers.add_parser("flash", help="ST-LINK 経由でファームウェアを書き込む")
    flash.add_argument("firmware", help=".elf / .hex / .bin ファイル")
    flash.add_argument("--address", default=DEFAULT_ADDRESS, help=".bin の書込先アドレス")
    flash.add_argument("--programmer", default=DEFAULT_PROGRAMMER, help="STM32CubeProgrammer CLI のパス又はコマンド名")
    flash.add_argument("--reset", action="store_true", help="書込み後にターゲットをリセットする")
    flash.set_defaults(handler=command_flash)

    reset = subparsers.add_parser("reset", help="ターゲットをリセットする")
    reset.add_argument("--method", choices=("stlink", "dtr-rts"), default="stlink")
    reset.add_argument("--programmer", default=DEFAULT_PROGRAMMER)
    reset.add_argument("--port", help="dtr-rts 使用時のシリアルポート")
    reset.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    reset.add_argument("--pulse", type=float, default=0.1, help="DTR/RTS の Low パルス時間（秒）")
    reset.set_defaults(handler=command_reset)

    list_ports = subparsers.add_parser("list-ports", help="利用可能なシリアルポートを列挙する")
    list_ports.set_defaults(handler=command_list_ports)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.subcommand == "reset" and args.method == "dtr-rts" and not args.port:
        parser.error("reset --method dtr-rts には --port が必要です")
    if args.subcommand == "diagnose":
        if args.duration <= 0 or args.max_events <= 0 or args.max_bytes <= 0:
            parser.error("diagnose の duration、max-events、max-bytes は正の値にしてください")
        if args.context_before < 0 or args.context_after < 0 or args.max_anomalies < 0:
            parser.error("diagnose の context-before、context-after、max-anomalies は 0 以上にしてください")
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
