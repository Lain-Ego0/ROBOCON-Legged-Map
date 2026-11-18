"""Bounded asynchronous bridge to the native GLIM CUDA process."""
from __future__ import annotations

import json
import os
import queue
import struct
import subprocess
import threading
from collections import deque
from pathlib import Path


class GLIMBridge:
    def __init__(self, binary: Path, config_dir: Path, library_dir: Path):
        self.binary = Path(binary)
        self.config_dir = Path(config_dir)
        self.library_dir = Path(library_dir)
        self.input: queue.Queue = queue.Queue(maxsize=16)
        self.lock = threading.Lock()
        self.poses = deque(maxlen=64)
        self.error = None
        self.dropped_scans = 0
        self.processed_scans = 0
        self.sync_warning_count = 0
        self.generation = 0
        self.process = None
        self.log_file = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, name="glim-bridge", daemon=True)
        self.thread.start()

    def submit(self, imu_samples, scan):
        if self.closed:
            return False
        try:
            self.input.put_nowait((imu_samples, scan))
            return True
        except queue.Full:
            self.dropped_scans += 1
            return False

    def reset(self):
        with self.lock:
            self.generation += 1
            generation = self.generation
        while True:
            try:
                self.input.get_nowait()
            except queue.Empty:
                break
        with self.lock:
            self.poses.clear()
            self.error = None
        try:
            self.input.put_nowait("reset")
        except queue.Full:
            pass
        return generation

    def drain_poses(self):
        with self.lock:
            result = list(self.poses)
            self.poses.clear()
        return result

    def status(self):
        with self.lock:
            return {
                "ready": self.process is not None and self.process.poll() is None,
                "error": self.error,
                "dropped_scans": self.dropped_scans,
                "processed_scans": self.processed_scans,
                "sync_warnings": self.sync_warning_count,
            }

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.input.put("stop", timeout=0.5)
        except queue.Full:
            pass
        self.thread.join(timeout=3.0)
        self._stop_process()

    def _start_process(self):
        if not self.binary.is_file():
            raise FileNotFoundError(f"GLIM bridge executable not found: {self.binary}")
        env = os.environ.copy()
        old_path = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = f"{self.library_dir}:{old_path}" if old_path else str(self.library_dir)
        log_path = Path(os.environ.get("GLIM_BRIDGE_LOG", "/tmp/robocon-glim-bridge.log"))
        self.log_file = log_path.open("w", encoding="utf-8")
        try:
            process = subprocess.Popen(
                [str(self.binary), str(self.config_dir), str(self.library_dir)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
                env=env,
            )
        except Exception:
            self.log_file.close()
            self.log_file = None
            raise
        self.process = process
        with self.lock:
            generation = self.generation
        threading.Thread(target=self._read_output, args=(process, generation), name="glim-pose-reader", daemon=True).start()
        self.stderr_thread = threading.Thread(
            target=self._read_stderr, args=(process, self.log_file), name="glim-log-reader", daemon=True
        )
        self.stderr_thread.start()

    def _read_stderr(self, process, log_file):
        warning = b"performing linearization in sync mode seriously affects the processing speed"
        count = 0
        assert process.stderr is not None
        for line in process.stderr:
            if warning in line:
                count += 1
            else:
                log_file.write(line.decode("utf-8", errors="replace"))
                log_file.flush()
        if count:
            with self.lock:
                self.sync_warning_count += count
            log_file.write(f"Suppressed repeated CUDA synchronous-linearization warnings: {count}\n")
            log_file.flush()

    def _stop_process(self):
        process, self.process = self.process, None
        if process is None:
            return
        if process.stdin:
            process.stdin.close()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if hasattr(self, "stderr_thread"):
            self.stderr_thread.join(timeout=1.0)
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None

    def _read_output(self, process, generation):
        assert process.stdout is not None
        for raw in process.stdout:
            try:
                record = json.loads(raw)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if record.get("valid"):
                with self.lock:
                    if generation == self.generation:
                        record["generation"] = generation
                        self.poses.append(record)
                        self.processed_scans += 1
            else:
                with self.lock:
                    if generation == self.generation:
                        self.processed_scans += 1
        code = process.poll()
        if code not in (None, 0) and not self.closed:
            with self.lock:
                self.error = f"GLIM process exited with status {code}"

    def _run(self):
        try:
            self._start_process()
            while True:
                item = self.input.get()
                if item == "stop":
                    break
                if item == "reset":
                    self._stop_process()
                    self._start_process()
                    continue
                imu_samples, scan = item
                self._write_batch(imu_samples, scan)
        except Exception as exc:  # noqa: BLE001
            with self.lock:
                self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self._stop_process()

    def _write_batch(self, imu_samples, scan):
        process = self.process
        if process is None or process.poll() is not None or process.stdin is None:
            raise RuntimeError("GLIM process is not running")
        stream = process.stdin
        for stamp, accel, gyro in imu_samples:
            stream.write(struct.pack("<B7d", ord("I"), stamp, *accel, *gyro))
        start, points, end = scan
        stream.write(struct.pack("<BdI", ord("L"), start, len(points)))
        if len(points):
            stream.write(points.astype("<f8", copy=False).tobytes(order="C"))
        stream.flush()
