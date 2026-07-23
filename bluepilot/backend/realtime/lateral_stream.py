"""
BluePilot portal: live lateral-debug feed for the phone graph (/lateral).

The MICI lateral debug screen shows desired vs actual steering angle on a display
that is honestly too small for more than a gut feeling. This module gives the same
signals to any phone on the device's hotspot/LAN as a 20 Hz snapshot stream the
portal serves over SSE.

The reader starts on the first /lateral view, holds CONFLATED sockets (only the
newest message is kept and parsed, so the running cost is three tiny capnp parses
per 50 ms tick), and stays up for the life of the process. Watchers only gate
whether samples are assembled.

Note (2026-07-24): an earlier version of this file registered the reader at portal
boot on the theory that a mid-drive SubMaster construction would gap carState for
the driving processes and disengage the car. That was TESTED and disproven — a
fresh msgq reader registering mid-stream does not disturb existing readers. The
real /lateral disengage cause was CPU: spawning the whole bp_portal process at
normal priority on a device already near 100% starved the driving stack. That is
fixed by demoting the portal process to SCHED_IDLE at import (see bp_portal.py),
so this feed can register whenever it likes without a boot pre-start.
"""

import threading
import time
import logging

logger = logging.getLogger(__name__)

_RATE_HZ = 20.0


class LateralFeed:
    """Singleton owner of the messaging reader. Thread-safe snapshot access."""

    _instance = None
    _instance_lock = threading.Lock()

    @classmethod
    def instance(cls) -> "LateralFeed":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self._watchers = 0
        self._seq = 0
        self._sample = {}

    # -- lifecycle --------------------------------------------------------------------------
    def ensure_started(self):
        """Start the permanent reader. Called once at portal startup so the message-queue
        reader registration happens offroad — NEVER lazily from a request handler (see the
        module docstring for why that disengaged the car)."""
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._run, daemon=True,
                                                name="lateral_feed")
                self._thread.start()

    def attach(self):
        self.ensure_started()  # belt and suspenders; the registration cost was paid at boot
        with self._lock:
            self._watchers += 1

    def detach(self):
        with self._lock:
            self._watchers = max(0, self._watchers - 1)

    def snapshot(self, last_seq: int):
        """(seq, sample) if newer than last_seq else (last_seq, None)."""
        with self._lock:
            if self._seq == last_seq:
                return last_seq, None
            return self._seq, dict(self._sample)

    # -- reader ---------------------------------------------------------------------------
    def _run(self):
        try:
            import cereal.messaging as messaging
        except Exception as exc:
            logger.error("lateral feed: messaging unavailable: %s", exc)
            return
        try:
            # Conflated: the queue keeps only the newest message, so the permanent reader
            # parses at most one message per service per tick regardless of publish rate.
            socks = {name: messaging.sub_sock(name, conflate=True, timeout=0)
                     for name in ("carState", "carControl", "controllerStateBP")}
        except Exception as exc:
            logger.error("lateral feed: sub_sock failed: %s", exc)
            return
        logger.info("lateral feed: permanent reader started (registered at boot)")
        latest = {}
        last_seen = {name: 0.0 for name in socks}
        period = 1.0 / _RATE_HZ
        while True:
            t0 = time.monotonic()
            try:
                for name, sock in socks.items():
                    msg = messaging.recv_one_or_none(sock)
                    if msg is not None:
                        latest[name] = msg
                        last_seen[name] = t0
                with self._lock:
                    watching = self._watchers > 0
                if watching and "carState" in latest and "carControl" in latest:
                    cs = latest["carState"].carState
                    cc = latest["carControl"].carControl
                    st = latest.get("controllerStateBP")
                    stb = st.controllerStateBP if st is not None else None
                    fresh = (t0 - last_seen["carState"] < 1.0
                             and t0 - last_seen["carControl"] < 1.0)
                    sample = {
                        't': time.time(),
                        'desired_deg': float(cc.actuators.steeringAngleDeg),
                        'actual_deg': float(cs.steeringAngleDeg),
                        'v_mph': float(cs.vEgo) * 2.23694,
                        'lat_active': bool(cc.latActive),
                        'torque_nm': float(cs.steeringTorque),
                        'low_factor': float(stb.bmsLowSpeedAdjustmentFactor) if stb is not None else 0.0,
                        'high_factor': float(stb.bmsHighSpeedAdjustmentFactor) if stb is not None else 0.0,
                        'autocal': str(getattr(stb, 'bmsAngleAutoCalState', '')) if stb is not None else '',
                        'alive': fresh,
                    }
                    if fresh:
                        with self._lock:
                            self._sample = sample
                            self._seq += 1
            except Exception as exc:
                # keep the reader alive through transient messaging hiccups
                logger.debug("lateral feed: update error: %s", exc)
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)


def serve_sse(handler):
    """Write the SSE stream onto a BaseHTTPRequestHandler until the client leaves.

    One thread per watching phone (the portal is a ThreadingHTTPServer); frames are
    only sent when the feed sequence advances, so a paused car costs near nothing.
    """
    import json as _json

    feed = LateralFeed.instance()
    feed.attach()
    try:
        handler.send_response(200)
        handler.send_header('Content-Type', 'text/event-stream')
        handler.send_header('Cache-Control', 'no-cache')
        handler.send_header('Connection', 'keep-alive')
        handler.send_header('Access-Control-Allow-Origin', '*')
        handler.end_headers()
        seq = 0
        last_beat = time.monotonic()
        while True:
            seq, sample = feed.snapshot(seq)
            now = time.monotonic()
            if sample is not None:
                payload = f"data: {_json.dumps(sample, separators=(',', ':'))}\n\n"
                handler.wfile.write(payload.encode())
                handler.wfile.flush()
                last_beat = now
            elif now - last_beat > 5.0:
                # comment-frame keepalive so phones detect a dead link promptly
                handler.wfile.write(b": keepalive\n\n")
                handler.wfile.flush()
                last_beat = now
            time.sleep(1.0 / _RATE_HZ / 2.0)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass  # phone left / screen locked — normal end of stream
    finally:
        feed.detach()
