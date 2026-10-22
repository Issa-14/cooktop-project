"""Hardware-independent scheduler, adapted from Fable Bridge._poll_keys.

One output at a time; bounded queue; completion only after release + settle.
Network work is queued separately, never performed inside tick().
"""
import math


class CommandError(Exception):
    pass


class Controller:
    def __init__(self, buttons, verified=False, trip_c=None, heartbeat_ms=5000,
                 lease_ms=60000, press_ms=150, gap_ms=250,
                 diff=lambda a, b: a-b, add=lambda a, b: a+b):
        self.buttons = buttons
        self.verified = verified
        self.trip_c = trip_c
        self.heartbeat_ms, self.lease_ms = heartbeat_ms, lease_ms
        self.press_ms, self.gap_ms = press_ms, gap_ms
        self.diff, self.add = diff, add
        self.armed = False
        self.reason = "BOOT_DISARMED"
        self.last_rx = 0
        self.lease_end = 0
        self.queue, self.active, self.settle = [], None, None
        self.replies, self.events, self.network = [], [], []
        self.pending, self.cache, self.cache_order = {}, {}, []
        self.temperature = {"c": None, "fault": "NOT_SAMPLED", "at_ms": None}
        self.power = {"watts": None, "output": None, "fault": "NOT_SAMPLED", "at_ms": None}
        self.off_pending = False
        self.generation = 0
        self.buttons.all_off()

    def _reply(self, cid, ok, result=None, error=None):
        msg = {"kind": "reply", "id": cid, "ok": ok}
        msg["result" if ok else "error"] = result if ok else error
        self.pending.pop(cid, None)
        self.cache[cid] = msg
        self.cache_order.append(cid)
        while len(self.cache_order) > 64:
            self.cache.pop(self.cache_order.pop(0), None)
        self.replies.append(msg)

    def disarm(self, reason, request_off=False):
        # Outputs first, as in Fable's _fail_pending: serial/network can stall.
        self.buttons.all_off()
        self.armed = False
        self.reason = reason
        self.generation += 1
        if request_off:
            self.off_pending = True
        ids = list(self.pending)  # includes a network command already in flight
        self.queue, self.active, self.settle, self.network = [], None, None, []
        for cid in ids:
            self._reply(cid, False, error=reason)
        self.events.append({"kind": "event", "event": "disarmed", "reason": reason})

    def update_temperature(self, c, fault, now):
        valid = type(c) in (int, float) and math.isfinite(c) and not fault
        self.temperature = {"c": c if valid else None, "fault": fault or (None if valid else "INVALID"), "at_ms": now}
        if self.armed and (not valid or c >= self.trip_c):
            self.disarm("TEMPERATURE_FAULT" if not valid else "SURFACE_LIMIT", True)

    def update_power(self, watts, output, fault, now):
        was_on = self.power['output'] is True
        valid = type(watts) in (int, float) and math.isfinite(watts) and watts >= 0 and type(output) is bool and not fault
        self.power = {"watts": watts if valid else None, "output": output if valid else None,
                      "fault": fault or (None if valid else "INVALID"), "at_ms": now}
        if self.armed and not valid:
            self.disarm("SHELLY_FAULT", True)
        elif self.armed and was_on and output is False:
            self.disarm("PLUG_TURNED_OFF")

    def _fresh(self, item, now, max_age):
        return item["at_ms"] is not None and not item["fault"] and 0 <= self.diff(now, item["at_ms"]) <= max_age

    def snapshot(self, now):
        def measurement(item, max_age):
            d = dict(item)
            d["age_ms"] = None if item["at_ms"] is None else self.diff(now, item["at_ms"])
            d["valid"] = self._fresh(item, now, max_age)
            return d
        return {"schema": "cib80.telemetry.v1", "device_ms": now,
                "armed": self.armed, "reason": self.reason,
                "active_button": self.active[1] if self.active else None,
                "queue_depth": len(self.queue), "temperature": measurement(self.temperature, 2000),
                "power": measurement(self.power, 5000),
                "cooktop": {"displayed_setpoint_c": None, "mode": None, "state_quality": "unknown"}}

    def handle(self, request, now):
        cid = request.get("id") if isinstance(request, dict) else None
        if not isinstance(cid, str) or not cid or len(cid) > 64:
            self.replies.append({"kind": "reply", "id": None, "ok": False, "error": "BAD_ID"})
            return
        if cid in self.cache:
            self.replies.append(self.cache[cid]); return
        if cid in self.pending:
            return  # Duplicate in flight: never actuate a second time.
        if len(self.pending) >= 16:
            self._reply(cid, False, error="BUSY"); return
        try:
            cmd = request.get("cmd")
            if cmd not in ("PING", "INFO", "SNAPSHOT", "ARM", "DISARM", "ABORT", "PRESS", "PLUG_SET"):
                raise CommandError("UNKNOWN_COMMAND")
            self.last_rx = now
            if cmd == "PING": self._reply(cid, True, {"pong": True})
            elif cmd == "INFO": self._reply(cid, True, {"protocol": 1, "version": "0.1.0", "buttons": list(self.buttons.names), "hardware_verified": self.verified})
            elif cmd == "SNAPSHOT": self._reply(cid, True, self.snapshot(now))
            elif cmd in ("DISARM", "ABORT"):
                self.disarm(cmd)
                self._reply(cid, True, {"buttons_released": True, "plug_unchanged": True})
            elif cmd == "ARM":
                if self.armed: raise CommandError("ALREADY_ARMED")
                if not self.verified: raise CommandError("HARDWARE_NOT_VERIFIED")
                if type(self.trip_c) not in (int, float) or not math.isfinite(self.trip_c):
                    raise CommandError("SURFACE_LIMIT_NOT_CONFIGURED")
                if self.off_pending: raise CommandError("OFF_REQUEST_PENDING")
                if not self._fresh(self.temperature, now, 2000) or self.temperature['c'] >= self.trip_c:
                    raise CommandError("TEMPERATURE_NOT_READY")
                if not self._fresh(self.power, now, 5000): raise CommandError("SHELLY_NOT_READY")
                self.armed, self.reason = True, "ARMED"
                self.lease_end = self.add(now, self.lease_ms)
                self._reply(cid, True, {"lease_ms": self.lease_ms})
            elif cmd == "PRESS":
                self.tick(now)
                if not self.armed: raise CommandError("NOT_ARMED")
                if self.power['output'] is not True: raise CommandError("PLUG_NOT_ON")
                key, ms = request.get('button'), request.get('ms', self.press_ms)
                if key not in self.buttons.names: raise CommandError("BAD_BUTTON")
                if type(ms) is not int or not 20 <= ms <= 500: raise CommandError("BAD_DURATION")
                if len(self.queue) >= 8: raise CommandError("QUEUE_FULL")
                self.pending[cid] = request
                self.queue.append((cid, key, ms))
            elif cmd == "PLUG_SET":
                on = request.get('on')
                if type(on) is not bool: raise CommandError("BAD_ON_VALUE")
                self.tick(now)
                if on and not self.armed: raise CommandError("NOT_ARMED")
                if not on: self.disarm("PLUG_OFF_REQUEST")
                self.pending[cid] = request
                self.network.append((cid, on, self.generation))
        except (CommandError, TypeError) as exc:
            self._reply(cid, False, error=str(exc) if isinstance(exc, CommandError) else "BAD_ARGUMENT")

    def tick(self, now):
        if self.armed:
            reason = None
            if self.diff(now, self.last_rx) >= self.heartbeat_ms: reason = "HOST_TIMEOUT"
            elif self.diff(now, self.lease_end) >= 0: reason = "LEASE_EXPIRED"
            elif not self._fresh(self.temperature, now, 2000): reason = "TEMPERATURE_STALE"
            elif not self._fresh(self.power, now, 5000): reason = "SHELLY_STALE"
            if reason:
                self.disarm(reason, True); return
        if self.active:
            cid, key, deadline = self.active
            if self.diff(now, deadline) >= 0:
                self.buttons.all_off()
                self.active = None
                self.settle = (cid, key, self.add(now, self.gap_ms))
            return
        if self.settle:
            cid, key, deadline = self.settle
            if self.diff(now, deadline) < 0: return
            self.settle = None
            self._reply(cid, True, {"button": key, "pulse_completed": True, "cooktop_acknowledged": False})
        if self.armed and self.queue:
            cid, key, ms = self.queue.pop(0)
            self.buttons.all_off()
            self.buttons.key(key, True)
            self.active = (cid, key, self.add(now, ms))

    def network_done(self, cid, ok, result=None, error=None):
        if cid in self.pending:
            self._reply(cid, ok, result, error)

    def drain(self):
        out = self.replies + self.events
        self.replies, self.events = [], []
        return out
