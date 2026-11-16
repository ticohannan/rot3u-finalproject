"""Serial link to servo_bridge.ino (115200 baud, text lines).

    link = BridgeLink(SerialTransport("COM3"), log_path="logs/session.csv")
    link.connect()                 # opening the port resets the Mega: waits for its HELLO line
    link.cmd("ON 0 1500")          # tagged command, waits for DONE, returns Result(ok, lines)
    link.targets([1520, None, None, 1010, None, None])   # fire and forget (used by the tracker)
    link.state                     # newest ST line (STREAM ms makes the sketch send them)
    link.stop()                    # X: stop and hold

Close the Arduino Serial Monitor first: only one program can hold the port.
Copyright (c) 2026 Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations
import collections, os, threading, time
from dataclasses import dataclass, field

DEFAULT_PORT = "COM3"
NUM_CH = 6


@dataclass
class State:
    t_ms: int
    pos: list            # commanded pulse now, per channel (us)
    tgt: list            # target pulse, per channel (us)
    active: int          # bit c = 1: channel c is holding a pulse
    moving: bool
    t_host: float = 0.0  # host clock when the line arrived

    def is_on(self, c: int) -> bool:
        return bool(self.active >> c & 1)


@dataclass
class Result:
    ok: bool
    lines: list = field(default_factory=list)
    timeout: bool = False


def parse_state(line: str, t_host: float = 0.0):
    f = line.strip().split(",")
    if len(f) != 16 or f[0] != "ST":
        return None
    try:
        v = [int(x) for x in f[1:]]
    except ValueError:
        return None
    return State(v[0], v[1:7], v[7:13], v[13], bool(v[14]), t_host)


class SerialTransport:
    """A USB serial port (pyserial). Opening it resets the Mega (DTR); the arm keeps holding.

    The port's timeout is set once, when it opens. Changing it later makes pyserial rewrite the port
    settings, which on Windows can pulse DTR and keep resetting the board (same code as rot3u_host)."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = 115200):
        import serial                                   # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=2)
        self._buf = b""

    def write_line(self, s: str):
        self.ser.write((s + "\n").encode("ascii", "replace"))
        self.ser.flush()

    def read_line(self, timeout: float = 0.05):
        """One complete line, or None if none arrived within the timeout."""
        end = time.monotonic() + timeout
        while True:
            if b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                line = line.decode("ascii", "replace").strip()
                if line:
                    return line
                continue
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self._buf += chunk
            elif time.monotonic() >= end:
                return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class BridgeLink:
    """threaded=True (normal use): a background thread reads the port. threaded=False (simulated time,
    tests): nothing runs in the background; cmd() and poll() read what the transport has ready."""

    def __init__(self, transport, log_path=None, clock=time.perf_counter, threaded=True):
        self.tr = transport
        self.clock = clock
        self.threaded = threaded
        self.hello = None
        self.state = None
        self.errors = []
        self.text = collections.deque(maxlen=400)       # human "# ..." lines, newest last
        self.connected = False
        self._id = 0
        self._wlock = threading.Lock()
        self._pending = {}                              # id -> [event, ok, lines]
        self._collect = None
        self._stop = False
        self._thread = None
        self._log = None
        if log_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
            self._log = open(log_path, "w", buffering=1)
            self._log.write("t_host_s,dir,line\n")
        self._t0 = self.clock()

    # ---- plumbing
    def _logline(self, d, s):
        if self._log:
            self._log.write(f"{self.clock() - self._t0:.4f},{d},{s}\n")

    def _write(self, s: str):
        with self._wlock:
            self.tr.write_line(s)
            self._logline("TX", s)

    def _handle(self, line: str):
        self._logline("RX", line)
        if line.startswith("ST,"):
            st = parse_state(line, self.clock())
            if st:
                self.state = st
            return
        if line.startswith("HELLO,"):
            self.hello = line.split(",")
            return
        if line.startswith("ACK,"):
            return
        if line.startswith("DONE,"):
            f = line.split(",")
            p = self._pending.get(int(f[1])) if len(f) >= 3 and f[1].isdigit() else None
            if p:
                p[1] = (f[2] == "OK")
                p[0].set()
            return
        if line.startswith("ERR,"):
            self.errors.append(line)
        self.text.append(line)
        for p in list(self._pending.values()):          # text between ACK and DONE belongs to the command
            p[2].append(line)

    def poll(self, timeout: float = 0.0) -> int:
        """Read and handle every line that is ready. Returns the number of lines."""
        n = 0
        while True:
            line = self.tr.read_line(timeout if n == 0 else 0.0)
            if line is None:
                return n
            self._handle(line)
            n += 1

    def _reader(self):
        while not self._stop:
            try:
                line = self.tr.read_line(0.05)
            except Exception as e:                      # cable pulled, port closed
                self.connected = False
                self.errors.append(f"link lost: {e}")
                return
            if line is not None:
                try:
                    self._handle(line)
                except Exception as e:                  # a damaged line must not end the reader
                    self.errors.append(f"bad line {line!r}: {e}")

    # ---- public
    def connect(self, timeout: float = 8.0):
        """Wait for the sketch's HELLO (the Mega resets when the port opens; its bootloader takes ~2 s)."""
        if self.threaded and self._thread is None:
            self._thread = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()
        t_end = self.clock() + timeout
        asked = self.clock() + 3.0
        while self.hello is None:
            if not self.threaded:
                if self.poll(0.05) == 0 and self.clock() > t_end:
                    break
            else:
                time.sleep(0.05)
            if self.hello is None and self.clock() > asked:   # no reset happened: ask
                self._write("HELLO")
                asked = self.clock() + 1.0
            if self.clock() > t_end:
                break
        if self.hello is None:
            raise ConnectionError("no HELLO from the sketch. Is servo_bridge uploaded, the port right, and the "
                                  "Serial Monitor closed? Last lines: " + " | ".join(list(self.text)[-5:]))
        if self.hello[1] != "servo_bridge":
            raise ConnectionError(f"the board runs '{self.hello[1]}', not servo_bridge: upload servo_bridge.ino")
        self.connected = True
        return self.hello

    def cmd(self, text: str, timeout: float = 2.0) -> Result:
        """Send a tagged command and wait for its DONE."""
        self._id = self._id % 60000 + 1
        i = self._id
        p = [threading.Event(), False, []]
        self._pending[i] = p
        self._write(f"@{i} {text}")
        if self.threaded:
            got = p[0].wait(timeout)
        else:
            t_end = self.clock() + timeout
            got = False
            while True:
                self.poll(0.0)
                if p[0].is_set():
                    got = True
                    break
                if self.clock() > t_end:
                    break
                self.poll(0.02)
        self._pending.pop(i, None)
        return Result(p[1] if got else False, p[2], timeout=not got)

    def send(self, text: str):
        """Send without waiting for the reply (the tracker's targets)."""
        self._id = self._id % 60000 + 1
        self._write(f"@{self._id} {text}")

    @staticmethod
    def targets_text(us) -> str:
        return "T " + " ".join("-" if u is None else f"{u:.1f}" for u in us)

    def targets(self, us):
        self.send(self.targets_text(us))

    def stop(self):
        self._write("X")

    def get_state(self, timeout: float = 1.0) -> State:
        """Ask for a fresh state line."""
        r = self.cmd("ST", timeout)
        if self.state is None or r.timeout:
            raise TimeoutError("no state line from the sketch")
        return self.state

    def close(self):
        self._stop = True
        if self._thread:
            self._thread.join(0.5)
        self.tr.close()
        self.connected = False
        if self._log:
            self._log.close()
            self._log = None
