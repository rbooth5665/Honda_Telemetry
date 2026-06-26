import serial
import serial.tools.list_ports
import time
import csv
import os
from datetime import datetime


class ECU:
    BAUD = 10400
    HEADER_LEN = 4
    WAKEUP_FRAME = [0xFE, 0x04, 0x72, 0x8C]
    WAKEUP_RESPONSE = bytearray(b'\x0E\x04\x72\x7C')

    REQ_ENGINE = [0x72, 0x05, 0x71, 0x11, 0x07]
    REQ_GEAR   = [0x72, 0x05, 0x71, 0xD1, 0x47]
    GEAR_STATE = {0x03: "N", 0x01: "CLUTCH", 0x00: "GEAR"}
    GEAR_RATIO_EDGES = [(114, 1), (87, 2), (74, 3), (66, 4), (59, 5), (0, 6)]

    def __init__(self, port=None, timeout=1):
        self.port = port
        self.timeout = timeout
        self.ser = None
        self.connected = False
        self._last_gear = 0    

    @staticmethod
    def honda_checksum(data):
        return ((sum(bytearray(data)) ^ 0xFF) + 1) & 0xFF

    def _find_port(self):
        for p in serial.tools.list_ports.comports():
            blob = f"{p.description or ''} {p.manufacturer or ''}".lower()
            if "ftdi" in blob:
                return p.device
        return None

    def open_port(self, wait=True, retry_delay=2):
        port = self.port or self._find_port()
        attempt = 0
        while port is None and wait:
            attempt += 1
            if attempt == 1 or attempt % 10 == 0:
                print(f"Waiting for serial port... (attempt {attempt})")
            time.sleep(retry_delay)
            port = self._find_port()
        if port is None:
            print("No serial port found")
            return False

        try:
            self.ser = serial.Serial(port, self.BAUD, serial.EIGHTBITS,
                                     serial.PARITY_NONE, serial.STOPBITS_ONE,
                                     timeout=self.timeout)
            self.ser.reset_input_buffer()
            print(f"Found serial port: {port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to open port: {e}")
            return False

    def wakeup(self):
        self.ser.break_condition = True
        time.sleep(0.070)
        self.ser.break_condition = False
        time.sleep(0.130)
        return self.send_recv(self.WAKEUP_FRAME, 4)

    def connect(self, max_attempts=None, attempt_delay=0.2):
        if self.ser is None and not self.open_port():
            return False
        attempt = 0
        while max_attempts is None or attempt < max_attempts:
            attempt += 1
            if self.wakeup() == self.WAKEUP_RESPONSE:
                self.connected = True
                print(f"Connected on attempt {attempt}")
                return True
            if attempt % 10 == 0:
                print(f"Still waiting on attempt {attempt}")
            time.sleep(attempt_delay)
        return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def send_recv(self, frame, recv_len=None, timeout=0.5):
        self.ser.reset_input_buffer()
        self.ser.write(bytes(frame))
        self.ser.read(len(frame))

        buf = bytearray()
        end = time.time() + timeout
        if recv_len is not None:
            while len(buf) < recv_len and time.time() < end:
                chunk = self.ser.read(recv_len - len(buf))
                if chunk:
                    buf.extend(chunk)
        else:                               
            original, self.ser.timeout = self.ser.timeout, 0.05
            try:
                last = time.time()
                while time.time() < end:
                    chunk = self.ser.read(64)
                    if chunk:
                        buf.extend(chunk)
                        last = time.time()
                    elif time.time() - last > self.ser.timeout:
                        break
            finally:
                self.ser.timeout = original
        return buf

    def _payload(self, resp):
        if resp is None or len(resp) < self.HEADER_LEN + 1:
            return None
        return list(resp[self.HEADER_LEN:-1])

    def poll_engine(self):
        return self._payload(self.send_recv(self.REQ_ENGINE))

    def poll_gear(self):
        return self._payload(self.send_recv(self.REQ_GEAR))

    def gear_from_ratio(self, rpm, speed):
        if speed is None or speed < 3 or rpm < 1000:
            return self._last_gear          
        ratio = rpm / speed
        for edge, g in self.GEAR_RATIO_EDGES:
            if ratio > edge:
                self._last_gear = g
                return g
        return self._last_gear

    def decode(self, eng, gear):
        out = {}
        if eng and len(eng) > 13:
            out["rpm"] = (eng[0] << 8) | eng[1]
            out["speed_kmh"] = eng[13]
            out["speed_mph"] = round(eng[13] * 0.621371, 1)
        if gear:
            raw = gear[0]
            out["clutch_state"] = self.GEAR_STATE.get(raw, f"0x{raw:02X}")
            if raw == 0x03:
                out["gear"] = 0             # neutral
            elif raw == 0x01:
                out["gear"] = self._last_gear   # clutch in: hold
            elif "rpm" in out and "speed_kmh" in out:
                out["gear"] = self.gear_from_ratio(out["rpm"], out["speed_kmh"])
        return out

class Logger:
    def __init__(self, ecu, log_dir="logs", width=20):
        self.ecu = ecu
        self.log_dir = log_dir
        self.width = width
        self._f = self._w = None
        self._sample = 0
        self._start = None

    def start(self, filename=None):
        os.makedirs(self.log_dir, exist_ok=True)
        if filename is None:
            stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"ecu_log_{stamp}.csv"
        path = os.path.join(self.log_dir, filename)
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(
            ["sample", "elapsed_s", "wall_clock",
             "rpm", "speed_kmh", "gear", "clutch_state"]
            + [f"t11_b{i}" for i in range(self.width)]
            + [f"d1_b{i}" for i in range(self.width)])
        self._start = time.time()
        self._sample = 0
        print(f"logging -> {path}")
        self.path = path
        return path

    @staticmethod
    def _fit(payload, width):
        if not payload:
            return [-1] * width
        return (payload + [0] * width)[:width]

    def log(self, eng, gear):
        dec = self.ecu.decode(eng, gear)
        elapsed = round(time.time() - self._start, 3)
        wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._w.writerow(
            [self._sample, elapsed, wall,
             dec.get("rpm", -1), dec.get("speed_kmh", -1),
             dec.get("gear", -1), dec.get("clutch_state", "NA")]
            + self._fit(eng, self.width)
            + self._fit(gear, self.width))
        self._sample += 1
        return dec

    def stop(self):
        if self._f:
            self._f.close()
            print(f"logged {self._sample} samples to {self.path}")
            self._f = self._w = None

if __name__ == "__main__":
    ecu = ECU()
    if ecu.connect():
        log = Logger(ecu)
        log.start()
        try:
            while True:
                eng = ecu.poll_engine()
                gear = ecu.poll_gear()
                dec = log.log(eng, gear)
                if log._sample % 40 == 0:
                    print(f"rpm {dec.get('rpm')}  "
                          f"{dec.get('speed_mph')} mph  gear {dec.get('gear')}")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nShutting down")
        finally:
            log.stop()
            ecu.close()