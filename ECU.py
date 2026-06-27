import serial
import serial.tools.list_ports
import time
import csv
import os
from datetime import datetime


class ECU:
    BAUD = 10400
    WAKEUP_FRAME = [0xFE, 0x04, 0x72, 0x8C]
    WAKEUP_RESPONSE = bytearray(b'\x0E\x04\x72\x7C')
    REQ_ENGINE = [0x72, 0x05, 0x71, 0x11, 0x07]    
    HEADER_LEN = 4          
    PAYLOAD_BYTES = 18      
    RESP_LEN = HEADER_LEN + PAYLOAD_BYTES + 1 
 
    GEAR_RATIO_EDGES = [(114, 1), (87, 2), (74, 3), (66, 4), (59, 5), (0, 6)]
    GEAR_HYST = 3          

    def __init__(self, port=None, timeout=1):
        self.port = port
        self.timeout = timeout
        self.ser = None
        self.connected = False
        self._last_gear = 0
        self._gcand = 0       
        self._gcount = 0

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

    def send_recv(self, frame, recv_len, timeout=0.5):
        self.ser.reset_input_buffer()
        self.ser.write(bytes(frame))
        self.ser.read(len(frame))

        buf = bytearray()
        end = time.time() + timeout
    
        while len(buf) < recv_len and time.time() < end:
            chunk = self.ser.read(recv_len - len(buf))
            if chunk:
                buf.extend(chunk)
       
        return buf

    def poll_engine(self):
        resp = self.send_recv(self.REQ_ENGINE, self.RESP_LEN, timeout=0.15)
        if len(resp) < self.HEADER_LEN + 14:
            return None
        return list(resp[self.HEADER_LEN:-1])


    def _gear(self, rpm, speed):   
        if speed < 3 or rpm < 1000:
            return self._last_gear             
        ratio = rpm / speed
        raw = next(g for edge, g in self.GEAR_RATIO_EDGES if ratio > edge)
        if raw == self._last_gear:
            self._gcand, self._gcount = raw, 0
        elif raw == self._gcand:
            self._gcount += 1
            if self._gcount >= self.GEAR_HYST:
                self._last_gear = raw
        else:
            self._gcand, self._gcount = raw, 1
        return self._last_gear

    UNKNOWN_BYTES = [14, 15, 16, 17, 18, 19]

    def decode(self, eng):
        if not eng or len(eng) <= 13:
            return None
        rpm = (eng[0] << 8) | eng[1]
        speed_kmh = eng[13]
        out = {
            "rpm": rpm,
            "speed_kmh": speed_kmh,
            "speed_mph": round(speed_kmh * 0.621371, 1),
            "gear": self._gear(rpm, speed_kmh),
            "tps": round(eng[3] / 16 * 10, 1),
            "ect_c": eng[5] - 40,
            "iat_c": eng[7] - 40,
            "map_kpa": eng[9],
            "batt": round(eng[12] / 10, 1),
        }

        for i in self.UNKNOWN_BYTES:
            out[f"b{i}"] = eng[i] if i < len(eng) else -1
        return out


class Logger:

    UNKNOWN_BYTES = ECU.UNKNOWN_BYTES
    COLS = (["sample", "elapsed_s", "dt_ms", "rpm", "speed_kmh", "speed_mph",
             "gear", "tps", "ect_c", "iat_c", "map_kpa", "batt"]
            + [f"b{i}" for i in UNKNOWN_BYTES])

    def __init__(self, log_dir="logs"):
        self.log_dir = log_dir
        self._f = self._w = None
        self._sample = 0
        self._start = self._last_t = None

    def start(self, filename=None):
        os.makedirs(self.log_dir, exist_ok=True)
        if filename is None:
            stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"ecu_log_{stamp}.csv"
        self.path = os.path.join(self.log_dir, filename)
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLS)
        self._start = self._last_t = time.time()
        self._sample = 0
        print(f"logging -> {self.path}")
        return self.path

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

    def log(self, dec):
        now = time.time()
        dt_ms = round((now - self._last_t) * 1000, 1)
        self._last_t = now
        self._w.writerow([self._sample, round(now - self._start, 3), dt_ms,
                          dec["rpm"], dec["speed_kmh"], dec["speed_mph"],
                          dec["gear"], dec["tps"], dec["ect_c"], dec["iat_c"],
                          dec["map_kpa"], dec["batt"]]
                         + [dec[f"b{i}"] for i in self.UNKNOWN_BYTES])
        self._sample += 1

    def stop(self):
        if self._f:
            self._f.close()
            total = time.time() - self._start
            rate = self._sample / total if total else 0
            print(f"logged {self._sample} samples to {self.path}  ({rate:.1f} Hz avg)")
            self._f = self._w = None


if __name__ == "__main__":
    ecu = ECU()
    if ecu.connect():
        log = Logger()
        log.start()
        try:
            while True:
                eng = ecu.poll_engine()
                if eng is None:
                    continue
                dec = ecu.decode(eng)
                if dec is None:
                    continue
                log.log(dec)
                if log._sample % 50 == 0:
                    print(f"rpm {dec['rpm']}  {dec['speed_mph']} mph  "
                          f"gear {dec['gear']}  ect {dec['ect_c']}C")
        except KeyboardInterrupt:
            print("\nShutting down")
        finally:
            log.stop()
            ecu.close()
