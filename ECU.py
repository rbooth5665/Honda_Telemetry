import serial
import serial.tools.list_ports
import time
import struct

class ECU:
    BAUD = 10400
    READ_BYTE = [0x72] #established message typing, 0x72 being read requests
    WAKEUP_RESPONSE = bytearray(b'\x0E\x04r|') #expected response from ECU after wakeup message
    WAKEUP_FRAME = [0xFE, 0x04, 0x72, 0x8C] #transmitted wakeup frame
    DIAGNOSTIC_RESPONSE = bytearray(b'\x02\x04\x00\xfa') #expected response from ECU after diagnostic request
    DATA_REQUEST = [0x72, 0x07, 0x72, 0x11, 0x00, 0x14, 0xF0] #frame to request 26 bytes of data from ECU

    def __init__(self, port=None, timeout=1):
        self.port = port
        self.timeout = timeout
        self.ser = None
        self.connected = False

        self.tps_closed = 25
        self.tps_open = 231
        



    @staticmethod
    def honda_checksum(data): #returns the honda checksum for a list of bytes as a hex value
        return ((sum(bytearray(data)) ^ 0xFF) + 1) & 0xFF
    
    @staticmethod
    def build_frame(mtype, data): #builds the message frame as a list of bytes
    #[type][length][data][checksum]
        length = len(mtype) + len(data) + 0x02 #length of the message, data, and 2 additional bytes
        frame = mtype + [length] + data
        cksum = ECU.honda_checksum(frame)
        return frame + [cksum]
    
    def _find_port(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = (p.description or "").lower()
            manf = (p.manufacturer or "").lower()
            if "ftdi" in desc or "ftdi" in manf:
                return p.device
        return None
    
    def open_port(self):
        port = self.port or self._find_port()
        if port is None:
            print("No Serial Port Found")
            return False
        
        try:
            self.ser = serial.Serial(
                                    port,
                                    self.BAUD,
                                    serial.EIGHTBITS,
                                    serial.PARITY_NONE, 
                                    serial.STOPBITS_ONE, 
                                    timeout=self.timeout
                                    )
            self.ser.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print("Failed to open port: {e}")
            return False
        
    def send_recv(self, frame, recv_len=None, timeout=0.5):
        #flush, send, read echo
        self.ser.reset_input_buffer()
        self.ser.write(bytes(frame))
        self.ser.read(len(frame))
    
        #byte structure
        buf = bytearray()
        end = time.time() + timeout
        
        #message length is known
        if recv_len is not None:
            while len(buf) < recv_len and time.time() < end:
                chunk = self.ser.read(recv_len - len(buf))
                if chunk:
                    buf.extend(chunk)
        
        #message length is not known, has timeout for raw probing
        else:
            original = self.ser.timeout
            self.ser.timeout = .05
            
            try:
                last_rx = time.time()
                while time.time() < end:
                    chunk = self.ser.read(64)
                    #if the ECU is actively transmitting, append to the buffer and reset the time
                    if chunk:
                        buf.extend(chunk)
                        last_rx = time.time()
                    #transmission hasn't occured recently, safe to assume the ECU is done communicating
                    elif time.time() - last_rx > self.ser.timeout:
                        break
            finally:
                self.ser.timeout = original

        #returns bytearray of the ECU response
        return buf
    
    def wakeup(self):
        self.ser.send_break(0.070)
        time.sleep(0.130)
        return self.send_recv(self.WAKEUP_FRAME, 4)
    
    def connect(self, max_attempts=None, attempt_delay=0.2):
        if self.ser is None and not self.open_port():
            return False

        attempt = 0
        while max_attempts is None or attempt < max_attempts:
            attempt += 1
            recv = self.wakeup()
            if recv == self.WAKEUP_RESPONSE:
                self.connected = True
                print(f"Connected on attempt: {attempt}")
                return True
            if recv:
                print(f"Unexpected response: {[hex(b) for b in recv]}")
            elif attempt % 10 == 0:
                print(f"Still waiting on attempt {attempt}")
            time.sleep(attempt_delay)
        
        return False
    
    def poll(self):
        return self.send_recv(self.DATA_REQUEST, 26)
    
    def parse(self, frame):
        if len(frame) < 26:
            return None
        
        d = frame[5:]

        rpm = (d[0] << 8) | d[1]
        tps = (d[2] - self.tps_closed) / (self.tps_open - self.tps_closed) * 100
        tps = max(0.0, min(100.0, tps))

        return {"rpm": rpm, "tps_pct": round(tps, 1)}
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        self.connected = False

if __name__ == "__main__":
    ecu = ECU()
    if ecu.connect():
        try:
            while True:
                frame = ecu.poll()
                data = ecu.parse(frame)
                if data:
                    print(f"RPM: {data['rpm']:5d}\nTPS: {data['tps_pct']:5.1f}")
        except KeyboardInterrupt:
            print("\nShutting Down")

        finally:
            ecu.close()