import serial
import serial.tools.list_ports
import time

#--------- Methods ---------#
def find_serial_port(): #connects to the FTDI serial port, active loop to probe for ports
    ports = serial.tools.list_ports.comports()
    while not ports:    
        print("No serial ports found. Retrying...")
        time.sleep(.1)
        ports = serial.tools.list_ports.comports()
        
    for ports in serial.tools.list_ports.comports():
        desc = ports.description.lower()
        manf = ports.manufacturer.lower()
        if "ftdi" in desc or "ftdi" in manf:
            return ports.device
    return "No FTDI serial port found"

def honda_checksum(data): #returns the honda checksum for a list of bytes as a hex value
    return ((sum(bytearray(data)) ^ 0xFF) + 1) & 0xFF

def build_frame(mtype, data): #builds the message frame as a list of bytes
    #[type][length][data][checksum]
    length = len(mtype) + len(data) + 0x02 #length of the message, data, and 2 additional bytes
    frame = mtype + [length] + data
    cksum = honda_checksum(frame)
    return frame + [cksum]

def send_recv(ser, frame, recv_len, timout=0.5): #sends byte frame from serial port, receives response and validates
    #flush, send, read echo
    ser.reset_input_buffer()
    ser.write(bytes(frame))
    ser.read(len(frame))
   
    #byte structure
    buf = bytearray()
    end = time.time() + timout
    
    #while timeout has not been met and the received bits is less than expected, read and append
    while len(buf) < recv_len and time.time() < end:
        chunk = ser.read(recv_len - len(buf))
        if chunk:
            buf.extend(chunk)
    
    #returns a hex array of the ECU response
    return buf

def validate_frame(frame): #checks for 4 byte frame, validates checksum
    if len(frame) != 4:
        return False
    return frame[-1] == honda_checksum(frame[:-1])


#--------- Program ---------#
#creates wakeup frame
wakeup_frame = build_frame([0xFE],[0x72])

try:
    #opens serial port, flushes buffer
    ser = serial.Serial(find_serial_port(),
                        10400,
                        serial.EIGHTBITS, 
                        serial.PARITY_NONE, 
                        serial.STOPBITS_ONE, 
                        timeout=1)
    ser.reset_input_buffer()

except serial.SerialException as e:
    print(f"FAILED to open serial port: {e}")
    SystemExit(1)

finally:
    if not ser.is_open:
        print("Serial port did not open")
        SystemExit(1)

#ECU wakeup pulse, clears input buffer noise
ser.send_break(.070)
time.sleep(.130)
ser.reset_input_buffer()

#send wakeup frame, receive ECU response
recv = send_recv(ser, wakeup_frame, 4)

#prints transmission results, checks checksum
print(f"TX: {hex(wakeup_frame)}")
if recv:
    print(f"RX: {recv}")
    if validate_frame(recv):
        print("Valid ECU response received")
    else:
        print("Checksum validation failed")
else:
    print("No response received from ECU")