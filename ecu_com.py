import serial
import serial.tools.list_ports
import time
#--------- Globals ---------#
READ_BYTE = [0x72] #established message typing, 0x72 being read requests



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
    if len(frame) < 3:
        return False
    return frame[-1] == honda_checksum(frame[:-1])

def wakeup(): #sends wakeup pulse and wakeup fram to ECU, returns response received
    #Creates wakeup frame, pulses K-Line
    wakeup_frame = build_frame([0xFE],[0x72])
    ser.send_break(.070)
    time.sleep(.130)
    
    return send_recv(ser, wakeup_frame, 4)


#--------- Program ---------#
try:
    #opens serial port, flushes buffer
    ser = serial.Serial(
                        find_serial_port(),
                        10400,
                        serial.EIGHTBITS, 
                        serial.PARITY_NONE, 
                        serial.STOPBITS_ONE, 
                        timeout=1
                        )
    ser.reset_input_buffer()

except serial.SerialException as e:
    print(f"FAILED to open serial port: {e}")
    SystemExit(1)

finally:
    if not ser.is_open:
        print("Serial port did not open")
        SystemExit(1)

#diagnostic frame candidates to test
diag_frame_candidate = {
                        build_frame(READ_BYTE,[0x00, 0xf0]), # PC37 diagnostic frame
                        build_frame(READ_BYTE,[0x00, 0x10]), #KWP2000 diagnostic protocol
                        
                        
                        
                        
                        
                        }                


#ECU wakeup protocol
recv = wakeup()

#prints transmission results, checks checksum
if recv:
    print(f"RX: {recv}")
    if validate_frame(recv):
        print("Valid ECU response received")
    else:
        print("Checksum validation failed")
        ser.close()
        SystemExit(1)
else:
    print("No response received from ECU")
    ser.close()
    SystemExit(1)