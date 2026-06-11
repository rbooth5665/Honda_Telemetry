import serial
import serial.tools.list_ports
import time
#--------- Globals ---------#
READ_BYTE = [0x72] #established message typing, 0x72 being read requests
WAKEUP_RESPONSE = bytearray(b'\x0E\x04r|') #expected response from ECU after wakeup message
DIAGNOSTIC_RESPONSE = bytearray(b'\x02\x04\x00\xfa') #expected response from ECU after diagnostic request


#--------- Methods ---------#
def find_serial_port(): #connects to the FTDI serial port, active loop to probe for ports
    ports = serial.tools.list_ports.comports()
    while not ports:    
        print("No serial ports found. Retrying...")
        time.sleep(.1)
        ports = serial.tools.list_ports.comports()
        
    for ports in serial.tools.list_ports.comports():
        desc = (ports.description or "").lower()
        manf = (ports.manufacturer or "").lower()
        if "ftdi" in desc or "ftdi" in manf:
            return ports.device
    return None

def honda_checksum(data): #returns the honda checksum for a list of bytes as a hex value
    return ((sum(bytearray(data)) ^ 0xFF) + 1) & 0xFF

def build_frame(mtype, data): #builds the message frame as a list of bytes
    #[type][length][data][checksum]
    length = len(mtype) + len(data) + 0x02 #length of the message, data, and 2 additional bytes
    frame = mtype + [length] + data
    cksum = honda_checksum(frame)
    return frame + [cksum]

def send_recv(ser, frame, recv_len=None, timout=0.5): #sends byte frame from serial port, receives response and validates. Conducts greedy algorithm if no bits received
    #flush, send, read echo
    ser.reset_input_buffer()
    ser.write(bytes(frame))
    ser.read(len(frame))
   
    #byte structure
    buf = bytearray()
    end = time.time() + timout
    
    #message length is known
    if recv_len is not None:
        while len(buf) < recv_len and time.time() < end:
            chunk = ser.read(recv_len - len(buf))
            if chunk:
                buf.extend(chunk)
    
    #message length is not known, has timeout for raw probing
    else:
        last_rx = time.time()
        while time.time() < end:
            chunk = ser.read(64)

            #if the ECU is actively transmitting, append to the buffer and reset the time
            if chunk:
                buf.extend(chunk)
                last_rx = time.time()
            #transmission hasn't occured recently, safe to assume the ECU is done communicating
            elif time.time() - last_rx > .05:
                break

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

def establish_diagnostic_session(): #sends diagnostic session initiation, returns response received
    diag_frame = build_frame(READ_BYTE, [0x00, 0xf0]) 
    return send_recv(ser, diag_frame, 4)
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
    raise SystemExit(1)

if ser is None or not ser.is_open:
    print("Serial port did not open")
    raise SystemExit(1)

#ECU wakeup protocol
recv = wakeup()

#prints transmission results, checks checksum
print("ECU WAKEUP VALIDATION")
if recv == WAKEUP_RESPONSE:
    print(f"RX: {recv}")
    if validate_frame(recv):
        print("Valid ECU response received\n")
    else:
        print("Checksum validation failed")
        ser.close()
        raise SystemExit(1)
elif not recv:
    print("No response received from ECU")
    ser.close()
    raise SystemExit(1)
else:
    print(f"Received unexpected message from ECU: {recv}")
    raise SystemExit(1)

diag_recv = establish_diagnostic_session()

#verifies the diagnostic response received
if diag_recv is not DIAGNOSTIC_RESPONSE:
    print(f"Diagnostic response not what was expected: {[hex(b) for b in diag_recv]}")
    ser.close()
    SystemExit(1)

#once diagnostic response is validated, begin polling for information

data_polling_candidates = [
                            build_frame(READ_BYTE, [0x72,0x11,0x00,0x14,0xF0]), #1000rr polling request
                            build_frame(READ_BYTE, [0x72,0x10,0x00,0x14,0xF0]), #polls data table 10, as per the AiM logging tech sheet
                            build_frame(READ_BYTE, [0x72,0x00,0x00,0x14,0xF0]), #polls any table at 0x00, fallback if 10, 11 return nothing
                            build_frame(READ_BYTE, [0x72,0x11,0x00,0x20,0xF0]), #counts 32 bytes back
                            build_frame(READ_BYTE, [0x72,0x11,0x00,0x10,0xF0])  #counts 16 bytes back
                          ]


#passes the data polling cadidates, listens for response
for data_frames in data_polling_candidates:
    data_polled = send_recv(data_frames)

    print(f"Data Polling TX: {[hex(b) for b in data_frames]}")
    if data_polled:
        if validate_frame(data_polled):
            print(f"Validation passed Data Polling RX: {[hex(b) for b in data_polled]}\n")
        else:
            print(f"Validation failed Data Polling RX: {[hex(b) for b in data_polled]}\n")
    else:
        print("ECU returned nothing from polling request")

    #wakeup and reinitialize so connection doesn't drop
    wakeup()
    establish_diagnostic_session()


ser.close()