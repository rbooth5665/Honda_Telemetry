import serial
import serial.tools.list_ports
import time
#--------- Globals ---------#
READ_BYTE = [0x72] #established message typing, 0x72 being read requests
WAKEUP_RESPONSE = bytearray(b'\x0E\x04r|') #expected response from ECU after wakeup message
DIAGNOSTIC_RESPONSE = bytearray(b'\x02\x04\x00\xfa') #expected response from ECU after diagnostic request
DATA_POLLING_REQUEST = [0x72, 0x07, 0x72, 0x11, 0x00, 0x14, 0xF0] #frame to request 26 bytes of data from ECU

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

def connect_ecu(delay = 0.2): #constantly polls the bike, waits to connect to ECU
    attempts = 0
    print("Turn Bike On")
    while(True): #while loop to constantly poll for ECU connection
        recv = wakeup()
        if recv == WAKEUP_RESPONSE and validate_frame(recv):
            print("ECU connected and validated")
            return recv
        time.sleep(delay)

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
recv = connect_ecu()

#prints transmission results, checks checksum
print("\nECU WAKEUP VALIDATION")
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
print("DIAGNOSTIC SESSION VALIDATION")
if diag_recv != DIAGNOSTIC_RESPONSE:
    print(f"Diagnostic response not what was expected: {[hex(b) for b in diag_recv]}")
    ser.close()
    SystemExit(1)

else:
    print(f"Diagnostic session open with: {[hex(b) for b in diag_recv]}\n")


try:
    print("Control+C to quit\n")
    poll = True
    while(poll):
        #poll for data, then wait .5 seconds
        data_received = send_recv(ser, DATA_POLLING_REQUEST, 26)
        print(f"{[hex(b) for b in data_received[5:-1]]}\n")
        time.sleep(.05)

        if not data_received:
            poll = False
except KeyboardInterrupt:
    print("User exited")

finally:
    ser.close()

    #received data from 20 bytes, stripped from 26 bytes
    # [0][1]   = RPM?
    # [2]      = TPS   
    # [3]      = TPS Secondary?
    # [4][5]   = ECT?
    # [6][7]   = IAT?
    # [8][9]   = unknown
    # [10][11] = constant padding
    # [12]     = unknown
    # [13]     = unknown
    # [14]     = gear?
    # [15]     = unknown (moves with RPM event)
    # [16]     = unknown, slow drift during 5 minutes
    # [17]     = temp? warmup drift
    # [18]     = unknown
    # [19]     = last byte varies