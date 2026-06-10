import serial
import serial.tools.list_ports
import time
#--------- Globals ---------#
READ_BYTE = [0x72] #established message typing, 0x72 being read requests
WAKEUP_RESPONSE = bytearray(b"\x0E\x04r|") #expected response from ECU


#--------- Methods ---------#
def find_serial_port(): #connects to the FTDI serial port, active loop to probe for ports
    ports = serial.tools.list_ports.comports()
    while not ports:    
        print("No serial ports found. Retrying...")
        time.sleep(.1)
        ports = serial.tools.list_ports.comports()
        
    for potential_port in serial.tools.list_ports.comports():
        desc = (potential_port.description or "").lower()
        manf = (potential_port.manufacturer or "").lower()
        if "ftdi" in desc or "ftdi" in manf:
            return potential_port.device
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

finally:
    if not ser.is_open:
        print("Serial port did not open")
        raise SystemExit(1)

#ECU wakeup protocol
recv = wakeup()

#prints transmission results, checks checksum
if recv == WAKEUP_RESPONSE:
    print(f"RX: {recv}")
    if validate_frame(recv):
        print("Valid ECU response received")
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

#diagnostic frame candidates to test
diag_frame_candidate = [
                        build_frame(READ_BYTE, [0x00, 0xf0]), # PC37 diagnostic frame
                        build_frame(READ_BYTE, [0x00, 0x10]), # KWP2000 diagnostic protocol
                        build_frame(READ_BYTE, [0x72, 0x11, 0x00, 0x14, 0xF0]), # skips diagnostic protocol and requests data directly
                        build_frame(READ_BYTE, [0x00, 0x72]), # 0x72 echo
                        build_frame(READ_BYTE, [0x00, 0x0E]), # addresses ECU node address 0x0E
                        build_frame(READ_BYTE, [0x00, 0x81])  # KWP2000 standard startCommunication message
                        ]    

#if ECU response has been validated, transmit diagnostic candidates
diag_failed = {}
diag_valid = {}
for diag_frame in diag_frame_candidate:
    #test each frame, receive any message from ECU
    diag_recv = send_recv(ser, diag_frame)
    
    #if a frame receives response
    if diag_recv:
        print(f"Diag TX: {[hex(b) for b in diag_frame]}")
        print(f"Diag RX: {[hex(b) for b in diag_recv]}")

        if validate_frame(diag_recv):
            #if validation is succesful, pass the valid frame to the valid received message as frame: response
            print("Diagnostic Message Received and validated")
            diag_valid.update({diag_frame: diag_recv})

        else:
            #something was received but it fails validation, pass to list that stores failed messages as frame: response
            print("Diagnostic Message Received but failed validation")
            diag_failed.update({diag_frame: diag_recv})
    
    #all candidate messages have been attempted and nothing has been received
    if diag_frame == diag_frame_candidate[-1] and not diag_recv:
        print("All Messages attempted, no message received")
    

ser.close()