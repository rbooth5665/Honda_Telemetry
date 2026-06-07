import serial
import serial.tools.list_ports
import time

#--------- Methods ---------#
def find_serial_port():
    for ports in serial.tools.list_ports.comports():
        desc = ports.description.lower()
        manf = ports.manufacturer.lower()
        if "ftdi" in desc or "ftdi" in manf:
            return ports.device
    return None

#--------- Program ---------#
PORT = find_serial_port() # finds the serial FTDI port
BAUD = 10400 #Honda baud
print(f"Using port: {PORT}")

try:
    ser = serial.Serial(PORT, BAUD, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=1)

except serial.SerialException as e:
    print(f"FAILED to open serial {PORT}: {e}")
    raise SystemExit(1)


print(PORT)

print("Connect probes")
print(ser.is_open)
time.sleep(10)


try:
    #phase 1 high
    print("=" * 45)
    print("Phase 1 IDLE - show voltage")
    ser.break_condition = False
    time.sleep(15)

    #phase 2 low
    print("=" * 45)
    print("Phase 2 LOW - no voltage")
    ser.break_condition = True
    time.sleep(15)

    #phase 3 high
    print("=" * 45)
    print("Phase 1 IDLE - show voltage")
    ser.break_condition = False
    time.sleep(15)

finally:
    ser.close()
    print("Test done")

