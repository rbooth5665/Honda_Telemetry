import serial
import serial.tools.list_ports
import time
import os
import csv
from datetime import datetime
from pylibftdi import Driver

#--------- Helper Methods ---------#
def list_serial_ports():
    ports = serial.tools.list_ports.comports()  
    if not ports:
        print("No serial ports found.")
    
    return ports

def print_device_info():
    for device in Driver().list_devices():
        print(device)

def initilization_protocol():



folder = "sensor_data"

if not os.path.exists(folder):
    os.makedirs(folder)

ports = serial.tools.list_ports.comports()

if not ports:
    while not ports:
        print("No serial ports found. Retrying...")
        ports = serial.tools.list_ports.comports()
        time.sleep(1)

for port in ports:
    print("Port Found!")
    print(f"Port: {port.device}")
    print(f"Description: {port.description}")
    print(f"HWID: {port.hwid}")
    print(f"manufacturer: {port.manufacturer}")

try:
    #opens the serial port, flushes buffer
    ser = serial.Serial(port.device, 9600, timeout = 1)
    ser.reset_input_buffer()

    #waits for serial connection, flushes any wakeup data
    time.sleep(2)
    ser.reset_input_buffer()

    #creates timestamped filename for csv output, joins with folder path
    filename = f"{datetime.now().strftime('%b-%d-%Y_%I-%M%p')}.csv"
    filepath = os.path.join(folder, filename)

    with open(filepath, "w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "potValue"])
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                
                if line:
                    print(line)
                    writer.writerow([datetime.now().strftime("%H:%M:%S.%f"), line])
                    file.flush()

except serial.SerialException as e:
    print(f"Error: {e}")
    exit()
except KeyboardInterrupt:
    print("Exiting...")
    exit()

finally:
    if ser.is_open: 
        ser.close()


