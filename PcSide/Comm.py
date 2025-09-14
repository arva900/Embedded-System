import time

import serial
import numpy as np


# ---------- CONSTANTS ----------
TICK_TIME_US = 0.666667   # 1/1.5MHz → 0.667 [µs]
SOUND_CM_PER_US = 0.034645      # speed of sound in [cm/µs]
ACK = 0xFF
EXIT = 0xFE
PORT = 'COM5'
BAUD_RATE = 9600
ser=None
calib_adc=np.zeros(10, dtype=np.uint16)
calib_dists=np.array([0,5,10,15,20,25,30,35,40,45])

# ---------- HELPERS ----------
def calc_checksum(frame):
    checksum = sum(frame) & 0xFF
    return (-checksum) & 0xFF
def read_byte():
    if ser is None: return print("no UART connection")
    byte=ser.read(1)
    return byte[0]
def send_byte(byt):
    if ser is None: return print("no UART connection")
    bytes_written = ser.write(byt)  # Convert to bytearray and send
    if bytes_written != 1: print("Timeout")

def send_frame(payload):
    if ser is None:
        print("No UART connection")
        return False
    frame = payload[:] + [calc_checksum(payload)]
    retries = 0
    while retries < 10:
        try:
            bytes_written = ser.write(bytearray(frame)) # Convert to bytearray and send
            if bytes_written != len(frame):
                print(f"Timeout: Sent {bytes_written}/{len(frame)} bytes")
                retries += 1
                continue
            print(f"Sent:{frame} ")
            # Wait for ACK
            ack = ser.read(1)
            if ack and ack[0] == ACK:
                print(f"ACK Received")
            return True
        except (serial.SerialTimeoutException, serial.SerialException) as e:
            print(f"Serial error: {e}, attempt {retries + 1}/10")
            retries += 1
    print("Max retries reached, transmission failed")
    return False

def read_frame(n,state_changed):
    if ser is None:
        print("No UART connection")
        return False
    retries = 0
    while retries < 5:
        try:
            # Read n+1 bytes (n bytes payload + 1 byte checksum)
            frame = ser.read(n + 1)
            # Check if we got complete frame
            if len(frame) != n + 1:
                print(f"Timeout: Received {len(frame)}/{n + 1}bytes")
                retries += 1
                continue
            # Convert to list and calculate checksum
            payload = list(frame)
            checksum = sum(payload) & 0xFF
            if checksum == 0:
                # Valid frame received, send ACK
                try:
                    send_byte([ACK-state_changed])
                    print("Received, ACK Sent")#{payload}
                    return payload[:-1]  # Return payload only
                except serial.SerialTimeoutException:
                    print("Timeout sending ACK after Read")
                    retries += 1
            else:
                print(f"Checksum error. Frame: {[hex(x) for x in payload]}, Checksum: {hex(checksum)}")
                retries += 1
        except (serial.SerialTimeoutException, serial.SerialException) as e:
            print(f"Serial error: {e}, attempt {retries + 1}/10")
            retries += 1
    print("Max retries reached, receiving failed")
    return [0,0,0]

def getOBJdist(data):
    """Process raw measurement data and return formatted values"""
    ang = data[-1]
    echo = (data[0] << 8) | data[1]  # Combine high and low bytes
    # Convert ticks to distance
    echo_time_us = echo * TICK_TIME_US
    dist_cm = (echo_time_us * SOUND_CM_PER_US) / 2
    if dist_cm >450 or dist_cm<2: dist_cm = 0
    return  int(dist_cm), ang, echo

def getLIGHTdist(data):
    global calib_adc
    ang = data[-1]
    adc = (data[0] << 8) | data[1]  # Combine high and low bytes
    dist_cm = np.interp(adc, calib_adc,calib_dists)
    return  int(dist_cm), ang, adc

def ADCcalib():
    global calib_adc
    frame = read_frame(13,0)
    values = [frame[i] for i in range(10)]  # Copy first 10 values
    # frame 10 → frame[`0..3]
    # frame 11 → frame[4..7]
    for i in range(4):
        msb = (frame[10] >>(6-2*i)) & 0x03
        values[i]  |= (msb << 8)

        msb = (frame[11] >>(6-2*i)) & 0x03
        values[i+4]|= (msb << 8)
    # frame 12 → frame[8..9]
    msb = (frame[12] >> 6) & 0x03
    values[8] |= (msb << 8)

    msb = (frame[12] >> 4) & 0x03
    values[9] |= (msb << 8)
    calib_adc[:] = values
    # calib_adc[0] = values[0]-50
    # calib_adc[1] = values[1]-200
    # calib_adc[2] = values[2]-200
    # calib_adc[3] = values[3] - 200
    # calib_adc[4] = values[4] - 200
    # calib_adc[5] = values[5] - 100
    # calib_adc[6] = values[6] - 10
    # calib_adc[7] = values[7] - 10
    print(f"ADCcalib: {calib_adc}")
def send_script(file_obj,header):
    """
    Reads a text file with instructions and send bytearray of encoded program.
    """
    bytecode = []
    for line in file_obj:
        line = line.strip()
        if not line or line.startswith("#"):continue  # skip empty or comments
        parts = line.split()
        cmd = parts[0].lower()
        if len(parts) > 1 and cmd != "servo_scan":oper = int(parts[1])  # single integer operand
        if cmd == "inc_lcd":
            bytecode.append(0x01)
            bytecode.append(oper)
        elif cmd == "dec_lcd":
            bytecode.append(0x02)
            bytecode.append(oper)
        elif cmd == "rra_lcd":
            bytecode.append(0x03)
            bytecode.append(oper)
        elif cmd == "set_delay":
            bytecode.append(0x04)
            bytecode.append(oper)
        elif cmd == "clear_lcd":
            bytecode.append(0x05)
        elif cmd == "servo_deg":
            bytecode.append(0x06)
            bytecode.append(oper)
        elif cmd == "servo_scan":
            a, b = map(int, parts[1].split(","))
            bytecode.append(0x07)
            bytecode.append(a)
            bytecode.append(b)
        elif cmd == "sleep":
            bytecode.append(0x08)
        else:raise ValueError(f"Unknown instruction: {line}")
    print("writing script file..")
    for char in header:
        #print(f"ch={char}")
        send_byte(char.encode("ascii"))
        time.sleep(0.05)  # msp store byte in flash
    for char in bytecode:
        #print(f"ch={char}")
        send_byte(bytes([char]))
        time.sleep(0.05)  # msp store byte in flash
    send_byte(bytes([ACK]))

def send_txt(file_obj,header):
     clean_file = file_obj.read().replace("\n", "").replace("\r", "").replace("\0", "")
     content = header + clean_file
     print("writing txt file..")
     for char in content:
         #print(f"ch={char}")
         send_byte(char.encode("ascii"))
         time.sleep(0.05)  # msp store byte in flash
     send_byte(bytes([ACK]))

def set_timout(timout):
    global ser
    ser.timeout=timout
def init_serial():
    global ser
    # ---------- CONFIGURE SERIAL PORT ----------
    ser = serial.Serial(
        port=PORT,  # <-- Change to your COM port
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.5,
        write_timeout=0.1
    )
    print(f"Connected to {PORT} at {BAUD_RATE} baud.\n")


