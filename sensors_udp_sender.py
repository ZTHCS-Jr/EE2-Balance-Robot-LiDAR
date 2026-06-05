import serial
import math
import time
import socket
import json
import sys

SERIAL_PORT = '/dev/serial0'
USB_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 230400
IMU_BAUD = 115200
IP = "10.144.216.133"
UDP_PORT = 31415

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"connected to {IP}:{UDP_PORT}")

try:
    lidar_ser = serial.Serial(SERIAL_PORT, LIDAR_BAUD, timeout=1)
    print(f"connected to {lidar_ser.name}")
except Exception as e:
    print(f"failed to connect: {e}")
    sys.exit(1) 

try:
    imu_ser = serial.Serial(USB_PORT, IMU_BAUD, timeout=0.01)
    print(f"connected to {imu_ser.name}")
except Exception as e:
    print(f"failed to connect: {e}")
    sys.exit(1)

def parse_packet(packet):
    # check packet size, header value, VerLen value
    if len(packet)!=47 or packet[0]!=0x54 or packet[1]!=0x2C:
        return []
    
    startAngle = (packet[5] << 8 | packet[4])/100.0
    endAngle = (packet[43] << 8 | packet[42])/100.0

    if endAngle < startAngle:
        step = (360-(startAngle-endAngle))/11.0
    else:
        step = (endAngle-startAngle)/11.0

    points = []
    for i in range(12):
        base = 6+(i*3) # data starts at index 6
        distance = packet[base+1] << 8 | packet[base]
        intensity = packet[base+2]
        angle = startAngle + step*i
        if angle >= 360.0: angle-=360.0
        points.append((math.radians(angle), distance, intensity))

    return points

time.sleep(1)

try:
    scanData = {}
    packetsRead = 0

    while True:
        # motor and imu send data at 50hz
        while imu_ser.in_waiting > 0:
            raw_line = imu_ser.readline().decode('utf-8', errors='ignore').strip()
            
            if raw_line.startswith("IMU:"):
                try:
                    latest_gyro_x = float(raw_line.split(":")[1])
                    imu_payload = { "imu": { "gyro_x": latest_gyro_x } }
                    message = json.dumps(imu_payload)
                    sock.sendto(message.encode("utf-8"), (IP, UDP_PORT))
                except ValueError:
                    pass 
            
            elif raw_line.startswith("MOTOR:"):
                try:
                    parts = raw_line.split(":")[1].split(",")
                    # uncomment for test using getSpeed() in controller
                    # left_rad = float(parts[0])
                    # right_rad = float(parts[1])
                    # odom_payload = { "odom": { "left_rad": left_rad, "right_rad": right_rad } }
                    left_steps = int(parts[0])
                    right_steps = int(parts[1])
                    odom_payload = { "odom": { "left_steps": left_steps, "right_steps": right_steps } }
                    message = json.dumps(odom_payload)
                    sock.sendto(message.encode("utf-8"), (IP, UDP_PORT))
                except (ValueError, IndexError):
                    pass 

        # lidar send data at 2.5hz
        if lidar_ser.in_waiting >= 47 and lidar_ser.read(1)[0] == 0x54:
            remaining = lidar_ser.read(46)
            if len(remaining) == 46:
                packet = bytes([0x54]) + remaining
                points = parse_packet(packet)
                
                for angle, radius, intensity in points:
                    if 0 < radius < 2000 and intensity >= 30:
                        deg = int(math.degrees(angle))
                        scanData[deg] = (angle, radius)
                packetsRead += 1

                if packetsRead >= 150:
                    lidar_payload = { "lidar": scanData }
                    message = json.dumps(lidar_payload)
                    sock.sendto(message.encode("utf-8"), (IP, UDP_PORT))
                    scanData = {}
                    packetsRead = 0

except KeyboardInterrupt:
    print("\nExit and closed port")
    lidar_ser.close()
    imu_ser.close()