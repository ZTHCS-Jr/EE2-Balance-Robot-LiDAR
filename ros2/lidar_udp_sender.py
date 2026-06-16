import serial
import math
import time
import socket
import json
import sys

PORT = '/dev/serial0'
BAUD_RATE = 230400
IP = "192.168.0.20"
UDP_PORT = 31415
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"connected to {IP}:{UDP_PORT}")

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"connected to {ser.name}")
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
    while True:
        scanData={} # points for a single 360 sweep
        packetsRead=0
        while packetsRead<150: # four 360 sweeps for redundancy
            if ser.in_waiting>=47 and ser.read(1)[0]==0x54:
                remaining = ser.read(46)
                if len(remaining)==46:
                    packet = bytes([0x54]) + remaining
                    points = parse_packet(packet)
                    for angle, radius, intensity in points:
                        if 0 < radius < 2000 and intensity >= 30:
                            deg = int(math.degrees(angle))
                            scanData[deg] = (angle, radius)
                    packetsRead+=1

        payload = json.dumps(scanData)
        sock.sendto(payload.encode("utf-8"), (IP, UDP_PORT))
        print(f"sent scan to {IP}")

except KeyboardInterrupt:
    print("\nExit and closed port")
    ser.close()