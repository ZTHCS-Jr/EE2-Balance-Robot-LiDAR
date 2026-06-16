import serial
import math
import plotext as plt
import time

PORT = '/dev/serial0'
BAUD_RATE = 230400

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"connected to {ser.name}")
except:
    print(f"failed to connect")

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
        x_coords=[]
        y_coords=[]
        for angle, radius in scanData.values():
            x = radius * math.sin(angle)
            y = radius * math.cos(angle)
            x_coords.append(x)
            y_coords.append(y)

        plt.clt()
        plt.cld()
        
        plt.scatter(x_coords, y_coords, marker="dot")
        plt.title("OKDO Lidar Live Terminal Plot")
        plt.plotsize(100, 190) # Width, Height in terminal characters
        
        plt.xlim(-2000, 2000)
        plt.ylim(-2000, 2000)
        
        plt.show()
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting and closing port...")
    ser.close()