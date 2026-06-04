import serial
import time

esp_port = '/dev/ttyUSB0' 
baud_rate = 115200

try:
    ser = serial.Serial(esp_port, baud_rate, timeout=0.1)
    print(f"Successfully connected to {esp_port}")
except Exception as e:
    print(f"Failed to connect: {e}")
    exit()

# read from port
try:
    while True:
        # Check if there are bytes waiting in the hardware buffer
        if ser.in_waiting > 0:
            # decode the bytes into a string, strip invisible newline characters (\r\n)
            raw_line = ser.readline().decode('utf-8').strip()
            
            if raw_line.startswith("IMU:"):
                try:
                    gyro_value_str = raw_line.split(":")[1]
                    gyro_x_dps = float(gyro_value_str)
                    
                    print(f"Received Yaw Speed: {gyro_x_dps} degrees/sec")
                    
                except ValueError:
                    print(f"Corrupt data received: {raw_line}")
                    
except KeyboardInterrupt:
    print("\nClosing port.")
    ser.close()