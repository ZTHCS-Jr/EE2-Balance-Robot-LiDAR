"""
Windows-side UDP relay for WSL2 on Windows 10 (no mirrored networking available).

The Raspberry Pi sends sensor UDP to this Windows host's LAN IP. WSL2 is NAT'd,
so those inbound packets never reach the ROS receiver inside WSL on their own.
This relay binds the LAN-facing port on Windows and forwards every datagram to
the WSL VM's current IP, where sensors_udp_receiver.py listens on 0.0.0.0:31415.

Run with WINDOWS python (python.exe), NOT inside WSL:
    python windows_udp_relay.py

- Point the Pi sender's IP at THIS Windows machine's LAN address (`ipconfig`).
- Allow inbound UDP 31415 through Windows Defender Firewall (see the steps).
- The WSL IP changes on `wsl --shutdown`/reboot; just restart this relay then.
"""
import socket
import subprocess

LISTEN_PORT = 31415   # port the Pi sends to (on this Windows host)
DEST_PORT = 31415     # port sensors_udp_receiver.py binds inside WSL


def wsl_ip():
    # Ask WSL for its current eth0 address (first token of `hostname -I`).
    out = subprocess.check_output(["wsl", "hostname", "-I"], text=True)
    return out.split()[0]


def main():
    dest = wsl_ip()
    print(f"Relaying UDP 0.0.0.0:{LISTEN_PORT}  ->  {dest}:{DEST_PORT} (WSL)")

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", LISTEN_PORT))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    count = 0
    while True:
        data, _ = rx.recvfrom(65536)
        tx.sendto(data, (dest, DEST_PORT))
        count += 1
        if count % 200 == 0:
            print(f"forwarded {count} packets")


if __name__ == "__main__":
    main()
