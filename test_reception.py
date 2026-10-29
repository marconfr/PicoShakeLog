#!/usr/bin/env python3
"""
PicoShakeLog - Test de réception WiFi TCP
Protocole : CMD_START='S', CMD_STOP='Q', RESP_ACK='A', RESP_NAK='N'

Format FIFO Packet 4 (20 bytes, big-endian) :
  [0]      Header
  [1-2]    Accel X  (int16 BE)
  [3-4]    Accel Y
  [5-6]    Accel Z
  [7-8]    Gyro X   (int16 BE)
  [9-10]   Gyro Y
  [11-12]  Gyro Z
  [13-14]  Temperature (int16 BE) -> C = raw/132.48 + 25
  [15-17]  Timestamp (20-bit sur 3 bytes)
  [18-19]  Extension

Facteurs de conversion (FIFO hi-res, +-4g / +-500dps par defaut) :
  Accel : 1/8192 g/LSB
  Gyro  : 1/65.5 dps/LSB (+-500dps)

Usage:
    python3 test_reception.py [--duration 5] [--ip 192.168.4.1] [--port 4242]
                              [--slot 1280] [--show 3]
"""

import socket
import struct
import time
import argparse

# --- Protocole ----------------------------------------------------------------
CMD_START    = b'S'
CMD_STOP     = b'Q'
RESP_ACK     = ord('A')   # 0x41
RESP_NAK     = ord('N')   # 0x4E

# --- Conversion ICM42688 FIFO Packet 4 ----------------------------------------
PACKET_SIZE = 20  # FIFO_PACKET_4_SIZE


def _timestamp_20b(ts_bytes, little_endian):
    """Decode 20-bit timestamp from 3 bytes depending on sensor data endianness."""
    if little_endian:
        # In little-endian mode bytes are LSB first.
        val = ts_bytes[0] | (ts_bytes[1] << 8) | (ts_bytes[2] << 16)
        return val & 0xFFFFF
    else:
        # Big-endian layout as in datasheet figure.
        return (ts_bytes[0] << 12) | (ts_bytes[1] << 4) | (ts_bytes[2] >> 4)


def parse_imu_packet(data, accel_g_per_lsb, gyro_dps_per_lsb, little_endian=False, hires=False):
    """Parse un paquet FIFO Packet 4 de 20 bytes (endianness configurable)."""
    if len(data) < PACKET_SIZE:
        return None

    header = data[0]
    if hires:
        # MDATA.py mapping: support both big-endian and little-endian word layouts.
        if little_endian:
            # base words are LSB first
            accelX = (data[2] << 10) | (data[1] << 2) | ((data[17] >> 6) & 0x03)
            accelY = (data[4] << 10) | (data[3] << 2) | ((data[18] >> 6) & 0x03)
            accelZ = (data[6] << 10) | (data[5] << 2) | ((data[19] >> 6) & 0x03)

            gyroX = (data[8] << 11) | (data[7] << 3) | ((data[17] >> 1) & 0x07)
            gyroY = (data[10] << 11) | (data[9] << 3) | ((data[18] >> 1) & 0x07)
            gyroZ = (data[12] << 11) | (data[11] << 3) | ((data[19] >> 1) & 0x07)

            temp_raw = (data[14] << 8) | data[13]
            if temp_raw & 0x8000:
                temp_raw -= 0x10000

            timestamp = (data[16] << 8) | data[15]
        else:
            # base words are MSB first (big-endian)
            accelX = (data[1] << 10) | (data[2] << 2) | ((data[17] >> 6) & 0x03)
            accelY = (data[3] << 10) | (data[4] << 2) | ((data[18] >> 6) & 0x03)
            accelZ = (data[5] << 10) | (data[6] << 2) | ((data[19] >> 6) & 0x03)

            gyroX = (data[7] << 11) | (data[8] << 3) | ((data[17] >> 1) & 0x07)
            gyroY = (data[9] << 11) | (data[10] << 3) | ((data[18] >> 1) & 0x07)
            gyroZ = (data[11] << 11) | (data[12] << 3) | ((data[19] >> 1) & 0x07)

            temp_raw = (data[13] << 8) | data[14]
            if temp_raw & 0x8000:
                temp_raw -= 0x10000

            timestamp = (data[15] << 8) | data[16]

        # sign-extend accel (18-bit) and gyro (19-bit)
        if accelX & 0x20000:
            accelX -= 0x40000
        if accelY & 0x20000:
            accelY -= 0x40000
        if accelZ & 0x20000:
            accelZ -= 0x40000

        if gyroX & 0x40000:
            gyroX -= 0x80000
        if gyroY & 0x40000:
            gyroY -= 0x80000
        if gyroZ & 0x40000:
            gyroZ -= 0x80000

    else:
        endian_prefix = "<" if little_endian else ">"
        ax_raw, ay_raw, az_raw = struct.unpack_from(f"{endian_prefix}hhh", data, 1)
        gx_raw, gy_raw, gz_raw = struct.unpack_from(f"{endian_prefix}hhh", data, 7)
        temp_raw = struct.unpack_from(f"{endian_prefix}h", data, 13)[0]
        ts_bytes = data[15:18]
        timestamp = _timestamp_20b(ts_bytes, little_endian)

    # Map names to variables depending on branch
    if hires:
        ax_val, ay_val, az_val = accelX, accelY, accelZ
        gx_val, gy_val, gz_val = gyroX, gyroY, gyroZ
        temp_val = temp_raw
    else:
        ax_val, ay_val, az_val = ax_raw, ay_raw, az_raw
        gx_val, gy_val, gz_val = gx_raw, gy_raw, gz_raw
        temp_val = temp_raw

    return {
        "header": header,
        "ax":     ax_val * accel_g_per_lsb,
        "ay":     ay_val * accel_g_per_lsb,
        "az":     az_val * accel_g_per_lsb,
        "gx":     gx_val * gyro_dps_per_lsb,
        "gy":     gy_val * gyro_dps_per_lsb,
        "gz":     gz_val * gyro_dps_per_lsb,
        "temp_c": temp_val / 132.48 + 25.0,
        "ts":     timestamp,
    }


# ---------------------------------------------------------------------------
# Conversion factors (match firmware behavior)

ACCEL_RANGE_SCALE = {
    2:  1.0 / 16384.0,
    4:  1.0 / 8192.0,
    8:  1.0 / 4096.0,
    16: 1.0 / 2048.0,
}

GYRO_RANGE_SCALE = {
    15.625: 1.0 / 2097.2,
    31.25:  1.0 / 1048.6,
    62.5:   1.0 / 524.3,
    125:    1.0 / 262.0,
    250:    1.0 / 131.0,
    500:    1.0 / 65.5,
    1000:   1.0 / 32.8,
    2000:   1.0 / 16.4,
}


def compute_scales(fifo_hires: bool, accel_range_g: int, gyro_range_dps: float):
    """Return (accel_g_per_lsb, gyro_dps_per_lsb) matching firmware rules.

    In FIFO hi-res, scales are fixed (1/8192 g/LSB, 1/131 dps/LSB) independent of ranges.
    Otherwise, use range-dependent scales.
    """
    if fifo_hires:
        return 1.0 / 8192.0, 1.0 / 131.0

    accel = ACCEL_RANGE_SCALE.get(accel_range_g, 1.0 / 8192.0)
    gyro  = GYRO_RANGE_SCALE.get(gyro_range_dps, 1.0 / 65.5)
    return accel, gyro


# --- Fonctions ----------------------------------------------------------------

def send_start(sock, timeout=3.0):
    """
    Envoie CMD_START et attend l'ACK.
    Robuste : lit jusqu'a plusieurs KB pour trouver l'ACK meme si des donnees
    IMU arrivent dans le meme segment TCP que la reponse.
    Retourne (ack_code, leftover_bytes).
    """
    sock.sendall(CMD_START)
    sock.settimeout(timeout)
    buf = b""
    try:
        while len(buf) < 8192:
            chunk = sock.recv(8192)
            if not chunk:
                break
            buf += chunk
            if len(buf) >= 1:
                break   # Premier byte = reponse, le reste = donnees IMU
    except socket.timeout:
        print("  -> Timeout: pas de reponse")
        return None, b""
    except ConnectionResetError:
        print("  -> ConnectionReset: le Pico a un PCB fantome (stale client_pcb)")
        return 'RESET', b""

    if not buf:
        return None, b""

    code     = buf[0]
    leftover = buf[1:]
    names = {RESP_ACK: 'ACK', RESP_NAK: 'NAK', ord('R'): 'RUNNING', ord('I'): 'IDLE'}
    print(f"  -> Reponse: {names.get(code, f'0x{code:02X}')}")
    return code, leftover


def receive_stream(sock, duration_s, slot_size, show_slots, prefill=b"", little_endian=False,
                   accel_g_per_lsb=1.0 / 8192.0, gyro_dps_per_lsb=1.0 / 131.0, hires=False):
    """
    Recoit le stream IMU pendant duration_s secondes.
    prefill : bytes deja recus avec l'ACK START.
    """
    packets_per_slot = slot_size // PACKET_SIZE
    print(f"\n[STREAM] Reception pendant {duration_s}s "
          f"(slot={slot_size}B = {packets_per_slot}x{PACKET_SIZE}B paquets IMU)...")
    sock.settimeout(2.0)

    total_bytes   = len(prefill)
    total_slots   = 0
    total_packets = 0
    buffer        = prefill
    t_start       = time.time()
    t_last_report = t_start

    try:
        while (time.time() - t_start) < duration_s:
            try:
                chunk = sock.recv(8192)
                if not chunk:
                    print("[STREAM] Connexion fermee par le Pico")
                    break
                buffer      += chunk
                total_bytes += len(chunk)

                while len(buffer) >= slot_size:
                    slot          = buffer[:slot_size]
                    buffer        = buffer[slot_size:]
                    total_slots  += 1
                    total_packets += packets_per_slot

                    if total_slots <= show_slots:
                        print(f"\n  -- Slot #{total_slots} --")
                        for i in range(min(3, packets_per_slot)):
                            raw = slot[i*PACKET_SIZE:(i+1)*PACKET_SIZE]
                            p   = parse_imu_packet(raw, accel_g_per_lsb, gyro_dps_per_lsb,
                                                   little_endian=little_endian, hires=hires)
                            if p:
                                print(f"  Pkt[{i:02d}] hdr=0x{p['header']:02X} "
                                      f"A({p['ax']:+.3f} {p['ay']:+.3f} {p['az']:+.3f})g "
                                      f"G({p['gx']:+7.2f} {p['gy']:+7.2f} {p['gz']:+7.2f})dps "
                                      f"T={p['temp_c']:.1f}C ts={p['ts']}")
                            else:
                                print(f"  Pkt[{i:02d}] raw: {raw.hex(' ')}")

                now = time.time()
                if now - t_last_report >= 1.0:
                    elapsed  = now - t_start
                    print(f"  [{elapsed:5.1f}s] {total_slots:5d} slots  "
                          f"{total_packets:7d} paquets  "
                          f"{total_bytes:8d} B  "
                          f"{total_bytes/elapsed/1024:6.1f} KB/s  "
                          f"{total_packets/elapsed:6.0f} Hz")
                    t_last_report = now

            except socket.timeout:
                print("[STREAM] Timeout (pas de donnees depuis 2s)")
                break

    except KeyboardInterrupt:
        print("\n[STREAM] Arrete par l'utilisateur")

    elapsed = time.time() - t_start
    print(f"\n-- Resultats --------------------------------------------------")
    print(f"  Duree         : {elapsed:.2f} s")
    print(f"  Total bytes   : {total_bytes} ({total_bytes/1024:.1f} KB)")
    print(f"  Total slots   : {total_slots}")
    print(f"  Total paquets : {total_packets}")
    if elapsed > 0:
        print(f"  Debit         : {total_bytes/elapsed/1024:.1f} KB/s")
        print(f"  Frequence IMU : {total_packets/elapsed:.0f} Hz  (attendu ~1000 Hz)")
    if total_slots > 0:
        print(f"  Bytes/slot    : {total_bytes/total_slots:.1f}  (attendu {slot_size})")
    if buffer:
        print(f"  Fragment fin  : {len(buffer)} bytes (slot incomplet)")


# --- Main ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="PicoShakeLog TCP reception test")
    parser.add_argument("--ip",       default="192.168.4.1")
    parser.add_argument("--port",     default=4242,  type=int)
    parser.add_argument("--duration", default=5,     type=int,  help="Duree (s)")
    parser.add_argument("--slot",     default=1280,  type=int,  help="Taille slot DMA (bytes)")
    parser.add_argument("--show",     default=3,     type=int,  help="Nb slots a decoder")
    parser.add_argument("--endian",   choices=["be", "le"], default="le",
                        help="Endianness des mots FIFO (bit4 SENSOR_DATA_ENDIAN, bit5 FIFO_COUNT_ENDIAN)")
    parser.add_argument("--fifo-hires", action="store_true", default=True,
                        help="FIFO hi-res active (bit4 FIFO_HIRES_EN)")
    parser.add_argument("--accel-range", choices=[2, 4, 8, 16], type=int, default=4,
                        help="Plage accel (g) si fifo-hires desactive")
    parser.add_argument("--gyro-range", choices=[15.625, 31.25, 62.5, 125, 250, 500, 1000, 2000],
                        type=float, default=500,
                        help="Plage gyro (dps) si fifo-hires desactive")
    args = parser.parse_args()

    print(f"[CONNECT] {args.ip}:{args.port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((args.ip, args.port))
    except (socket.timeout, ConnectionRefusedError) as e:
        print(f"[ERREUR] Connexion impossible: {e}")
        return
    print("[CONNECT] Connecte !\n")

    print("[CMD] START")
    resp, leftover = send_start(sock)

    # Retry une fois si le Pico a rejete la connexion (stale client_pcb)
    if resp == 'RESET':
        print("  -> Attente 1s puis reconnexion...")
        sock.close()
        time.sleep(1.0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        try:
            sock.connect((args.ip, args.port))
        except (socket.timeout, ConnectionRefusedError) as e:
            print(f"[ERREUR] Reconnexion impossible: {e}")
            return
        print("[CONNECT] Reconnecte !\n")
        print("[CMD] START (retry)")
        resp, leftover = send_start(sock)

    if resp != RESP_ACK:
        print("[ERREUR] START refuse ou pas de reponse")
        sock.close()
        return

    little_endian = (args.endian == "le")
    accel_g_per_lsb, gyro_dps_per_lsb = compute_scales(
        fifo_hires=args.fifo_hires, accel_range_g=args.accel_range, gyro_range_dps=args.gyro_range
    )

    receive_stream(sock, args.duration, args.slot, args.show, prefill=leftover,
                   little_endian=little_endian,
                   accel_g_per_lsb=accel_g_per_lsb,
                   gyro_dps_per_lsb=gyro_dps_per_lsb,
                   hires=args.fifo_hires)

    print("\n[CMD] STOP")
    try:
        sock.sendall(CMD_STOP)
        sock.settimeout(2.0)
        leftover = b""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                leftover += chunk
                if any(b == RESP_ACK for b in leftover[-10:]):
                    print("  -> ACK recu")
                    break
            except socket.timeout:
                break
    except OSError:
        pass

    sock.close()
    print("\n[DONE] Connexion fermee.")


if __name__ == "__main__":
    main()
