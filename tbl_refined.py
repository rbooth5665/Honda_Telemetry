"""
speed_hunt_logger.py - Focused full-rate logger for the speed hunt.

Polls THREE tables EVERY loop (no round-robin), so the
RPM-independent candidate bytes get sampled fast enough for a clean
deceleration test:

    engine (0x1100)  -> RPM/TPS reference (every row)
    D000   (0xD000)  -> the rich new table (10 movers, incl. b12)
    D200   (0xD200)  -> the single-mover table (b0, 0-180)

Unlike the round-robin scanner, this polls all three on every loop,
so D000/D200 bytes update at the full sample rate. That makes the
deceleration test reliable: a SPEED byte stays elevated while RPM
drops during a coast; an RPM-correlated byte falls with RPM.

SAFETY: all three requests are read-only 0x72 commands. Only the
address differs. No write/erase/security commands anywhere.

Usage on the Pi:
    sudo systemctl stop honda-logger      # free the serial port
    cd ~/Honda_Log
    source venv/bin/activate
    python speed_hunt_logger.py
    # ride with VARIED speeds + COASTING + a full STOP, Ctrl+C to end
"""

import time
import csv
import os
from datetime import datetime
from ECU import ECU


# All read-only 0x72 requests. Verified checksums.
ENGINE_REQUEST = [0x72, 0x07, 0x72, 0x11, 0x00, 0x14, 0xF0]
D000_REQUEST   = [0x72, 0x07, 0x72, 0xD0, 0x00, 0x14, 0x31]
D200_REQUEST   = [0x72, 0x07, 0x72, 0xD2, 0x00, 0x14, 0x2F]

PAYLOAD_WIDTH = 20  # bytes of payload to record per table


def payload(frame, width=PAYLOAD_WIDTH):
    """Strip 5-byte header + 1-byte checksum; pad/truncate to width.
    Returns width copies of -1 if the response is missing/invalid."""
    if frame is None or len(frame) < 6:
        return [-1] * width
    p = list(frame[5:-1])
    if len(p) < width:
        p += [0] * (width - len(p))
    return p[:width]


def main():
    ecu = ECU()
    if not ecu.connect():
        print("Could not connect to ECU.")
        return

    os.makedirs("logs", exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    path = os.path.join("logs", f"speed_hunt_{stamp}.csv")
    f = open(path, "w", newline="")
    w = csv.writer(f)
    print(f"speed-hunt logging -> {path}")

    # Header: engine bytes (e0..), D000 bytes (d0..), D200 bytes (p0..)
    header = (["sample", "elapsed_s", "wall_clock"]
              + [f"e{i}" for i in range(PAYLOAD_WIDTH)]
              + [f"d{i}" for i in range(PAYLOAD_WIDTH)]
              + [f"p{i}" for i in range(PAYLOAD_WIDTH)])
    w.writerow(header)

    start = time.time()
    sample = 0

    try:
        while True:
            # Poll all three tables EVERY loop (full rate)
            e = payload(ecu.send_recv(ENGINE_REQUEST, 26))
            d = payload(ecu.send_recv(D000_REQUEST, 26))
            p = payload(ecu.send_recv(D200_REQUEST, 26))

            elapsed = round(time.time() - start, 3)
            wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            w.writerow([sample, elapsed, wall] + e + d + p)
            sample += 1

            # heartbeat every ~5s so you can confirm it's alive + reading
            if sample % 50 == 0:
                rpm = (e[0] << 8) | e[1] if e[0] >= 0 else -1
                print(f"sample {sample}  rpm {rpm}  d12 {d[12]}  p0 {p[0]}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping")
    finally:
        f.close()
        print(f"logged {sample} samples to {path}")
        ecu.close()


if __name__ == "__main__":
    main()