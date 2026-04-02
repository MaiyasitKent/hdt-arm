import dynamixel_sdk as dxl
import time

PORT  = "/dev/ttyUSB0"
BAUD  = 57600
IDS   = [0, 1, 2, 3, 4]

ph = dxl.PortHandler(PORT)
pk = dxl.PacketHandler(2.0)
ph.openPort()
ph.setBaudRate(BAUD)

print("Pinging each servo 5 times...\n")
for ID in IDS:
    ok = 0
    for _ in range(5):
        model, result, error = pk.ping(ph, ID)
        if result == dxl.COMM_SUCCESS:
            ok += 1
        time.sleep(0.05)
    status = "OK" if ok == 5 else f"FAIL ({ok}/5)"
    print(f"  ID {ID}: {status}")

ph.closePort()