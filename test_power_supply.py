import dynamixel_sdk as dxl
import time

PORT = "/dev/ttyUSB0"
BAUD = 57600
IDS  = [0, 1, 2, 3, 4]

ADDR_VOLTAGE = 144  # Present Input Voltage (X-series)
ADDR_TEMP    = 146  # Present Temperature

ph = dxl.PortHandler(PORT)
pk = dxl.PacketHandler(2.0)
ph.openPort()
ph.setBaudRate(BAUD)

print(f"{'ID':<5} {'Voltage(V)':<12} {'Temp(C)':<10}")
print("-" * 30)
for ID in IDS:
    v, r1, _ = pk.read2ByteTxRx(ph, ID, ADDR_VOLTAGE)
    t, r2, _ = pk.read1ByteTxRx(ph, ID, ADDR_TEMP)
    voltage = v / 10.0 if r1 == dxl.COMM_SUCCESS else -1
    temp    = t        if r2 == dxl.COMM_SUCCESS else -1
    print(f"  {ID:<3} {voltage:<12.1f} {temp:<10}")

ph.closePort()