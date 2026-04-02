import dynamixel_sdk as dxl
import time

PORT = "/dev/ttyUSB0"
BAUD = 57600
IDS  = [0, 1, 2, 3, 4]

ADDR_PRESENT_POS = 132
LEN_PRESENT_POS  = 4

ph = dxl.PortHandler(PORT)
pk = dxl.PacketHandler(2.0)
ph.openPort()
ph.setBaudRate(BAUD)

groupSyncRead = dxl.GroupSyncRead(ph, pk, ADDR_PRESENT_POS, LEN_PRESENT_POS)
for ID in IDS:
    groupSyncRead.addParam(ID)

print("SyncRead 20 ครั้ง...\n")
ok = 0
fail = 0
for i in range(20):
    result = groupSyncRead.txRxPacket()
    if result != dxl.COMM_SUCCESS:
        fail += 1
        print(f"  [{i:02d}] FAIL: {pk.getTxRxResult(result)}")
    else:
        ok += 1
        positions = [groupSyncRead.getData(ID, ADDR_PRESENT_POS, LEN_PRESENT_POS) for ID in IDS]
        print(f"  [{i:02d}] OK: {positions}")
    time.sleep(0.02)  # 50Hz

print(f"\nสรุป: OK={ok}  FAIL={fail}/{ok+fail}")
ph.closePort()