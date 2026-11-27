from machine import UART, Pin

# UPDIは 8bit / 偶数パリティ / ストップ2bit
u = UART(0, baudrate=115200, bits=8, parity=0, stop=2, tx=Pin(0), rx=Pin(1))

def name(b):
    if b == 0x55:
        return "SYNCH"
    return {0x80:"LDCS", 0xC0:"STCS", 0xE0:"KEY", 0x00:"LDS",
            0x40:"STS", 0x20:"LD", 0x60:"ST", 0xA0:"REPEAT"}.get(b & 0xE0, "")

print("UPDI sniffer start")
prev = None
while True:
    if u.any():
        for b in u.read():
            tag = name(b) if (prev == 0x55 or b == 0x55) else ""
            print("%02X %s" % (b, tag))
            prev = b