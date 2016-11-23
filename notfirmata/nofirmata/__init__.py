import struct
import serial
from kbhit import KBHit
from time import sleep

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=3)
kb = KBHit()

def slow_write(data):
    for byte in data:
        byte = struct.pack("B", byte)
        ser.write(byte)
        print("wrote", byte)
        sleep(0.05)
    print("done")

while True:
    slow_write(b"\xaa\xaa\xff\xff\xff")
    sleep(1)
    slow_write(b"\xaa\xaa\x00\x00\x00")
    sleep(1)
    if ser.inWaiting() > 0:
        data_str = ser.read(ser.inWaiting())
        print(' '.join('{:02x}'.format(x) for x in data_str))
    continue
    for x in range(0, 0xff, 10):
        packet = struct.pack("5B", 0xaa, 0xaa, x, x, x)
        print(">", x, end=': ')
        print(' '.join('{:02x}'.format(x) for x in packet), end='')
        ser.write(packet)
        sleep(0.5)
        if ser.inWaiting() > 0:
            data_str = ser.read(ser.inWaiting())
            print(" <", ' '.join('{:02x}'.format(x) for x in data_str))
            #print(struct.unpack('5B', data_str), end='')
        else:
            pass
            #print("nodata", end='')
        #print()

while True:
    if kb.kbhit():
        c = kb.getch()
        print(c)
    if ser.inWaiting() > 0:
        data_str = ser.read(ser.inWaiting()).decode('ascii')
        print(data_str, end='')
