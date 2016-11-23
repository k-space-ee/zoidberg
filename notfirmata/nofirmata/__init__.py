import struct
import serial
from kbhit import KBHit
from time import sleep

ser = serial.Serial("/dev/ttyACM0", 9600,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1,
                    xonxoff=0,
                    rtscts=0)
print("reset arduino")
ser.setDTR(False)
sleep(0.022)
ser.flushInput()
ser.setDTR(True)
print("wait for arduino boot to end")
sleep(1)
print("Done")
kb = KBHit()

def slow_write(data):
    for byte in data:
        byte = struct.pack("B", byte)
        ser.write(byte)
        #print("wrote", byte)
        sleep(0.1)


def write_packet(packet):
    checksum = 0
    final = [0xaa, 0xaa]
    for byte in packet:
        final.append(byte)
        checksum += byte & 0xff
        if checksum > 255:
            checksum &= 0x00FF
    final.append(checksum)
    print(' '.join('{:02x}'.format(x) for x in final))
    final = struct.pack("6B", *final)
    slow_write(final)
    return final

def speed(m1, m2, m3):
    write_packet((int(m1)&0xff, int(m2)&0xff, int(m3)&0xff))

s = int(64)
#speed(127, 127, 127)
#speed(0, 127, 0)
#speed(s, s, s)
#exit(0)
while True:
    #slow_write(b"\xaa\xaa\xff\xff\xff")
    #sleep(1)
    #slow_write(b"\xaa\xaa\x00\x00\x00")
    #sleep(1)
    #if ser.inWaiting() > 0:
    #    data_str = ser.read(ser.inWaiting())
    #    print(' '.join('{:02x}'.format(x) for x in data_str))
    #continue
    step = 1
    l = list(range(-128, 127, step))
    l.extend(list(range(127, -128, -step)))

    for x in l:
        print(">", "{:02x}".format(x), end=': ')
        speed(x, x, x)
        sleep(0.00)
        if ser.inWaiting() > 0:
            data_str = ser.read(ser.inWaiting())
            print(" <", ' '.join('{:02x}'.format(x) for x in data_str))
            #print(struct.unpack('5B', data_str), end='')
        else:
            pass
            #print("nodata", end='')

while True:
    if kb.kbhit():
        c = kb.getch()
        print(c)
    if ser.inWaiting() > 0:
        data_str = ser.read(ser.inWaiting()).decode('ascii')
        print(data_str, end='')
