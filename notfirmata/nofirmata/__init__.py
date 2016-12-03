import struct
import serial
from kbhit import KBHit
from time import sleep
import itertools

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
        #sleep(0.05)


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
    final = struct.pack("7B", *final)
    slow_write(final)
    return final

def speed(*speeds, grabber=False, kicker=False):
    enables = []
    directions = []
    pwms = []
    for n, speed in enumerate(speeds):
        enables.append(True)
        directions.append(speed > 0)
        pwm = 24 + abs(int(float(speed) * 205))
        pwms.append(pwm & 0xff)
    bitlist = itertools.chain((kicker, grabber), directions[::-1], enables[::-1])
    bits = "".join(["1" if i else "0" for i in bitlist][::1])
    bits = int(bits, 2)
    print(bin(bits), end=" ")
    pwms.append(bits)
    write_packet(pwms)

s = int(64)
#speed(127, 127, 127)
#speed(0, 127, 0)
#speed(s, s, s)
#exit(0)
grabber = True
while True:
    grabber = not grabber
    print(grabber)
    speed(0.1, 0.1, 0.1, grabber=grabber, kicker=True)
    sleep(0.01)
step = 1
r = 75
l = list(range(-r, r, step))
l.extend(list(range(r, -r, -step)))
for s in range(0, -r, -step):
    x = s/100
    print(">", "{}".format(x), end=': ')
    speed(x, x, x)
    sleep(0.01)
while True:
    #slow_write(b"\xaa\xaa\xff\xff\xff")
    #sleep(1)
    #slow_write(b"\xaa\xaa\x00\x00\x00")
    #sleep(1)
    #if ser.inWaiting() > 0:
    #    data_str = ser.read(ser.inWaiting())
    #    print(' '.join('{:02x}'.format(x) for x in data_str))
    #continue

    for x in l:
        x = x/100;
        print(">", "{}".format(x), end=': ')
        speed(x, x, x)
        sleep(0.01)
        if ser.inWaiting() > 0:
            data_str = ser.read(ser.inWaiting())
            print("<", data_str.decode())
            #print(" <", ' '.join('{:02x}'.format(x) for x in data_str))
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
