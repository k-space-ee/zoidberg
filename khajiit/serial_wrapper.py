import serial.tools.list_ports


def find_serial(name) -> dict:
    ports = serial.tools.list_ports.comports()
    result = dict((port.device, port) for port in ports if port.product and name.lower() in port.product.lower())
    return result


def info(ports):
    for k, p in ports.items():
        print()
        print(p.device)
        print(p.name)
        print(p.description)
        print(p.hwid)
        print(p.vid)
        print(p.pid)
        print(p.serial_number)
        print(p.location)
        print(p.manufacturer)
        print(p.product)
        print(p.interface)


if __name__ == '__main__':
    info(find_serial(''))
