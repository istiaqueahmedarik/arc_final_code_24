import struct
import crcmod
import serial
import time

# CRC-16-CCITT calculation function
crc16 = crcmod.mkCrcFun(0x11021, initCrc=0xFFFF, rev=False, xorOut=0x0000)

def encode_message(msg_id, body):
    start_byte = 0x7E
    body_length = len(body)
    frame = struct.pack('>BBB', start_byte, msg_id, body_length) + body
    checksum = crc16(frame)
    frame += struct.pack('>H', checksum)
    return frame

def send_message(serial_port, msg_id, body):
    message = encode_message(msg_id, body)
    serial_port.write(message)
    print(f"Sent Message: {message.hex()}")

def read_message(serial_port):
    while True:
        if serial_port.read(1) == b'\x7E':
            msg_id = serial_port.read(1)
            body_length = serial_port.read(1)
            body = serial_port.read(ord(body_length))
            checksum = serial_port.read(2)
            frame = b'\x7E' + msg_id + body_length + body + checksum
            start_byte, msg_id, body_length = struct.unpack('>BBB', frame[:3])
            body = frame[3:-2]
            received_checksum = struct.unpack('>H', frame[-2:])[0]
            calculated_checksum = crc16(frame[:-2])

            if received_checksum != calculated_checksum:
                raise ValueError("Checksum does not match")

            return msg_id, body

def main():
    port = '/dev/ttyUSB0'  # Replace with your serial port
    baud_rate = 9600  # Replace with your baud rate

    # Open serial port
    ser = serial.Serial(port, baud_rate)

    try:
        latitude = float(input("Enter latitude: "))
        longitude = float(input("Enter longitude: "))
        body = struct.pack('>dd', latitude, longitude)
        send_message(ser, 0x02, body)  # Sending NavigateToGPS message

        while True:
            msg_id, body = read_message(ser)
            if msg_id == 0:  # Acknowledge message
                print("Received Acknowledge message")
            elif msg_id == 3:  # TaskCompleted message
                print("Received Task Completed message")
                break
    finally:
        ser.close()

if __name__ == "__main__":
    main()
