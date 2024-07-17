import struct
import crcmod
import serial
import time

import crc


# CRC-16-CCITT calculation function
# crc16 = crcmod.mkCrcFun(0x1021, initCrc=0x0000, rev=False, xorOut=0x0000)

def encode_message(msg_id, body):
    start_byte = 0x7E
    body_length = len(body)
    frame = struct.pack('>BBB', start_byte, msg_id, body_length) + body
    checksum = crc.Calculator(crc.Crc16.XMODEM.value).checksum(frame)
    frame += struct.pack('>H', checksum)
    return frame

def send_message(serial_port, msg_id, body):
    message = encode_message(msg_id, body)
    serial_port.write(message)
    print(f"Sent Message: {message.hex()}")

def decode_message(frame):
    start_byte, msg_id, body_length = struct.unpack('>BBB', frame[:3])
    body = frame[3:-2]
    # received_checksum = struct.unpack('>H', frame[-2:])[0]
    received_checksum = struct.unpack('>H', frame[-2:])[0]
    print(received_checksum)
    print(frame[:-2])
    calculated_checksum =  crc.Calculator(crc.Crc16.XMODEM.value).checksum(frame[:-2])

    print(calculated_checksum)
    
    if received_checksum != calculated_checksum:
        raise ValueError("Checksum does not match")

    return msg_id, body

# decode_message(b"\x7E\x22\x03\x01\x02\x03")
def decode_body(msg_id, body):
    if msg_id == 0x00:
        return "Acknowledge", {}
    elif msg_id == 0x01:
        armed = struct.unpack('>B', body)[0]
        return "ArmDisarm", {"armed": armed}
    elif msg_id == 0x02:
        latitude, longitude = struct.unpack('>dd', body)
        return "NavigateToGPS", {"latitude": latitude, "longitude": longitude}
    elif msg_id == 0x03:
        return "TaskCompleted", {}
    elif msg_id == 0x04:
        stage_id = struct.unpack('>B', body)[0]
        return "SetStage", {"stage_id": stage_id}
    elif msg_id == 0x05:
        text_message = body.decode('utf-8')
        return "Text", {"text_message": text_message}
    elif msg_id == 0x06:
        tag_id, dictionary = struct.unpack('>IB', body)
        return "ArucoTag", {"tag_id": tag_id, "dictionary": dictionary}
    elif msg_id == 0x07:
        tags = [struct.unpack('>IB', body[i:i+5]) for i in range(0, len(body), 5)]
        tag_list = [{"tag_id": t[0], "dictionary": t[1]} for t in tags]
        return "LocateArucoTags", {"tag_list": tag_list}
    elif msg_id == 0x08:
        x, y, z = struct.unpack('>fff', body[:12])
        reference = body[12:].decode('utf-8') if len(body) > 12 else ''
        return "Location3D", {"x": x, "y": y, "z": z, "reference": reference}
    elif msg_id == 0x09:
        distance = struct.unpack('>f', body[:4])[0]
        color = body[4:].decode('utf-8') if len(body) > 4 else ''
        return "Detection", {"distance": distance, "description": color}
    elif msg_id == 0x0A:
        yaml_dict = body.decode('utf-8')
        return "SetParameters", {"yaml_serialized_dict": yaml_dict}
    else:
        return "Unknown", {}

def read_message(serial_port):
    while True:
        if serial_port.read(1) == b'\x7E':
            msg_id = serial_port.read(1)
            print(f"Received Message ID: {msg_id.hex()}")
            body_length = serial_port.read(1)
            body = serial_port.read(ord(body_length))
            checksum = serial_port.read(2)
            frame = b'\x7E' + msg_id + body_length + body + checksum
            print(frame)
            return decode_message(frame)

def choose_message_type():
    print("\nChoose a message type to send:")
    print("1. Acknowledge (0x00)")
    print("2. ArmDisarm (0x01)")
    print("3. NavigateToGPS (0x02)")
    print("4. TaskCompleted (0x03)")
    print("5. SetStage (0x04)")
    print("6. Text (0x05)")
    print("7. ArucoTag (0x06)")
    print("8. LocateArucoTags (0x07)")
    print("9. Location3D (0x08)")
    print("10. Detection (0x09)")
    print("11. SetParameters (0x0A)")
    choice = input("Enter the number of your choice: ")
    return int(choice)

def get_message_data(choice):
    if choice == 1:
        return 0x00, b''
    elif choice == 2:
        armed = int(input("Enter 1 to arm, 0 to disarm: "))
        return 0x01, struct.pack('>B', armed)
    elif choice == 3:
        latitude = float(input("Enter latitude: "))
        longitude = float(input("Enter longitude: "))
        return 0x02, struct.pack('>dd', latitude, longitude)
    elif choice == 4:
        return 0x03, b''
    elif choice == 5:
        stage_id = int(input("Enter stage ID: "))
        return 0x04, struct.pack('>B', stage_id)
    elif choice == 6:
        text_message = input("Enter text message: ")
        return 0x05, text_message.encode('utf-8')
    elif choice == 7:
        tag_id = int(input("Enter tag ID: "))
        dictionary = int(input("Enter dictionary: "))
        return 0x06, struct.pack('>IB', tag_id, dictionary)
    elif choice == 8:
        tag_count = int(input("Enter number of tags: "))
        tags = []
        for _ in range(tag_count):
            tag_id = int(input("Enter tag ID: "))
            dictionary = int(input("Enter dictionary: "))
            tags.append(struct.pack('>IB', tag_id, dictionary))
        return 0x07, b''.join(tags)
    elif choice == 9:
        x = float(input("Enter x coordinate: "))
        y = float(input("Enter y coordinate: "))
        z = float(input("Enter z coordinate: "))
        reference = input("Enter reference (optional): ")
        return 0x08, struct.pack('>fff', x, y, z) + reference.encode('utf-8')
    elif choice == 10:
        distance = float(input("Enter distance: "))
        color = input("Enter color description: ")
        return 0x09, struct.pack('>f', distance) + color.encode('utf-8')
    elif choice == 11:
        yaml_dict = input("Enter YAML serialized dict: ")
        return 0x0A, yaml_dict.encode('utf-8')

def main():
    port = '/dev/ttyUSB0'  # Replace with your serial port
    baud_rate = 115200  # Replace with your baud rate
    
    # Open serial port
    ser = serial.Serial(port, baud_rate)

    try:
        while True:
            print("\n1. Send a message")
            print("2. Receive a message")
            choice = input("Enter your choice: ")

            if choice == '1':
                message_choice = choose_message_type()
                msg_id, body = get_message_data(message_choice)
                send_message(ser, msg_id, body)
                try:
                    while True:
                        try:
                            msg_id, body = read_message(ser)
                            if msg_id == 0:  # Acknowledge message
                                print("Received Acknowledge message")
                                break
                            
                        except Exception as e:
                            print(f"Error reading message: {e}")
                            time.sleep(1)
                
                except KeyboardInterrupt:
                    print("Exiting...")
                    break
            
            elif choice == '2':
                msg_id, body = read_message(ser)
                print(f"Received Message: {msg_id} {body}")
                msg_type, decoded_body = decode_body(msg_id, body)
                print(f"Received Message Type: {msg_type}, Decoded Body: {decoded_body}")
            else:
                print("Invalid choice. Please try again.")
                
    finally:
        ser.close()

if __name__ == "__main__":
    main()