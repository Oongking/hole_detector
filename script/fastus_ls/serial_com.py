import serial
import time

# Configure the serial port
port = serial.Serial('/dev/ttyUSB1', 9600)  # Change 'COM1' to your serial port name
port.timeout = 1  # Set timeout for readline()

# Define function to send and receive data
def send_and_receive(data_to_send):
    port.write(data_to_send)  # Send data
    time.sleep(0.1)  # Wait for response
    response = port.read_all()
    return response

# Main function
def main():
    try:


        # data = bytes([0x02,0x02,0xc0,0x00,0x00,0x02,0x00,0x01,0x03,0xC1])
        data = bytes([0x02,0x00,0xc0,0x10,0x03,0xD0])

        for res in data:
            print(f"send: {hex(res)}")
        response = send_and_receive(data)
        for res in response:
            print(f"Received: {hex(res)}")
    except KeyboardInterrupt:
        pass
    finally:
        port.close()  # Close the serial port

if __name__ == "__main__":
    main()
