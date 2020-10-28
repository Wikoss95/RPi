# import Adafruit_ADS1x15
# def ads1115_potentiometer(value):
#     value_out = 300 * value / 26700 - 50
#     return value_out

# import
import logging
import time
import board
import busio
import socket
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS
# from signal import signal, SIGPIPE, SIG_DFL
#
# signal(SIGPIPE, SIG_DFL)

# Create bus instance
i2c = busio.I2C(board.SCL, board.SDA)

# Create device instances
ADC1 = ADS.ADS1115(i2c, address=0x48)
ADC2 = ADS.ADS1115(i2c, address=0x49)

channels_names = [ADS.P0, ADS.P1, ADS.P2, ADS.P3]
channels = []
# Create channels
for channel in channels_names:
    channels.append(AnalogIn(ADC1, channels_names[channel]))
for channel in channels_names:
    channels.append(AnalogIn(ADC2, channels_names[channel]))


def data_handling(values, channels):
    for x in range(len(values)):
        values.insert(x, round((360*1000*(channels[x].voltage)/4096), 1))
        # values.insert(x, round((channels[x].voltage-0.085), 3))
    return values

# Print nice channel column headers.
# print("{:>3}  {:>3}  {:>3}  {:>3}  {:>3}  {:>3}".format("CHAN1", "CHAN2", "CHAN3", "CHAN4", "CHAN5", "CHAN6"))


SERVER = "192.168.1.100"
PORT = 9999
ADDR = (SERVER, PORT)
print(SERVER)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create socket instance
# s.bind(('192.168.1.99', 9999))  # Binding socket

try:
    server.listen(5)  # Number of connection tries
    # Wait for a connection
    print('waiting for a connection')
    conn, addr = server.accept()
except Exception as e:
        print(e)


print("connected succesfully")

# Main loop.
while True:
    # Read all the ADC channel values in a list.
    values = [0, 0, 0, 0, 0, 0, 0, 0]
    sensors_read = data_handling(values, channels)
    sensor_value1 = sensors_read.pop()
    sensor_value2 = sensors_read.pop()

    data_to_send = ''
    for data in range(6):
        data_to_send = data_to_send + str(sensors_read[data])
        if data < 5:
            data_to_send = data_to_send + "/"
    try:
        print(data_to_send)
        msg = conn.send(bytes(data_to_send, 'utf-8'))
    except Exception as e:
        print(e)
    finally:
        logging.info('Data successfully sent! /from thread')
    time.sleep(0.1)

    # print(sensor_value1, sensor_value2)
    # print("{:>2f}  {:>2f}  {:>2f}  {:>2f}  {:>2f}  {:>2f}".format(val[0], val[1], val[2], val[3], val[4], val[5]))
    # time.sleep(0.1)






