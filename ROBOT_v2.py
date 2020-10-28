import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS
import time
import board
import busio
import math
import sys
import concurrent.futures
import socket
import logging
from threading import Thread
from queue import Queue


# DEFINIOWANIE ZMIENNYCH

frequency = 500
steps_per_rev = 200
alpha_rad = 2 * math.pi / steps_per_rev
alpha_deg = 360 / steps_per_rev
tt = 0.1

ADC1_addr = 0x48
ADC2_addr = 0x49

# stepper gripper
dirPIN_grip = 35
stepPIN_grip = 36
ratio_grip = 1

dirPIN = [15, 21, 23, 31, 11, 37]
stepPIN = [16, 22, 24, 32, 12, 38]
ratio = [9, 36, 36, 18, 8, 8]

accels = [math.pi, 0.1*math.pi, 0.2*math.pi, 0.5*math.pi, math.pi, math.pi]
speeds = [0.5*math.pi, 0.1*math.pi, 0.1*math.pi, 0.25*math.pi, 0.5*math.pi, 0.5*math.pi]
lower_boundary = [30, 90, 90, 90, 90, 90]
upper_boundary = [300, 270, 270, 270, 270, 270]


GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BOARD)


# METODY
def data_reading(channels, queueII, queueIII):
    logging.info('Method >data_reading< started...')
    values = []
    for n in range(8):
        if n < 6:
            values.insert(n, round((360 * 1000 * (channels[n].voltage) / 4096), 1))
        else:
            values.insert(n, channels[n])
        print(values)
    encoder_values = values[0-5]
    encoder_values2 = encoder_values
    # gripper_values = values[6-7]
    queueII.put(encoder_values)
    queueIII.put(encoder_values2)
    # queueIV.put(gripper_values)


def sensors_data_send(queueII):
    logging.info('Method >sensors_data_sending< started...')

    while True:
        sensors_readings = queueII.get()
        data_to_send = ''
        for data in range(6):
            data_to_send = data_to_send + str(sensors_readings[data])
            if data < 5:
                data_to_send = data_to_send + "/"
        try:
            conn.sendall(bytes(data_to_send, 'utf-8'))
        except Exception as e:
            print(e)
        finally:
            logging.info('Data successfully sent!')
        time.sleep(0.1)


def order_recieve(queueI):
    logging.info('Method >order_recieve< started...')

    while True:
        order_data_raw_raw = conn.recv(32)
        order_data_raw = order_data_raw_raw.decode("utf-8")
        order_data = list(order_data_raw.split("/"))
        print("{:>3}  {:>3}  {:>3}  {:>3}  {:>3}  {:>3}".format(order_data[0], order_data[1],
                                                                order_data[2], order_data[3], order_data[4],
                                                                order_data[5]))
        queueI.put(order_data)


def stepper_move_smooth(n, dir_pin, step_pin, ratio, accels, speeds, lower, upper, current_position, dataQ):
    print('watek silnika ' + n + ' wystartowal')
    angle = dataQ.get()
    # current_position = readingsQ.get()
    steps = (current_position[n] - angle[n]) / (alpha_deg / ratio[n])
    if steps < 0:
        direction = 0
    else:
        direction = 1
    GPIO.output(dir_pin[n], direction)
    steps = abs(round(steps))

    c0 = (0.676 / tt) * (math.sqrt(2 * alpha_rad / accels[n] * ratio[n]))
    deceleration = 1.5 * accels[n]

    accel_dist = steps * deceleration / (accels[n] + deceleration)
    # print('acceleration distance: ' + str(accel_dist))
    speed_reach = math.pow(speeds[n], 2) / (2 * alpha_rad * accels[n] / ratio[n])
    # print('speed reached: ' + str(speed_reach))
    decel_dist = 0
    if speed_reach < accel_dist:
        decel_dist = -speed_reach * accels[n] / deceleration
        logging.info('Standard speed ramp - Trapezoid velocity mode')
    elif speed_reach >= accel_dist:
        decel_dist = -(steps - speed_reach)
        logging.info('Up/Down speed ramp - Triangular velocity mode')
    # print('deceleration distance: ' + str(decel_dist))
    delay = c0
    counter = 0

    for n in range(steps):
        steady_move = True
        # print('GPIO.output(step_PIN, GPIO.HIGH)')
        GPIO.output(step_pin[n], GPIO.HIGH)
        high_time = 0.1 * delay * tt
        # print('sleep: ' + str(high_time))
        time.sleep(high_time)
        # print('GPIO.output(step_PIN, GPIO.LOW)')
        GPIO.output(step_pin[n], GPIO.LOW)
        low_time = 0.9 * delay * tt
        # print('sleep: ' + str(low_time))
        time.sleep(low_time)

        if n < speed_reach:
            counter = n
        elif n == (steps + decel_dist):
            counter = int(decel_dist)
        elif n > (steps + decel_dist):
            counter = counter + 1
        else:
            steady_move = True
        if steady_move:
            continue
        else:
            new_delay = abs(delay - (2 * delay) / (4 * counter + 1))
            delay = new_delay


# ##### MAIN BODY #####
# TWORZENIE INSTANCJI POSZCZEGOLNYCH ELMENTOW SKLADOWYCH

# i2c = busio.I2C(board.SCL, board.SDA)
# ADC1 = ADS.ADS1115(i2c, address=ADC1_addr)
# ADC2 = ADS.ADS1115(i2c, address=ADC2_addr)

# channels_names = [ADS.P0, ADS.P1, ADS.P2, ADS.P3]
# channels = []
# # Create channels
# for channel in channels_names:
#     channels.append(AnalogIn(ADC1, channels_names[channel]))
# for channel in channels_names:
#     channels.append(AnalogIn(ADC2, channels_names[channel]))

# Create steppers instances
# stepper_gripper = Stepper(stepPIN_grip, dirPIN_grip, ratio_grip, [0, 360], 0, 0.1*math.pi, 0.1*math.pi)

SERVER = "192.168.1.100"
PORT = 9999
ADDR = (SERVER, PORT)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create socket instance
server.bind(ADDR)  # Binding socket

try:
    server.listen(5)  # Number of connection tries
    # Wait for a connection
    print('waiting for a connection')
    conn, addr = server.accept()
except Exception as e:
        print(e)
print("Connected succesfully...")


sensors_data_queue = Queue()
internal_sensor_data_queue = Queue()
order_data_queue = Queue()
futures = []
current_position_passed = [0, 0, 0, 0, 0, 0]

with concurrent.futures.ThreadPoolExecutor(max_workers=7) as executor:
    executor.submit(order_recieve, order_data_queue)
    # executor.submit(data_reading, channels, sensors_data_queue, internal_sensor_data_queue)
    executor.submit(sensors_data_send, sensors_data_queue)
    for n in range(6):
        executor.submit(stepper_move_smooth, n, dirPIN, stepPIN, ratio, accels, speeds, lower_boundary, upper_boundary, current_position_passed, order_data_queue)
    # executor.submit(stepper_move_smooth, 1, dirPIN, stepPIN, ratio, accels, speeds, lower_boundary, upper_boundary, current_position_passed, order_data_queue)


# threadA = Thread(target=order_recieve, args=("ThreadA", order_data_queue))
# threadB = Thread(target=MOVE, args=("ThreadB", order_data_queue, steppers))

