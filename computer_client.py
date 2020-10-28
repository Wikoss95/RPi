import socket
import time
import logging
from threading import Thread
from queue import Queue
SERVER = '192.168.1.100'
PORT = 9999
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

# welcome_note_from_server = s.recv(32)
# print(welcome_note_from_server.decode("utf-8"))


def convert_data(string):
    li = list(string.split("/"))
    return li


def odbieranie_danych():
    while True:
        sensors_data_raw = client.recv(32)
        val = convert_data(sensors_data_raw.decode("utf-8"))
        print("{:>2f}  {:>2f}  {:>2f}  {:>2f}  {:>2f}  {:>2f}".format(val[0], val[1], val[2], val[3], val[4], val[5]))
        print(val)
        time.sleep(0.1)


def wysylanie_danych(name, socket):
    while True:
        order = [0, 0, 0, 0, 0, 0]
        order_str = ""
        print("Podaj katy o jakie powinny przesunac sie silniki: \n")

        order[0] = input("Os 1: ")
        order[1] = input("Os 2: ")
        order[2] = input("Os 3: ")
        order[3] = input("Os 4: ")
        order[4] = input("Os 5: ")
        order[5] = input("Os 6: ")

        for x in range(len(order)):
            order_str = order_str + str(order[x])
            if x < len(order) - 1:
                order_str = order_str + "/"
        try:
            socket.sendall(bytes(order_str, 'utf-8'))
        except Exception as e:
            print(e)
        finally:
            logging.info('Data successfully sent! /from thread: ' + name)
            print('data sent...')


# thread1 = Thread(target=odbieranie_danych)
thread2 = Thread( target=wysylanie_danych, args=("Thread2", client))
# thread2 = Thread( target=number_counter, args=("WATEK2", start_with, queue, queue_ch))

thread2.start()
