import Adafruit_DHT
import control_class as cc
import time
import RPi.GPIO as GPIO
import threading
import sys

def manage_airstone():
    print("Airstone thread started")
    while True:
        airstone.airstone_auto()
        time.sleep(10)

def manage_waterpump():
    print("Waterpump thread started")
    while True:
        pump.waterpump_auto()
        time.sleep(10)

def manage_heater():
    print("Heater thread started")
    while True:
        heater.heater_auto()
        time.sleep(10)

def manage_light():
    print("Light thread started")
    while True:
        light.light_auto()
        time.sleep(10)

def manage_fan():
    print("Fan thread started")
    while True:
        fan.fan_auto()
        time.sleep(10)

heater = cc.Heater()
light = cc.Light()
fan = cc.Fan()
airstone = cc.Airstone()
pump = cc.WaterPump()
#sensor = cc.DHT11_Sensor()

try:
    # Thread creation
    t1 = threading.Thread(target=manage_airstone)
    t2 = threading.Thread(target=manage_waterpump)
    t3 = threading.Thread(target=manage_heater)
    t4 = threading.Thread(target=manage_light)
    t5 = threading.Thread(target=manage_fan)

    t1.daemon = True
    t2.daemon = True
    t3.daemon = True
    t4.daemon = True
    t5.daemon = True

    # Thread execution
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()

    # Wait for all threads to complete (in truth, this program will never end...)
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()

except KeyboardInterrupt:
    GPIO.cleanup()
    print("Program stopped by user")
    exit()