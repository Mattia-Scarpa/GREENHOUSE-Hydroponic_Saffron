import Adafruit_DHT
import RPi.GPIO as GPIO
import time
#import control_class as cc


# define sensor
DHT_SENSOR = Adafruit_DHT.DHT22 # DHT11, DHT22, AM2302
DHT_PIN = 26 # read from GPIO26

Rele_1 = 27
Rele_2 = 17
Rele_3 = 24
Rele_4 = 23
Rele_5 = 22

test_rele = True
time_delay = 10

#sensor = cc.DHT11_Sensor()


GPIO.setmode(GPIO.BCM) # GPIO numbering
GPIO.setup(Rele_1, GPIO.OUT) # set GPIO27 as output
GPIO.setup(Rele_2, GPIO.OUT) # set GPIO17 as output
GPIO.setup(Rele_3, GPIO.OUT) # set GPIO24 as output
GPIO.setup(Rele_4, GPIO.OUT) # set GPIO23 as output
GPIO.setup(Rele_5, GPIO.OUT) # set GPIO22 as output


# try sensor read
while True:
    try:
        humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN) #sensor.read_sensor()
        print(f"T = {temperature} °C, H = {humidity} %")
        
        #time.sleep(10) # wait 5 seconds
        # test relay

        if test_rele:

                # test relay 1
                GPIO.output(Rele_1, GPIO.HIGH) # set GPIO27 high, relay off
                print("Relay 1 off")
                time.sleep(time_delay) # wait 5 seconds
                GPIO.output(Rele_1, GPIO.LOW) # set GPIO27 low, relay on
                print("Relay 1 on")
                time.sleep(time_delay) # wait 5 seconds

                # test relay 2
                GPIO.output(Rele_2, GPIO.HIGH) # set GPIO17 high, relay off
                print("Relay 2 off")
                time.sleep(time_delay) # wait 5 seconds
                GPIO.output(Rele_2, GPIO.LOW) # set GPIO17 low, relay on
                print("Relay 2 on")
                time.sleep(time_delay) # wait 5 seconds

                # test relay 3
                GPIO.output(Rele_3, GPIO.HIGH) # set GPIO24 high, relay off
                print("Relay 3 off")
                time.sleep(time_delay) # wait 5 seconds
                GPIO.output(Rele_3, GPIO.LOW) # set GPIO24 low, relay on
                print("Relay 3 on")
                time.sleep(time_delay) # wait 5 seconds

                # test relay 4
                GPIO.output(Rele_4, GPIO.HIGH) # set GPIO23 high, relay off
                print("Relay 4 off")
                time.sleep(time_delay) # wait 5 seconds
                GPIO.output(Rele_4, GPIO.LOW) # set GPIO23 low, relay on
                print("Relay 4 on")
                time.sleep(time_delay) # wait 5 seconds

                # test relay 5
                GPIO.output(Rele_5, GPIO.HIGH) # set GPIO22 high, relay off
                print("Relay 5 off")
                time.sleep(time_delay) # wait 5 seconds
                GPIO.output(Rele_5, GPIO.LOW) # set GPIO22 low, relay on
                print("Relay 5 on")
                time.sleep(time_delay) # wait 5 seconds

                # test all relays
                GPIO.output(Rele_1, GPIO.HIGH) # set GPIO27 high, relay off
                GPIO.output(Rele_2, GPIO.HIGH) # set GPIO17 high, relay off
                GPIO.output(Rele_3, GPIO.HIGH) # set GPIO24 high, relay off
                GPIO.output(Rele_4, GPIO.HIGH) # set GPIO23 high, relay off
                GPIO.output(Rele_5, GPIO.HIGH) # set GPIO22 high, relay off

                print("All relays off")
                time.sleep(time_delay) # wait 5 seconds

                GPIO.output(Rele_1, GPIO.LOW) # set GPIO27 low, relay on
                GPIO.output(Rele_2, GPIO.LOW) # set GPIO17 low, relay on
                GPIO.output(Rele_3, GPIO.LOW) # set GPIO24 low, relay on
                GPIO.output(Rele_4, GPIO.LOW) # set GPIO23 low, relay on
                GPIO.output(Rele_5, GPIO.LOW) # set GPIO22 low, relay on

                print("All relays on")
                time.sleep(time_delay) # wait 5 seconds
        
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program stopped by user")
        break
