import Adafruit_DHT
import RPi.GPIO as GPIO
import time
import datetime

import threading


####################### DEFINE SENSORS ####################### 
# Temperature and Humidity Sensor DHT22
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 26 # read from GPIO4

################### DEFINE CONTROL RELAYS ####################
# Relay 1 - Heater Control
RELAY_PIN_HEATER = 17
# Relay 2 - Light Control
RELAY_PIN_LIGHT = 27
# Relay 3 - Fan Control
RELAY_PIN_FAN = 24
# Relay 4 - AIRSTONE Control
RELAY_PIN_AIRSTONE = 23
# Relay 5 - WATER PUMP Control
RELAY_PIN_WATER_PUMP = 22

############# DEFINE PWM PIN FOR FAN CONTROL #################

TRIAC_PIN_1 = 12
TRIAC_PIN_2 = 13

ZC_PIN_DIMMER = 25


##################### DEFINE PARAMETERS #######################
# Temperature parameters
TEMP_HEATER_THRESHOLD_OFF = 24.0 # temperature threshold to turn off heater
TEMP_HEATER_THRESHOLD_ON = 17.0 # temperature threshold to turn on heater

# Light parameters
LIGHT_ON = 7 # light on at 7am
LIGHT_OFF = 19 # light off at 7pm

# Fan parameters
TEMP_FAN_THRESHOLD_HIGH = 25.0 # temperature threshold to turn on fan
TEMP_FAN_THRESHOLD_LOW = 20.0 # temperature threshold to turn off fan

H_FAN_THRESHOLD_HIGH = 85.0 # humidity threshold to turn on fan
H_FAN_THRESHOLD_LOW = 80.0 # humidity threshold to turn off fan

T_FAN_MAX_TIME = 180.0 # maximum time for fan to be on
T_FAN_WAIT_TIME = 600.0 # time to wait before next fan activation
T_FAN_STANDBY_TIME = 300.0 # time to wait before next fan activation

T_START_HOUR = 8 # start hour for fan activation
T_STOP_HOUR = 22 # stop hour for fan activation

# Airstone parameters
AIRSTONE_ON = 4 # airstone on at 4am
AIRSTONE_OFF = 21 # airstone off at 9pm

# Water pump parameters
WATER_PUMP_ON = 6 # water pump on at 6am
WATER_PUMP_OFF = 18 # water pump off at 6pm



class DHT22_Sensor():
    
    def __init__(self):
        self.DHT_SENSOR = DHT_SENSOR
        self.DHT_PIN = DHT_PIN
        self.humidity = 0.
        self.temperature = 0.
        
        self.temperature_T, self.humidity_T = 24.0, 35.0 # temperature and humidity for testing

    def read_sensor(self):
        # try sensor read
        while True:
            humidity, temperature = Adafruit_DHT.read_retry(self.DHT_SENSOR, self.DHT_PIN)
            if self.humidity is not None and self.temperature is not None and self.humidity < 100:
                self.humidity = humidity
                self.temperature = temperature
                return self.temperature, self.humidity
            else:
                print("Sensor read error")
                return self.temperature, self.humidity #self.temperature_T, self.humidity_T #self.temperature, self.humidity


class Heater(): 

    def __init__(self):


        self.T_high = TEMP_HEATER_THRESHOLD_OFF
        self.T_low = TEMP_HEATER_THRESHOLD_ON

        GPIO.setmode(GPIO.BCM) # GPIO numbering
        GPIO.setup(RELAY_PIN_HEATER, GPIO.OUT) # set GPIO27 as output

        self.DHT22_sensor = DHT22_Sensor()

        self.heater_status = True # heater virtually on
        print("HEAT: \tINITIALIZED")
        self.heater_off() # heater off
    
    def heater_on(self):
        GPIO.output(RELAY_PIN_HEATER, GPIO.LOW)
        if not self.heater_status:
            print(f"{datetime.datetime.now()}\tHEAT: \tON")
        self.heater_status = True

    def heater_off(self):
        GPIO.output(RELAY_PIN_HEATER, GPIO.HIGH)
        if self.heater_status:
            print(f"{datetime.datetime.now()}\tHEAT: \tOFF")
        self.heater_status = False
    
    def set_new_threshold(self, T_high, T_low):
        self.T_high = T_high
        self.T_low = T_low

    def heater_auto(self):
        
        # check temperature 
        temperature, humidity = self.DHT22_sensor.read_sensor()
        print(f"{datetime.datetime.now()}\tHEAT-> \tTemperature={temperature:.1f}°, Humidity={humidity:.1f}%")
        
        if temperature > self.T_high and self.heater_status:
            self.heater_off()
            self.heater_status = False

        if temperature < self.T_low and not self.heater_status:
            self.heater_on()
            self.heater_status = True
        
        print(f"{datetime.datetime.now()}\tHEAT:\tWaiting for {1} minute before next check")
        time.sleep(50)        

class Light(): 

    def __init__(self):

        self.light_status = False # light off
        self.time_on = LIGHT_ON
        self.time_off = LIGHT_OFF

        GPIO.setmode(GPIO.BCM) # GPIO numbering
        GPIO.setup(RELAY_PIN_LIGHT, GPIO.OUT) # set GPIO17 as output

        print("LGHT: \tINITIALIZED")
        self.light_off()

    def set_new_time(self, time_on, time_off):
        self.time_on = time_on
        self.time_off = time_off

    def light_on(self):
        GPIO.output(RELAY_PIN_LIGHT, GPIO.LOW) # set RELAY_PIN_LIGHT LOW -> relay off
        self.light_status = True
        print(f"{datetime.datetime.now()}\tLGHT: \tON")

    def light_off(self):
        GPIO.output(RELAY_PIN_LIGHT, GPIO.HIGH) # set RELAY_PIN_LIGHT HIGH -> relay on
        self.light_status = False
        print(f"{datetime.datetime.now()}\tLGHT: \tOFF")

    def light_auto(self):

        # check time
        t = time.localtime()
        hour = t.tm_hour

        if hour >= self.time_on and hour < self.time_off and not self.light_status:
            self.light_on()
        elif (hour < self.time_on or hour >= self.time_off) and self.light_status:
            self.light_off()
        
        return
    

class Fan():

    """ 
    Fan control class:
    Activate fan when temperature is above T_high or humidity is above H_high.
    Keep on until temperature is below T_high-thresh and/or humidity is below H_low or for at most t_max seconds.
    Repeat after t_wait seconds. 
    """

    def __init__(self):

        self.fan_status = True # fan virtually on
        self.T_high = TEMP_FAN_THRESHOLD_HIGH
        self.T_low = TEMP_FAN_THRESHOLD_LOW

        self.H_high = H_FAN_THRESHOLD_HIGH
        self.H_low = H_FAN_THRESHOLD_LOW

        self.t_max = T_FAN_MAX_TIME
        self.t_wait = T_FAN_WAIT_TIME
        self.t_standby = T_FAN_WAIT_TIME

        self.start_hour = T_START_HOUR
        self.stop_hour = T_STOP_HOUR

        self.base_speed = 80.0
        self.speed_percentage = self.base_speed

        # set up relay pin        
        GPIO.setmode(GPIO.BCM) # GPIO numbering
        GPIO.setup(RELAY_PIN_FAN, GPIO.OUT)

        # set up TRIAC pins
        GPIO.setup(TRIAC_PIN_1, GPIO.OUT)
        GPIO.setup(TRIAC_PIN_2, GPIO.OUT)

        # set up zero-crossing pin
        GPIO.setup(ZC_PIN_DIMMER, GPIO.IN)

        # Initialize DHT22 sensor
        self.DHT22_sensor = DHT22_Sensor()

        # setup ZC interrupt
        GPIO.add_event_detect(ZC_PIN_DIMMER, GPIO.RISING, callback=self.callback_zerocross)

        print("FAN : \tINITIALIZED")
        self.fan_standby() # fan off


    #def callback_zerocross(self, channel):
    #    t = threading.Thread(target=self.handle_zerocross, args=(channel,))
    #    t.start()


    def callback_zerocross(self, channel):

        delay_fan_1 = 0.019 * (100 - self.speed_percentage)*.95 / 100 # delay fan 1 
        delay_fan_2 = 0.019 * (100 - self.speed_percentage) / 100 # delay fan 2

        time.sleep(max(delay_fan_1, delay_fan_2)) # wait for the appropriate delay

        if delay_fan_1 > delay_fan_2:
            GPIO.output(TRIAC_PIN_1, GPIO.HIGH)
            time.sleep(delay_fan_1 - delay_fan_2)
            GPIO.output(TRIAC_PIN_2, GPIO.HIGH)
        else:
            GPIO.output(TRIAC_PIN_2, GPIO.HIGH)
            time.sleep(delay_fan_2 - delay_fan_1)
            GPIO.output(TRIAC_PIN_1, GPIO.HIGH)

        time.sleep(0.001)
        GPIO.output(TRIAC_PIN_1, GPIO.LOW)
        GPIO.output(TRIAC_PIN_2, GPIO.LOW)


    def set_new_threshold(self, T_high=25.0, T_low=22.0, H_high=75.0, H_low=65.0):
        self.T_high = T_high
        self.T_low = T_low
        self.H_high = H_high
        self.H_low = H_low

    def set_new_time(self, t_max=180.0, t_wait=180.0):
        self.t_max = t_max
        self.t_wait = t_wait

    def fan_on(self):
        GPIO.output(RELAY_PIN_FAN, GPIO.LOW)
        if not self.fan_status:
            print(f"{datetime.datetime.now()}\tFAN : \tON at {self.speed_percentage:.1f}%") 
        self.fan_status = True

    
    def fan_standby(self):
        GPIO.output(RELAY_PIN_FAN, GPIO.LOW)
        self.speed_percentage = self.base_speed
        if not self.fan_status:
            print(f"{datetime.datetime.now()}\tFAN : \tON at {self.speed_percentage:.1f}%") 
        self.fan_status = False

    def fan_off(self):
        GPIO.output(RELAY_PIN_FAN, GPIO.HIGH)
        if self.fan_status:
            print(f"{datetime.datetime.now()}\tFAN : \tOFF")
        self.fan_status = False

    def set_fan_speed(self, temperature, humidity):
        """
        Compute fan speed based on the difference between current 
        temperature/humidity and the respective threshold values.
        """
        # Check how far temperature is above threshold.
        temp_diff = max(temperature - self.T_high, 0)
        # Check how far humidity is above threshold.
        hum_diff = max(humidity - self.H_high, 0)

        # Compute a proportional speed for the fans.
        # These coefficients determine how quickly the fan speed ramps up based on the temperature/humidity.
        temp_speed = temp_diff * 2.5  # increase the speed by 20% for every degree above the threshold
        hum_speed = hum_diff * 1.5  # increase the speed by 15% for every % above the threshold

        # Choose the largest of the two values to use for the fan speed.
        speed = max(temp_speed, hum_speed)
        # Ensure the fan speed is within the range 30-100%.
        self.speed_offset = min(speed, 20)  # clip speed to between 0% and 20%
        self.speed_percentage = self.base_speed + self.speed_offset
        if not self.fan_status:
            print(f"{datetime.datetime.now()}\tFAN -> \tSpeed set to {self.speed_percentage:.1f}%")
        return self.speed_percentage
    
        
    def fan_auto(self):

            # check temperature and humidity
            temperature, humidity = self.DHT22_sensor.read_sensor()
            print(f"{datetime.datetime.now()}\tFAN -> \tTemperature={temperature:.1f}°, Humidity={humidity:.1f}%")

            if (((datetime.datetime.now().minute % 30) < 2) and (datetime.datetime.now().hour >= self.start_hour and datetime.datetime.now().hour < self.stop_hour)) and not self.fan_status and temperature > self.T_low-5:
                print(f"{datetime.datetime.now()}\tFAN -\tTime criteria activated")
                # activate fan for 2 minutes
                t0 = time.time()
                while time.time()-t0 < 120: # 2 minutes
                    self.speed_percentage = self.base_speed + 10 # set fan speed to 90% by default
                    self.fan_on() # turn on fan
                    time.sleep(1) # wait for 1 seconds
                print(f"{datetime.datetime.now()}\tFAN :\tMax time reached - turning off fan")
                self.fan_standby() # turn off fan

            elif (temperature > self.T_high and temperature > self.T_low) or humidity > self.H_high and (datetime.datetime.now().hour >= self.start_hour and datetime.datetime.now().hour < self.stop_hour) and not self.fan_status: #  or humidity > self.H_high
                print(f"{datetime.datetime.now()}\tFAN :\tTemperature/Humidity criteria activated")
                # activate fan for at most t_max seconds
                t0 = time.time()
                while time.time()-t0 < self.t_max:
                    self.set_fan_speed(temperature, humidity) # compute fan speed
                    self.fan_on() # turn on fan
                    print(f"{datetime.datetime.now()}\tFAN :\tSpeed set to {self.speed_percentage:.1f}%")
                    time.sleep(20) # wait for 20 seconds
                    temperature, humidity = self.DHT22_sensor.read_sensor()

                # check for a second cycle if temperature and humidity are still above threshold
                if (temperature > self.T_high and temperature > self.T_low) or humidity > self.H_high:
                    # go in standby for t_standby seconds
                    print(f"{datetime.datetime.now()}\tFAN :\tMax time reached - going in standby for {self.t_standby/60} minutes")
                    self.fan_standby()
                    time.sleep(self.t_standby) # wait for t_standby seconds

                    # reacquire temperature and humidity
                    temperature, humidity = self.DHT22_sensor.read_sensor()
                    print(f"{datetime.datetime.now()}\tFAN :\tTemperature={temperature:.1f}°, Humidity={humidity:.1f}%")
                    # check if temperature and humidity are still above threshold
                    if temperature > self.T_high or humidity > self.H_high:
                        # activate fan for at most t_max seconds
                        print(f"{datetime.datetime.now()}\tFAN :\tTemperature/Humidity criteria activated")
                        t0 = time.time()
                        while time.time()-t0 < self.t_max:
                            self.set_fan_speed(temperature, humidity)
                            self.fan_on() # turn on fan
                            time.sleep(20) # wait for 20 seconds


                print(f"{datetime.datetime.now()}\tFAN :\tMax time reached - turning off fan")
                self.fan_off() # turn off fan
                print(f"{datetime.datetime.now()}\tFAN :\tWaiting for {self.t_wait/60} minutes")
                time.sleep(self.t_wait) # wait for t_wait seconds
            elif not self.fan_status:
                print(f"{datetime.datetime.now()}\tFAN : \tNo criteria activated - keeping fan off")
                self.fan_off()
            else:
                print(f"{datetime.datetime.now()}\tFAN : \tNo criteria activated - Shutting down fan")
                self.fan_off()

            print(f"{datetime.datetime.now()}\tFAN :\tWaiting for {1} minute before next check")
            time.sleep(50)        
            return


class Airstone():

    def __init__(self):

        self.airstone_status = False

        self.start_hour = AIRSTONE_ON
        self.stop_hour = AIRSTONE_OFF

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN_AIRSTONE, GPIO.OUT)

        print("AIR : \tINITIALIZED")
        self.airstone_off()

    def airstone_on(self):
        GPIO.output(RELAY_PIN_AIRSTONE, GPIO.LOW)
        self.airstone_status = True
        print(f"{datetime.datetime.now()}\tAIR : \tON")

    def airstone_off(self):
        GPIO.output(RELAY_PIN_AIRSTONE, GPIO.HIGH)
        self.airstone_status = False
        print(f"{datetime.datetime.now()}\tAIR : \tOFF")

    def airstone_auto(self):
        # check time
        t = time.localtime()
        hour = t.tm_hour

        if hour >= self.start_hour and hour < self.stop_hour and not self.airstone_status:
            self.airstone_on()
        elif (hour < self.start_hour or hour >= self.stop_hour) and self.airstone_status:
            self.airstone_off()
        
        return

    
class WaterPump():

    def __init__(self):

        self.waterpump_status = True # water pump virtually on

        self.start_hour = WATER_PUMP_ON
        self.stop_hour = WATER_PUMP_OFF

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN_WATER_PUMP, GPIO.OUT)
        print("PUMP: \tINITIALIZED")
        self.waterpump_off() # water pump off

    def waterpump_on(self):
        GPIO.output(RELAY_PIN_WATER_PUMP, GPIO.LOW)
        if not self.waterpump_status:
            print(f"{datetime.datetime.now()}\tPUMP: \tON")
        self.waterpump_status = True

    def waterpump_off(self):
        GPIO.output(RELAY_PIN_WATER_PUMP, GPIO.HIGH)
        if self.waterpump_status:
            print(f"{datetime.datetime.now()}\tPUMP: \tOFF")
        self.waterpump_status = False

    def waterpump_auto(self):

        t = time.localtime()
        hour = t.tm_hour

        if (hour >= self.start_hour and hour < self.stop_hour) and not self.waterpump_status:
            self.waterpump_on()
        elif (hour < self.start_hour or hour >= self.stop_hour) and self.waterpump_status:
            self.waterpump_off()
        
        return
