from time import sleep
from datetime import datetime
from statistics import mean
import math
import threading
import busio
import adafruit_bme280
import pigpio

## BME280 Temp/humidity sensors
INNER_SENSOR_I2C_PINS = (3, 2)
OUTER_SENSOR_I2C_PINS = (27, 17)
INNER_TEMP_OFFSET = 0.7
OUTER_TEMP_OFFSET = 0.0
INNER_HUMIDITY_OFFSET = 0.0
OUTER_HUMIDITY_OFFSET = 1.0

# PWM PC fan
FAN_PWM_PIN = 18
FAN_TACH_PIN = 15
FAN_PWM_FREQ = 25000
FAN_PWM_MIN = 28
FAN_PWM_MAX = 100

## MPDMv4.1 AC light dimmer
LIGHT_POWER_PIN = 19
LIGHT_PWM_PIN = 13
LIGHT_PWM_FREQ = 6000


class ConnectionException(Exception):
    pass


class Sensor:
    I2C_ADDRESS = 0x76
    READINGS_CACHE = 20

    def __init__(self, pins, temp_offset=0.0, humidity_offset=0.0):
        self._pins = pins
        self._temp_offset = temp_offset
        self._humidity_offset = humidity_offset
        self._sensor = adafruit_bme280.Adafruit_BME280_I2C(busio.I2C(*pins), Sensor.I2C_ADDRESS)
        self._readings_cache = {"temperature": [], "humidity": []}
        print("filling sensor cache...")
        while len(self._readings_cache['temperature']) < self.READINGS_CACHE and len(self._readings_cache['humidity']) < self.READINGS_CACHE:
            self._get_readings()
            sleep(0.5)
        print("done filling sensor cache")
        reader_thread = threading.Thread(target=self._readings_thread, args=())
        reader_thread.daemon = True 
        reader_thread.start()

    def _get_readings(self):
        for reading_type, reading_cache in self._readings_cache.items():
            readings = reading_cache.copy()
            reading = getattr(self._sensor, reading_type)
            readings.append(reading)
            if len(readings) > self.READINGS_CACHE:
                readings = readings[len(readings) - self.READINGS_CACHE:]
            self._readings_cache[reading_type] = readings

    def _readings_thread(self):
        while True:
            self._get_readings()
            sleep(0.5)

    @property
    def temperature(self):
        return self._normalise_readings(self._readings_cache['temperature'])
    
    @property
    def humidity(self):
        return self._normalise_readings(self._readings_cache['humidity'])

    @staticmethod
    def _normalise_readings(readings):
        if len(readings) > 8:
            readings = sorted(readings)[4:len(readings)-4]
        return mean(readings)


class FanTach:
    def __init__(self, pi, gpio, pulses_per_rev=2.0, min_RPM=120.0):
      self.pi = pi
      self.gpio = gpio
      self.pulses_per_rev = pulses_per_rev
      if min_RPM > 1000.0:
         min_RPM = 1000.0
      elif min_RPM < 1.0:
         min_RPM = 1.0
      self.min_RPM = min_RPM
      self._watchdog = 200 # Milliseconds.
      self._new = 1.0 
      self._old = 0.0
      self._high_tick = None
      self._period = None
      pi.set_mode(gpio, pigpio.INPUT)
      self._cb = pi.callback(gpio, pigpio.RISING_EDGE, self._cbf)
      pi.set_watchdog(gpio, self._watchdog)

    def _cbf(self, gpio, level, tick):
      if level == 1: # Rising edge.
         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)
            if self._period is not None:
               self._period = (self._old * self._period) + (self._new * t)
            else:
               self._period = t
         self._high_tick = tick
      elif level == 2: # Watchdog timeout.
         if self._period is not None:
            if self._period < 2000000000:
               self._period += (self._watchdog * 1000)

    @property
    def RPM(self):
      RPM = 0.0
      if self._period is not None:
         RPM = 60000000.0 / (self._period * self.pulses_per_rev)
         if RPM < self.min_RPM:
            RPM = 0.0
      return RPM

    def cancel(self):
      self.pi.set_watchdog(self.gpio, 0) # cancel watchdog
      self._cb.cancel()


class Fan:
    RPM_DIVISOR=2.0

    def __init__(self, pin, tach_pin=0, freq=25000, min_dutycycle=28, max_dutycycle=100, pi=pigpio.pi()):
        if not pi.connected:
            raise ConnectionException("Cannot connect to pigpiod")
        self._pi = pi
        self._pin = pin
        self._tach_pin = tach_pin
        self._freq = freq
        self._min = min_dutycycle
        self._max = max_dutycycle
        self._range = self._max - self._min
        self._tach = None
        self._percentage = 0
        if tach_pin:
            pi.set_pull_up_down(tach_pin, pigpio.PUD_UP)
            self._tach = FanTach(pi, tach_pin, Fan.RPM_DIVISOR)
        self.speed(1)

    def _percent_to_dutycycle(self, percent):
        return int((self._range * (percent / 100) + self._min))

    def speed(self, percentage=None):
        if percentage is None:
            return self._percentage
        if percentage < 1 or percentage > 100:
            raise ValueError("percentage must be a number between 1 and 100")
        self._percentage = percentage
        self._pi.hardware_PWM(self._pin, self._freq, self._percent_to_dutycycle(percentage)*10000)
    
    @property
    def rpm(self):
        RPM = -1.0
        if self._tach is not None:
            RPM = self._tach.RPM
        return RPM


class Light:
    ON = 1
    OFF = 0
    MAX = 740

    def __init__(self, pwm_pin, power_pin = None, freq=6000, pi=pigpio.pi()):
        if not pi.connected:
            raise ConnectionException("Cannot connect to pigpiod")
        self._pi = pi
        self._pwm_pin = pwm_pin
        self._power_pin = power_pin
        self._freq = freq
        self._brightness = 0
        self.brightness(0)

    def _percent_to_dutycycle(self, percentage):
        val = int((math.log((100-percentage)+1) / math.log(100)) * Light.MAX)
        return val if val >= 0 else 0
    
    def brightness(self, brightness=None):
        if brightness is None:
            return self._brightness
        if brightness < 0 or brightness > 100:
            raise ValueError("percentage must be a number between 0 and 100")
        if brightness < 1 and self._brightness != brightness:
            self._pi.write(self._power_pin, Light.OFF)
            self._brightness = brightness
            return
        if self._brightness == 0:
            self._pi.write(self._power_pin, Light.ON)
        self._pi.hardware_PWM(self._pwm_pin, self._freq, self._percent_to_dutycycle(brightness)*1000)
        self._brightness = brightness


if __name__ == "__main__":
    
    inner_sensor = Sensor(INNER_SENSOR_I2C_PINS, INNER_TEMP_OFFSET, INNER_HUMIDITY_OFFSET)
    outer_sensor = Sensor(OUTER_SENSOR_I2C_PINS, OUTER_TEMP_OFFSET, OUTER_HUMIDITY_OFFSET)

    pi = pigpio.pi()
    fan = Fan(FAN_PWM_PIN, FAN_TACH_PIN, pi=pi)
    light = Light(LIGHT_PWM_PIN, LIGHT_POWER_PIN, LIGHT_PWM_FREQ, pi)

    light.brightness(0)
    fan.speed(1)
    recent_change = 0

    while True:
        timestamp = str(datetime.now())
        inner_temp = inner_sensor.temperature
        outer_temp = outer_sensor.temperature
        inner_humidity = inner_sensor.humidity
        outer_humidity = outer_sensor.humidity
        print(
            "%s TEMP: I: %0.2f O: %0.2f D: %0.2f HUMIDITY: I: %0.2f O: %0.2f D: %0.2f "
            "LAMP: %s%% FAN: %s%% %0.0f RPM" 
            % (
                timestamp, inner_temp, outer_temp, inner_temp - outer_temp, inner_humidity, outer_humidity, inner_humidity - outer_humidity, light.brightness(), fan.speed(), fan.rpm
            ))
        if recent_change < 1:
            if outer_temp > 25.0 and inner_humidity < 45.0 and light.brightness() > 0:
                print("%s turning off lamp, external temperature sufficient" % timestamp)
                light.brightness(0)
                recent_change = 6
            elif inner_temp > 23.0 and outer_humidity < 50.0 and inner_humidity < 50.0 and inner_humidity > 40.0 and outer_humidity > 40.0:
                if outer_humidity < inner_humidity - 0.2 and light.brightness() < 100:
                    print("%s increasing lamp brightness to reduce humidity" % timestamp)
                    light.brightness(light.brightness()+1)
                elif outer_humidity > inner_humidity + 0.2 and light.brightness() > 0:
                    print("%s decreasing lamp brightness to increase humidity" % timestamp)
                    light.brightness(light.brightness()-1)
            elif inner_temp > 23.0 and light.brightness() > 0 and inner_humidity < 40.0:
                print("%s turning off lamp, external temperature sufficient" % timestamp)
                light.brightness(0)
                recent_change = 6
            elif outer_temp > 24.0 and inner_temp > 25.0 and light.brightness() > 0 and outer_humidity > 40.0 and inner_humidity < 38.0:
                print("%s lowering lamp brightness to increase humidity" % timestamp)
                light.brightness(light.brightness()-1)
            #elif inner_temp < 24.0 and light.brightness() < 100 and inner_humidity > 40.0:
            #    print("%s increasing lamp brightness to up temperature" % timestamp)
            #    light.brightness(light.brightness()+1)
            elif inner_temp > 28.0 and light.brightness() > 0 and inner_humidity < 50.0:
                print("%s lowering lamp brightness to drop temperature" % timestamp)
                light.brightness(light.brightness()-1)
            elif inner_temp > 30.0 and light.brightness() > 0:
                print("%s lowering lamp brightness to drop temperature" % timestamp)
                light.brightness(light.brightness()-1)
            if inner_humidity < 40.0 and outer_humidity > 45.0 and fan.speed() < 100:
                print("%s increasing fan speed to bring in more moist air" % timestamp)
                fan.speed(fan.speed()+1)
            elif inner_humidity > 40.0 and inner_humidity > outer_humidity + 1.0 and fan.speed() < 100:
                print("%s increasing fan speed, inner humidity is higher than outer" % timestamp)
                fan.speed(fan.speed()+1)
            elif inner_humidity > 50.0 and fan.speed() < 100:
                print("%s max fan speed to counter high humidity" % timestamp)
                fan.speed(100)
                recent_change = 6
            elif inner_humidity < 40.0 and fan.speed() > 1:
                print("%s min fan speed to increase humidity" % timestamp)
                fan.speed(1)
                recent_change = 6
                if light.brightness() > 0:
                    print("%s lowering lamp brightness to lessen humidity drop" % timestamp)
                    light.brightness(light.brightness()-1)
        else:
            recent_change -= 1
            print("waiting out this cycle due to recent change")
        sleep(10)
