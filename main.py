from machine import Pin, PWM, SoftI2C
import time
import dht
from i2c_lcd import I2cLcd

# === Setup pins ===
led = Pin(12, Pin.OUT)

fan_inA = PWM(Pin(19), freq=10000)
fan_inB = PWM(Pin(18), freq=10000)

button_left = Pin(16, Pin.IN, Pin.PULL_UP)
button_right = Pin(27, Pin.IN, Pin.PULL_UP)

motion_sensor = Pin(14, Pin.IN)
buzzer = PWM(Pin(25))

# === DHT Sensor ===
sensor = dht.DHT11(Pin(17))

# === LCD I2C ===
DEFAULT_I2C_ADDR = 0x27
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)
lcd = I2cLcd(i2c, DEFAULT_I2C_ADDR, 2, 16)
lcd.backlight_on()

# === States ===
led_on = False
fan_on = False
last_left = 1
last_right = 1
last_lcd_update = 0
last_motion_state = -1

# === Helper functions ===
def toggle_led():
    global led_on
    led_on = not led_on
    led.value(1 if led_on else 0)

def toggle_fan():
    global fan_on
    fan_on = not fan_on
    if fan_on:
        fan_inA.duty(700)
        fan_inB.duty(0)
    else:
        fan_inA.duty(0)
        fan_inB.duty(0)

def update_lcd(temp, hum, motion):
    lcd.move_to(0, 0)
    lcd.putstr("Temp: {:>2}C Hum:{:>2}%".format(temp, hum))
    lcd.move_to(0, 1)
    lcd.putstr("Motion: " + ("Yes " if motion else "No  "))

def make_sound():
    buzzer.freq(100)       # Very low frequency (deep)
    buzzer.duty(50)        # Very low volume (duty out of 1023)
    time.sleep(0.02)       # Very short duration (20ms)
    buzzer.duty(0)

# === Main loop ===
while True:
    # Toggle LED
    if button_left.value() == 0 and last_left == 1:
        toggle_led()
        time.sleep(0.2)
    last_left = button_left.value()

    # Toggle Fan
    if button_right.value() == 0 and last_right == 1:
        toggle_fan()
        time.sleep(0.2)
    last_right = button_right.value()

    # Read motion
    motion_now = motion_sensor.value()

    # Play F1 radio beep on new motion
    if motion_now == 1 and last_motion_state == 0:
        make_sound()

    # Read temp and humidity
    try:
        sensor.measure()
        temperature = sensor.temperature()
        humidity = sensor.humidity()
    except Exception:
        temperature = 0
        humidity = 0

    # Update LCD on motion change or every 2s
    now = time.ticks_ms()
    if motion_now != last_motion_state or time.ticks_diff(now, last_lcd_update) > 2000:
        update_lcd(temperature, humidity, motion_now)
        last_lcd_update = now
        last_motion_state = motion_now

    time.sleep(0.05)

