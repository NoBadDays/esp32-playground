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

motion_sensor = Pin(23, Pin.IN)

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
motion_detected = False
motion_last_time = 0
MOTION_DISPLAY_TIME = 3000  # milliseconds to show "Yes" after motion

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

def update_lcd():
    global motion_detected, motion_last_time
    try:
        sensor.measure()
        t = sensor.temperature()
        h = sensor.humidity()

        # Check for motion
        if motion_sensor.value() == 1:
            motion_detected = True
            motion_last_time = time.ticks_ms()

        # Clear motion after timeout
        if motion_detected and time.ticks_diff(time.ticks_ms(), motion_last_time) > MOTION_DISPLAY_TIME:
            motion_detected = False

        # Update display
        lcd.move_to(0, 0)
        lcd.putstr("Temp: {:>2}C Hum:{:>2}%".format(t, h))
        lcd.move_to(0, 1)
        lcd.putstr("Motion: " + ("Yes " if motion_detected else "No  "))
    except Exception:
        lcd.move_to(0, 0)
        lcd.putstr("Sensor Error     ")
        lcd.move_to(0, 1)
        lcd.putstr("                ")

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

    # Update LCD every 2 seconds or on motion
    now = time.ticks_ms()
    if time.ticks_diff(now, last_lcd_update) > 2000 or motion_sensor.value() == 1:
        update_lcd()
        last_lcd_update = now

    time.sleep(0.01)
