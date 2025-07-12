from machine import Pin, PWM, SoftI2C
import time
import dht
from i2c_lcd import I2cLcd
import sys

# === CONFIG ===
GAS_ALERT_DURATION_MS = 7000        # How long to keep fan + alert on after gas clears
FAN_DUTY = 500                      # Speed of fan (The range of duty cycle is 0-1023)
BEEP_TONE = [659, 523, 659, 784]    # F1 beep pattern

# === Setup pins ===
led = Pin(12, Pin.OUT)

fan_inA = PWM(Pin(19), freq=10000)
fan_inB = PWM(Pin(18), freq=10000)

button_left = Pin(16, Pin.IN, Pin.PULL_UP)
button_right = Pin(27, Pin.IN, Pin.PULL_UP)

motion_sensor = Pin(14, Pin.IN)
gas_sensor = Pin(23, Pin.IN)
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
fan_auto = False
last_left = 1
last_right = 1
last_lcd_update = 0
last_motion_state = -1
last_gas_state = -1
gas_clear_time = None
gas_alert_active = False
last_state_line = ""

# === Helper functions ===
def toggle_led():
    global led_on
    led_on = not led_on
    led.value(1 if led_on else 0)

def toggle_fan(on=None):
    global fan_on
    if on is None:
        fan_on = not fan_on
    else:
        fan_on = on

    if fan_on:
        fan_inA.duty(FAN_DUTY)
        fan_inB.duty(0)
    else:
        fan_inA.duty(0)
        fan_inB.duty(0)

def update_lcd(temp, hum, motion):
    lcd.move_to(0, 0)
    lcd.putstr("Temp: {:>2}C Hum:{:>2}%".format(temp, hum))
    lcd.move_to(0, 1)
    lcd.putstr("Motion: " + ("Yes " if motion else "No  "))

def show_gas_alert():
    lcd.move_to(0, 1)
    lcd.putstr("!! GAS ALERT !! ")

def f1_radio_beep():
    buzzer.duty(512)
    for freq in BEEP_TONE:
        buzzer.freq(freq)
        time.sleep(0.15)
    buzzer.duty(0)

def print_state_line():
    global last_state_line
    motion = "Yes" if motion_sensor.value() else "No"
    gas = "Yes" if not gas_sensor.value() else "No"  # flipped logic
    fan = "On" if fan_on else "Off"
    led = "On" if led_on else "Off"
    fan_mode = "Auto" if fan_auto else "Manual"

    line = f"Motion: {motion} | Gas: {gas} | Fan: {fan} ({fan_mode}) | LED: {led}"

    if line != last_state_line:
        print("\r" + line + " " * 10, end="")  # overwrite line
        last_state_line = line

# === Main loop ===
while True:
    # Toggle LED
    if button_left.value() == 0 and last_left == 1:
        toggle_led()
        time.sleep(0.2)
    last_left = button_left.value()

    # Toggle Fan manually
    if button_right.value() == 0 and last_right == 1:
        toggle_fan()
        fan_auto = False
        time.sleep(0.2)
    last_right = button_right.value()

    # Read motion
    motion_now = motion_sensor.value()
    last_motion_state = motion_now

    # Read DHT
    try:
        sensor.measure()
        temperature = sensor.temperature()
        humidity = sensor.humidity()
    except Exception:
        temperature = 0
        humidity = 0

    # === GAS detection ===
    gas_now = gas_sensor.value()

    if gas_now == 0 and last_gas_state == 1:  # 0 = gas detected
        f1_radio_beep()
        show_gas_alert()
        toggle_fan(on=True)
        fan_auto = True
        gas_alert_active = True
        gas_clear_time = None

    elif gas_now == 1 and last_gas_state == 0:  # 1 = air clean again
        gas_clear_time = time.ticks_ms()

    if gas_alert_active and gas_clear_time:
        if time.ticks_diff(time.ticks_ms(), gas_clear_time) > GAS_ALERT_DURATION_MS:
            gas_alert_active = False
            update_lcd(temperature, humidity, motion_now)
            if fan_auto:
                toggle_fan(on=False)
                fan_auto = False
            gas_clear_time = None

    last_gas_state = gas_now

    # LCD update
    now = time.ticks_ms()
    if not gas_alert_active and time.ticks_diff(now, last_lcd_update) > 2000:
        update_lcd(temperature, humidity, motion_now)
        last_lcd_update = now

    print_state_line()
    time.sleep(0.05)
