#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from smbus import SMBus
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.port import LegoPort
from ev3dev2.led import Leds

left_motor = LargeMotor(OUTPUT_B)
right_motor = LargeMotor(OUTPUT_C)
ultrasonic = UltrasonicSensor(INPUT_4)
leds = Leds()

PIXY_ADDRESS = 0x54
pixy_ok = False
try:
    LegoPort(INPUT_1).mode = 'other-i2c'
    time.sleep(0.5)
    pixy_bus = SMBus(3)
    pixy_ok = True
    print("Pixy2: قناة I2C فتحت")
except Exception as e:
    print("Pixy2: فشل -", e)

def get_pixy_block(sig):
    if not pixy_ok:
        return None
    try:
        data = [174, 193, 32, 2, sig, 1]
        pixy_bus.write_i2c_block_data(PIXY_ADDRESS, 0, data)
        time.sleep(0.01)
        block = pixy_bus.read_i2c_block_data(PIXY_ADDRESS, 0, 20)
        got_sig = block[6] + block[7] * 256
        w = block[12] + block[13] * 256
        if got_sig == sig and w > 0:
            x = block[8] + block[9] * 256
            return (x, w)
    except Exception:
        pass
    return None

SIG_RED, SIG_GREEN = 1, 2
FRAME_WIDTH = 316
CENTER_X = FRAME_WIDTH // 2
TARGET_RED_X = CENTER_X - 60
TARGET_GREEN_X = CENTER_X + 60

BASE_SPEED = 35
KP = 0.25
MAX_TURN = 25
SAFE_DISTANCE = 8

def drive(l, r):
    left_motor.on(l)
    right_motor.on(r)

def stop():
    left_motor.off(brake=True)
    right_motor.off(brake=True)

def steer_to(target_x, actual_x):
    error = actual_x - target_x
    turn = max(-MAX_TURN, min(MAX_TURN, KP * error))
    drive(BASE_SPEED + turn, BASE_SPEED - turn)

print("🚀 بدء")

try:
    while True:
        if ultrasonic.distance_centimeters < SAFE_DISTANCE:
            stop()
            time.sleep(0.05)
            continue

        red = get_pixy_block(SIG_RED)
        green = get_pixy_block(SIG_GREEN)

        if red and green:
            if red[1] >= green[1]:
                green = None
            else:
                red = None

        if red:
            leds.set_color("LEFT", "red")
            leds.set_color("RIGHT", "red")
            steer_to(TARGET_RED_X, red[0])
        elif green:
            leds.set_color("LEFT", "green")
            leds.set_color("RIGHT", "green")
            steer_to(TARGET_GREEN_X, green[0])
        else:
            leds.set_color("LEFT", "green")
            leds.set_color("RIGHT", "green")
            drive(BASE_SPEED, BASE_SPEED)

        time.sleep(0.03)

except KeyboardInterrupt:
    print("⛔ توقف")
    stop()
