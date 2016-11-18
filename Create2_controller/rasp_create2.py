import cv2
import sys
import time
import create2api

bot = create2api.Create2()
bot.start()
bot.safe()

try:
    while True:
        cmd = raw_input("Command, r: right, l: left, f: forward, b: backward, s: stop")
        if len(cmd) > 0:
            direction = cmd[0]
        if direction == "r":
            print "turn right"
            bot.turn_clockwise(15)
        if direction == "l":
            print "turn left"
            bot.turn_clockwise(-15)
        if direction == "f":
            print "drive forward"
            bot.drive_straight(15)
        if direction == "b":
            print "drive backward"
            bot.drive_straight(-15)
        if direction == "s":
            print "stopped"
            bot.drive_straight(0)
            bot.turn_clockwise(0)

except KeyboardInterrupt:
    bot.drive_straight(0)
    bot.turn_clockwise(0)
    print "\nkeyboard interrupt, stopped"
    bot.destroy()