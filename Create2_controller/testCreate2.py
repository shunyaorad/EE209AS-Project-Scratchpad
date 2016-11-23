import create2api
import time

bot = create2api.Create2()
bot.start()
bot.safe()

t = 4.15

def turn90CW():
	bot.turn_clockwise(15)
	time.sleep(t)


def driveStraight():
	bot.drive_straight(15)
	time.sleep(15)

def killBot():
	bot.drive_straight(0)
	bot.turn_clockwise(0)
	bot.destroy()

driveStraight()

#1
turn90CW()
driveStraight()

# #2
turn90CW()
driveStraight()


# #3
turn90CW()
driveStraight()


# #4
turn90CW()

killBot()