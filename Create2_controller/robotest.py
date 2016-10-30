from breezycreate2 import Robot
import time

# Create a Create2. This will automatically try to connect to your robot over serial
bot = Robot()

# Play a note to let us know you're alive!
bot.playNote('A4', 100)

# Tell the Create2 to turn right slowly
bot.setTurnSpeed(-50)

# Wait a second
time.sleep(1)

# Stop
bot.setTurnSpeed(0)

# Report bumper hits and wall proximity for 30 seconds
start_time = time.time()
while (time.time() - start_time) < 30:
    print('Bumpers: ' + str(bot.getBumpers()) + '    Wall: ' + str(bot.getWallSensor()))

# Close the connection
bot.close()