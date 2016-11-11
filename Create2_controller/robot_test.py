import time
import create2api

bot = create2api.Create2()
bot.start()
bot.safe()

bot.turn_clockwise(15)
time.sleep(5)
bot.turn_clockwise(0)
bot.destroy()