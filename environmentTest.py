import math

class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def distance(self):
		global special
		distance = int(round(((self.x - xx)**2 + (self.y - yy)**2)**0.5))
		special = distance
		return distance

def distance():
	global special
	special = 100

special = 0
xx = 0
yy = 0
pA = Point(3,3)
pA.distance()
distance()
print special
