import random

white = (255,255,255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
black = (0, 0, 0)

def floatPtToIntPt(pt):
	return (int(pt[0]), int(pt[1]))

def getRandomPoint(minX, maxX, minY, maxY):
	return (getRandomValue(minX,maxX), getRandomValue(minY,maxY))

def getRandomValue(min=0, max=1):
	return random.random() * (max - min) + min