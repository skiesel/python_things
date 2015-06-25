import pygame
import random
import sys
from pygame.locals import *

maxX = 50
maxY = 25

white = (255,255,255,255)
black = (0,0,0,255)

class Image:
	def __init__(self, surface):
		self.width, self.height = surface.get_size()
		self.pixels = []
		for y in range(self.height):
			row = []
			for x in range(self.width):
				if surface.get_at((x,y)) == white:
					row.append(0)
				else:
					row.append(1)
			self.pixels.append(row)

	def __str__(self):
		str = ""
		for row in self.pixels:
			for val in row:
				if val == 0:
					str += " "
				else:
					str += "#"
			str += "\n"
		return str


def randomDrawing(screen, lines=None):
	width, height = screen.get_size()
	for i in range(lines):
		pt1 = floatPtToIntPt(getRandomPoint(0,width,0,height))
		pt2 = floatPtToIntPt(getRandomPoint(0,width,0,height))
		pygame.draw.line(screen, black, pt1, pt2, 1)

def main():
	pygame.init()
	pygame.display.set_caption('RANSAC')
	screen = pygame.display.set_mode((maxX, maxY))
	screen.fill(white)

	randomDrawing(screen, 10)

	image = Image(screen)
	print(image)

	pygame.display.update()

	while (True):
		event = pygame.event.wait()
		if event.type == pygame.QUIT:
			pygame.quit(); sys.exit();

def floatPtToIntPt(pt):
	return (int(pt[0]), int(pt[1]))

def getRandomPoint(minX, maxX, minY, maxY):
	return (getRandomValue(minX,maxX), getRandomValue(minY,maxY))

def getRandomValue(min=0, max=1):
	return random.random() * (max - min) + min

main()

