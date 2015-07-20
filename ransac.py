import pygame
import random
import sys
import time
import math
from helpers import *
from pygame.locals import *

maxX = 500
maxY = 250

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

	def sample(self):
		for row in range(len(self.pixels)):
			for col in range(len(self.pixels[row])):
				if self.pixels[row][col] == 1:
					if getRandomValue() < 0.25:
						self.pixels[row][col] = 1
						continue
				self.pixels[row][col] = 0

	def makeNoisy(self, noiseDistance):
		for row in range(len(self.pixels)):
			for col in range(len(self.pixels[row])):
				if self.pixels[row][col] == 1:
					value = getRandomValue()
					dist = int(getRandomValue(1, noiseDistance+1))
					if value < 0.20:
						if value < 0.05 and row > dist:
							self.pixels[row][col] = 0
							self.pixels[row-dist][col] = 1
						elif value < 0.1 and col > dist:
							self.pixels[row][col] = 0
							self.pixels[row][col-dist] = 1
						elif value < 0.15 and row < (len(self.pixels)-dist):
							self.pixels[row][col] = 0
							self.pixels[row+dist][col] = 1
						elif col < (len(self.pixels[row])-dist):
							self.pixels[row][col] = 0
							self.pixels[row][col+dist] = 1
					continue
				self.pixels[row][col] = 0

	def draw(self, screen):
		for row in range(len(self.pixels)):
			for col in range(len(self.pixels[row])):
				if self.pixels[row][col] == 0:
					screen.set_at((col,row), white)
				else:
					screen.set_at((col,row), black)

	def getPixelCoords(self):
		coords = []
		for row in range(len(self.pixels)):
			for col in range(len(self.pixels[row])):
					if self.pixels[row][col] == 1:
						coords.append((col,row))
		return coords

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

class RansacLines:

	def __init__(self, screen, image, inlierThreshold, errorThreshold):
		coords = image.getPixelCoords()
		self.goodLines = []

		screen.fill(white)
		image.draw(screen)

		inlierSet = []

		for i in range(5000):
			index1 = int(getRandomValue() * len(coords))
			index2 = int(getRandomValue() * len(coords))
			while index1 == index2:
				index2 = int(getRandomValue() * len(coords))

			coord1 = coords[index1]
			coord2 = coords[index2]

			inliers = self.__checkInliersLines(screen, coords, (coord1, coord2), errorThreshold)

			if len(inliers[1]) >= inlierThreshold:
				inlierSet.append((inliers[0], inliers[1], (coord1, coord2)))

		sorted(inlierSet, key=lambda inlier: inlier[0])
		howmany = min(10, len(inlierSet))
		for i in range(howmany):
			pt1, pt2 = inlierSet[i][2]
			pygame.draw.line(screen, red, pt1, pt2, 2)
		pygame.display.update()

	def __checkInliersCircle(self, screen, coods, pt1, pt2, errorThreshold):
		center = ((pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2)
		radiusSqr = self.__pointToPointSqrDistance(center, pt1)
		for coord in coords:
			distSqr = self.__pointToPointSqrDistance(center, coord)
			if math.fabs(distSqr - radiusSqr) <= errorThreshold:
				inlierError += distSqr
				inliers.append(coord)

		return inlierError, inliers


	def __checkInliersLines(self, screen, coords, line, errorThreshold):
		pt1, pt2 = line

		slope = 0.
		if pt2[0] - pt1[0] == 0:
			slope = float("inf")
		else:
			slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])

		inliers = []
		inlierError = 0

		sqrThresh = errorThreshold * errorThreshold
		for coord in coords:
			near = self.__nearestPointOnLine(coord, line, slope)

			err = self.__pointToPointSqrDistance(coord, near)
			if err < sqrThresh:
				inlierError += err
				inliers.append(coord)

		return inlierError, inliers

	def __nearestPointOnLine(self, pt, line, slope):
		if math.fabs(slope) < 0.000001: #horizontal line
			return (pt[0], line[0][1])
		elif math.isinf(slope):  #vertical line
			return (line[0][0], pt[1])

		v1 = (line[1][0] - line[0][0], line[1][1] - line[0][1])
		v2 = (pt[0] - line[0][0], pt[1] - line[0][1])

		factor = self.__dotProduct(v2, v1) / self.__dotProduct(v1, v1)
		projection = (factor * v1[0] + line[0][0], factor * v1[1] + line[0][1])

		return projection

	def __dotProduct(self, v1, v2):
		return v1[0] * v2[0] + v1[1] * v2[1]
		
	def __pointToPointSqrDistance(self, pt1, pt2):
		dx = pt1[0] - pt2[0]
		dy = pt1[1] - pt2[1]
		return dx * dx + dy * dy
	

class RansacCircles:

	def __init__(self, screen, image, inlierThreshold, errorThreshold):
		coords = image.getPixelCoords()
		self.goodLines = []

		screen.fill(white)
		image.draw(screen)

		inlierSet = []

		for i in range(10000):
			index1 = int(getRandomValue() * len(coords))
			index2 = int(getRandomValue() * len(coords))
			while index1 == index2:
				index2 = int(getRandomValue() * len(coords))

			coord1 = coords[index1]
			coord2 = coords[index2]

			center = ((coord1[0] + coord2[0]) / 2, (coord1[1] + coord2[1]) / 2)
			radiusSqr = self.__pointToPointSqrDistance(center, coord1)

			inliers = self.__checkInliersCircle(screen, coords, center, radiusSqr, errorThreshold)

			if len(inliers[1]) >= inlierThreshold:
				inlierSet.append((inliers[0], inliers[1], (floatPtToIntPt(center), int(math.sqrt(radiusSqr)))))

		sorted(inlierSet, key=lambda inlier: inlier[0])
		howmany = min(10, len(inlierSet))
		for i in range(howmany):
			center, radius = inlierSet[i][2]
			pygame.draw.circle(screen, red, center, radius, 1)
		pygame.display.update()

	def __checkInliersCircle(self, screen, coords, center, radiusSqr, errorThreshold):
		inliers = []
		inlierError = 0
		for coord in coords:
			distSqr = self.__pointToPointSqrDistance(center, coord)
			if math.fabs(distSqr - radiusSqr) <= errorThreshold:
				inlierError += distSqr
				inliers.append(coord)

		return inlierError, inliers

	def __pointToPointSqrDistance(self, pt1, pt2):
		dx = pt1[0] - pt2[0]
		dy = pt1[1] - pt2[1]
		return dx * dx + dy * dy

def randomLineDrawing(screen, numLines):
	width, height = screen.get_size()
	for i in range(numLines):
		pt1 = floatPtToIntPt(getRandomPoint(0,width,0,height))
		pt2 = floatPtToIntPt(getRandomPoint(0,width,0,height))
		pygame.draw.line(screen, black, pt1, pt2, 1)

def randomCircleDrawing(screen, numCircles):
	width, height = screen.get_size()

	maxRadius = min(width, height) / 2

	for i in range(numCircles):
		radius = int(getRandomValue(10, maxRadius))
		center = floatPtToIntPt(getRandomPoint(radius,width-radius,radius,height-radius))
		pygame.draw.circle(screen, black, center, radius, 1)

def main():
	pygame.init()
	pygame.display.set_caption('RANSAC')
	screen = pygame.display.set_mode((maxX, maxY))
	screen.fill(white)

	time.sleep(10)

#	randomLineDrawing(screen, 10)
	randomCircleDrawing(screen, 5)

	image = Image(screen)
	# print(image)

	pygame.display.update()

	time.sleep(1)

	screen.fill(white)

	image.sample()

	image.draw(screen)

	pygame.display.update()
	time.sleep(1)

	screen.fill(white)

	image.makeNoisy(10)

	image.draw(screen)

	pygame.display.update()

	# RansacLines(screen, image, 50, 5)
	RansacCircles(screen, image, 25, 25)

	while (True):
		event = pygame.event.wait()
		if event.type == pygame.QUIT:
			pygame.quit(); sys.exit();

main()

