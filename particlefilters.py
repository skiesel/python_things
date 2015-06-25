import fileinput
import math
import pygame
import pyqtree
import random
import scipy.stats
import sys
from pygame.locals import *

maxX = 640
maxY = 480

motionDx = 10
motionDy = 10

sensorMean = 0
sensorStd = 5

white = (255,255,255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
black = (0, 0, 0)


class Map:
	def __init__(self, numPolys, numVertices, radius, minX, maxX, minY, maxY):
		self.minX = minX
		self.maxX = maxX
		self.minY = minY
		self.maxY = maxY
		self.numVertices = numVertices
		self.polygons = self.__generateMap(numPolys, numVertices, radius, minX, maxX, minY, maxY)
		self.spatialIndex = pyqtree.Index(bbox=[minX, minY, maxX, maxY])

		for i in range(len(self.polygons)):
			self.spatialIndex.insert(item=tuple(self.polygons[i]), bbox=self.__getPolygonBBox(self.polygons[i]))

	def __getPolygonBBox(self, poly):
		xmin = float("inf")
		xmax = -float("inf")
		ymin = float("inf")
		ymax = -float("inf")
		for i in range(len(poly)):
			xmin = self.__getMin(xmin, poly[i][0])
			xmax = self.__getMax(xmax, poly[i][0])
			ymin = self.__getMin(ymin, poly[i][1])
			ymax = self.__getMax(ymax, poly[i][1])
		return xmin, ymin, xmax, ymax


	def draw(self, screen):
		for i in range(len(self.polygons)):
			self.__drawPolygon(screen, self.polygons[i])
	
	def contains(self, point):
		return point[0] >= self.minX and point[0] <= self.maxX and point[1] >= self.minY and point[1] <= self.maxY

	def isContainedByObstacle(self, point):
		pointBBox = (point[0], point[1], point[0], point[1])
		matches = self.spatialIndex.intersect(pointBBox)
		for poly in matches:
			if self.__getWindingNumber(poly, point) != 0:
				return True
		return False

	def __determinant2(self, a, b, c, d):
		return (a*d) - (b*c)

	def __determinant3(self, a, b, c, d, e, f, g, h, i):
		return (a*e*i) + (b*f*g) + (c*d*h) - (c*e*g) - (b*d*i) - (a*f*h)

	def __isLeft(self, p1, p2, p3):
		return self.__determinant3(1, p1[0], p1[1],
								   1, p2[0], p2[1],
								   1, p3[0], p3[1]) > 0

	def __isRight(self, p1, p2, p3):
		return self.__determinant3(1, p1[0], p1[1],
								   1, p2[0], p2[1],
								   1, p3[0], p3[1]) < 0

	def __getWindingNumber(self, polygon, point):
		wn = 0
		polygon = tuple(polygon[:]) + (polygon[0],) #close up the polygon
		
		for i in range(len(polygon)-1):
			if polygon[i][1] <= point[1]:
				if polygon[i+1][1] > point[1]:
					if self.__isLeft(polygon[i], polygon[i+1], point):
						wn += 1
			else:
				if polygon[i+1][1] <= point[1]:
					if self.__isRight(polygon[i], polygon[i+1], point):
						wn -= 1
		return wn

	def __distanceSquared(self, p1, p2):
		dx = p1[0] - p2[0]
		dy = p1[1] - p2[1]
		return dx*dx + dy*dy

	def isThereObstacleIntersection(self, line):
		bBox = self.__getPolygonBBox(line)
		matches = self.spatialIndex.intersect(bBox)
		for poly in matches:
			for j in range(len(poly)):
				k = j + 1
				if k >= len(poly):
					k = 0
				if self.__getLineSegLineSegIntersection(line, (poly[j], poly[k])):

					return True
		return False

	def nearestObstacleIntersection(self, line):
		minDist = float("inf")
		intersection = None
		bBox = self.__getPolygonBBox(line)
		matches = self.spatialIndex.intersect(bBox)
		for poly in matches:
			for j in range(len(poly)):
				k = j + 1
				if k >= len(poly):
					k = 0
				intersectionCandidate = self.__getLineSegLineSegIntersection(line, (poly[j], poly[k]))
				if intersectionCandidate:
					dist = self.__distanceSquared(line[0], intersectionCandidate)
					if dist < minDist:
						minDist = dist
						intersection = intersectionCandidate

		return intersection

	def __getLineSegLineSegIntersection(self, l1, l2):
		intersection = self.__getLineLineIntersection(l1, l2)
		if intersection and self.__pointInLineBBox(l1, intersection) and self.__pointInLineBBox(l2, intersection):
			return intersection
		return None

	def __pointInLineBBox(self, line, point):
		xmin = self.__getMin(line[0][0], line[1][0])
		xmax = self.__getMax(line[0][0], line[1][0])
		ymin = self.__getMin(line[0][1], line[1][1])
		ymax = self.__getMax(line[0][1], line[1][1])
		return point[0] + 0.00001 >= xmin and point[0] - 0.00001 <= xmax and point[1] + 0.00001 >= ymin and point[1] - 0.00001 <= ymax

	def __getMax(self, a, b):
		if a > b:
			return a
		return b

	def __getMin(self, a, b):
		if a < b:
			return a
		return b

	def __getLineLineIntersection(self, l1, l2):
		e = self.__determinant2(l1[0][0], 1, l1[1][0], 1)
		f = self.__determinant2(l1[0][1], 1, l1[1][1], 1)
		g = self.__determinant2(l2[0][0], 1, l2[1][0], 1)
		h = self.__determinant2(l2[0][1], 1, l2[1][1], 1)

		denom = self.__determinant2(e, f, g, h)

		if denom == 0:
			return None

		a = self.__determinant2(l1[0][0], l1[0][1], l1[1][0], l1[1][1])
		b = self.__determinant2(l1[0][0], 1, l1[1][0], 1)
		c = self.__determinant2(l2[0][0], l2[0][1], l2[1][0], l2[1][1])
		d = self.__determinant2(l2[0][0], 1, l2[1][0], 1)

		x = self.__determinant2(a, b, c, d) / denom
		y = self.__determinant2(a, f, c, h) / denom

		return (x, y)

	def __drawPolygon(self, screen, polyPoints):
		pygame.draw.polygon(screen, black, polyPoints)

	def __pointInPoly(self, point, poly):
		point = ShapelyPoint(point.x, point.y)
		return 

	def __generateMap(self, numPolys, numVertices, radius, minX, maxX, minY, maxY):
		polygons = []
		for i in range(numPolys):
			polygons.append(self.__getRandomPolygon(numVertices, radius, minX, maxX, minY, maxY))
		return polygons

	def __getRandomPolygon(self, numVertices, radius, minX, maxX, minY, maxY):
		vertices = []
		thetaInc = 2*math.pi / numVertices
		center = getRandomPoint(minX,maxX,minY,maxY)
		for i in range(numVertices):
			curRadius = getRandomValue(0,radius)
			vertices.append((center[0] + math.cos(i*thetaInc) * curRadius,
							 center[1] + math.sin(i*thetaInc) * curRadius))
		return vertices

	def __buildShapelyPolgyons(self):
		shapelyPolygons = []
		for i in range(len(self.polygons)):
			arr = ()
			for j in range(len(self.polygons[i])):
				arr = arr + ((self.polygons[i][j][0], self.polygons[i][j][1]),)
			shapelyPolygons.append(ShapelyPolgon(arr))
		return shapelyPolygons


class Agent:
	def __init__(self, scanCount, maxScanLength):
		self.x = 0
		self.y = 0
		self.scanCount = scanCount
		self.maxScanLength = maxScanLength
		self.scans = []
		self.noise = scipy.stats.norm(sensorMean, sensorStd)

	def randomNotInCollision(self, map):
		pt = getRandomPoint(map.minX, map.maxX, map.minY, map.maxY)
		while map.isContainedByObstacle(pt):
			pt = getRandomPoint(map.minX, map.maxX, map.minY, map.maxY)
		self.x = pt[0]
		self.y = pt[1]
	
	def applyMotion(self, motion, map, externalPoint=None):
		dx = 0
		dy = 0
		if motion == 0:
			dx = motionDx
		elif motion == 1:
			dx = -motionDx
		elif motion == 2:
			dy = motionDy
		elif motion == 3:
			dy = -motionDy

		dxNoise = getRandomValue()
		dyNoise = getRandomValue()
		if dxNoise < 0.16:
			if dxNoise < 0.8:
				dx += motionDx * 0.25
			else:
				dx -= motionDx * 0.25
		if dyNoise < 0.16:
			if dyNoise < 0.8:
				dy += motionDy * 0.25
			else:
				dy -= motionDy * 0.25

		if externalPoint:
			newPt = (externalPoint[0] + dx, externalPoint[1] + dy)
			traj = ((externalPoint[0], externalPoint[1]), newPt)
			if map.contains(newPt) and not map.isThereObstacleIntersection(traj):
				return newPt
			return externalPoint
		else:
			newPt = (self.x + dx, self.y + dy)
			traj = ((self.x, self.y), newPt)
			if map.contains(newPt) and not map.isThereObstacleIntersection(traj):
				self.x = newPt[0]
				self.y = newPt[1]

	def scan(self, map, groundTruth=False):
		self.scans = []
		scanDeltas = []
		scanLengths = []
		thetaInc = 2*math.pi / self.scanCount
		start = (self.x, self.y)
		for i in range(self.scanCount):
			end = (start[0] + math.cos(i*thetaInc) * self.maxScanLength,
					start[1] + math.sin(i*thetaInc) * self.maxScanLength)
			scan = ((start[0], start[1]), (end[0], end[1]))
			
			intersection = map.nearestObstacleIntersection(scan)
			
			hit = end
			if intersection:
				hit = intersection
			if not groundTruth:
				err = self.noise.rvs()
				hit = (hit[0] + math.cos(i*thetaInc) * err,
					   hit[1] + math.sin(i*thetaInc) * err)

			self.scans.append(hit)
			scanLengths.append(math.hypot(start[0] - hit[0], start[1] - hit[1]))
			scanDeltas.append((hit[0] - start[0], hit[1] - start[1]))
		return scanLengths, scanDeltas

	def draw(self, screen, drawScan=False):
		pygame.draw.circle(screen, blue, floatPtToIntPt((self.x, self.y)), 2)
		if drawScan:
			for i in range(len(self.scans)):
				pygame.draw.line(screen, blue, floatPtToIntPt((self.x, self.y)), floatPtToIntPt(self.scans[i]))

class ParticleFilter:
	def __init__(self, map, count=0):
		self.particles = []
		self.weights = []
		self.mostLikely = -1
		for i in range(count):
			self.particles.append(self.__getRandomNotInCollision(map))

	def addParticle(self, x, y, w):
		self.particles.append((x,y))
		self.weights.append(w)

	def draw(self, screen, scanLines=None):
		for i in range(len(self.particles)):
			pygame.draw.circle(screen, red, floatPtToIntPt(self.particles[i]), 2)

		if scanLines and self.mostLikely >= 0:
			likely = floatPtToIntPt((self.particles[self.mostLikely][0], self.particles[self.mostLikely][1]))
			for i in range(len(scanLines)):
				endPt = floatPtToIntPt((likely[0] + scanLines[i][0], likely[1] + scanLines[i][1]))
				pygame.draw.line(screen, red, likely, endPt)

	def applyMotion(self, motion, map, agent):
		updatedParticles = []
		for i in range(len(self.particles)):
			newPt = agent.applyMotion(motion, map, self.particles[i])
			if newPt:
				updatedParticles.append(newPt)
			else:
				updatedParticles.append(self.particles[i])
		self.particles = updatedParticles

	def applyScan(self, scanLengths, agent, map):
		thetaInc = 2*math.pi / len(scanLengths)
		self.weights = []
		total = 0.
		for i in range(len(self.particles)):
			weight = 1
			for j in range(len(scanLengths)):
				weight *= self.__getProbabilityOfScanLine(self.particles[i], map, agent, scanLengths[j], j*thetaInc)
			self.weights.append(weight)
			total += weight
		
		maxWeight = 0
		for i in range(len(self.weights)):
			self.weights[i] /= total
			if self.weights[i] > maxWeight:
				maxWeight = self.weights[i]
				self.mostLikely = i


	def __getProbabilityOfScanLine(self, particle, map, agent, scanLength, theta):
		end = (particle[0] + math.cos(theta) * agent.maxScanLength, particle[1] + math.sin(theta) * agent.maxScanLength)
		scan = ((particle[0], particle[1]), (end[0], end[1]))

		intersection = map.nearestObstacleIntersection(scan)
		hit = end
		if intersection:
			hit = intersection

		sensorEnd = (particle[0] + math.cos(theta) * scanLength,
					 particle[1] + math.sin(theta) * scanLength)

		diff = math.hypot(hit[0] - sensorEnd[0], hit[1] - sensorEnd[1])

		return agent.noise.pdf(diff)

	def resampleParticles(self, map, rejuvenate):
		newParticles = []

		howMany = int(len(self.particles) * (1 -rejuvenate))

		hist = dict()

		for i in range(howMany):
			randomVal = getRandomValue()
			
			sum = 0
			selected = -1
			while sum <= randomVal:
				selected += 1
				sum += self.weights[selected]

			newParticles.append(self.particles[selected])
			if not selected in hist:
				hist[selected] = 1
			else:
				hist[selected] += 1

		howMany = len(self.particles) - howMany
		for i in range(howMany):
			newParticles.append(self.__getRandomNotInCollision(map))

		self.particles = newParticles

	def __getRandomNotInCollision(self, map):
		pt = getRandomPoint(map.minX, map.maxX, map.minY, map.maxY)
		while map.isContainedByObstacle(pt):
			pt = getRandomPoint(map.minX, map.maxX, map.minY, map.maxY)
		return pt

def main():
	# random.seed(2)

	pygame.init()
	pygame.display.set_caption('Particle Filter')
	screen = pygame.display.set_mode((maxX, maxY))

	map = Map(15, 6, 150, 0, maxX, 0, maxY)

	agent = Agent(13, 100)
	agent.randomNotInCollision(map)

	filter = ParticleFilter(map, 100)

	first = True

	while (True):
		screen.fill(white)

		if not first:
			dir = getDirection(screen, agent, 1)
			filter.resampleParticles(map, 0.25)
			agent.applyMotion(dir, map)
			filter.applyMotion(dir, map, agent)
			#printMoveMessage(dir)

		first = False

		map.draw(screen)
		scanLengths, scanLines = agent.scan(map)

		#printScanMessage(scanLengths)

		agent.draw(screen, True)

		filter.applyScan(scanLengths, agent, map)

		filter.draw(screen, scanLines)

		# update the screen
		pygame.display.update()

def printScanMessage(scans):
	print("Current scan is:", end=" ")
	for scan in scans:
		print(scan, end=" ")
	print("")

def printMoveMessage(dir):
	print("Previous action was:", end=" ")
	if dir == 0:
		print("RIGHT")
	elif dir == 1:
		print("LEFT")
	elif dir == 2:
		print("DOWN")
	elif dir == 3:
		print("UP")

def readParticles(str):
	tokens = str.split()
	particleCount = int(tokens[0])
	filter = ParticleFilter()
	for i in range(particleCount):
		coord = input().split()
		filter.addParticle(float(coord[0]), float(coord[1]), float(coord[2]))
	return filter

def readMostLikely(str):
	tokens = str.split(":")
	coords = tokens[1].split()
	return (coords[0], coords[1])


def getDirection(screen, agent, mouseKeyboardStdin):
	if mouseKeyboardStdin == 0:
		command = input()
		if command == "RIGHT":
			return 0
		elif command == "LEFT":
			return 1
		elif command == "DOWN":
			return 2
		elif command == "UP":
			return 3
	elif mouseKeyboardStdin == 1:
		while True:
			event = pygame.event.wait()
			if event.type == pygame.QUIT:
				pygame.quit(); sys.exit();
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_RIGHT:
					return 0
				elif event.key == pygame.K_LEFT:
					return 1
				elif event.key == pygame.K_DOWN:
					return 2
				elif event.key == pygame.K_UP:
					return 3
	return -1

def floatPtToIntPt(pt):
	return (int(pt[0]), int(pt[1]))

def getRandomPoint(minX, maxX, minY, maxY):
	return (getRandomValue(minX,maxX), getRandomValue(minY,maxY))

def getRandomValue(min=0, max=1):
	return random.random() * (max - min) + min

main()