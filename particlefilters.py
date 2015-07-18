import fileinput
import math
import pygame
import random
import scipy.stats
import sys
import polymap
from helpers import *
from pygame.locals import *

maxX = 640
maxY = 480

motionDx = 10
motionDy = 10

sensorMean = 0
sensorStd = 5

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

	map = polymap.PolyMap(15, 6, 150, 0, maxX, 0, maxY)

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

main()