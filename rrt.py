import fileinput
import math
import pygame
import time
import polymap
import kdtree
from helpers import *
from pygame.locals import *

maxX = 640
maxY = 480

white = (255,255,255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
black = (0, 0, 0)

FRICTION=0.01

class Hovercraft:
	def __init__(self, initX=0., initY=0., initTheta=0.):
		self.x = initX
		self.y = initY
		self.theta = initTheta

		self.dx = 0.
		self.dy = 0.
		self.dtheta = 0.

	def build(self, vals):
		self.x = vals[0]
		self.y = vals[1]
		self.theta = vals[2]
		self.dx = vals[3]
		self.dy = vals[4]
		self.dtheta = vals[5]

	def getPoint(self):
		return (self.x, self.y, self.theta, self.dx, self.dy, self.dtheta)

	def step(self, linearAcceleration, rotationAcceleration, dt):
		self.dx += linearAcceleration * math.cos(self.theta) * dt
		self.dy += linearAcceleration * math.sin(self.theta) * dt
		self.dtheta += rotationAcceleration * dt

		if self.dx > 10 or self.dx < -10:
			self.dx = math.copysign(10, self.dx)

		if self.dy > 10 or self.dy < -10:
			self.dy = math.copysign(10, self.dy)

		if self.dtheta > 0.05 or self.dtheta < -0.05:
			self.dtheta = math.copysign(0.05, self.dtheta)

		self.x += self.dx * dt
		self.y += self.dy * dt
		self.theta += self.dtheta * dt

class HovercraftSprite(pygame.sprite.Sprite):
	def __init__(self, initX, initY, initTheta):
		pygame.sprite.Sprite.__init__(self)
		self.imageList = []
		for i in range(4):
			self.imageList.append(pygame.transform.rotate(pygame.transform.scale(pygame.image.load('images/Rocket'+str(i)+'.bmp'), (20,40)), -90))
		
		self.image = self.imageList[0]
		self.rect = self.image.get_rect()
		self.rect.center = (initX, initY)
		self.hovercraft = Hovercraft(initX, initY, initTheta)
		
		
		self.surface = pygame.Surface((20, 10))
		self.surface.set_colorkey(white, pygame.SRCALPHA)

	def step(self, linearAcceleration, rotationAcceleration, dt):
		self.hovercraft.step(linearAcceleration, rotationAcceleration, dt)

		imageIndex = 0
		if linearAcceleration > 0:			
			imageIndex = 1
		elif rotationAcceleration < 0:
			imageIndex = 2
		elif rotationAcceleration > 0:
			imageIndex = 3
		self.__getReadyToDraw(imageIndex)


	def __getReadyToDraw(self, imageIndex):
		self.rect.center = (self.hovercraft.x, self.hovercraft.y)
		oldCenter = self.rect.center
		self.image = pygame.transform.rotate(self.imageList[imageIndex], (-self.hovercraft.theta * 180. / math.pi))
		self.rect = self.image.get_rect()
		self.rect.center = oldCenter

	def build(self, vals):
		self.hovercraft.build(vals)
		self.__getReadyToDraw(0)

class Edge:
	def __init__(self, p0, p1, id, parent):
		self.p0 = p0
		self.p1 = p1
		self.id = id
		self.parent = parent

	def getLine(self):
		return (self.p0, self.p1)

class RRT:
	def __init__(self, start, goal, goalTolerance, samplingBounds, controlBounds):
		startEdge = Edge(start, start, 0, 0)
		self.dimensionality = len(start)
		self.kdtree = kdtree.KDTree(self.dimensionality)
		self.kdtree.insert(kdtree.KDPoint(start, startEdge))
		self.edgeArray = [startEdge]

		self.goal = goal
		self.goalTolerance = goalTolerance
		self.solution = []

		self.samplingBounds = samplingBounds
		self.controlBounds = controlBounds

		hovercraft = HovercraftSprite(0., 0., 0.)
		self.hovercraftSprite = pygame.sprite.Group(hovercraft)

	def __getRandomPoint(self):
		point = ()
		for i in range(self.dimensionality):
			point += (getRandomValue(self.samplingBounds[i][0], self.samplingBounds[i][1]),)
		return point

	def step(self, map, dt):
		if len(self.solution) > 0:
			return

		point = self.__getRandomPoint()
		nearestNode, dist = self.kdtree.nearest(kdtree.KDPoint(point))
		nearest = nearestNode.point
		edge = self.__steer(nearest, point, dt)
		if not map.isThereObstacleIntersection(edge.getLine()):
			edge.parent = nearestNode.data.id
			edge.id = len(self.edgeArray)
			self.edgeArray.append(edge)
			self.kdtree.insert(kdtree.KDPoint(edge.p1, edge))

			if self.__isGoal(edge.p1):
				cur = len(self.edgeArray) - 1
				while self.edgeArray[cur].parent != cur:
					self.solution.append(cur)
					cur = self.edgeArray[cur].parent
				self.solution = list(reversed(self.solution))
				print("found goal")

	def __isGoal(self, point):
		return math.fabs(point[0] - self.goal[0]) < self.goalTolerance and math.fabs(point[1] - self.goal[1]) < self.goalTolerance

	def __steer(self, point, target, dt):
		linearAcceleration = getRandomValue(self.controlBounds[0][0], self.controlBounds[0][1])
		angularAcceleration = getRandomValue(self.controlBounds[1][0], self.controlBounds[1][1])

		steps = int(dt / 0.01)
		hovercraft = Hovercraft()
		hovercraft.build(point)

		for i in range(steps):
			hovercraft.step(linearAcceleration, angularAcceleration, 0.01)

		return Edge(point, hovercraft.getPoint(), -1, -1)

	def draw(self, screen):

		if len(self.solution) > 0:
			self.hovercraftSprite.sprites()[0].build(self.edgeArray[self.solution[0]].p0)
			self.hovercraftSprite.draw(screen)
			self.solution = self.solution[1:len(self.solution)]
			time.sleep(0.5)

		for edge in self.edgeArray:
			pygame.draw.line(screen, red, edge.p0[0:2], edge.p1[0:2])
		pygame.draw.circle(screen, green, self.goal[0:2], self.goalTolerance)


def main():
	# random.seed(2)

	pygame.init()
	pygame.display.set_caption('RRT Hovercraft')
	screen = pygame.display.set_mode((maxX, maxY))

	samplingBounds = [(0,maxX), (0,maxY), (-math.pi, math.pi), (-20,20), (-20,20), (-math.pi, math.pi)]
	controlBounds = [(0,0.1), (-0.01, 0.01)]

	rrt = RRT((50,50,0,0,0,0), (600, 400), 50, samplingBounds, controlBounds)

	map = polymap.PolyMap(2, 6, 150, 0, maxX, 0, maxY, white)

	while (True):
		screen.fill(black)
		# linearAcceleration, angularAcceleration = getKeyboard(hovercraft)

		map.draw(screen)

		rrt.step(map, 10)
		rrt.draw(screen)

		pygame.display.flip()
		time.sleep(0.075)

def getKeyboard(hovercraft):
	for event in pygame.event.get():
		if event and event.type == pygame.QUIT:
			pygame.quit(); sys.exit();
		elif event and event.type == pygame.KEYDOWN:
			if event.key == pygame.K_RIGHT:
				return (0, math.pi * -0.01)
			elif event.key == pygame.K_LEFT:
				return (0, math.pi * 0.01)
			# elif event.key == pygame.K_DOWN:
			# 	return (-1, 0)
			elif event.key == pygame.K_UP:
				return (1, 0)
	return (0,0)

main()