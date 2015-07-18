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

class Hovercraft(pygame.sprite.Sprite):
	def __init__(self, initX, initY, initTheta):
		pygame.sprite.Sprite.__init__(self)
		self.imageList = []
		for i in range(4):
			self.imageList.append(pygame.transform.rotate(pygame.transform.scale(pygame.image.load('images/Rocket'+str(i)+'.bmp'), (20,40)), -90))
		
		self.image = self.imageList[0]
		self.rect = self.image.get_rect()
		self.rect.center = (initX, initY)

		self.x = initX
		self.y = initY
		self.theta = initTheta

		self.dx = 0.
		self.dy = 0.
		self.dtheta = 0.
		
		self.length = 20.
		self.surface = pygame.Surface((20, 10))
		self.surface.set_colorkey(white, pygame.SRCALPHA)

	def step(self, linearAcceleration, rotationAcceleration, dt):
		self.dx += linearAcceleration * math.cos(self.theta) * dt
		self.dy += linearAcceleration * math.sin(self.theta) * dt
		self.dtheta += rotationAcceleration * dt
		self.x += self.dx * dt
		self.y += self.dy * dt
		self.theta += self.dtheta * dt

		imageIndex = 0
		if linearAcceleration > 0:			
			imageIndex = 1
		elif rotationAcceleration < 0:
			imageIndex = 2
		elif rotationAcceleration > 0:
			imageIndex = 3

		self.rect.center = (self.x, self.y)
		oldCenter = self.rect.center
		self.image = pygame.transform.rotate(self.imageList[imageIndex], (-self.theta * 180. / math.pi))
		self.rect = self.image.get_rect()
		self.rect.center = oldCenter

class Edge:
	def __init__(self, p0, p1, parent):
		self.p0 = p0
		self.p1 = p1
		self.parent = parent

	def getLine(self):
		return (self.p0, self.p1)

class RRT:
	def __init__(self, start):
		startEdge = Edge(start, start, 0)
		self.kdtree = kdtree.KDTree(2)
		self.kdtree.insert(kdtree.KDPoint(start, startEdge))
		self.edgeArray = [startEdge]

	def step(self, map, dt):
		point = getRandomPoint(0, maxX, 0, maxY)
		nearest, dist = self.kdtree.nearest(kdtree.KDPoint(point))
		nearest = nearest.point
		edge = self.steer(nearest, point, dt)
		if not map.isThereObstacleIntersection(edge.getLine()):
			edge.parent = len(self.edgeArray)
			self.edgeArray.append(edge)
			self.kdtree.insert(kdtree.KDPoint(edge.p1, edge))

	def steer(self, point, target, dt):
		dx = target[0] - point[0]
		dy = target[1] - point[1]
		distance = math.sqrt(dx * dx + dy * dy)
		if distance < dt:
			return Edge(point, target, -1)
		dx *= dt / distance
		dy *= dt / distance
		endpoint = (point[0] + dx, point[1] + dy)
		return Edge(point, endpoint, -1)

	def draw(self, screen):
		for edge in self.edgeArray:
			pygame.draw.line(screen, red, edge.p0, edge.p1)


def main():
	# random.seed(2)

	pygame.init()
	pygame.display.set_caption('RRT Hovercraft')
	screen = pygame.display.set_mode((maxX, maxY))

	rrt = RRT((0,0))

	map = polymap.PolyMap(2, 6, 150, 0, maxX, 0, maxY, white)

	hovercraft = Hovercraft(100., 100., 0.)
	hovercraftSprite = pygame.sprite.Group(hovercraft)

	while (True):
		screen.fill(black)
		linearAcceleration, angularAcceleration = getKeyboard(hovercraft)
		hovercraft.step(linearAcceleration, angularAcceleration, 0.5)
		hovercraftSprite.draw(screen);

		map.draw(screen)

		rrt.step(map, 25)
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