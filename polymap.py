import math
import pyqtree
import pygame
from helpers import *

class PolyMap:
	def __init__(self, numPolys, numVertices, radius, minX, maxX, minY, maxY, color=black):
		self.minX = minX
		self.maxX = maxX
		self.minY = minY
		self.maxY = maxY
		self.color = color
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
		pygame.draw.polygon(screen, self.color, polyPoints)

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