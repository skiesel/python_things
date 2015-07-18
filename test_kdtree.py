import random
import math
import kdtree

def getRandomPoint(dimensionality):
    point = ()
    for j in range(dimensionality):
        point += (random.random(),)
    return point

def getRandomPoints(howmany, dimensionality):
    pointList = []
    for i in range(howmany):
        pointList.append(getRandomPoint(dimensionality))
    return pointList

def linearScan(pointList, queryPoint):
    bestNode = None
    bestDistance = float("inf")
    for point in pointList:
        distance = kdtree.defaultDistance(kdtree.KDPoint(point), kdtree.KDPoint(queryPoint))
        if distance < bestDistance:
            bestDistance = distance
            bestNode = kdtree.KDPoint(point)
    return bestNode, bestDistance

def test():
    dimensionality = 3
    tree = kdtree.KDTree(dimensionality)
    pointList = getRandomPoints(1000, dimensionality)

    for point in pointList:
        tree.insert(kdtree.KDPoint(point))

    queryList = getRandomPoints(1000, dimensionality)
    for point in queryList:
        kdNode, kdDist = tree.nearest(kdtree.KDPoint(point))
        lNode, lDist = linearScan(pointList, point)
        if math.fabs(kdDist - lDist) > 0.0001:
            print(pointList)
            print("mistake")
            print(point)
            print("------------")
            print(lDist)
            print(lNode.point)
            print("------------")
            print(kdDist)
            print(kdNode.point)
            exit()

test()