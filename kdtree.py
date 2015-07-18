import math

class KDPoint:
    def __init__(self, point, data=None):
        self.point = point
        self.data = data
        self.split = 0
        self.left = None
        self.right = None

def defaultDistance(kdp1, kdp2):
    sum = 0.
    for i in range(len(kdp1.point)):
        diff = kdp1.point[i] - kdp2.point[i]
        sum += diff*diff
    return math.sqrt(sum)

class KDTree:
    def __init__(self, dimensionality, pointList=[], distanceFunction=defaultDistance):
        self.dimensionality = dimensionality
        self.distanceFunction=distanceFunction
        self.root = None

    def insert(self, kdpoint):
        assert(len(kdpoint.point) == self.dimensionality)
        self.root = self.__insert(self.root, 0, kdpoint)
        # print("")

    def __insert(self, root, depth, kdpoint):
        if root == None:
            # print("ROOT")
            kdpoint.split = depth % self.dimensionality
            return kdpoint
        if kdpoint.point[root.split] < root.point[root.split]:
            # print("LEFT")
            root.left = self.__insert(root.left, depth+1, kdpoint)
        else:
            # print("RIGHT")
            root.right = self.__insert(root.right, depth+1, kdpoint)
        return root

    def nearest(self, kdpoint):
        return self.__nearest(self.root, kdpoint)

    def __nearest(self, root, kdpoint):
        bestNode = root
        bestDistance = self.distanceFunction(kdpoint, root)

        def lookLeft(bestNode, bestDistance):
            # print("look left")
            candidateNode, candidateDistance = self.__nearest(root.left, kdpoint)
            if bestDistance > candidateDistance:
                return candidateNode, candidateDistance
            else:
                return bestNode, bestDistance
        
        def lookRight(bestNode, bestDistance):
            # print("look right")
            candidateNode, candidateDistance = self.__nearest(root.right, kdpoint)
            if bestDistance > candidateDistance:
                return candidateNode, candidateDistance
            else:
                return bestNode, bestDistance

        if root.left != None and (root.right == None or kdpoint.point[root.split] < root.point[root.split]):
            bestNode, bestDistance = lookLeft(bestNode, bestDistance)
            if root.right != None:
                if root.point[root.split] < kdpoint.point[root.split] + bestDistance:
                    bestNode, bestDistance = lookRight(bestNode, bestDistance)
        elif root.right != None and (root.left == None or kdpoint.point[root.split] > root.point[root.split]):
            bestNode, bestDistance = lookRight(bestNode, bestDistance)
            if root.left != None:
                if root.point[root.split] < kdpoint.point[root.split] + bestDistance:
                    bestNode, bestDistance = lookLeft(bestNode, bestDistance)

        return bestNode, bestDistance