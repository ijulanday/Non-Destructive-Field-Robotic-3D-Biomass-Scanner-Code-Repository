import os
import sys

outputFile = open("epic.txt", "w")

class point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
    def __str__(self):
        return (str(self.x) + ' ' + str(self.y) + ' ' + str(self.z))
    def setVal(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

# formula for number of points defining hollow cube: 
#   p = n^3 - (n-2)^3
#   p -> total number of points
#   n -> number of points on an edge
#   i.e. a 2x2 cube would have 2^3 - 0^3 = 8 total points
#   below defines a 10x10 cube 

points = [point()] * 488
ppos = 0
xmin, xmax = 0, 10
ymin, ymax = -5, 5
zmin, zmax = 0, 10
scalemax = 10

for z in range(zmin,zmax):
    for y in range(ymin,ymax):
        for x in range(xmin,xmax):
            if y == ymin or y == ymax-1 or z == zmin or z == zmax-1 or x == xmin or x == xmax-1:
                points[ppos].setVal(x/scalemax, y/scalemax, z/scalemax)
                outputFile.write(str(points[ppos]) + '\n')
                ppos += 1

outputFile.close()
print('Finished creating point cloud')

