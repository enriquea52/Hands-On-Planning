#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

'''
Bresenham's line drawing algorithm for a general 2D case taking
into consideration 4 quadrants in a discretized cartesian plane.
'''

def bresenham_line(start, end, width = 1):

    x1,y1 = start[1],start[0]
    x2,y2 = end[1],end[0]

    if abs(y2-y1) < abs(x2-x1):
        if x1 > x2:
            return np.asarray(pointsLow(end,start,width))[::-1]
        else:
            return np.asarray(pointsLow(start,end,width))
    else:
        if y1 > y2:
            return np.asarray(pointsHigh(end,start,width))[::-1]
        else:
            return np.asarray(pointsHigh(start,end,width))


def pointsHigh(start,end,width):
    points = []
    x1,y1 = start[1],start[0]
    x2,y2 = end[1],end[0]

    dx = x2 - x1
    dy = y2 - y1

    xi = width

    if dx < 0:
        xi = -width
        dx = -dx
    
    D = (2*dx) - dy
    x = x1

    for y in range(y1,y2+1):
        points.append((y,x))
        if D > 0:
            x += xi
            D = D + (2*(dx-dy))
        else:
            D = D + 2*dx

    return points

def pointsLow(start,end,width ):
    points = []
    x1,y1 = start[1],start[0]
    x2,y2 = end[1],end[0]

    dx = x2 - x1
    dy = y2 - y1

    yi = width

    if dy < 0:
        yi = -width
        dy = -dy
    
    D = (2*dy) - dx
    y = y1

    for x in range(x1,x2+1):
        points.append((y,x))
        if D > 0:
            y += yi 
            D = D + (2*(dy-dx))
        else:
            D = D + 2*dy

    return points

if __name__ == "__main__":

    img = np.zeros((10,10), dtype = np.int8)
    start = (0,0) #(9,9)
    end = (9,9)   #(4,2)
    points = bresenham_line(start, end)
    print(points)
    for point in points:
        img[point[0],point[1]] = 1
    plt.matshow(img)
    plt.show()
