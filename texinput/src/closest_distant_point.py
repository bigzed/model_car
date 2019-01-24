import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import imread

def closest_point(point, laneID, distance):
    """Returns closest point on trajectory."""
    # Top or bottom center of circle
    x, y = point[0], point[1]
    if laneID == 1: #INNERLANE
        laned = 16
    elif laneID == 2: #OUTERLANE
        laned = 48

    if x == 215 and y == 196:
        return np.array([215, 76 + laned])
    if x == 215 and y == 404:
        return np.array([215, 524 + laned])

    cp = [0,0]
    closepoint = [0,0]
    # Top half circle:
    for i in range(2):
        oldcp = closepoint
        if y <= 196:
            cp,distance,closepoint = closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121 + laned, distance)
            if distance > 0:
                x,y = 94 - laned, 196
                print("Top Circle - overshoot")
            else:
                print("Top Circle - fit")
        # Bottom half circle:
        elif y >= 404:
            cp,distance,closepoint = closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121 + laned, distance)
            if distance > 0:
                x,y = 336 + laned, 404
                print("Bottom Circle - overshoot")
            else:
                print("Bottom Circle - fit")
        # Left line
        elif x <= 215:
            closepoint = np.array([94 - laned, y])
            if y + distance > 404:
                distance -= (404 - y)
                cp = np.array([94 - laned, 404])
                print("Right Line - overshoot")
            else:
                distance = 0
                cp = np.array([94 - laned, y + distance])
                print("Left Line - fit")
        # Right line
        else:
            closepoint = np.array([336 + laned, y])
            if y - distance < 196:
                distance -= (y - 196)
                cp = np.array([336 + laned, 196])
                print("Right Line - overshoot")
            else:
                distance = 0
                cp = np.array([336 + laned, y + distance])
                print("Right Line - fit")
        x,y = cp[0],cp[1]
    return cp, oldcp

def closest_point_on_circle(p, c, r, d):
    v = p - c
    closepoint = c + ((v * r) / np.linalg.norm(v))
    dist = 0
    turnpoint = closepoint
    if d > 0:
        distpoint = closepoint - c
        if c[1] == 196: # top circle
            ca = np.array([1,0])
        elif c[1] == 404: # bottom circle
            ca = np.array([-1,0])
        pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
        U = (2 * np.pi * r)
        distangle = (360 * (d / U))
        tangle = -np.radians(pangle + distangle)
        print(U,d,(d/U), distangle, pangle, tangle)
        if tangle > 180:
            print("OVERFLOW")
            dangle = tangle - 180
            dist = (dangle * U / 360)
        cos, sin = np.cos(tangle), np.sin(tangle)
        print(tangle, cos, sin)
        turnpoint = [(distpoint[0]*cos) - (distpoint[1]*sin), (distpoint[0]*sin) + (distpoint[1]*cos)]
        plt.plot([distpoint[0], turnpoint[0]], [distpoint[1], turnpoint[1]], 'yx-', label=['foo'] )
        turnpoint += c
    return turnpoint, dist, closepoint

def main(args):
    img = imread('../pictures/map.png')
    plt.ion()
    plt.imshow(img)
    '''
    ax = plt.gcf().gca()
    ax.add_artist(plt.Circle((215, 196), 121 + 16, color='y'))
    ax.add_artist(plt.Circle((215, 404), 121 + 48, color='y'))
    #'''
    
    for point in [([100, 100], 2, 120), ([300, 200], 1, 100), ([20, 426], 1, 100), ([240,426], 2, 120)]:
        closest_p,cp = closest_point(point[0], point[1], point[2])
        print(f'Point ({point[0][0]}, {point[0][1]}) -> {closest_p}')
        print(f'IMG-COLOR: {img[point[0][0], point[0][1]]}')
        '''
        plt.plot(closest_p[0], closest_p[1], marker='x', color='g')
        '''
        plt.plot([point[0][0], cp[0]], [point[0][1], cp[1]], 'gx-')
        plt.plot([cp[0], closest_p[0]], [cp[1], closest_p[1]], 'rx-')
        #plt.plot([0, closest_p[0]], [1, closest_p[1]], 'bx-')
        #'''

    plt.show(block=True)

if __name__ == '__main__':
    main(sys.argv)
