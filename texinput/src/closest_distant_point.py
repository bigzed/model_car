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
    # Top half circle:
    for i in range(2):
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
                print("Top Circle - overshoot")
            else:
                print("Top Circle - fit")
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
            closepoint = np.array([94 - laned, y])
            if y - distance < 196:
                distance -= (y - 196)
                cp = np.array([336 + laned, 196])
                print("Right Line - overshoot")
            else:
                distance = 0
                cp = np.array([336 + laned, y + distance])
                print("Right Line - fit")
        x,y = cp[0],cp[1]
    return cp, closepoint

def closest_point_on_circle(p, c, r, d):
    # get closest point
    v = p - c
    closepoint = c + ((v * r) / np.linalg.norm(v))
    dist = 0
    distpoint = closepoint
    if d > 0:
        # shift circle to (0,0)
        distpoint = closepoint - c
        ca = np.array([1,0]) # top right # top left
        #ca = np.array([-1,0]) # bottom left # bottom right
        # calculate angle of v
        pangle = np.arccos((ca[0] * v[0] + ca[1] * v[1]) / (np.linalg.norm(v)  * np.linalg.norm(ca)))
        # calculate angle of distance
        U = (2 * np.pi * r)
        distangle = (360 * (d / U))
        # calculate angle of distance with respect to circle
        #tangle = 360 - (pangle + distangle) # bottom right # bottom left
        #tangle = 360 - (pangle - distangle) # top left
        #tangle = (pangle + distangle)
        tangle = (pangle - distangle) # bottom left # top right
        if tangle > 180:
            dangle = tangle - 180
            dist = (dangle * U / 360)
        # shift distpoint by distance angle
        cos, sin = np.cos(tangle), np.sin(tangle)
        distpoint = [distpoint[0]*cos + distpoint[1]*(-sin), distpoint[0]*sin + distpoint[1]*(cos)]
        # move distpoint back to original circle center
        distpoint += c
    return distpoint, dist, closepoint

def main(args):
    img = imread('../pictures/map.png')
    plt.ion()
    plt.imshow(img)
    '''
    ax = plt.gcf().gca()
    ax.add_artist(plt.Circle((215, 196), 121, color='y'))
    ax.add_artist(plt.Circle((215, 404), 121, color='y'))
    #'''
    for point in [([100, 100], 2, 20), ([300, 200], 1, 50), ([20, 426], 1, 20), ([240,426], 2, 20)]:
        closest_p,cp = closest_point(point[0], point[1], point[2])
        print(f'Point ({point[0][0]}, {point[0][1]}) -> {closest_p}')
        print(f'IMG-COLOR: {img[point[0][0], point[0][1]]}')
        '''
        plt.plot(closest_p[0], closest_p[1], marker='x', color='g')
        '''
        plt.plot([point[0][0], cp[0]], [point[0][1], cp[1]], 'gx-')
        plt.plot([closest_p[0], cp[0]], [closest_p[1], cp[1]], 'gx-')
        #'''

    plt.show(block=True)

if __name__ == '__main__':
    main(sys.argv)
