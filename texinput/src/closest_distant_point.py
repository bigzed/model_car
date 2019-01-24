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
    circle_remap = 1
    while(circle_remap >= 0):
        # Top half circle:
        if y <= 196:
            cp = closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121 + laned, distance * circle_remap)
            print("Top Half Circle")
        # Bottom half circle:
        elif y >= 404:
            cp = closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121 + laned, distance * circle_remap)
            print("Bottom Half Circle")
        # Left line
        elif x <= 215:
            if y + distance >= 404:
                cp = closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121 + laned, distance * circle_remap)
                print("Bottom Half Circle")
            else:
                circle_remap = -1
                cp = np.array([94 - laned, y + distance* circle_remap])
                print("Left Line")
        # Right line
        else:
            if y + distance <= 196:
                cp = closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121 + laned, distance * circle_remap)
                print("Top Half Circle")
            else:
                circle_remap = -1
                cp = np.array([336 + laned, y - distance* circle_remap])
                print("Right Line")
        print("%d: cp[%f, %f]" % (circle_remap, cp[0], cp[1]))
        circle_remap -= 1
    return cp

def closest_point_on_circle(p, c, r, d):
    v = p - c
    distpoint = c + ((v * r) / np.linalg.norm(v))
    if d > 0:
        distanceangle = 360 * (d / 2 * np.pi * r)
        c, s = np.cos(distanceangle), np.sin(distanceangle)
        distpoint = [distpoint[0]*c + distpoint[1]*(-s), distpoint[0]*s + distpoint[1]*(c)]
    return distpoint
def main(args):
    img = imread('../pictures/map.png')
    plt.ion()
    plt.imshow(img)
    '''
    ax = plt.gcf().gca()
    ax.add_artist(plt.Circle((215, 196), 121, color='y'))
    ax.add_artist(plt.Circle((215, 404), 121, color='y'))
    #'''
    for point in [([100, 100], 2, 20), ([300, 200], 1, 50)]:
        closest_p = closest_point(point[0], point[1], point[2])
        print(f'Point ({point[0][0]}, {point[0][1]}) -> {closest_p}')
        print(f'IMG-COLOR: {img[point[0][0], point[0][1]]}')
        '''
        plt.plot(closest_p[0], closest_p[1], marker='x', color='g')
        '''
        plt.plot([point[0][0], closest_p[0]], [point[0][1], closest_p[1]], 'gx-')
        #'''

    plt.show(block=True)

if __name__ == '__main__':
    main(sys.argv)
