import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import imread

def closest_point(x, y):
    """Returns closest point on trajectory."""
    # Top or bottom center of circle
    if x == 215 and y == 196:
        return np.array([215, 76])
    if x == 215 and y == 404:
        return np.array([215, 524])

    # Top half circle:
    if y <= 196:
        return closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121)
    # Bottom half circle:
    elif y >= 404:
        return closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121)
    # Left line
    elif x <= 215:
        return np.array([94, y])
    # Right line
    else:
        return np.array([336, y])

def closest_point_on_circle(p, c, r):
    v = p - c
    return c + v / np.linalg.norm(v) * r

def main(args):
    img = imread('../pictures/map.png')
    plt.ion()
    plt.imshow(img)
    '''
    ax = plt.gcf().gca()
    ax.add_artist(plt.Circle((215, 196), 121, color='y'))
    ax.add_artist(plt.Circle((215, 404), 121, color='y'))
    #'''
    for point in [[0, 0], [400, 200], [300, 100], [240, 420]]:
        closest_p = closest_point(point[0], point[1])
        print(f'Point ({point[0]}, {point[1]}) -> {closest_p}')
        print(f'IMG-COLOR: {img[point[0], point[1]]}')
        '''
        plt.plot(closest_p[0], closest_p[1], marker='x', color='g')
        '''
        plt.plot([point[0], closest_p[0]], [point[1], closest_p[1]], 'gx-')
        #'''

    plt.show(block=True)

if __name__ == '__main__':
    main(sys.argv)
