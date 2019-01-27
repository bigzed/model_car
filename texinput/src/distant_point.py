import sys
import numpy as np
import matplotlib.pyplot as plt
from imageio import imread

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

    dp = [0,0]
    cp = [point]
    # Top half circle:
    distance += 1
    first = True
    while (distance > 0 or first):
        first = False
        if y <= 196:
            dp,distance,ncp = closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121 + laned, distance)
            if distance > 0:
                dp[0],dp[1] = 94 - laned, 197
                print("Top Circle - overshoot")
            else:
                print("Top Circle - fit")
        # Bottom half circle:
        elif y >= 404:
            dp,distance,ncp = closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121 + laned, distance)
            if distance > 0:
                dp[0],dp[1] = 336 + laned, 403
                print("Bottom Circle - overshoot")
            else:
                print("Bottom Circle - fit")
        # Left line
        elif x <= 215:
            ncp = np.array([94 - laned, y])
            if y + distance > 404:
                distance -= (404 - y)
                dp = np.array([94 - laned, 404])
                print("Left Line - overshoot")
            else:
                dp = np.array([94 - laned, y + distance])
                distance = 0
                print("Left Line - fit")
        # Right line
        else:
            ncp = np.array([336 + laned, y])
            if y - distance < 196:
                distance -= (y - 196)
                dp = np.array([336 + laned, 196])
                print("Right Line - overshoot")
            else:
                dp = np.array([336 + laned, y - distance])
                distance = 0
                print("Right Line - fit")
        x,y = dp[0],dp[1]
        cp.append(ncp)
    cp.append(dp)
    return cp

def closest_point_on_circle(p, c, r, d):
    v = p - c
    closepoint = c + ((v * r) / np.linalg.norm(v))

    if d > 0:
        top = False
        if c[1] == 196:
            top = True
        dist = 0
        turnpoint = closepoint
        U = (2 * np.pi * r)
        if d > (U/2):
            temp = d % (U/2)
            dist = d - temp
            d = temp
        distangle = (360 * (d / U))
        print(dist)
        if top:
            ca = np.array([1,0])
            pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
            tangle = 360-(pangle + distangle) # reverse target angle
            if tangle < 180:
                print("TOP OVERSHOOT")
                dist += (180 - tangle) * (U/360)
        else: #bot
            ca = np.array([-1,0])
            pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
            tangle = 180-(pangle + distangle) # reverse target angle
            print(U,d,(d/U), distangle, pangle, tangle)
            if tangle < 0:
                print("BOT OVERSHOOT")
                dist += abs(tangle) * (U/360)
                print(tangle, dist)
        cos = r * np.cos(np.deg2rad(tangle))
        sin = r * np.sin(np.deg2rad(tangle))
        turnpoint = np.array([cos, sin]) + c
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
    #'''
    for point in [([100, 100], 2, 20), ([300, 200], 1, 50), ([15,354],1,65), ([350,570],2,30)]:
        cp = closest_point(point[0], point[1], point[2])
        print(f'Point {point}')
        print("---------------------------------------------------------")
        lc = ["r","b","g","y","c","m","k","w"]
        for i in range(len(cp)-1):
            plt.plot([cp[i][0], cp[i+1][0]], [cp[i][1], cp[i+1][1]], '.-', color=lc[i%len(lc)])

    sys.stdout.flush()
    plt.show(block=True)

if __name__ == '__main__':
    main(sys.argv)
