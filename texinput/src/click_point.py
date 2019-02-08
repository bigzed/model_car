import sys
import numpy as np
from matplotlib.widgets import Slider
from matplotlib.widgets import RadioButtons
import matplotlib.pyplot as plt
from imageio import imread

class Closepoint:
    def __init__(self, debug):
        self.debug = debug
        self.dist = 50
        self.lane = 1
        img = imread('../pictures/map.png')
        plt.imshow(img)
        plt.ion()
        self.fig = plt.gcf()
        self.ax =  plt.gca()
        
        self.dax = self.fig.add_axes([.82,.1,.1,.15])
        self.rax = self.fig.add_axes([.78,.25,.2,.2])
        distance = Slider(self.dax, 'distance', 0, 400, valinit = self.dist)
        distance.on_changed(self.adaptdist)
        self.rbtn = RadioButtons(self.rax, ("inner lane","outer lane"))
        self.rbtn.on_clicked(self.adaptlane)
        self.lastlines = 0
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        plt.show(block=True)
        
    def printdebug(self, *args):
        if self.debug == True:
            if len(args) > 0:
                print(args)
    
    def adaptdist(self, val):
        self.dist = int(val)
        
    def adaptlane(self, val):
        if val == "inner lane":
            self.lane = 1
        if val == "outer lane":
            self.lane = 2
    
    def onclick(self, event):
        if event.inaxes == self.ax:
            cp = self.closest_point([event.xdata, event.ydata])
            lc = ["r","b","g","y","c","m","k","w"]
            for i in range(len(cp)-1):
                self.ax.plot([cp[i][0], cp[i+1][0]], [cp[i][1], cp[i+1][1]], '.-', color=lc[i%len(lc)])
            for i in range(self.lastlines):
                self.ax.lines.pop(0)
            self.lastlines = len(cp) -1
            sys.stdout.flush()

    def closest_point(self, point):
        laneID = self.lane
        distance = self.dist
        """Returns closest point on trajectory."""
        # Top or bottom center of circle
        x, y = point[0], point[1]
        sys.stdout.flush()
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
                dp,distance,ncp = self.closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121 + laned, distance)
                if distance > 0:
                    dp[0],dp[1] = 94 - laned, 197
                    self.printdebug("Top Circle - overshoot")
                else:
                    self.printdebug("Top Circle - fit")
            # Bottom half circle:
            elif y >= 404:
                dp,distance,ncp = self.closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121 + laned, distance)
                if distance > 0:
                    dp[0],dp[1] = 336 + laned, 403
                    self.printdebug("Bottom Circle - overshoot")
                else:
                    self.printdebug("Bottom Circle - fit")
            # Left line
            elif x <= 215:
                ncp = np.array([94 - laned, y])
                if y + distance > 404:
                    distance -= (404 - y)
                    dp = np.array([94 - laned, 404])
                    self.printdebug("Left Line - overshoot")
                else:
                    dp = np.array([94 - laned, y + distance])
                    distance = 0
                    self.printdebug("Left Line - fit")
            # Right line
            else:
                ncp = np.array([336 + laned, y])
                if y - distance < 196:
                    distance -= (y - 196)
                    dp = np.array([336 + laned, 196])
                    self.printdebug("Right Line - overshoot")
                else:
                    dp = np.array([336 + laned, y - distance])
                    distance = 0
                    self.printdebug("Right Line - fit")
            x,y = dp[0],dp[1]
            cp.append(ncp)
        cp.append(dp)
        return cp

    def closest_point_on_circle(self, p, c, r, d):
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
            if top:
                ca = np.array([1,0])
                pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
                tangle = 360-(pangle + distangle) # reverse target angle
                if tangle < 180:
                    self.printdebug("TOP OVERSHOOT")
                    dist += (180 - tangle) * (U/360)
            else: #bot
                ca = np.array([-1,0])
                pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
                tangle = 180-(pangle + distangle) # reverse target angle
                if tangle < 0:
                    self.printdebug("BOT OVERSHOOT")
                    dist += abs(tangle) * (U/360)
            cos = r * np.cos(np.deg2rad(tangle))
            sin = r * np.sin(np.deg2rad(tangle))
            turnpoint = np.array([cos, sin]) + c
        return turnpoint, dist, closepoint

def main(args):
    cp = Closepoint(False)

if __name__ == '__main__':
    main(sys.argv)
