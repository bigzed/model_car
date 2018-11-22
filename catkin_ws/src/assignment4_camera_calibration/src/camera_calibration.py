#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraCalibration:
    def __init__(self):
        # Image publisher
        self.image_gray_pub = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
        # Image source
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 1)
        # OpenCV
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert to grayscale
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        try:
            self.image_gray_pub.publish(self.bridge.cv2_to_imgmsg(gray_img, "mono8"))
        except CvBridgeError as e:
            print(e)

        # Convert to B/W image
        bi_gray_max = 255
        bi_gray_min = 250
        ret, black_img = cv2.threshold(gray_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        try:
            self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(black_img, "mono8"))
        except CvBridgeError as e:
            print(e)

        # Scan for white pixels and remember coordinates
        coordinates = []
        for y in range(480):
            for x in range(640):
                if black_img[y, x] == 255:
                    coordinates.append([x, y])
        print("%s Coordinates: \n %s" % (str(len(coordinates)), ', '.join(str(x) for x in coordinates)))

        # Compute the extrinsic parameters with SolvePNP
        fx = 614.1699
        fy = 614.9002
        cx = 329.9491
        cy = 237.2788
        k1 = 0.1115
        k2 = -0.1089
        p1 = 0
        p2 = 0

        camera_mat = np.zeros((3, 3, 1))
        camera_mat[:, :, 0] = np.array([[fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]])
        dist_coeffs = np.zeros((4, 1))
        dist_coeffs[:, 0] = np.array([[k1, k2, p1, p2]])

        # Object points
        obj_points = np.zeros((6, 3, 1))
        obj_points[:, :, 0] = np.array([
            [00.0, 00.0, 00.0], [40.2, 00.0, 00.0],
            [00.0, 27.7, 00.0], [39.7, 28.0, 00.0],
            [00.0, 55.8, 00.0], [39.8, 55.5, 00.0]])
        # Cluser image points
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, label ,center = cv2.kmeans(np.float32(np.array(coordinates)), 6, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        retval, rvec, tvec = cv2.solvePnP(obj_points, center, camera_mat, dist_coeffs)
        print("RETVAL: \n %s \n RVEC: \n %s \n TVEC: \n %s" % (retval, rvec, tvec))

        # Calculate rotation matrix
        rmat = np.zeros((3,3))
        cv2.Rodrigues(rvec, rmat, jacobian=0)
        print("Rotation Matrix: \n %s \n" % rmat)
        print("Inverse Rotation Matrix: \n %s \n" % np.linalg.inv(rmat))
        print("Translation Vec: \n %s \n" % tvec)
        print("Point: \n %s \n" % obj_points[-1])

        print("inverse of Homogenous transform: \n %s \n" % np.multiply(np.linalg.inv(rmat), tvec))

        # Calculate angles
        print("Euler Angles: \n %s" % rmat)

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(R) :

        assert(isRotationMatrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

def main(args):
    rospy.init_node('camera_calibration', anonymous = True)
    cc = CameraCalibration()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")

if __name__ == '__main__':
    main(sys.argv)
