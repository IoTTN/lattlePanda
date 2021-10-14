# V3.6.9
# ls /dev/tty*
# sudo chmod a+rw /dev/ttyUSB0
# Khong dat ten file trung ten serial.py
# import the necessary packages
from imutils.video import VideoStream
import cv2
from cv2 import aruco
import argparse
import imutils
import time
import cv2
import sys
import math
import serial

import time

version = "V3.6.9"

ArUco_marker_angles = {}
oX = 0
oY = 0
oA = 0
chY = 0
agvY = 0
agvX = 0
message = None
dAGVY = 0
dAGVX = 0
dAngle = 0
bcsID = 0
agvID = 0
agvA = 0
a = 0  # vi tri duong ke vang tieu chuan
charger = 0
i = 0
j = 0
limit = 0
comPort = None
useSerial =None



try:
    from configparser import ConfigParser
except ImportError:
    from ConfigParser import ConfigParser  # ver. < 3.0

# instantiate
config = ConfigParser()
# parse existing file
config.read('D:\Cuong\config.ini')  # dien day du duong dan file
# read values from a section
bcs_id = config.get('parameter', 'bcs_id')
alarmAngle = int(config.get('parameter', 'alarmAngle'))
useSerial = config.get('parameter', 'useSerial')
sendTimer = int(config.get('parameter', 'sendTimer'))
xReal = int(config.get('parameter', 'xReal'))
xMeansure = int(config.get('parameter', 'xMeansure'))
yReal = int(config.get('parameter', 'yReal'))
yMeansure = int(config.get('parameter', 'yMeansure'))
yellowLine = int(config.get('parameter', 'yellowLine'))
fBuzzer = int(config.get('parameter', 'fBuzzer'))
limit = int(config.get('parameter', 'limit'))
comPort = config.get('parameter', 'comPort')


xRate = float(xReal / xMeansure)
yRate = float(yReal / yMeansure)

print("[INFO] Load parameter ...")

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters_create()
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
if useSerial == 'True':
    ser = serial.Serial(comPort, 9600)

# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 1000 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=800)
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
                                                       arucoDict, parameters=arucoParams)
    aruco.drawDetectedMarkers(frame, corners, ids, (255, 0, 255))

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()

        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            doi = abs(topLeft[1] - topRight[1])
            ke = abs(topLeft[0] - topRight[0])
            if ke != 0:
                tan = float(doi / ke)
                angle = 90 - int(57.3 * math.atan(tan))
            else:
                angle = 0
            ArUco_marker_angles[markerID] = angle

            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            if markerID == 0:
                oX = int(cX)
                oY = int(cY)
                oA = int(angle)
                a = oY - yellowLine  # Vach vang
                bcsID = int(bcs_id)
            elif markerID == 1:
                chY = int(cY)
                charger = int(chY)  # toa do vi tri charging pin
                cA = int(angle)
            else:
                agvX = int(cX)
                agvY = int(cY)
                agvA = int(angle)
                agvID = int(markerID)
        dAGVY = int(yRate * (oY - agvY))
        dAGVX = int(xRate * (agvX - oX))
        dAngle = int(oA - agvA)
        x1 = oX - limit
        x2 = oX + limit
        y1 = oY - 2*limit
        y2 = oY - 4*limit
        j = j + 1
        if j == 6000:
            messageStatus = "BC{0:0=4d}RUN0000000000000".format(bcsID)
            print(messageStatus)
            if useSerial == 'True':
                ser.write(messageStatus.encode('utf-8'))
        if (len(ids) > 2) and (charger <= a) and (agvX > x1) and (agvX < x2) and (agvY < y1) and (agvY > y2):
            message = "BC{0:0=4d}{1:0=3d}{2:0=3d}{3:0=3d}{4:0=3d}".format(bcsID, agvID, dAGVX, dAGVY, dAngle)

            print(message)
            if useSerial == 'True':
                ser.write(message.encode('utf-8'))
            i = i + 1

            print("i = ", i)

            if i == 9:
                message = "BC{0:0=4d}{1:0=3d}{2:0=3d}{3:0=3d}{4:0=3d}".format(bcsID, agvID, dAGVX, dAGVY, dAngle)

                print(message)
                if useSerial == True:
                    ser.write(message.encode('utf-8'))
                cv2.circle(frame, (20, 50), 9, (0, 255, 0), -1)  # den tin hieu hien thi gui lenh ve moxa
                if dAngle > alarmAngle:
                    timestr = time.strftime("%H%M%S")
                    cv2.imwrite('D:\Cuong\Pictures/' + str(message) + '-' + timestr + '.jpg', frame)
            if i == sendTimer:
                i = 0

        else:
            cv2.circle(frame, (20, 50), 9, (0, 0, 255), -1)  # den tin hieu hien thi khong gui lenh ve moxa
        cv2.line(frame, (1, oY), (1279, oY), (0, 0, 255), 2)
        cv2.line(frame, (427, a), (547, a), (0, 255, 255), 2)  # duong ke vang
        cv2.line(frame, (oX, 1), (oX, 1019), (0, 0, 255), 2)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        cv2.line(frame, (oX, oY), (agvX, agvY), (0, 255, 0), 2)
        cv2.putText(frame, "{0}".format(str(ArUco_marker_angles[markerID])),
                    (topLeft[0] - 30, topLeft[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2)
        cv2.putText(frame,
                    version + "bcsID:{0:0=4d};agvID:{1:0=3d};X:{2:0=3d};Y:{3:0=3d};R:{4:0=3d}".format(bcsID, agvID,
                                                                                                      dAGVX, dAGVY,
                                                                                                      dAngle),
                    (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 255), 2)

    # show the output frame
    cv2.imshow("BCS Check", frame)
    cv2.moveWindow("BCS Check", 0, 0)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()