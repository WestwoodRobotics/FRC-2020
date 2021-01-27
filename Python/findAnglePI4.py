import cv2
import numpy as np

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink, CvSource, VideoMode
from networktables import NetworkTables, NetworkTablesInstance
import ntcore

configFile = "/boot/frc.json"

sd = NetworkTables.getTable("SmartDashboard")

team = None
server = False

cameras = []
cameraConfigs = []

sinks = []
srcs = []

goalContour = None

class CameraConfig(): pass

j = None

def readJSONConfig():
	global team
	global server

	try:
		with open(configFile, "rt", encoding="utf-8") as f:
			j = json.load(f)
	except OSError as err:
		print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
		return False

	if not isinstance(j, dict):
		parseError("must be JSON object")
		return False

	try:
		team = j["team"]
	except KeyError:
		parseError("could not read team number")
		return False

	try:
		cameras = j["cameras"]
	except KeyError:
		parseError("could not read cameras")
		return False

	for camera in cameras:
		if not readConfig(camera):
			return False

	return True

def readConfig(config):
	cam = CameraConfig()

	# name
	try:
		cam.name = config["name"]
	except KeyError:
		parseError("could not read camera name")
		return False

	# path
	try:
		cam.path = config["path"]
	except KeyError:
		parseError("camera '{}': could not read path".format(cam.name))
		return False

	# stream properties
	cam.streamConfig = config.get("stream")
	cam.config = config

	cam.h = config["h"]
	cam.s = config["s"]
	cam.v = config["v"]

	cam.fov = config["fov"]

	cam.height = config["height"]
	cam.width = config["width"]

	cam.degPerPix = cam.fov/cam.width

	cameraConfigs.append(cam)

	return True

def getGoalContours():
	img1 = cv2.imread('goal_contour.png', 0)
	
	ret, thresh = cv2.threshold(img1, 127, 255, 0)
	
	#__, contours, hierarchy = cv2.findContours(thresh, 2, 1)
	__, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	#print(contours.len())
	cnt1 = None
	if(len(contours) != 0):
		cnt1 = contours[0]

	x, y, w, h = cv2.boundingRect(cnt1)

	print(float(cv2.contourArea(cnt1))/(w*h))

	return cv2.convexHull(cnt1)

def startCamera(config):
	print("Starting camera '{}' on {}".format(config.name, config.path))
	inst = CameraServer.getInstance()
	camera = UsbCamera(config.name, config.path)
	server = inst.startAutomaticCapture(camera=camera, return_server=True)

	camera.setConfigJson(json.dumps(config.config))
	camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

	if config.streamConfig is not None:
		server.setConfigJson(json.dumps(config.streamConfig))

	cvSink = inst.getVideo(name=config.name)
	cvSrc = inst.putVideo(config.name + " Vision", config.height, config.width)

	return camera, cvSink, cvSrc

def processVision(cvSink, cvSrc, num, goalContour):
	fov = cameraConfigs[num].fov

	frame = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
	time, frame = cvSink.grabFrame(frame)
	
	if time == 0:
		cvSrc.notifyError(cvSink.getError());
	
	blur = cv2.blur(frame, (3,3))

	#converting to HSV
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

	# Normal masking algorithm
	lower = np.array([cameraConfigs[num].h[0], cameraConfigs[num].s[0], cameraConfigs[num].v[0]])
	upper = np.array([cameraConfigs[num].h[1], cameraConfigs[num].s[1], cameraConfigs[num].v[1]])

	structure = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

	mask = cv2.inRange(hsv, lower, upper)

	openImg = cv2.morphologyEx(mask, cv2.MORPH_OPEN, structure, iterations=2)
	closeImg = cv2.morphologyEx(openImg, cv2.MORPH_CLOSE, structure, iterations=2)

	#result = cv2.bitwise_and(frame, frame, mask=openImg)

	__, contours, hierarchy = cv2.findContours(closeImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	rectCenter = 0.0
	#distFromFrameCenter = 0.0

	if len(contours) != 0.0:
		cv2.drawContours(frame, contours, -1, 255, 3)

		c = max(contours, key=cv2.contourArea)

		rotateRect = cv2.minAreaRect(c)
		box = cv2.boxPoints(rotateRect)
		box = np.int0(box)

		x, y, w, h = cv2.boundingRect(c)

		ratio = float(cv2.contourArea(c))/(w*h)

		if ratio < .2 and ratio > .09:
			rectCenter = x + w/(2.0)

			cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

			cv2.drawContours(frame,[box],0,(0,0,255),2)
	
		sd.putNumber("ratio", float(cv2.contourArea(c))/(w*h))

	frameH, frameW = frame.shape[:2]
	distFromCenter = rectCenter - frameW/2.0

	degrees = distFromCenter*cameraConfigs[num].degPerPix

	#print("degrees per pixel", cameraConfigs[num].degPerPix)
	
	sd.putNumber("degrees", degrees)

	#sd.putNumber("rect center", rectCenter)
	#print("degrees", degrees)

	#print("processing")
	cvSrc.putFrame(frame)

def init():
	if len(sys.argv) >= 2:
		configFile = sys.argv[1]

	# read configuration
	if not readJSONConfig():
		sys.exit(1)

	# start NetworkTables
	ntinst = NetworkTablesInstance.getDefault()
	if server:
		print("Setting up NetworkTables server")
		ntinst.startServer()
	else:
		print("Setting up NetworkTables client for team {}".format(team))
		ntinst.startClientTeam(team)

	# start cameras
	for config in cameraConfigs:
		serv, sink, src = startCamera(config)
		cameras.append(serv)
		sinks.append(sink)
		srcs.append(src)

	goalContour = getGoalContours()

	#if goalContour == None:
		#print("uh oh")

	while True:
		#time.sleep(10)
		processVision(sinks[0], srcs[0], 0, goalContour)
		#processVision(sinks[1], srcs[1], 1, goalContour)


if __name__ == "__main__":
	init()