import cv2
import numpy as np

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

<<<<<<< Updated upstream
sd = NetworkTables.getTable('SmartDashboard')

#Reports a parse error to pi4 console
def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

#Reads camera's configuration and adds camera to cameraConfigs[]
def readCameraConfig(config, switched):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        if(switched):
            parseError("could not read switched camera name")
        else:
            parseError("could not read camera name")
        
        return False

    # path
    try:
        if(switched):
            cam.key = config["key"]
        else:
            cam.path = config["path"]
    except KeyError:
        if(switched):
            parseError("switched camera '{}': could not read key".format(cam.name))
        else:
            parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    if(not switched):
        cam.streamConfig = config.get("stream")
        cam.config = config

    if(not switched):
        cameraConfigs.append(cam)
    else:
        switchedCameraConfigs.append(cam)

    return True

#Read camera config
def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera, False):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readCameraConfig(camera, True):
                return False

    return True

#Start running camera
def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)
=======
sinks = []
srcs = []

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
>>>>>>> Stashed changes

	if not isinstance(j, dict):
		parseError("must be JSON object")
		return False

	try:
		team = j["team"]
	except KeyError:
		parseError("could not read team number")
		return False

<<<<<<< Updated upstream
    cvSink = inst.getVideo()
    cvSrc = inst.putVideo(config.name + " Vision", 160, 120) #Change to get values from config file

    return camera, cvSink, cvSrc
=======
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
>>>>>>> Stashed changes

	# stream properties
	cam.streamConfig = config.get("stream")
	cam.config = config

	cam.height = config["height"]
	cam.width = config["width"]

	cameraConfigs.append(cam)

	return True

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

def processVision(cvSink, cvSrc):
<<<<<<< Updated upstream
    frame = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
    time, frame = cvSink.grabFrame(frame)
    
    if time == 0:
        cvSrc.notifyError(cvSink.getError());
    
    blur = cv2.blur(frame, (3,3))
=======
	frame = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
	time, frame = cvSink.grabFrame(frame)
	
	if time == 0:
		cvSrc.notifyError(cvSink.getError());
	
	blur = cv2.blur(frame, (3,3))
>>>>>>> Stashed changes

	#converting to HSV
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

	# Normal masking algorithm
	lower = np.array([sd.getNumber("h1", 0),sd.getNumber("s1", 0),sd.getNumber("v1", 0)])
	upper = np.array([sd.getNumber("h2", 180),sd.getNumber("s2", 255),sd.getNumber("v2", 255)])

	structure = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

	mask = cv2.inRange(hsv, lower, upper)

	openImg = cv2.morphologyEx(mask, cv2.MORPH_OPEN, structure, iterations=1)

	result = cv2.bitwise_and(frame, frame, mask=openImg)

	#print("processing")
	cvSrc.putFrame(result)

def init():
<<<<<<< Updated upstream
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    cvSink = None;
    cvSrc = None;

    # start cameras
    for config in cameraConfigs:
        serv, cvSink, cvSrc = startCamera(config)
        cameras.append(serv)

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    sd.putNumber("h1", 0)
    sd.putNumber("s1", 0)
    sd.putNumber("v1", 0)
    sd.putNumber("h2", 180)
    sd.putNumber("v2", 255)
    sd.putNumber("s2", 255)

    while True:
        processVision(cvSink, cvSrc)
=======
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

	sd.putNumber("h1", 0)
	sd.putNumber("s1", 0)
	sd.putNumber("v1", 0)
	sd.putNumber("h2", 180)
	sd.putNumber("v2", 255)
	sd.putNumber("s2", 255)

	while True:
		processVision(sinks[0], srcs[0])
		#processVision(sinks[1], srcs[1])

>>>>>>> Stashed changes

if __name__ == "__main__":
	init()