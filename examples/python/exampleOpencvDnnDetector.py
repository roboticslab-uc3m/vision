#!/usr/bin/env python3

import yarp
import roboticslab_vision

detectorOptions = yarp.Property()
detectorOptions.put("device", "OpencvDnnDetector")
detectorOptions.put("trainedModel", "yolov3-tiny.weights")
detectorOptions.put("configDNNModel", "yolov3-tiny.cfg")
detectorOptions.put("classesTrainedModel", "coco-object-categories.txt")

detectorDevice = yarp.PolyDriver(detectorOptions)

if not detectorDevice.isValid():
    print("Device not available")
    raise SystemExit

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

rf = yarp.ResourceFinder()
rf.setDefaultContext("OpencvDnnDetector")
sampleFullName = rf.findFileByName("tests/teddy-bear.png")
yarpImgRgb = yarp.ImageRgb()

if not yarp.read(yarpImgRgb, sampleFullName, yarp.FORMAT_PNG):
    print("Image file not available")
    raise SystemExit

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects):
    print("Detector failed")
    raise SystemExit

detectedObject = detectedObjects.get(0).asDict()

print("tlx: %d" % detectedObject.find("tlx").asInt32()) # 102
print("tly: %d" % detectedObject.find("brx").asInt32()) # 264
print("brx: %d" % detectedObject.find("tly").asInt32()) # 21
print("bry: %d" % detectedObject.find("bry").asInt32()) # 250

print("confidence: %f" % detectedObject.find("confidence").asFloat64()) # 0.250131
print("category: %s" % detectedObject.find("category").asString()) # teddy bear <3
