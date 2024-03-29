#!/usr/bin/env python3

import yarp
import roboticslab_vision

detectorOptions = yarp.Property()
detectorOptions.put("device", "ColorRegionDetector")
detectorOptions.put("algorithm", "redMinusGreen")
detectorDevice = yarp.PolyDriver(detectorOptions)

if not detectorDevice.isValid():
    print("Device not available")
    raise SystemExit

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImgRgb = yarp.ImageRgb()
yarpImgRgb.resize(300, 200)
yarpImgRgb.zero()

for r in range(100, 200):
    for c in range(50, 150):
        yarpImgRgb.pixel(r, c).r = 255

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects) or detectedObjects.size() == 0:
    print('Detector failed')
    raise SystemExit

detectedObject = detectedObjects.get(0).asDict()

print("tlx: %d" % detectedObject.find("tlx").asInt32()) # 100
print("tly: %d" % detectedObject.find("brx").asInt32()) # 200
print("brx: %d" % detectedObject.find("tly").asInt32()) # 50
print("bry: %d" % detectedObject.find("bry").asInt32()) # 150
