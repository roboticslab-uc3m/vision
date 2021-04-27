#!/usr/bin/env python3

import yarp
import roboticslab_vision

detectorOptions = yarp.Property()
detectorOptions.put("device", "ColorRegionDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

if not detectorDevice.isValid():
    print("Device not available")
    raise SystemExit

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImgRgb = yarp.ImageRgb()
yarpImgRgb.zero()

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects) or detectedObjects.size() == 0:
    print('Detector failed')
    raise SystemExit
