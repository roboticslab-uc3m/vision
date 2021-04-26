#!/usr/bin/env python

import yarp
import roboticslab_vision

#yarp.Network.init()
#if not yarp.Network.checkNetwork():
#    print("[error] Please try running yarp server")
#    quit()

detectorOptions = yarp.Property()
detectorOptions.put("device", "ColorRegionDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImgRgb = yarp.ImageRgb()
yarpImgRgb.zero()

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects):
    print('Detector failed')
    raise SystemExit

detectorDevice.close()
