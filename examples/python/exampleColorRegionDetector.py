#!/usr/bin/env python

import yarp
import roboticslab_vision

#yarp.Network.init()
#if not yarp.Network.checkNetwork():
#    print("[error] Please try running yarp server")
#    quit()

detectorOptions = yarp.Property()
detectorOptions.put("device","ColorRegionDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImgRgb = yarp.ImageRgb()
yarpImgRgb.zero()

yarpImgFlex = yarp.FlexImage()
yarpImgFlex.setPixelCode(yarpImgRgb.getPixelCode())
yarpImgFlex.setQuantum(yarpImgRgb.getQuantum())
yarpImgFlex.setExternal(yarpImgRgb.getRawImage(), yarpImgRgb.width(), yarpImgRgb.height())

print("detect()")
detectedObjects = iDetector.detect(yarpImgFlex)

detectorDevice.close()
