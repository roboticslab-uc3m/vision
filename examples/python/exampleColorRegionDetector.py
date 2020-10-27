import yarp
import roboticslab_vision

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print("[error] Please try running yarp server")
    quit()

detectorOptions = yarp.Property()
detectorOptions.put("device","ColorRegionDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImg = yarp.ImageRgb()
yarpImg.zero()

print("detect()")
#iDetector.detect(yarpImg, std::vector< yarp::os::Property > &)

detectorDevice.close()