#include "SharedArea.hpp"

namespace roboticslab {

/************************************************************************/
void SharedArea::init() {
    lineCoords[0] =  0;  lineCoords[1] = 0;  lineCoords[2] = 0;
    lineCoords[3] =  -0.2;  lineCoords[4] = 0;  lineCoords[5] = 1;
}

/************************************************************************/
void SharedArea::setLC(const double _lineCoords[6]) {
    lcMutex.wait();
    for(int i=0;i<6;i++) lineCoords[i] = _lineCoords[i];
    lcMutex.post();
    printf("[SharedArea] set shortCoords1: %f %f %f\t",lineCoords[0],lineCoords[1],lineCoords[2]);
    printf("[SharedArea] set shortCoords2: %f %f %f\n",lineCoords[3],lineCoords[4],lineCoords[5]);
}

/************************************************************************/
void SharedArea::getLC(double _lineCoords[6]) {
    lcMutex.wait();
    for(int i=0;i<6;i++) _lineCoords[i] = lineCoords[i];
    lcMutex.post();
    // printf("[SharedArea] get coord1: %f %f %f\t",lineCoords[0],lineCoords[1],lineCoords[2]);
    // printf("[SharedArea] get coord2: %f %f %f\n",lineCoords[3],lineCoords[4],lineCoords[5]);
}

/************************************************************************/
void SharedArea::getLongLC(double _longLineCoords[6]) {
    double shortCoords[6];
    lcMutex.wait();
    for(int i=0;i<6;i++) shortCoords[i] = lineCoords[i];
    lcMutex.post();
    printf("[SharedArea] get shortCoords1: %f %f %f\t",shortCoords[0],shortCoords[1],shortCoords[2]);
    printf("[SharedArea] get shortCoords2: %f %f %f\n",shortCoords[3],shortCoords[4],shortCoords[5]);
    double direction[3] = { shortCoords[3] - shortCoords[0],
                            shortCoords[4] - shortCoords[1],
                            shortCoords[5] - shortCoords[2] };

    double gain = 15;  // 15 okay using dist from elbow, 30 too much
    _longLineCoords[0] = shortCoords[0];
    _longLineCoords[1] = shortCoords[1];
    _longLineCoords[2] = shortCoords[2];
    _longLineCoords[3] = shortCoords[0] + (direction[0] * gain);
    _longLineCoords[4] = shortCoords[1] + (direction[1] * gain);
    _longLineCoords[5] = shortCoords[2] + (direction[2] * gain);
    printf("[SharedArea] get longCoords1: %f %f %f\t",_longLineCoords[0],_longLineCoords[1],_longLineCoords[2]);
    printf("[SharedArea] get longCoords2: %f %f %f\n",_longLineCoords[3],_longLineCoords[4],_longLineCoords[5]);
}

/************************************************************************/
}  // namespace roboticslab

