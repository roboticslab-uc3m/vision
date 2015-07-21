// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateMachine.hpp"

/************************************************************************/

bool StateMachine::threadInit() {
    _machineState = 0;
    return true;
}

/************************************************************************/

void StateMachine::run() {
    while(!isStopping()) {
        if(_machineState==-1) {
            ttsSay( ConstString("Sorry, I do not know what that is. Let's start over again.") );
            _machineState=0;
        } else if(_machineState==0) {
            ttsSay( ConstString("I am ready. Please tell me.") );
            _machineState=1;
        } else if(_machineState==1) {
            ConstString inStr = asrListen();
            // Blocking
            _inStrState1 = inStr;
            ConstString outStr("I understood ");
            outStr += inStr + ". Is that correct?";
            ttsSay(outStr);
            _machineState=2;
        } else if (_machineState==2) {
            ConstString inStr = asrListen();
            //j//ConstString inStr = "yes";  //j// shortcut
            // Blocking 
            if (inStr == "yes") {
                if( _inStrState1.find("this is ") != ConstString::npos ) _machineState=3;
                else if ( _inStrState1.find("imagine ") != ConstString::npos ) _machineState=4;
                else if ( _inStrState1.find("touch a ") != ConstString::npos ) _machineState=5;
                else _machineState=-1;
            } else if (inStr == "no") {
                ttsSay( ConstString("Oh, sorry. Could you please repeat?") );  // My bad, let's start over again.
                _machineState=1;
            } else {
                ttsSay( ConstString("What? Was it correct, yes or no?") );
                _machineState=2;  // stay
            }
        } else if (_machineState==3) {
            ConstString words( _inStrState1.substr( _inStrState1.find("this is a ")+10, _inStrState1.length()) );
            Bottle bWords(words);
            outTextPort->write(bWords);
            ConstString outStr("Okay, perfect. I've grounded this information on");
            for(int i=0;i<bWords.size();i++) {
                outStr += " ";
                outStr += bWords.get(i).asString();
            }
            outStr += ".";
            ttsSay( outStr );
            _machineState=0;
        } else if (_machineState==4) {
            ttsSay( ConstString("Let me think...") );
            ConstString words( _inStrState1.substr( _inStrState1.find("imagine a ")+10, _inStrState1.length()) );
            Bottle bWords(words);
            Bottle cmd,response;
            cmd.addVocab(VOCAB_SET);
            //if(bWords.size() > 0) cmd.addInt(1);
            //if(bWords.size() > 1) cmd.addInt(2);
            //if(bWords.size() > 2) cmd.addInt(3);
            cmd.addInt(1);
            cmd.addInt(2);
            cmd.addInt(3);
            //_fittingClient->write(cmd,response);
            cmd.clear();
            response.clear();
            cmd.addVocab(VOCAB_FIT);
            cmd.append(bWords);
            //_fittingClient->write(cmd,response);
            ConstString outStr("That would be somewhere around coordinates ");
            outStr += response.toString();
            outStr += ", i believe.";
            ttsSay( outStr );
            _machineState=0;
        }else if (_machineState==5) {
            ttsSay( ConstString("Okay. Let me think...") );
            ConstString words( _inStrState1.substr( _inStrState1.find("touch a ")+8, _inStrState1.length()) );
            Bottle bWords(words);
            Bottle cmd,response;
            cmd.addVocab(VOCAB_SET);
            cmd.addInt(1);
            cmd.addInt(2);
            cmd.addInt(3);
            cmd.addInt(4);
            cmd.addInt(5);
            //cmd.addInt(6);  // for real
            //bool ok = _fittingClient->write(cmd,response);
            //if (!ok) printf("[WARNING] fitting failed!!!\n");
            cmd.clear();
            response.clear();
            cmd.addVocab(VOCAB_FIT);
            cmd.append(bWords);
            //_fittingClient->write(cmd,response);

            printf("Fit returned: %s.\n",response.toString().c_str());
            
            const int iters = 1; // 10 !!!!!!!!!!!!!!!!!!!!!!!!!!!
            vector< vector <double> > objFeats;
            for(int scene=0; scene<iters; scene++) {

                Bottle* allFeatsBottle = inCvPort->read(true);  //true->blocking
                size_t numFeatures = allFeatsBottle->size();

                Bottle* tmpFeatBottle = allFeatsBottle->get(0).asList();
                size_t numObjsKept = objFeats.size();
                size_t numObjsCame = tmpFeatBottle->size();

                objFeats.resize( numObjsKept + numObjsCame );

                for(int objIdx=numObjsKept; objIdx < (numObjsKept + numObjsCame); objIdx++)
                    objFeats[objIdx].resize(numFeatures);

                for(int featIdx=0; featIdx < allFeatsBottle->size() ; featIdx++) {
                    Bottle* objFeatsBottle = allFeatsBottle->get(featIdx).asList();
                    for(int objIdx=0; objIdx < numObjsCame; objIdx++) {
                        objFeats[objIdx+numObjsKept][featIdx] = objFeatsBottle->get(objIdx).asDouble() ;
                    }
                }

                Time::delay(.2);

            }

            Vector euclDistances;
            for(int objIdx=0; objIdx < objFeats.size(); objIdx++) {
                printf("Obj %d (%f,%f)\n",objIdx,objFeats[objIdx][5],objFeats[objIdx][6]);
                double sumDiffSquared = 0;                
                for(int featIdx=0; featIdx<5; featIdx++) {

                    Bottle cmdG;
                    Bottle responseG;
                    cmdG.addVocab(VOCAB_MAX);
                    cmdG.addInt(featIdx+1);
                    //if (! _groundingClient->write(cmdG,responseG) ) printf("[WARNING] grounding failed!!!\n");
                    double numMax = responseG.get(0).asDouble();  // get(0):min, get(1):max
                    printf("* Feature: %d ",featIdx+1);
                    double target = response.get(featIdx).asDouble();
                    printf("| target: %f ",target);
                    double object = objFeats[objIdx][featIdx];
                    printf("| object: %f ",object);
                    //double numMax = std::max(target,object);
                    double diffNorm = (target-object)/numMax;  // pow((target-object),2);
                    printf("| diffNorm: %f ",diffNorm);                    
                    double diffSquared = diffNorm*diffNorm;  // pow((target-object),2);
//                    double diffSquared = fabs(diffNorm);  // pow((target-object),2);
                    printf("| diffNormSquared: %f ",diffSquared);                    
                    sumDiffSquared += diffSquared;
                    printf("(featureMax: %f)\n",numMax);                    
                }
                double euclDistance = sqrt(sumDiffSquared);
                printf("* euclDistance: %f\n",euclDistance);
                euclDistances.push_back(euclDistance);
            }

            //double minDist = std::numeric_limits<double>::infinity();  // does not work, gets fixed
            //double minDist = 99999999999999999999999.0;  // works
            double minDist = std::numeric_limits<double>::max();
            int minDistIdx = -1;
            for(int idx=0; idx < euclDistances.size() ; idx++) {
                if( euclDistances[idx] < minDist ) {
                    minDist = euclDistances[idx];
                    minDistIdx = idx;
                }
            }
            printf("Closest out of %zd: obj %d (dist: %f)\n",euclDistances.size(),minDistIdx,minDist);
            printf("Coordinates of obj %f %f (6,7) (pxXpos,pxYpos)\n",objFeats[minDistIdx][5],objFeats[minDistIdx][6]);
            printf("Coordinates of obj %f %f %f (8,9,10) (mX0,mY0,mZ0)\n",objFeats[minDistIdx][7],objFeats[minDistIdx][8],objFeats[minDistIdx][9]);
            Bottle outPoints;
            outPoints.addInt(int(objFeats[minDistIdx][5]));
            outPoints.addInt(int(objFeats[minDistIdx][6]));
            // write at end

            //double minDistS = std::numeric_limits<double>::infinity();  // does not work, gets fixed
            //double minDistS = 99999999999999999999999.0;  // works
            double minDistS = std::numeric_limits<double>::max();
            int minDistIdxS = -1;
            for(int idxS=0; idxS < euclDistances.size() ; idxS++) {
                if( (euclDistances[idxS] < minDistS) && ( minDistIdx != idxS) ) {
                    minDistS = euclDistances[idxS];
                    minDistIdxS = idxS;
                }
            }

            printf("Second closest out of %zd: objS %d (dist: %f)\n",euclDistances.size(),minDistIdxS,minDistS);
            printf("Coordinates of objS %f %f (6,7) (pxXpos,pxYpos)\n",objFeats[minDistIdxS][5],objFeats[minDistIdxS][6]);
            printf("Coordinates of objS %f %f %f (8,9,10) (mX0,mY0,mZ0)\n",objFeats[minDistIdxS][7],objFeats[minDistIdxS][8],objFeats[minDistIdxS][9]);

            outPoints.addInt(int(objFeats[minDistIdxS][5]));
            outPoints.addInt(int(objFeats[minDistIdxS][6]));
            outPointsPort->write(outPoints);
            
			if( _inStrState1 == "touch a red large circle") {
				system("redLargeCircle.py");

			} else if( _inStrState1 == "touch a large green square") {
				system("largeGreenSquare.py");

			} else if( _inStrState1 == "touch a white small square") {
				system("whiteSmallSquare.py");

			} else if( _inStrState1 == "touch a small black circle") {
				system("smallBlackCircle.py");
			}
			
            /*ConstString outStr("That would be somewhere around coordinates ");
            outStr += response.toString();
            outStr += ", i believe.";
            ttsSay( outStr );

            cmd.clear();
            response.clear();
            cmd.addVocab(VOCAB_MOVJ);
            cmd.addDouble(objFeats[minDistIdx][7]);
            cmd.addDouble(objFeats[minDistIdx][8]);
            cmd.addDouble(objFeats[minDistIdx][9]);
            _solverClient->write(cmd,response);

            ConstString outStr2("Performing action touch a ");
            outStr2 += _inStrState1;
            outStr2 += ".";
            ttsSay( outStr2 );
            yarp::os::Time::delay(45.0);
            ttsSay( ConstString("Action performed.") );*/
            _machineState=0;
        } else {
            ttsSay( ConstString("ANOMALY") );
        }         
    }
}

/************************************************************************/

void StateMachine::ttsSay(const ConstString& sayConstString) {
    Bottle bOut;
    bOut.addString(sayConstString);
    outTtsPort->write(bOut);
    printf("[StateMachine] Said: %s\n", sayConstString.c_str());
}

/************************************************************************/

ConstString StateMachine::asrListen() {
    Bottle* bIn = inSrPort->read(true);  // shouldWait
    printf("[StateMachine] Listened: %s\n", bIn->toString().c_str());
    return bIn->get(0).asString();
}

/************************************************************************/

int StateMachine::getMachineState() {
    return _machineState;
}

/************************************************************************/

void StateMachine::setInSrPort(yarp::os::BufferedPort<yarp::os::Bottle>* inSrPort) {
    this->inSrPort = inSrPort;
}

/************************************************************************/

void StateMachine::setInCvPort(yarp::os::BufferedPort<yarp::os::Bottle>* inCvPort) {
    this->inCvPort = inCvPort;
}

/************************************************************************/

void StateMachine::setOutPointsPort(yarp::os::Port* outPointsPort) {
    this->outPointsPort = outPointsPort;
}

/************************************************************************/

void StateMachine::setOutTtsPort(yarp::os::Port* outTtsPort) {
    this->outTtsPort = outTtsPort;
}

/************************************************************************/
