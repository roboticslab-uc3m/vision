// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <math.h>

#include <vector>
#include <algorithm>
#include <limits>

#define VOCAB_FIT VOCAB3('f','i','t')
#define VOCAB_MAX VOCAB3('m','a','x')

#define VOCAB_MY_STOP VOCAB4('s','t','o','p')

#define VOCAB_STAT VOCAB4('s','t','a','t')
#define VOCAB_MOVL VOCAB4('m','o','v','l')
#define VOCAB_MOVJ VOCAB4('m','o','v','j')
#define VOCAB_INV VOCAB3('i','n','v')

#define VOCAB_FWD VOCAB3('f','w','d')
#define VOCAB_BKWD VOCAB4('b','k','w','d')
#define VOCAB_POSE VOCAB4('p','o','s','e')
#define VOCAB_ROT VOCAB3('r','o','t')
#define VOCAB_VMOS VOCAB4('v','m','o','s')

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using std::vector;

/**
 * @ingroup StateMachine
 *
 * StateMachine class implements a specific state machine.
 */
class StateMachine : public Thread {
protected:

    yarp::os::BufferedPort<yarp::os::Bottle> *_inAsrPort;
    yarp::os::BufferedPort<yarp::os::Bottle> *_inFeaturesPort;
    yarp::os::Port *_outPointsPort;
    yarp::os::Port *_outTextPort;
    yarp::os::Port *_outTtsPort;
    yarp::os::RpcClient *_fittingClient;
    yarp::os::RpcClient *_groundingClient;
    yarp::os::RpcClient *_solverClient;

    int _machineState;

    ConstString _inStrState1;

    void ttsSay(const ConstString& sayConstString);
    ConstString asrListen();

public:

    /**
     * Initialization method. The thread executes this function
     * when it starts and before "run". This is a good place to 
     * perform initialization tasks that need to be done by the 
     * thread itself (device drivers initialization, memory 
     * allocation etc). If the function returns false the thread 
     * quits and never calls "run". The return value of threadInit()
     * is notified to the class and passed as a parameter 
     * to afterStart(). Note that afterStart() is called by the 
     * same thread that is executing the "start" method.
     */
    bool threadInit();

    /**
     * Loop function. This is the thread itself.
     */
    void run();

    /**
     * Get its state.
     */
    int getMachineState();

    /** Register an input callback port for asr. */
    void setInAsrPort(yarp::os::BufferedPort<yarp::os::Bottle>* inAsrPort);

    /** Register an input callback port for features. */
    void setInFeaturesPort(yarp::os::BufferedPort<yarp::os::Bottle>* inFeaturesPort);

    /** Register an output Port for points. */
    void setOutPointsPort(yarp::os::Port* outPointsPort);

    /** Register an output Port for text. */
    void setOutTextPort(yarp::os::Port* outTextPort);

    /** Register an output Port for tts. */
    void setOutTtsPort(yarp::os::Port* outTtsPort);

    /** Register an rpc client for tts. */
    void setFittingClient(yarp::os::RpcClient* fittingClient);

    /** Register an rpc client for g. */
    void setGroundingClient(yarp::os::RpcClient* groundingClient);

    /** Register an rpc client for the solver. */
    void setSolverClient(yarp::os::RpcClient* solverClient);

};

#endif

