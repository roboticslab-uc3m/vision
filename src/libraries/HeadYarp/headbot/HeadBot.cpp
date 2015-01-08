// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HeadBot.hpp"

// -----------------------------------------------------------------------------

bool teo::HeadBot::indexWithinRange(const int& idx) {
    if (idx >= numMotors ){
        CD_WARNING("Index out of range!! (%d >= %d)!!!\n",idx,numMotors);
        return false;
    }
    return true;
}

// ------------------- DeviceDriver Related ------------------------------------

bool teo::HeadBot::open(Searchable& config) {

    numMotors = config.check("numMotors",Value(NDOF_HEAD),"Number of motors").asInt();

    targetDegs[0] = 0.0;
    targetDegs[1] = 0.0;

    RT_TASK *task;
    M3Sds * sys;
    int cntr=0;

    rt_allow_nonroot_hrt();

    if (sys = (M3Sds*)rt_shm_alloc(nam2num(MEKA_ODOM_SHM),sizeof(M3Sds),USE_VMALLOC))
        printf("Found shared memory starting shm_humanoid_controller.\n");
    else
    {
        printf("Rtai_malloc failure for %s\n",MEKA_ODOM_SHM);
        return 0;
    }

    rt_allow_nonroot_hrt();
    /*if (!(task = rt_task_init_schmod(nam2num("TSHM"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
    {
        rt_shm_free(nam2num(TORQUE_SHM));
        printf("Cannot init the RTAI task %s\n","TSHM");
        return 0;
    }*/
    hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
    usleep(100000); //Let start up
    if (!sys_thread_active)
    {
        rt_task_delete(task);
        rt_shm_free(nam2num(MEKA_ODOM_SHM));
        printf("Startup of thread failed.\n",0);
        return 0;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::close() {
    //-- If here, got crtl-c, so close...
    printf("Removing RT thread...\n",0);
    sys_thread_end=1;
    //rt_thread_join(hst);
    usleep(1250000);
    if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
    //rt_task_delete(task);
    rt_shm_free(nam2num(MEKA_ODOM_SHM));
    return true;
}

// ------------------ IEncoder Related -----------------------------------------

bool teo::HeadBot::resetEncoder(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::resetEncoders() {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setEncoder(int j, double val) {  // encExposed = val;
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setEncoders(const double *vals) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < numMotors; i++)
        ok &= setEncoder(i,vals[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getEncoder(int j, double *v) {
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    *v = status.head.theta[j];

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getEncoders(double *encs) {
    //CD_INFO("\n");  //-- Too verbose in stream.

    bool ok = true;
    for(unsigned int i=0; i < numMotors; i++)
        ok &= getEncoder(i,&encs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getEncoderSpeed(int j, double *sp) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getEncoderSpeeds(double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<numMotors;i++)
        ok &= getEncoderSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getEncoderAcceleration(int j, double *spds) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getEncoderAccelerations(double *accs) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// ------------------ IPosition Related ----------------------------------------

bool teo::HeadBot::getAxes(int *axes) {
    *axes = numMotors;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setPositionMode() {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::positionMove(int j, double ref) {  // encExposed = ref;
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    targetDegs[j] = DEG2RAD( ref );

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::positionMove(const double *refs) {  // encExposed = refs;
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::relativeMove(int j, double delta) {
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::relativeMove(const double *deltas) {  // encExposed = deltas + encExposed

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::checkMotionDone(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::checkMotionDone(bool *flag) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<numMotors; j++)
    {
        ok &= this->checkMotionDone(j,&flag[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setRefSpeed(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setRefSpeeds(const double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<numMotors;i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setRefAcceleration(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::setRefAccelerations(const double *accs) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<numMotors;i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getRefSpeed(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getRefSpeeds(double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<numMotors;i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getRefAcceleration(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::getRefAccelerations(double *accs) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<numMotors;i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::stop(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::stop() {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<numMotors;i++)
        ok &= stop(i);
    return ok;
}

// ------------------ IVelocity Related ----------------------------------------

bool teo::HeadBot::setVelocityMode() {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::velocityMove(int j, double sp) {
    CD_INFO("(%d)\n",j);

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::HeadBot::velocityMove(const double *sp) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<numMotors; j++)
    {
        ok &= this->velocityMove(j,sp[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

