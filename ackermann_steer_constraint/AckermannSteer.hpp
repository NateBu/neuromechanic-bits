#ifndef AckermannSteer_H
#define AckermannSteer_H

#include <iostream>
#include "nmwidgets.h"

using namespace std;
using namespace arma;
class ackermann: public NM::Widget {
    NM::DOM::Node* localNode;
    NM::DOM::Node* rootNode;
    NM::DOM::Node* state;
    NM::DOM::Node* manifold;
    NM::DOM::Node* pErr;
    NM::DOM::Node* vErr;
    NM::DOM::Node* aErr;
    NM::DOM::Node* accel;
    NM::DOM::Node* bodyRotation;
    NM::DOM::Node* dbodyRotationdt;
    NM::DOM::Node* constraintNode;
    int bodyindex, steerdofi;
    mat yawaxis, lataxis, fwdaxis;
    double pgain, vgain, wheelbase;

    void ConstraintInit();
    int getDoFIndex(string dofname);
  public:
    ackermann(NM::DOM::Node *rootTree, int *treeLoc);

    int environment();
    int intrinsicdynamics();
    int muscle();
    int neuron();
    int presolve();
    int postsolve();

};

#endif
