#ifndef SimpleServo_H
#define SimpleServo_H

#define double_msg natebunderson_m_double
#define nmext_widget natebunderson_w_nmexternalresources


#include <thread>
#include <iostream>
#include "nmwidgets.h"
#include "double_msg/msg/double.hpp"
#include "nmext_widget/nm_external_resources.hpp"


using namespace std;
using namespace arma;
class simpleservo: public NM::Widget {
    NM::DOM::Node* localNode = NULL;
    NM::DOM::Node* rootNode = NULL;
    NM::DOM::Node* state = NULL;
    NM::DOM::Node* torque = NULL;
    NM::DOM::Node* time = NULL;
    std::shared_ptr< rclcpp::node::Node > ros2node;
    std::shared_ptr< rclcpp::publisher::Publisher<double_msg::msg::Double> > ros2pub;
    std::shared_ptr< double_msg::msg::Double > ros2msg;
    std::shared_ptr< rclcpp::subscription::Subscription<double_msg::msg::Double> > ros2sub;
    mat iterm = zeros(1,1);
    double pgain, dgain, igain, tlast, targetposition;
    int ndof, dofindex;
    void Init();
    void nodestartup(string nodename, string topic);
    void commandCallback(const double_msg::msg::Double::SharedPtr msg);

  public:
    simpleservo(NM::DOM::Node *rootTree, int *treeLoc);

    int environment();
    int intrinsicdynamics();
    int muscle();
    int neuron();
    int presolve();
    int postsolve();

};
#endif
