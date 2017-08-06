#ifndef SimpleVelocityMotor_H
#define SimpleVelocityMotor_H

#include <thread>
#include <iostream>
#include "natebunderson_w_nmexternalresources/nm_external_resources.hpp"
#include "natebunderson_m_double/msg/double.hpp"
#include "natebunderson_m_estop/msg/estop.hpp"

using namespace std;
using namespace arma;
class simplevelocitymotor: public NM::Widget {
    NM::DOM::Node* localNode = NULL;
    NM::DOM::Node* rootNode = NULL;
    NM::DOM::Node* state = NULL;
    NM::DOM::Node* torque = NULL;
    NM::DOM::Node* time = NULL;
    std::shared_ptr< rclcpp::node::Node > ros2node;
    //std::shared_ptr< rclcpp::publisher::Publisher<natebunderson_m_double::msg::Double> > ros2pub;
    std::shared_ptr< natebunderson_m_double::msg::Double > ros2msg;
    std::shared_ptr< rclcpp::subscription::Subscription<natebunderson_m_double::msg::Double> > ros2vsub;
    std::shared_ptr< rclcpp::subscription::Subscription<natebunderson_m_estop::msg::Estop> > ros2ssub;
    mat iterm = zeros(1,1);
    bool estop=false;
    double pgain, dgain, igain, tlast, targetvelocity;
    int ndof, dofindex;
    void Init();
    void nodestartup(string nodename, string vtopic, string stopic);
    void commandCallback(const natebunderson_m_double::msg::Double::SharedPtr msg);
    void estopCallback(const natebunderson_m_estop::msg::Estop::SharedPtr msg);
    
  public:
    simplevelocitymotor(NM::DOM::Node *rootTree, int *treeLoc);

    int environment();
    int intrinsicdynamics();
    int muscle();
    int neuron();
    int presolve();
    int postsolve();

};
#endif
