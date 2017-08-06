#include "SimpleVelocityMotor.hpp"

NM::Widget *widgetmaker(NM::DOM::Node *rootTree, int *treeLoc){
   return new simplevelocitymotor(rootTree, treeLoc);
};
class widgetproxy {
public:
   widgetproxy(){
      NM::widgetfactory["natebunderson_w_simplevelocitymotor"] = widgetmaker;  // register the widgetmaker with the factory
   }
};
widgetproxy wp;   // our one instance of the widgetproxy

// Initialize static variables for objects of simplevelocitymotor class
int simplevelocitymotor::intrinsicdynamics() {return 0;};
int simplevelocitymotor::muscle() {return 0;};
int simplevelocitymotor::environment() {return 0;};
int simplevelocitymotor::presolve() {return 0;};
int simplevelocitymotor::postsolve() {return 0;};

void simplevelocitymotor::estopCallback(const natebunderson_m_estop::msg::Estop::SharedPtr msg) {
  estop = (msg->stop);
  //std::cout << "SimpleVelocityMotor command received: [" << msg->value << "]" << std::endl;
}


void simplevelocitymotor::commandCallback(const natebunderson_m_double::msg::Double::SharedPtr msg) {
  targetvelocity = msg->value;
  //std::cout << "SimpleVelocityMotor command received: [" << msg->value << "]" << std::endl;
}

void simplevelocitymotor::Init() {
  time = rootNode->child("dynamic")->child("time");
	state = rootNode->child("dynamic")->child("state");
	torque = localNode->child("Torque");
	auto dofname = localNode->child("DegreeOfFreedom")->text();
  auto dof = rootNode->child("bodies")->progeny("degreeoffreedom","name",dofname);
  if (dof == NULL) {
      cout << "Degree of freedom '" << dofname << "' referenced in simplevelocitymotor does not exist" << endl;
      throw 0;
  }
	ndof = NM::NM_MODEL_NDOFS();
  dofindex = dof->child("dofindex")->imatrix()(0,0);
	tlast = 0;
	iterm.fill(0);
}

int simplevelocitymotor::neuron() {
  if (torque==NULL)
    Init();
  mat State = state->dmatrix();
  double t = time->dmatrix()(0,0), estopvelocity=0.0;
  mat verr = (estop) ? State.col(dofindex+ndof)-estopvelocity : State.col(dofindex+ndof)-targetvelocity;
  iterm = iterm + (t-tlast)*verr;
  mat trq = -pgain*verr - igain*iterm;
  // std::cout << "State(dofindex) = " << State.col(dofindex) << " Torque:" << trq << std::endl;
  torque->dmatrix(trq);
  tlast = t;
  
  // Publish
  if (fmod(t,1.0)<.000001) {
    ros2msg->value = t;
    //std::cout << "NMPublishing: t=" << ros2msg->value << std::endl;
    //ros2pub->publish(ros2msg);
    //rclcpp::spin_some(ros2node);
  }

  return 0;
};

void spin(std::shared_ptr< rclcpp::node::Node > node) {
  rclcpp::spin(node);
}

void simplevelocitymotor::nodestartup(string nodename, string vtopic, string stopic) {
  std::cout << "Attempting to initialize node " << nodename << " of type NateBunderson:SimpleVelocityMotor" << std::endl;
  NmExternalResources::nmros2 init_rcl_cpp;
  ros2node = rclcpp::Node::make_shared(nodename);

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;
  
  std::cout << "Node " << nodename << " listening to velocity topic:" << vtopic << " and E-stop topic:" << stopic << std::endl;
  ros2msg = std::make_shared<natebunderson_m_double::msg::Double>(); 
  //ros2pub = ros2node->create_publisher<natebunderson_m_double::msg::Double> (topic, custom_qos_profile);
  ros2vsub = ros2node->create_subscription<natebunderson_m_double::msg::Double>(
    vtopic, std::bind(&simplevelocitymotor::commandCallback, this, std::placeholders::_1),
    custom_qos_profile);
  ros2ssub = ros2node->create_subscription<natebunderson_m_estop::msg::Estop>(
    stopic, std::bind(&simplevelocitymotor::estopCallback, this, std::placeholders::_1),
    custom_qos_profile);

  std::thread listenerThread(spin, ros2node);
  listenerThread.detach();
}

simplevelocitymotor::simplevelocitymotor(NM::DOM::Node *rootTree, int *treeLoc) {
  rootNode = rootTree;
  localNode = rootTree->getModelSubTree(treeLoc);
  pgain = localNode->child("PGain")->dmatrix()(0,0);
  dgain = localNode->child("DGain")->dmatrix()(0,0);
  igain = localNode->child("IGain")->dmatrix()(0,0);
  targetvelocity = 0;
  nodestartup(localNode->child("NodeName")->text(),
    localNode->child("CommandTopic")->text(),localNode->child("EstopTopic")->text());
}
