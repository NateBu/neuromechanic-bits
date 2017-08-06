#include "AckermannSteer.hpp"

NM::Widget *widgetmaker(NM::DOM::Node *rootTree, int *treeLoc){
   return new ackermann(rootTree, treeLoc);
};
class widgetproxy {
public:
   widgetproxy(){
      NM::widgetfactory["biomechanico_w_ackermannsteer"] = widgetmaker;  // register the widgetmaker with the factory
   }
};
// our one instance of the widgetproxy
widgetproxy wp;

// Initialize static variables for objects of ackermann class
int ackermann::intrinsicdynamics() {return 0;};
int ackermann::muscle() {return 0;};
int ackermann::neuron() {return 0;};
int ackermann::presolve() {return 0;};
int ackermann::postsolve() {return 0;};

int ackermann::environment() {
	// Two constraints: Lateral velocity is zero, and the curvature (k) is proportional to the steer angle (alpha) and wheel base (wb)
  // k = tan(alpha)/wb
	//    yawrate*wb - tan(alpha)*velocity = 0          
  //    (yawaxis*wb*Jr-tan(alpha)*fwdaxis*Jt)*qdot = 0
	// J*qdotdot - (sec^2(alpha)*alphadot*fwdaxis)*J*qdot = 0
	if (manifold==NULL)
    ConstraintInit();
    
	int ndof = NM::NM_MODEL_NDOFS();
  mat State = state->dmatrix();
  mat origin = zeros(1,3);
  mat R = bodyRotation->dmatrix();
  mat dRdt = dbodyRotationdt->dmatrix();
  NM::NMJacobian J = NM::Jacobian(origin, bodyindex);
  mat Jr = J.J.rows(3,5);     mat dJr = J.dJdt_dQdt.rows(3,5);
  mat Jt = J.J.rows(0,2);     mat dJt = J.dJdt_dQdt.rows(0,2);
  
	mat localVelocity = zeros(1,3);
	mat vOrigint = NM::GlobalVelocity(origin, localVelocity, bodyindex); // = J*qdot
  mat vOriginr = (Jr*State.cols(ndof,2*ndof-1).t()).t();
  mat yax = R*yawaxis;    mat dyax = dRdt*yawaxis;
	mat vax = R*fwdaxis;    mat dvax = dRdt*fwdaxis;
	mat lax = R*lataxis;    mat dlax = dRdt*lataxis;
  double tA = tan(State(0,steerdofi));
  double dtA = State(0,ndof+steerdofi) * pow(1/cos(State(0,steerdofi)),2);
  // Build constraint
	mat cmanifold = zeros(2,ndof);
  // CAN I DO THIS (yawaxis.t()*Jr)???
	cmanifold.row(0) = wheelbase*yax.t()*Jr-tA*vax.t()*Jt;
	cmanifold.row(1) = lax.t()*Jt;
  
  //cmanifold.raw_print("CM:");
	mat positionerror = zeros(1,2);
	mat velocityerror = -State.cols(ndof,2*ndof-1)*cmanifold.t();
	mat acceleration  = zeros(1,2);
  
  mat acc0 = -wheelbase*(yax.t()*dJr + vOriginr*dyax)
            -tA*(vax.t()*dJt+vOrigint*dvax) + dtA * (vOrigint*vax);
  mat acc1 = -lax.t()*dJt - vOrigint*dRdt*lataxis;
  acceleration(0,0) = acc0(0,0);
  acceleration(0,1) = acc1(0,0);
  //lax.t().raw_print("LateralAxis_Global:");
  //(vOrigint*lax).raw_print("VelocityY");
	mat RHS = acceleration + pgain*positionerror + vgain*velocityerror;
	// Set constraints

  manifold->dmatrix(cmanifold);
	pErr->dmatrix(positionerror);
	vErr->dmatrix(velocityerror);
	aErr->dmatrix(RHS);
  return 0;
};

int ackermann::getDoFIndex(string dofname) {
    auto steerdof_ = rootNode->child("bodies")->progeny("degreeoffreedom","name",dofname);
    if (steerdof_ == NULL) {
        cout << "Degree of freedom '" << dofname << "' referenced in AckermannSteer does not exist" << endl;
        throw 0;
    }
    return steerdof_->child("dofindex")->imatrix()(0,0);
}

void ackermann::ConstraintInit() {
	// Get wheelbase and dofs
  wheelbase = abs(localNode->child("WheelBase")->dmatrix()(0,0));
  
  vec yawax = normalise(arma::conv_to<vec>::from(localNode->child("YawAxis")->dmatrix().row(0)));
  vec fwdax = normalise(arma::conv_to<vec>::from(localNode->child("VelocityAxis")->dmatrix().row(0)));
  vec latax = cross(yawax,fwdax);
  if (norm(latax)==0) throw 0;
  latax = normalise(latax);
  fwdax = cross(latax,yawax); // Replace user specified fwd ax with one that is perpendicular to yaw and lateral axes
  yawaxis = yawax;
  fwdaxis = fwdax;
  lataxis = latax;
  
  steerdofi = getDoFIndex(localNode->child("SteeringDoF")->text());
  NM::DOM::Node* body = rootNode->child("Bodies")->child("RigidBody","Name",localNode->child("Body")->text());
  bodyindex = body->child("BodyIndex")->imatrix()(0,0);
  bodyRotation = body->child("RotationMatrix");
  dbodyRotationdt = body->child("dRotationMatrixdt");
  
	// Get constraint stuff
	manifold = constraintNode->child("ConstraintManifold");
}

ackermann::ackermann(NM::DOM::Node *rootTree, int *treeLoc) {
  rootNode = rootTree;
  localNode = rootTree->getModelSubTree(treeLoc);
  accel = NULL;
  state = rootNode->child("dynamic")->child("state");
  
  constraintNode = rootNode->getAddElement("environment")->getAddElement("constraints")->addElement("GenericConstraint");
	int nc = 2;
  mat znc = zeros(1,nc);
  mat z1 = zeros(1,1);
	string InstanceName = localNode->child("Name")->text();
	constraintNode->addAttribute("Type",(string)"generic");
	constraintNode->addAttribute("Parent",(string)"ground");
	constraintNode->addAttribute("Name",InstanceName+"_constraint");
	constraintNode->addElement("MaxConstraints",nc);
	manifold = NULL; // We'll attach to this later
	pErr = constraintNode->addElement("PositionError",znc);
	vErr = constraintNode->addElement("VelocityError",znc);
	aErr = constraintNode->addElement("Acceleration",znc);
	pgain = 0;	z1(0,0) = pgain;	constraintNode->addElement("PositionErrorGain",z1);
	vgain = 0;	z1(0,0) = vgain;	constraintNode->addElement("VelocityErrorGain",z1);
	
}
