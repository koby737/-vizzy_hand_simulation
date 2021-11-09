/*!
    \file plugins/HandPlugin.cc
    \brief Hand Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#include "HandPlugin.hh"

namespace gazebo {

/// \brief Class for joint and respective mimics data.
class JointGroup
{
    /// Actuated joint
    public: physics::JointPtr actuated;
    /// Actuated joint default target value
    public: double target {0.0};
    /// Vector of mimic joints
    public: std::vector<physics::JointPtr> mimic;
    /// Vector of corresponding mimic joint's multipliers
    public: std::vector<double> multipliers;

    /// Constructor
    public: JointGroup(physics::JointPtr actuated_, double target_=0.0):
        actuated(actuated_), target(target_)
    {
    }
};

/// \brief Class for joints under periodic force
class JointForcePair
{
    /// Actuated joint
    public: physics::JointPtr joint;
    /// Value of the force/torque
    public: double force {0.0};

    public: JointForcePair(physics::JointPtr joint_, double force_=0.0):
        joint(joint_), force(force_)
    {
    }
};

/// \brief Class for private hand plugin data.
class HandPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo topic subscriber
    public: transport::SubscriberPtr sub;
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HandPlugin)

/////////////////////////////////////////////////
HandPlugin::HandPlugin() : ModelPlugin(),
    data_ptr(new HandPluginPrivate)
{
    gzmsg << "[HandPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
HandPlugin::~HandPlugin()
{
    gzmsg << "[HandPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void HandPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Validate model to which plugin is attached
    if (_model->GetJointCount() != 0)
    {
        this->model = _model;
        this->world = _model->GetWorld();
    }
    else
    {
        gzerr << "[HandPlugin] Invalid model." << std::endl;
        return;
    }

    // Extract parameters from SDF element

    // Finger joints
    if (loadJointGroups(_sdf) != true) return;
    // Virtual joints for unconstrained movement
    if (loadVirtualJoints(_sdf) != true) return;
    // Load PID controllers
    if (loadControllers(_sdf) != true) return;
    // Enable/disable gravity
    if (_sdf->HasElement(PARAM_GRAVITY))
    {
        bool gravity_enabled = _sdf->Get<bool>(PARAM_GRAVITY);
        this->model->SetGravityMode(gravity_enabled);
    }

    // Set default pose
    setPose(init_pose);

    // Connect to world update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HandPlugin::onUpdate, this));

    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(REQUEST_TOPIC,
        &HandPlugin::onRequest, this);
    // Publish to the hand plugin topic
    this->data_ptr->pub = this->data_ptr->node->
        Advertise<HandMsg>(RESPONSE_TOPIC);

    gzmsg << "[HandPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
bool HandPlugin::loadMimicJoints(sdf::ElementPtr _sdf, JointGroup & joint)
{
    std::string mimic_name;
    double multiplier;

    if (_sdf->HasElement(PARAM_MIMIC_JOINT))
    {
        sdf::ElementPtr mimic_sdf;
        for (mimic_sdf = _sdf->GetElement(PARAM_MIMIC_JOINT);
            mimic_sdf != NULL;
            mimic_sdf = mimic_sdf->GetNextElement())
        {
            if (mimic_sdf->HasAttribute(PARAM_NAME) &&
                mimic_sdf->HasAttribute(PARAM_MULTIPLIER))
            {
                mimic_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(mimic_name);
                mimic_sdf->GetAttribute(PARAM_MULTIPLIER)->Get<double>(multiplier);
                joint.mimic.push_back(this->model->GetJoint(mimic_name));
                joint.multipliers.push_back(multiplier);

                gzdbg << "   Mimic joint " << mimic_name << " k = " << multiplier << std::endl;
            }
            else if (mimic_sdf->GetName() == PARAM_MIMIC_JOINT)
            {
                gzerr << "[HandPlugin] No joint name provided." << std::endl;
                return false;
            }
        }
    }
}

/////////////////////////////////////////////////
bool HandPlugin::loadJointGroups(sdf::ElementPtr _sdf)
{
    std::string joint_name;
    double target;
    int inserted = 0;

    if (_sdf->HasElement(PARAM_JOINT_GROUP))
    {
        sdf::ElementPtr joint_sdf;
        for (joint_sdf = _sdf->GetElement(PARAM_JOINT_GROUP);
            joint_sdf != NULL;
            joint_sdf = joint_sdf->GetNextElement())
        {
            if (joint_sdf->HasAttribute(PARAM_NAME))
            {
                joint_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(joint_name);
                joint_sdf->GetAttribute(PARAM_TARGET)->Get<double>(target);
                this->joint_groups.emplace_back(this->model->GetJoint(joint_name), target);
                gzdbg << "Actuated joint " << joint_name << "target = " << target << std::endl;
                loadMimicJoints(joint_sdf, this->joint_groups.at(inserted++));
            }
            else if (joint_sdf->GetName() == PARAM_JOINT_GROUP)
            {
                gzerr << "[HandPlugin] No joint name provided." << std::endl;
                return false;
            }
        }
    }
    else
    {
        gzerr << "[HandPlugin] No joints specified." << std::endl;
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
bool HandPlugin::loadVirtualJoints(sdf::ElementPtr _sdf)
{
    if (_sdf->HasElement(PARAM_VIRTUAL_JOINTS))
    {
        std::string virtual_joints_str = _sdf->Get<std::string>(PARAM_VIRTUAL_JOINTS);

        // Convert string to array of strings
        std::vector<std::string> virtual_joint_names;
        boost::split(virtual_joint_names, virtual_joints_str,
            boost::is_any_of(" "), boost::token_compress_on);
        int num_virtual_joints = virtual_joint_names.size();

        for (auto joint_name : virtual_joint_names) {
            this->virtual_joints.push_back(this->model->GetJoint(joint_name));
        }

        if (this->virtual_joints.size() != num_virtual_joints)
        {
            gzerr << "[HandPlugin] Invalid reference link or joint specified." << std::endl;
            return false;
        }
    }
    else
    {
        gzerr << "[HandPlugin] Unspecified reference link or joint." << std::endl;
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
void HandPlugin::setPIDController(
    physics::JointControllerPtr controller,
    int type,
    physics::JointPtr joint,
    double p, double i, double d,
    double initial_value)
{
    std::string joint_name = joint->GetScopedName();
    if (type == POSITION)
    {
        controller->SetPositionPID(joint_name, common::PID(p,i,d));
        controller->SetPositionTarget(joint_name, initial_value);
    }
    else if (type == VELOCITY)
    {
        controller->SetVelocityPID(joint_name, common::PID(p,i,d));
        controller->SetVelocityTarget(joint_name, initial_value);
    }
}

/////////////////////////////////////////////////
bool HandPlugin::loadControllers(sdf::ElementPtr _sdf)
{
    physics::JointControllerPtr controller = model->GetJointController();

    // Reset existing PIDs
    std::map<std::string, common::PID> vel = controller->GetVelocityPIDs();
    for (const auto &entry : vel) {
        controller->SetVelocityPID(entry.first, common::PID(0,0,0));
    }
    std::map<std::string, common::PID> pos = controller->GetPositionPIDs();
    for (const auto &entry : pos) {
        controller->SetPositionPID(entry.first, common::PID(0,0,0));
    }

    // Controller pid gains and controller types
    double v_p = 10.0, v_i = 0.0, v_d = 0.0;
    double r_p = 3.0, r_i = 0.0, r_d = 0.0;
    int v_type = POSITION, r_type = POSITION;
    std::string type;

    if (_sdf->HasElement(PARAM_CONTROLLERS))
    {
        sdf::ElementPtr ctrl_sdf = _sdf->GetElement(PARAM_CONTROLLERS);
        if (ctrl_sdf->HasElement(PARAM_CTRL_REAL) &&
            ctrl_sdf->HasElement(PARAM_CTRL_VIRTUAL))
        {
            sdf::ElementPtr virtual_sdf = ctrl_sdf->GetElement(PARAM_CTRL_VIRTUAL);
            sdf::ElementPtr real_sdf = ctrl_sdf->GetElement(PARAM_CTRL_REAL);
            virtual_sdf->GetAttribute(PARAM_CTRL_TYPE)->Get<std::string>(type);
            if (type == "velocity") v_type = VELOCITY; // else position
            real_sdf->GetAttribute(PARAM_CTRL_TYPE)->Get<std::string>(type);
            if (type == "velocity") r_type = VELOCITY; // else position
            virtual_sdf->GetAttribute("p")->Get<double>(v_p);
            virtual_sdf->GetAttribute("i")->Get<double>(v_i);
            virtual_sdf->GetAttribute("d")->Get<double>(v_d);
            real_sdf->GetAttribute("p")->Get<double>(r_p);
            real_sdf->GetAttribute("i")->Get<double>(r_i);
            real_sdf->GetAttribute("d")->Get<double>(r_d);
        }
    }

    for (const auto & joint : virtual_joints)
    {
       setPIDController(controller, v_type,
            joint, v_p, v_i, v_d, 0.0);
    }
    for (const auto & group : joint_groups)
    {
        setPIDController(controller, r_type,
            group.actuated, r_p, r_i, r_d, 0.0);
            setPIDTarget(r_type, group.actuated, group.target, true);
        for (int i = 0; i < group.mimic.size(); i++)
        {
            setPIDController(controller, r_type,
                group.mimic.at(i), r_p, r_i, r_d, 0.0);
            setPIDTarget(r_type, group.mimic.at(i), 
                group.target * group.multipliers.at(i), true);
        }
    }

    // DEBUG
    std::map<std::string, common::PID> new_vel = controller->GetVelocityPIDs();
    std::map<std::string, common::PID> new_pos = controller->GetPositionPIDs();
    gzdbg << "Position Controllers:" << std::endl;
    for (const auto &entry : new_pos) {
        gzdbg << "   " << entry.first
            << " P: " << entry.second.GetPGain()
            << " I: " << entry.second.GetIGain()
            << " D: " << entry.second.GetDGain()
            << "\n";
    }
    gzdbg << "Velocity Controllers:" << std::endl;
    for (const auto &entry : new_vel) {
        gzdbg << "   " << entry.first
            << " P: " << entry.second.GetPGain()
            << " I: " << entry.second.GetIGain()
            << " D: " << entry.second.GetDGain()
            << "\n";
    }

    return true;
}

/////////////////////////////////////////////////
void HandPlugin::onUpdate()
{
    std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

    // Periodic routine
    if (routine_active) {
        updateForces();
    }

    // Process request
    if (msg_req)
    {
        updatePose(msg_req);
        updatePIDTargets(msg_req);
        checkReset(msg_req);

	if(check(msg_req)){
		std::string col("r_ind_phal_1_joint");
		debug_joint = model->GetJoint(col);

		type = 2;
		std::string date = "2021_02_09";
		if(type == 1){
			ref = 150/2*3.14159265359/180;
			data = "Time,Position,Ref\n";
			file_name = "Dados" + date + "/Position/dados.csv";
		}
		else if(type == 2){
			ref = 2;
			data = "Time,Velocity,Ref\n";
			file_name = "Dados" + date + "/Velocity/dados.csv";
		}
		else if(type == 3){
			ref = 2;
			data = "Time,Torque,Ref\n";
			file_name = "Dados" + date + "/Torque/dados.csv";
		}
		else{
			data = "Time,Torque,Position,Velocity\n";
			file_name = "Dados" + date + "/dados.csv";
		}
		abs_start = std::chrono::high_resolution_clock::now();
		start = std::chrono::high_resolution_clock::now();
		loop = true;

		/*std::string col2("r_ind_phal_2_joint");
		debug_joint2 = model->GetJoint(col2);
		if(type == 1){
			data2 = "Time,Position,Ref\n";
			file_name2 = "Dados" + date + "/Position/dados2.csv";
		}
		else if(type == 2){
			data2 = "Time,Velocity,Ref\n";
			file_name2 = "Dados" + date + "/Velocity/dados2.csv";
		}
		else if(type == 3){
			file_name2 = "Dados" + date + "/Torque/dados2.csv";
		}
		else{
			file_name2 = "Dados" + date + "/dados2.csv";
		}


		std::string col3("r_ind_phal_3_joint");
		debug_joint3 = model->GetJoint(col3);
		if(type == 1){
			data3 = "Time,Position,Ref\n";
			file_name3 = "Dados" + date + "/Position/dados3.csv";
		}
		else if(type == 2){
			data3 = "Time,Velocity,Ref\n";
			file_name3 = "Dados" + date + "/Velocity/dados3.csv";
		}
		else if(type == 3){
			file_name3 = "Dados" + date + "/Torque/dados3.csv";
		}
		else{
			file_name3 = "Dados" + date + "/dados3.csv";
		}


		std::string col4("base_footprint");
		debug_link = model->GetLink(col4);
		for(auto link:model->GetLinks())
			std::cout << link->GetName() << std::endl;

		if(debug_link == NULL)
			std::cout << "There is no link..." << std::endl;
		if(type == 1){
			data4 = "Time,Position,Ref\n";
			file_name4 = "Dados" + date + "/Position/dados4.csv";
		}
		else if(type == 2){
			data4 = "Time,Velocity,Ref\n";
			file_name4 = "Dados" + date + "/Velocity/dados4.csv";
		}
		else if(type == 3){
			file_name4 = "Dados" + date + "/Torque/dados4.csv";
		}
		else{
			file_name4 = "Dados" + date + "/dados4.csv";
		}*/
	}



        msg_req.reset();
    }



	if(loop){
		stop = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span = stop - abs_start;
		std::chrono::duration<double> time_span_ = stop - start;
		start = std::chrono::high_resolution_clock::now();

		ignition::math::Vector3d force1 = debug_joint->GetForceTorque(0).body1Force;
		ignition::math::Vector3d force2 = debug_joint->GetForceTorque(0).body2Force;
		ignition::math::Vector3d torque1 = debug_joint->GetForceTorque(0).body1Torque;
		ignition::math::Vector3d torque2 = debug_joint->GetForceTorque(0).body2Torque;
		//gzdbg << force1[0] << ", " << force1[1] << ", " << force1[2] << std::endl;
		//gzmsg << force2[0] << ", " << force2[1] << ", " << force2[2] << std::endl;
		double my_force =  sqrt(pow(force1[0],2)  + pow(force1[1],2)  + pow(force1[2],2));
		double my_torque = sqrt(pow(torque1[0],2) + pow(torque1[1],2) + pow(torque1[2],2));

		double value = 0.0;
		if(type == 1) value = debug_joint->Position();
		else if(type == 2) value = debug_joint->GetVelocity(0);//time_span_.count();
		else if(type == 3) value = debug_joint->GetForce(0);

		std::cout << time_span.count() << "," << value << std::endl;
		data = data + std::to_string(time_span.count()) + "," + std::to_string(value)  + "," + std::to_string(ref) + "\n";

		if(time_span.count() > 10){
			std::ofstream MyFile(file_name);
			MyFile << data;
			MyFile.close();
			loop = false;
		}

		/*if(type == 1) value = debug_joint2->Position();
		else if(type == 2) value = debug_joint2->GetVelocity(0);
		else if(type == 3) value = debug_joint2->GetForce(0);

		data2 = data2 + std::to_string(time_span.count()) + "," + std::to_string(value)  + "," + std::to_string(ref) + "\n";

		if(time_span.count() > 10){
			std::ofstream MyFile(file_name2);
			MyFile << data2;
			MyFile.close();
			loop = false;
		}

		if(type == 1) value = debug_joint3->Position();
		else if(type == 2) value = debug_joint3->GetVelocity(0);
		else if(type == 3) value = debug_joint3->GetForce(0);

		std::cout << time_span.count() << "," << value << std::endl;
		data3 = data3 + std::to_string(time_span.count()) + "," + std::to_string(value)  + "," + std::to_string(ref) + "\n";

		if(time_span.count() > 10){
			std::ofstream MyFile(file_name3);
			MyFile << data3;
			MyFile.close();
			loop = false;
		}

		ignition::math::Vector3d my_vec;
		my_vec = debug_link->WorldLinearVel();//time_span_.count();
		value = sqrt(pow(my_vec[0],2) + pow(my_vec[1],2) + pow(my_vec[2],2));

		std::cout << time_span.count() << "," << value << std::endl;
		data4 = data4 + std::to_string(time_span.count()) + "," + std::to_string(value)  + "," + std::to_string(ref) + "\n";

		if(time_span.count() > 10){
			std::ofstream MyFile(file_name4);
			MyFile << data4;
			MyFile.close();
			loop = false;
		}*/
	}


}

/////////////////////////////////////////////////
void HandPlugin::onRequest(HandMsgPtr &_msg)
{
    std::lock_guard<std::mutex> lock(data_ptr->mutex);
    // If no pending request exists
    if (!msg_req) { msg_req = _msg; }
}

/////////////////////////////////////////////////
void HandPlugin::setPose(const ignition::math::Pose3d & pose)
{
    ignition::math::Vector3d pos = pose.Pos();
    ignition::math::Quaterniond rot = pose.Rot();

    virtual_joints.at(0)->SetPosition(0, pos.X());
    virtual_joints.at(1)->SetPosition(0, pos.Y());
    virtual_joints.at(2)->SetPosition(0, pos.Z());
    virtual_joints.at(3)->SetPosition(0, rot.Roll());
    virtual_joints.at(4)->SetPosition(0, rot.Pitch());
    virtual_joints.at(5)->SetPosition(0, rot.Yaw());

    // DEBUG
    gzdbg << "Set pose " << pose << std::endl;

    //resetJoints();
    imobilise();
}

/////////////////////////////////////////////////
void HandPlugin::updatePose(HandMsgPtr &_msg)
{
    if (_msg->has_pose())
    {
        setPose(msgs::ConvertIgn(_msg->pose()));
    }
}



/////////////////////////////////////////////////
void HandPlugin::updateForces()
{

	int i = 0;
	for (const auto & pair : joint_force_pairs){
		pair.joint->SetForce(0, pair.force);
	}
	//gzdbg << "Applying direct forces/torques." << std::endl;
}

/////////////////////////////////////////////////
void HandPlugin::setPIDTarget(
    int type,
    physics::JointPtr joint,
    double value,
    bool force)
{
    physics::JointControllerPtr ctrl = model->GetJointController();
    std::string joint_name = joint->GetScopedName();

    if (type == VELOCITY)
    {
        ctrl->SetVelocityTarget(joint_name, value);
    }
    else if (type == POSITION)
    {
        ctrl->SetPositionTarget(joint_name, value);
        if (force) {
            joint->SetPosition(0, value);
        }

        // DEBUG
        std::map<std::string, double> positions;
        positions = ctrl->GetPositions();
        gzdbg << joint_name << " : " << positions[joint_name] << "\n";
    }
    else if (type == FORCE)
    {
        joint_force_pairs.emplace_back(joint, value);
        routine_active = true;
    }
}

/////////////////////////////////////////////////
void HandPlugin::updatePIDTargets(HandMsgPtr &_msg)
{
	if (_msg->pid_targets_size() > 0){
	        bool force = (_msg->has_force_target())? _msg->force_target() : false;
		
		routine_active = false;

		if(_msg->interruptor()){
			gzdbg << "Reset position control.\n";
			model->GetJointController()->Reset(); //Turn off the position control to pass to force control
			return;
		}

		for (const auto target : _msg->pid_targets()){
			int type = target.type();
			std::string name = target.joint();
			double value = target.value();

			physics::JointPtr joint = model->GetJoint(name);
			// Check joint is in model
			if (!joint) {
				gzdbg << name << " not found\n";
				return;
			}

			// Update PID target
			// Update PID target
			setPIDTarget(type, joint, value, force);
			// Update mimic joints' PIDs
			for (const auto & group : joint_groups){
				if (group.actuated == joint){
					for (int i = 0; i < group.mimic.size(); i++){
						physics::JointPtr mimic = group.mimic.at(i);
						setPIDTarget(type, mimic, value * group.multipliers.at(i), force);
					}
				}
			}
		}
	}
}

/////////////////////////////////////////////////
void HandPlugin::checkReset(HandMsgPtr &_msg)
{
    if (_msg->has_reset()) {
        if (_msg->reset()) {

            imobilise();
            //resetWorld();

            // Reset timers
            routine_active = false;

            // Reset joints under constant force/torque actuation
            joint_force_pairs.clear();
        }
    }
}

/////////////////////////////////////////////////
void HandPlugin::imobilise()
{
    physics::Link_V links = model->GetLinks();
    for (auto link : links) { link->ResetPhysicsStates(); }
    physics::Joint_V joints = model->GetJoints();
    for (auto joint : joints) { joint->SetVelocity(0, 0); }
    model->GetJointController()->Reset();
}

/////////////////////////////////////////////////
void HandPlugin::resetJoints()
{
    for (const auto &group : joint_groups) {
        group.actuated->SetPosition(0, 0);
        for (const auto &mimic_joint : group.mimic) {
            mimic_joint->SetPosition(0, 0);
        }
    }
}


/////////////////////////////////////////////////
void HandPlugin::resetWorld()
{
    world->SetPhysicsEnabled(false);
    world->Reset();
    world->SetPhysicsEnabled(true);
}

/////////////////////////////////////////////////
bool HandPlugin::check(HandMsgPtr &_msg)
{	
	if(_msg->has_measure()){
		return _msg->measure();
	}
	else{
		return false;
	}
}

}
