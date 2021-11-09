/*!
    \file utils/Interface.cc
    \brief Grasp interface

    \author João Borrego : jsbruglie
*/

#include "Interface.hh"

//////////////////////////////////////////////////
Interface::Interface(){

	node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	node->Init();
	pub = node->Advertise<grasp::msgs::Hand>(HAND_PLUGIN_TOPIC);
	sub = node->Subscribe("~/my_contact", &Interface::read_values, this);
	pub->WaitForConnection();
	pub_call = node->Advertise<grasp::msgs::ContactRequest>("~/grasp/contact");
	pub_call->WaitForConnection();

	std::vector<double> values;

	// Initial configuration

	joints = {
		"virtual_px_joint", "virtual_py_joint", "virtual_pz_joint",
		"virtual_rr_joint", "virtual_rp_joint", "virtual_ry_joint"
	};

	values = {-0.05, -0.03, 0.05, 0.0, 0.0, 0.0}; // power
	//values = {-0.045, -0.05, 0.05, 0.0, 0.0, 0.0}; // precision
	//values = {-0.05, -0.18, 0.05, 0.0, 0.0, 0.0}; // test

	setJoints(joints, values);

	srand(time(NULL));
}

//////////////////////////////////////////////////
bool Interface::init(const std::string & config_file, const std::string & robot){

	std::string grasp_name, joint;
	double pre_value, post_value, force_value; 

	try{
		YAML::Node config = YAML::LoadFile(config_file);
		// Read robot name
		robot_name = config[robot]["name"].as<std::string>();
		// Read grasp configurations
		YAML::Node grasp_shapes = config[robot]["grasp_configurations"];

		for (auto const & grasp : grasp_shapes){

			grasp_name = grasp.first.as<std::string>();
			grasps.emplace_back(grasp_name);
			YAML::Node joint_value_pairs = config[robot]["grasp_configurations"][grasp_name];

			for (auto const & pair : joint_value_pairs){

				joint = pair.first.as<std::string>();

				pre_value = pair.second["pre"].as<double>();
				grasps.back().pre.emplace_back(joint, pre_value);

				post_value = pair.second["post"].as<double>();
				post_value = post_value*3.14159265358979323846/180; // degrees to radians
				grasps.back().post.emplace_back(joint, post_value);

				if (pair.second["force"]){
					force_value = pair.second["force"].as<double>();
					grasps.back().forces.emplace_back(joint, force_value);
				}

			}
			debugPrintTrace("Grasp configuration " << grasps.back().name << " : " << grasps.back().pre.size() << " joints.");
		}

		// Read transformation matrix from pose
		std::vector<double> p = config[robot]["pose"].as<std::vector<double>>();
		ignition::math::Pose3d pose(p.at(0),p.at(1),p.at(2),p.at(3),p.at(4),p.at(5));
		t_base_gripper = ignition::math::Matrix4d(pose);

	}
	catch (YAML::Exception& yamlException){
		errorPrintTrace("Unable to parse " << config_file);
		return false;
	}


	usleep(1000);
	openFingers();

	return true;
}

/////////////////////////////////////////////////
ignition::math::Matrix4d Interface::getTransformBaseGripper(){
	return t_base_gripper;
}

/////////////////////////////////////////////////
std::string Interface::getRobotName(){
	return robot_name;
}

/////////////////////////////////////////////////
void Interface::setPose(ignition::math::Pose3d pose, double timeout){

	grasp::msgs::Hand msg;
	gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
	gazebo::msgs::Set(pose_msg, pose);
	msg.set_allocated_pose(pose_msg);

	if (timeout > 0)
		msg.set_timeout(timeout);

	pub->Publish(msg);
}

/////////////////////////////////////////////////
void Interface::reset(){

	grasp::msgs::Hand msg;
	msg.set_reset(true);
	pub->Publish(msg);

	std::vector<double> values;

	joints = {
		"virtual_px_joint", "virtual_py_joint", "virtual_pz_joint",
		"virtual_rr_joint", "virtual_rp_joint", "virtual_ry_joint"
	};

	values = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

	setJoints(joints, values);
	usleep(1000);
	openFingers();
}

/////////////////////////////////////////////////
void Interface::openFingers(double timeout, bool set_position){

	grasp::msgs::Hand msg;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;

	int i = 0;
	for (auto const & pair : grasp.pre){
		target = msg.add_pid_targets();
		target->set_type(POSITION);
		target->set_joint(pair.first);
		target->set_value(pair.second);
		actuation_pose[i++] = pair.second;   
	}

	for(size_t i = 0; i < FINGERS; i++){
		for(size_t j = 0; j < PHALANGES; j++){
			force_measured[i][j] = 0;
			contacts[i][j] = false;
		}
	}

	if (timeout > 0)
		msg.set_timeout(timeout);
	if (set_position)
		msg.set_force_target(true);

	pub->Publish(msg);

	turn_off_contact_sensors();
}

/////////////////////////////////////////////////
void Interface::closeFingers(double timeout, bool apply_force){

	grasp::msgs::Hand msg;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;

	int i = 0;
	if (apply_force){
		if (grasp.forces.empty())
			debugPrintTrace("No force values for closing fingers specified in config."); 

		for (auto const & pair : grasp.forces){
			target = msg.add_pid_targets();
			target->set_type(FORCE);
			target->set_joint(pair.first);
			target->set_value(pair.second); 
		}

	}else{
		for (auto const & pair : grasp.post){
			target = msg.add_pid_targets();
			target->set_type(POSITION);
			target->set_joint(pair.first);
			target->set_value(pair.second);
			actuation_pose[i++] = pair.second;
		}
	}


	if (timeout > 0)
		msg.set_timeout(timeout);
	pub->Publish(msg);
}

/////////////////////////////////////////////////
void Interface::raiseHand(double timeout){

	grasp::msgs::Hand msg;
	std::vector<std::string> virtual_joints;
	std::vector<double> values;

	// TODO - Read virtual joints from config file
	virtual_joints = {
		"virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
		"virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint"
	};
	values = {0,0,0.6,0,0,0};

	for (unsigned int i = 0; i < virtual_joints.size(); i++){
		grasp::msgs::Target *target = msg.add_pid_targets();
		target->set_type(POSITION);
		target->set_joint(virtual_joints.at(i));
		target->set_value(values.at(i));
	}

	if (timeout > 0)
		msg.set_timeout(timeout);
	pub->Publish(msg);
}

//////////////////////////////////////////////////
void Interface::loop(){

	bool loop = true;
	char key = -1;

	std::chrono::high_resolution_clock::time_point stop, start;

	start = std::chrono::high_resolution_clock::now();
	while (loop){
		// Keyboard input
		if (kbhit()){
			usleep(2000);
			key = getchar();
			if (key == KEY_EXIT)
				loop = false;
			else if (key == 'n')
				pose_noise = !pose_noise;
			else
				processKeypress(key);
		}

		stop = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span = stop - start;
		if(grasp_control && time_span.count() > T){
			//std::cout << time_span.count() << std::endl;
			start = std::chrono::high_resolution_clock::now();
			control();
		}
	}

}

//////////////////////////////////////////////////
int Interface::processKeypress(char key){

	float translation_value = 0.005;
	float rotation_value = 0.25;

	switch (key){
		// Translate base
		case 'w':
			moveJoint("virtual_px_joint",  translation_value); break;
		case 's':
			moveJoint("virtual_px_joint", -translation_value); break;
		case 'a':
			moveJoint("virtual_py_joint",  translation_value); break;
		case 'd':
			moveJoint("virtual_py_joint", -translation_value); break;
		case 'e':
			moveJoint("virtual_pz_joint",  translation_value); break;
		case 'q':
			moveJoint("virtual_pz_joint", -translation_value); break;

		// Rotate base
		case 'i':
			moveJoint("virtual_rr_joint",  rotation_value); break;
		case 'k':
			moveJoint("virtual_rr_joint", -rotation_value); break;
		case 'j':
			moveJoint("virtual_rp_joint",  rotation_value); break;
		case 'l':
			moveJoint("virtual_rp_joint", -rotation_value); break;
		case 'u':
			moveJoint("virtual_ry_joint",  rotation_value); break;
		case 'o':
			moveJoint("virtual_ry_joint", -rotation_value); break;

		// Move fingers
		case 'r':
			openFingers(); break;
		case 'f':
			closeFingers(); break;
		case 'b':
			raiseHand(); break;
		case '\n':
			reset(); break;
		case 'g':
			grasp_init(); break;
		case 't':
			for(int i = 0; i < FINGERS - 1; ++i)
				if(expected_output[i] > 0.0)
					expected_output[i] -= 0.1;
			std::cout << expected_output[0] << std::endl;
			break;
			//test_relation_force(); break;
		case 'p':
			test_force_control(); break;
		default:
			break;
	}
	return 0;
}

//////////////////////////////////////////////////
void Interface::moveJoint(const char *joint, double value){

	state[joint] += value;
	grasp::msgs::Hand msg;
	grasp::msgs::Target *target;

	target = msg.add_pid_targets();
	target->set_type(POSITION);
	target->set_joint(joint);
	target->set_value(state[joint]);

	pub->Publish(msg);

	std::cout << joint << ": " << value << ", " << state[joint] << std::endl;
	gzdbg << "message" << std::endl;
}

//////////////////////////////////////////////////
void Interface::setJoints(std::vector<std::string> & joints, std::vector<double> & values){

	if (joints.size() == values.size()){

		grasp::msgs::Hand msg;
		for (unsigned int i = 0; i < joints.size(); i++){

			const char *joint = joints.at(i).c_str();
			state[joint] = values.at(i);
			grasp::msgs::Target *target = msg.add_pid_targets();
			target->set_type(POSITION);
			target->set_joint(joint);
			target->set_value(values.at(i));
		}
		pub->Publish(msg);
	}
}

/////////////////////////////////////////////////
void Interface::read_values(ContactPtr & _msg){

	std::string fingers_id[FINGERS] = {"thumb", "ind", "med", "min"};
	std::string phalanges_id[PHALANGES] = {"1", "2", "3"};

	mtx.lock();
	for(int i = 0; i < FINGERS; ++i){
		if(strstr(_msg->name().c_str(), fingers_id[i].c_str())){
			for(int j = 0; j < PHALANGES; ++j){
				if(strstr(_msg->name().c_str(), phalanges_id[j].c_str())){
					contacts[i][j] = true;
					force_measured[i][j] = sqrt( pow(_msg->xforce(),2) + pow(_msg->yforce(),2) + pow(_msg->zforce(),2) );
					break;
				}
			}
			break;
		}
	}
	mtx.unlock();
}

/////////////////////////////////////////////////
void Interface::control(){

	grasp::msgs::Hand msg;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;

	float torque;

	int i = 0;
	if(missing_contacts){
		missing_contacts = false;
		for(auto const & pair : grasp.post){
			//if(!( (contacts[i][0] || contacts[i][1] || contacts[i][2]) || ((i == MED) && (contacts[MIN][0] || contacts[MIN][1] || contacts[MIN][2])) ) && actuation_pose[i] <= pair.second){
			if(!( contacts[i][2] || ((i == MED) && contacts[MIN][2]) ) && actuation_pose[i] <= pair.second && expected_output[i] != 0){
				missing_contacts = true;
				target = msg.add_pid_targets();
				target->set_type(POSITION);
				target->set_joint(pair.first);
				actuation_pose[i] += pair.second/400*(1 + (i == THUMB));
				target->set_value(actuation_pose[i]);
			}
			else{
				target = msg.add_pid_targets();
				target->set_type(POSITION);
				target->set_joint(pair.first);
				target->set_value(actuation_pose[i]);
			}
			i++;
		}
		if(!missing_contacts)
			msg.set_interruptor(true);
		pub->Publish(msg);
		usleep(10000);
	}
	else{
		if (warning){
			std::cout << "Force control." << std::endl;
			warning = false;
			msg.set_interruptor(false);
		}

		for(auto const & pair : grasp.forces){
			if(expected_output[i] <= 0.0)
				continue;
			target = msg.add_pid_targets();
			target->set_type(FORCE);
			target->set_joint(pair.first);

			// Getting samples
			for(int p = 0; p < PHALANGES; p++){
				mean_[i][p] -= saved_samples[i][p][index_]/mean_samples_;
				mtx.lock();
				saved_samples[i][p][index_] = force_measured[i][p];
				mtx.unlock();
				mean_[i][p] += saved_samples[i][p][index_]/mean_samples_;

				/////////////////// MEDIAN ///////////////////////////////////////////
				if(true){
					float y_[mean_samples_]{};
					std::copy(saved_samples[i][p], saved_samples[i][p] + mean_samples_, y_);
					std::sort(y_, y_ + mean_samples_); 

					if (mean_samples_ % 2 != 0) 
					    mean_[i][2] = y_[mean_samples_ / 2]; 
					else
						mean_[i][2] = (y_[(mean_samples_ - 1) / 2] + y_[mean_samples_ / 2]) / 2.0;
				}
				//////////////////////////////////////////////////////////////////////
			}

			//std::cout << "Joint: " << pair.first << std::endl;
			//std::cout << "Forces measured: P - " <<  force_measured[i][2] << " M - " << force_measured[i][1] << " B - " << force_measured[i][0] << std::endl;

			//if(contacts[i][2]){
				torque = fingers_array[i].calc(expected_output[i], mean_[i][2], 0);

				//std::cout << i << ": " << expected_output[i] << ", " << mean_[i][2] << ", " << torque << std::endl;

				//force_measured[i][2] = 0;
				//contacts[i][2] = false;
			//}
			/*else if(contacts[i][1]){
				force_to_apply = fingers_array[i].calc(expected_output[i], mean_[i][1], pair.second/400);
				force_measured[i][1] = 0;
				contacts[i][1] = false;
			}
			else if(contacts[i][0]){
				force_to_apply = fingers_array[i].calc(expected_output[i], mean_[i][0], pair.second/400);
				force_measured[i][0] = 0;
				contacts[i][0] = false;
			}
			else if(i == MED){
				if(contacts[MIN][2]){
					force_to_apply = fingers_array[MED].calc(expected_output[i], mean_[MIN][2], pair.second/400);
					force_measured[MIN][2] = 0;
					contacts[MIN][2] = false;	
				}
				else if(contacts[MIN][1]){
					force_to_apply = fingers_array[MED].calc(expected_output[i], mean_[MIN][1], pair.second/400);
					force_measured[MIN][1] = 0;
					contacts[MIN][1] = false;	
				}
				else if(contacts[MIN][0]){
					force_to_apply = fingers_array[MED].calc(expected_output[i], mean_[MIN][0], pair.second/400);
					force_measured[MIN][0] = 0;
					contacts[MIN][0] = false;	
				}
			}*/

			target->set_value(torque);
			target->set_reference(expected_output[i]);
			i++;				
		}
		if(pose_noise){
			int possible_values = 101; // has to be impar
			int offset = -floor(possible_values/2);
			float factor = 2000;

			joints = {
				"virtual_px_joint", "virtual_py_joint", "virtual_pz_joint",
				"virtual_rr_joint", "virtual_rp_joint", "virtual_ry_joint"
			};

			for(auto joint : joints){
				state[joint] += (rand() % possible_values + offset)/factor;
				grasp::msgs::Target *target;

				target = msg.add_pid_targets();
				target->set_type(POSITION);
				target->set_joint(joint);
				target->set_value(state[joint]);
			}
		}

		pub->Publish(msg);
	}
	check_contact_index_++;
	index_++;
	if(index_ == mean_samples_) index_ = 0;
	if(check_contact_index_ == mean_samples_) check_contact_index_ = 0;
}

/////////////////////////////////////////////////
void Interface::grasp_init(){
	
	//PID constants
	float kp = 0.3;
	float ki = 40;
	float kd = 0.002;
	float a = 10;

	float factor = 0.6;

	kp = kp*factor;
	ki = ki*factor;
	kd = kd*factor;

	fingers_array[THUMB] = pid(kp, ki, kd, T, a);
	fingers_array[IND] = pid(kp, ki, kd, T, a);
	fingers_array[MED] = pid(kp, ki, kd, T, a); // medium and minimum finger share the same motor

	turn_on_contact_sensors();
	grasp_control = true;
	missing_contacts = true;
}

/////////////////////////////////////////////////
void Interface::test_relation_position(){

	grasp::msgs::Hand msg;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;
	grasp::msgs::Target *target_finger;

	// Create and open a text file
	std::ofstream MyFile("Dados2020_10_29/position/dados.csv");

	int finger = IND;
	float limit;

	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(grasp_control){
		std::cout << "Fail to test" << std::endl;
		return;
	}
	turn_on_contact_sensors();
	////////////////////////////////////////////////////////////


	int i = 0;
	for(auto const & pair : grasp.post){

		target = msg.add_pid_targets();
		target->set_type(POSITION);
		target->set_joint(pair.first);
		target->set_value(actuation_pose[i]); 

		if(finger == i || (finger == MIN && i == MED) ){
			target_finger = target;
			limit = pair.second;
		}
		i++;
	}

  	MyFile <<  "Position,Force" << std::endl;
	while(actuation_pose[finger] <= limit){

		if(contacts[finger][2]){

			// Write to the file
  			MyFile << actuation_pose[finger] << "," << force_measured[finger][2] << std::endl;
  			std::cout << actuation_pose[finger] << ", " << force_measured[finger][2] << std::endl;
		}

		actuation_pose[finger] += limit/400;
		target_finger->set_value(actuation_pose[finger]);
		pub->Publish(msg);
		usleep(100000);
	}
	
	// Reset the hand position
	contacts[finger][2] = false;
	force_measured[finger][2] = 0;
	actuation_pose[finger] = 0;
	target_finger->set_value(actuation_pose[finger]);
	pub->Publish(msg);

	// Close the file
	MyFile.close();

	turn_off_contact_sensors();
}

/////////////////////////////////////////////////
void Interface::test_relation_velocity(){

	grasp::msgs::Hand msg;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;
	grasp::msgs::Target *target_finger;

	// Create and open a text file
	std::ofstream MyFile("Dados2020_10_29/velocity/dados.csv");

	int finger = IND;
	float limit = 3.68; // rad/s 3 joints with 1, 0.5 and 0.5 of multiplier
	float velocity = 0;

	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(grasp_control){
		std::cout << "Fail to test" << std::endl;
		return;
	}
	turn_on_contact_sensors();
	////////////////////////////////////////////////////////////


	int i = 0;
	for(auto const & pair : grasp.post){

		target = msg.add_pid_targets();
		target->set_type(VELOCITY);
		target->set_joint(pair.first);
		target->set_value(actuation_pose[i]); 

		if(finger == i || (finger == MIN && i == MED) ){
			target_finger = target;
		}
		i++;
	}

  	MyFile <<  "Velocity,Force" << std::endl;
	while(velocity <= limit){

		if(contacts[finger][2]){

			// Write to the file
  			MyFile << velocity << "," << force_measured[finger][2] << std::endl;
		}
  			std::cout << velocity << ", " << force_measured[finger][2] << std::endl;

		velocity += limit/400;
		target_finger->set_value(velocity);
		pub->Publish(msg);
		usleep(1000000);
	}
	
	// Reset the hand position
	contacts[finger][2] = false;
	force_measured[finger][2] = 0;
	target_finger->set_value(0);
	pub->Publish(msg);

	// Close the file
	MyFile.close();

	turn_off_contact_sensors();
}

/////////////////////////////////////////////////
void Interface::test_relation_force(){

	grasp::msgs::Hand msg;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;
	grasp::msgs::Target *target_finger;

	// Create and open a text file
	std::ofstream MyFile("Dados2020_12_25/dados.csv");

	int finger = IND;
	float force = 0;
	float limit = 1.2; // N.m

	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(grasp_control){
		std::cout << "Fail to test" << std::endl;
		return;
	}
	turn_on_contact_sensors();
	////////////////////////////////////////////////////////////

	int i = 0;
	for(auto const & pair : grasp.post){

		target = msg.add_pid_targets();
		target->set_type(FORCE);
		target->set_joint(pair.first);
		target->set_value(actuation_pose[i]);

		if(finger == i || (finger == MIN && i == MED) ){
			target_finger = target;
		}
		i++;
	}

	msg.set_interruptor(true);
	pub->Publish(msg);
	msg.clear_reset();

  	MyFile <<  "Input Force,Measured Force" << std::endl;
	while(force <= limit){

		if(contacts[finger][2]){
			// Write to the file
  			MyFile << force << "," << force_measured[finger][2] << std::endl;
  			std::cout << force << ", " << force_measured[finger][2] << std::endl;
		}

		force += limit/400;
		target_finger->set_value(force); // three joints
		msg.set_interruptor(false);
		pub->Publish(msg);
		usleep(100000);
	}
	
	// Reset the hand position
	contacts[finger][2] = false;
	force_measured[finger][2] = 0;
	target_finger->set_value(0);
	pub->Publish(msg);

	// Close the file
	MyFile.close();

	turn_off_contact_sensors();
}

/////////////////////////////////////////////////
void Interface::turn_on_contact_sensors(){

	usleep(100000);
	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(!grasp_control){
		grasp::msgs::ContactRequest msg;
		grasp::msgs::CollisionRequest *pair = msg.add_collision();
		pair->set_collision1("H");
		pub_call->Publish(msg);
	}
	////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////
void Interface::turn_off_contact_sensors(){

	usleep(100000);
	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(grasp_control){
		grasp::msgs::ContactRequest msg;
		grasp::msgs::CollisionRequest *pair = msg.add_collision();
		pair->set_collision1("H");
		pub_call->Publish(msg);
		grasp_control = false;
		warning = false;
	}
	////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////
void Interface::test_force_control(){

	grasp::msgs::Hand msg_pose, msg_force;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;
	grasp::msgs::Target *target_finger;

	int finger = IND;
	float limit;

	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(grasp_control){
		std::cout << "Fail to test" << std::endl;
		return;
	}
	turn_on_contact_sensors();
	////////////////////////////////////////////////////////////

	float force_reference = 1.0;

	//PID constants
	float kp = 0.3;
	float ki = 40;
	float kd = 0.002;
	float a = 10;

	pid fingers_pid = pid(kp, ki, kd, T, a);

	// Create and open a text file
	std::ofstream MyFile("Dados2021_02_09/dados.csv");
	MyFile << "Time,Force,Torque,MF,ref" << std::endl;

	int i = 0;
	for(auto const & pair : grasp.post){

		target = msg_pose.add_pid_targets();
		target->set_type(POSITION);
		target->set_joint(pair.first);
		target->set_value(actuation_pose[i]); 

		if(finger == i || (finger == MIN && i == MED) ){
			limit = pair.second;
			target_finger = target;
		}
		i++;
	}

	msg_pose.set_measure(false);

	while(!contacts[finger][2]){
		actuation_pose[finger] += limit/400;
		target_finger->set_value(actuation_pose[finger]);
		pub->Publish(msg_pose);
		msg_pose.set_measure(false);
		usleep(10000); // 10 ms
	}

	i = 0;
	for(auto const & pair : grasp.forces){

		target = msg_force.add_pid_targets();
		target->set_type(FORCE);
		target->set_joint(pair.first);
		target->set_value(0); 

		if(finger == i || (finger == MIN && i == MED) ){
			limit = pair.second;
			target_finger = target;
		}
		i++;
	}

	bool loop = true;
	char key = -1;

	float torque = 0;

	int mean_samples = 20;
	float y[mean_samples]{};
	int index = 0;
	float mean = 0;
		
	std::chrono::high_resolution_clock::time_point stop, start, abs_start;
	start = std::chrono::high_resolution_clock::now();
	abs_start = std::chrono::high_resolution_clock::now();

	float tau = 0.010;
	float sample;
	float initial_force = 0.0;
	float expected_force = initial_force;

	bool first = true;
	int last_time = 1;

	while(loop){

		if (kbhit()){
			key = getchar();
			if (key == KEY_EXIT)
				loop = false;
		}

		stop = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span = stop - start;

		if(time_span.count() < T)
			continue;

		start = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double> abs_time_span = stop - abs_start;
		expected_force = force_reference;// - (force_reference - initial_force)*exp(-abs_time_span.count()/tau);

		/*mtx.lock();
		sample = force_measured[finger][2];
		mtx.unlock(); */

		mean -= y[index]/mean_samples;
		mtx.lock();
		y[index] = force_measured[finger][2];
		mtx.unlock();
		mean += y[index]/mean_samples;


		/////////////////// MEDIAN ///////////////////////////////////////////
		if(true){
			float y_[mean_samples]{};
			std::copy(y, y + mean_samples, y_);
			std::sort(y_, y_ + mean_samples); 

			if (mean_samples % 2 != 0) 
			    mean = y_[mean_samples / 2]; 
			else
				mean = (y_[(mean_samples - 1) / 2] + y_[mean_samples / 2]) / 2.0;
		}
		//////////////////////////////////////////////////////////////////////

		if(int(abs_time_span.count()) > last_time){
			last_time = int(abs_time_span.count());
			if(int(abs_time_span.count())%2 == 0)
				force_reference -= 0.3;
			else
				force_reference += 0.3;
		}
		
		torque = fingers_pid.calc(expected_force, mean, 0);
		//torque = (int(abs_time_span.count())%3)*1.2/2;

		target_finger->set_value(torque);
		pub->Publish(msg_force);

		std::cout << abs_time_span.count() << ", " << expected_force << ", " << mean << ", " << torque << std::endl;
		MyFile << abs_time_span.count() << "," << mean << "," << torque << ", " << y[index++] << ", " << expected_force << std::endl;

		if(index == mean_samples) index = 0;

		if(first){
			first = false;
			msg_force.set_interruptor(true);
			msg_force.set_measure(false);
		}
		else{
			msg_force.set_interruptor(false);
			msg_force.set_measure(false);
		}
	}
	
	// Close the file
	MyFile.close();

	// Reset the hand position
	contacts[finger][2] = false;
	force_measured[finger][2] = 0;
	actuation_pose[finger] = 0;
	target_finger->set_value(actuation_pose[finger]);
	usleep(1000);
	pub->Publish(msg_pose);

	turn_off_contact_sensors();
}

/////////////////////////////////////////////////
void Interface::test_response_time(){

	grasp::msgs::Hand msg_pose, msg_force;
	// TODO allow grasp to be chosen
	GraspShape grasp = grasps.back();

	grasp::msgs::Target *target;
	grasp::msgs::Target *target_finger;

	// Create and open a text file
	std::ofstream MyFile("Dados2020_11_03/dados.csv");
  	MyFile << "Time,Force" << std::endl;

	int finger = IND;
	float limit;

	////////////////////////////////////////////////////////////
	// INIT CONTACT SENSORES ///////////////////////////////////
	////////////////////////////////////////////////////////////
	if(grasp_control){
		std::cout << "Fail to test" << std::endl;
		return;
	}
	turn_on_contact_sensors();
	////////////////////////////////////////////////////////////
	std::chrono::high_resolution_clock::time_point stop, start, abs_start;

	int i = 0;
	for(auto const & pair : grasp.post){
		target = msg_pose.add_pid_targets();
		target->set_type(POSITION);
		target->set_joint(pair.first);
		target->set_value(actuation_pose[i]); 

		if(finger == i || (finger == MIN && i == MED) ){
			target_finger = target;
			limit = pair.second;
		}
		i++;
	}

	while(!contacts[finger][2]){
		actuation_pose[finger] += limit/400;
		target_finger->set_value(actuation_pose[finger]);
		pub->Publish(msg_pose);
		usleep(1000);
	}


	i = 0;
	for(auto const & pair : grasp.forces){
		target = msg_force.add_pid_targets();
		target->set_type(FORCE);
		target->set_joint(pair.first);
		target->set_value(1.2*(i++==finger)); 
	}
	msg_pose.set_interruptor(true);
	pub->Publish(msg_force);

	start = std::chrono::high_resolution_clock::now();
	stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_span = stop - start;
	while(time_span.count() < 0.01){
		MyFile << time_span.count() << "," << force_measured[finger][2] << std::endl;
		std::cout << time_span.count() << ", " << force_measured[finger][2] << std::endl;
		stop = std::chrono::high_resolution_clock::now();
		time_span = stop - start;
		usleep(100);
	}

	// Reset the hand position
	contacts[finger][2] = false;
	force_measured[finger][2] = 0;

	// Close the file
	MyFile.close();

	turn_off_contact_sensors();
	openFingers();
}

