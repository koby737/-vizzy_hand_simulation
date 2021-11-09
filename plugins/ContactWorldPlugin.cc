/*!
    \file plugins/ContactWorldPlugin.cc
    \brief Contact manager Gazebo world plugin

    \author João Borrego : jsbruglie
*/

#include "ContactWorldPlugin.hh"

namespace gazebo {


/// \brief Class for private Target plugin data.
class ContactWorldPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo request topic subscriber
    public: transport::SubscriberPtr sub_req;
    /// Gazebo contacts topic subscriber
    public: transport::SubscriberPtr sub_con;
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ContactWorldPlugin)

/////////////////////////////////////////////////
ContactWorldPlugin::ContactWorldPlugin() : WorldPlugin(),
    data_ptr(new ContactWorldPluginPrivate)
{
    gzmsg << "[ContactWorldPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
ContactWorldPlugin::~ContactWorldPlugin()
{
    gzmsg << "[ContactWorldPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void ContactWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    world = _world;

    // Communications - TODO
    std::string req_topic = REQUEST_TOPIC;

    // Setup transport node
    data_ptr->node = transport::NodePtr(new transport::Node());
    data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    data_ptr->sub_req = data_ptr->node->Subscribe(req_topic, &ContactWorldPlugin::onRequest, this);
    // Publish to the response topic
    data_ptr->pub = data_ptr->node->Advertise<grasp::msgs::Contact>("~/my_contact");

	manager = world->Physics()->GetContactManager();
	// world->Physics()->SetTargetRealTimeFactor(1.0);
	// gzmsg << world->Physics()->GetTargetRealTimeFactor() << std::endl;

    gzmsg << "[ContactWorldPlugin] Loaded plugin." << std::endl;
}


/////////////////////////////////////////////////
void ContactWorldPlugin::onContact(ConstContactsPtr & _msg)
{
	std::lock_guard<std::mutex> lock(data_ptr->mutex);
        // Check for collisions between entities

	grasp::msgs::Contact msg;

	for (int i = 0; i < manager->GetContactCount(); i++){

		physics::Contact *contact = manager->GetContact(i);
		std::string tmp_col1(contact->collision1->GetScopedName());
		std::string tmp_col2(contact->collision2->GetScopedName());

		if ( !(strstr(tmp_col1.c_str(), col.c_str()) || strstr(tmp_col2.c_str(), col.c_str()) ) )
			continue;

		// Pay attention to this part
		pose = contact->collision2->GetLink()->WorldPose();

		//gzdbg << "Contact: " << tmp_col1 << " and " << tmp_col2 << " with " << contact->count << " contacts." << std::endl;

		// no specific frames specified, use identity pose, keeping
		// relative frame at inertial origin
		// frame_pos = ignition::math::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
		frame_rot = ignition::math::Quaterniond(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());  // gazebo u,x,y,z == identity

		float total_wrench_force_x = 0;
		float total_wrench_force_y = 0;
		float total_wrench_force_z = 0;
		float total_wrench_torque_x = 0;
		float total_wrench_torque_y = 0;
		float total_wrench_torque_z = 0;
 
		for(int j = 0; j < contact->count; ++j){

			// Force
			ignition::math::Vector3d force = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
					    contact->wrench[j].body1Force[0],
					    contact->wrench[j].body1Force[1],
					    contact->wrench[j].body1Force[2]));
			total_wrench_force_x += force[0];
			total_wrench_force_y += force[1];
			total_wrench_force_z += force[2];

			/*if( sqrt( pow(force[0],2) + pow(force[1],2) + pow(force[2],2) ) > sqrt( pow(total_wrench_force_x,2) + pow(total_wrench_force_y,2) + pow(total_wrench_force_z,2) ) ){

				total_wrench_force_x = force[0];
				total_wrench_force_y = force[1];
				total_wrench_force_z = force[2];

			}*/

			// Torque
			/*ignition::math::Vector3d torque = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
					    contact->wrench[j].body1Torque[0],
					    contact->wrench[j].body1Torque[1],
					    contact->wrench[j].body1Torque[2]));
			total_wrench_torque_x += torque[0];
			total_wrench_torque_y += torque[1];
			total_wrench_torque_z += torque[2];*/

			/*position = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
						   contact->positions[j][0],
						   contact->positions[j][1],
						   contact->positions[j][2]) - frame_pos);
			normal = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
						   contact->normals[j][0],
						   contact->normals[j][1],
						   contact->normals[j][2]));*/
		}
		/*gzdbg << "Position: " << position << std::endl;
		gzdbg << "Normal: " << normal << "\n" << std::endl;*/
		/*gzdbg << "Force: " << total_wrench_force_x << " "
				   << total_wrench_force_y << " "
				   << total_wrench_force_z << " "
				   << "\n\n" << std::endl;*/
		/*gzdbg << "Torque: " << total_wrench_torque_x << " "
				    << total_wrench_torque_y << " "
				    << total_wrench_torque_z << " "
				    << "\n\n" << std::endl;*/

		/*float depth = 0;
		for(int i = 0; i < contact->count; ++i)
			depth += contact->depths[i];*/

		double limit = 3;

		if(total_wrench_force_x > limit)
			total_wrench_force_x = limit;
		else if(total_wrench_force_x < -limit)
			total_wrench_force_x = -limit;

		if(total_wrench_force_y > limit)
			total_wrench_force_y = limit;
		else if(total_wrench_force_y < -limit)
			total_wrench_force_y = -limit;

		if(total_wrench_force_z > limit)
			total_wrench_force_z = limit;
		else if(total_wrench_force_z < -limit)
			total_wrench_force_z = -limit;


		msg.set_name(tmp_col2);
		msg.set_xforce(total_wrench_force_x);
		msg.set_yforce(total_wrench_force_y);
		msg.set_zforce(total_wrench_force_z);
		data_ptr->pub->Publish(msg);

	}

}

/////////////////////////////////////////////////
void ContactWorldPlugin::onRequest(ContactRequestPtr & _msg)
{

	std::lock_guard<std::mutex> lock(data_ptr->mutex);
	gzdbg << "Message received" << std::endl;
	
	if(msg_flag){
		gzdbg << "Contact sensors turned off" << std::endl;
		data_ptr->sub_con.reset();
		msg_flag = false;
	}
	else{
		gzdbg << "Contact sensors turned on" << std::endl;
		// Subcribe to the monitored contacts topic
		data_ptr->sub_con = data_ptr->node->Subscribe("~/physics/contacts", &ContactWorldPlugin::onContact, this);
		msg_flag = true;
	}

}


}
