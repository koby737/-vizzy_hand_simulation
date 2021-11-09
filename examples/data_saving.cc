/*!
    \file utils/Data.cc
    \brief Grasp interface

    \author João Borrego : jsbruglie
*/

#include "data_saving.hh"


int main(int _argc, char **_argv){

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Data
    Data api;

    api.open_files();

    // Main loop - Keyboard input
    api.loop();

    api.close_files();

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

//////////////////////////////////////////////////
Data::Data(){

	node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	node->Init();
	// passar a sub pub = node->Advertise<grasp::msgs::Hand>(HAND_PLUGIN_TOPIC);
	sub = node->Subscribe("~/my_contact", &Data::read_values, this);
    	sub_t = node->Subscribe("~/hand", &Data::read_torque, this);
	// Subcribe to the monitored requests topic
	// sub = node->Subscribe(REQUEST_TOPIC, &Data::onRequest, this);
}

/////////////////////////////////////////////////
void Data::read_values(ContactPtr & _msg){

	if(first_read){
		start = std::chrono::high_resolution_clock::now();
		first_read = false;
	}

	stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_span = stop - start;

	// Getting samples
	for(int i = 0; i < FINGERS; ++i){
		if(strstr(_msg->name().c_str(), fingers_id[i].c_str())){
			for(int j = 0; j < PHALANGES; ++j){
				if(strstr(_msg->name().c_str(), phalanges_id[j].c_str())){
					if(joints[i] == false || j != 2) break;
					if(contacts[i][j] == false)
						my_files[i][j] << "Time,Force" << std::endl;
					contacts[i][j] = true;
					/*if(i == 0){
						std::cout << _msg->name().c_str() << std::endl;
						std::cout << _msg->xforce() << ", " << _msg->yforce() << ", " << _msg->zforce() << std::endl;
						std::cout << pow(_msg->xforce(),2) << ", " << pow(_msg->yforce(),2) << ", " << pow(_msg->zforce(),2) << std::endl;
						std::cout << sqrt( pow(_msg->xforce(),2) + pow(_msg->yforce(),2) + pow(_msg->zforce(),2) ) << "\n" << std::endl;  
					}*/

					mean_[i][j] -= saved_samples[i][j][index_[i][j]]/mean_samples_;
					saved_samples[i][j][index_[i][j]] = sqrt( pow(_msg->xforce(),2) + pow(_msg->yforce(),2) + pow(_msg->zforce(),2) );
					mean_[i][j] += saved_samples[i][j][index_[i][j]]/mean_samples_;
					time_of_last_sample[i][j] = std::chrono::high_resolution_clock::now();

					index_[i][j]++;
					if(index_[i][j] == mean_samples_)
						index_[i][j] = 0;

					my_files[i][j] << time_span.count() << "," << mean_[i][j] << std::endl;
					break;
				}
			}
			break;
		}
	}
}

/////////////////////////////////////////////////
void Data::read_torque(HandMsgPtr & _msg){

	stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_span = stop - start;


	if (_msg->pid_targets_size() > 0){
		int i = 0;
		for (const auto target : _msg->pid_targets()){
			int type = target.type();
			if(type != FORCE){
				stop = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> time_span = stop - start;
				std::cout << time_span.count() << std::endl;
				break;
			}

			std::string name = target.joint();
			double value = target.value();
			reference = target.reference();

			// Getting samples
			if(joints[i] == false)
				joint_files[i] << "Time,Torque,Reference" << std::endl;
			joints[i] = true;

			joint_files[i++] << time_span.count() << "," << value << "," << reference << std::endl;
		
		}
		joints[3] = joints[2];
	}


}

/////////////////////////////////////////////////
void Data::onRequest(HandMsgPtr &_msg)
{

	for (const auto target : _msg->pid_targets())
		std::cout << target.type() << std::endl;
}


/////////////////////////////////////////////////
void Data::open_files(){

	for(int i = 0; i < FINGERS; ++i){
		for(int j = 0; j < PHALANGES; ++j){
			my_files[i][j].open("Dados2021_02_22/" + fingers_id[i] + "_" + phalanges_id[j] + ".csv");
		}
	}

	for(int i = 0; i < FINGERS - 1; ++i){
		joint_files[i].open("Dados2021_02_22/" + fingers_id[i] + ".csv");
	}

}


/////////////////////////////////////////////////
void Data::close_files(){

	for(int i = 0; i < FINGERS; ++i){
		for(int j = 0; j < PHALANGES; ++j){
			my_files[i][j].close();
		}
	}

	for(int i = 0; i < FINGERS - 1; ++i){
		joint_files[i].close();
	}

}


//////////////////////////////////////////////////
void Data::loop(){

	bool loop = true;
	char key = -1;

	std::chrono::high_resolution_clock::time_point stop_loop, start_loop;

	start_loop = std::chrono::high_resolution_clock::now();

        std::ofstream file;
	file.open("Dados2021_02_22/file.csv");
	file << "Time,Force,Reference" << std::endl;

	double sum = 0;

	while (loop){
		// Keyboard input
		if (kbhit()){
			key = getchar();
			if (key == KEY_EXIT)
				loop = false;
		}

		stop_loop = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span_loop = stop_loop - start_loop;
		if(!first_read && time_span_loop.count() > 1.0/1000){
			start_loop = std::chrono::high_resolution_clock::now();

			stop = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> time_span = stop - start;

			for(int i = 0; i < FINGERS; ++i)
				for(int j = 0; j < PHALANGES; ++j){
					std::chrono::duration<double> time_span_ = stop - time_of_last_sample[i][j];
					if(time_span_.count() < 1)
						sum += mean_[i][j];
			}

			file << time_span.count() << "," << sum << "," << reference << std::endl;
			sum = 0;
		}
	}

	file.close();

}

//////////////////////////////////////////////////
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
