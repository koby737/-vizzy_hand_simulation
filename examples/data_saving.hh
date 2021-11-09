/*!
    \file utils/Interface.hh
    \brief Grasp interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _DATA_SAVING_
#define _DATA_SAVING_

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>

// Custom messages
#include "target.pb.h"
#include "hand.pb.h"
#include "my_msg.pb.h"

// DEBUG
#include <iostream>
// SQRT
#include <math.h>
// TIME
#include <chrono>
// MUTEX
#include <mutex>
// FILES
#include <fstream>

#define KEY_EXIT 'e'
#define FINGERS 4
#define PHALANGES 3
#define THUMB 0
#define IND 1
#define MED 2
#define MIN 3

/// Topic for incoming requests
#define REQUEST_TOPIC    "~/hand"
/// \brief Hand interface

/// Apply force/torque directly
#define FORCE grasp::msgs::Target::FORCE

class Data{

    typedef const boost::shared_ptr<const grasp::msgs::Contact> ContactPtr;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::Hand> HandMsgPtr;
    /// Gazebo communication node
    private: gazebo::transport::NodePtr node;
    /// Gazebo publisher
    private: gazebo::transport::SubscriberPtr sub, sub_t;

		bool contacts[FINGERS][PHALANGES]{};
		bool joints[FINGERS]{};
		double force_measured[FINGERS][PHALANGES]{};

		std::chrono::high_resolution_clock::time_point time_of_last_sample[FINGERS][PHALANGES]{};

		double actuation_pose[FINGERS]{};

        std::string fingers_id[FINGERS] = {"thumb", "ind", "med", "min"};
        std::string phalanges_id[PHALANGES] = {"1", "2", "3"};

        // Create and open a text file
        std::ofstream my_files[FINGERS][PHALANGES];
        std::ofstream joint_files[FINGERS-1];

        std::chrono::high_resolution_clock::time_point stop, start;

        bool first_read = true;

        int mean_samples_ = 20;
        float saved_samples[FINGERS][PHALANGES][20]{};
        int index_[FINGERS][PHALANGES]{};
        float mean_[FINGERS][PHALANGES]{};

	double reference = 0;

    /// \brief Constructor
    public: Data();
        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(HandMsgPtr & _msg);
	//TODO
	public: void read_values(ContactPtr & _msg);
	//TODO
	public: void read_torque(HandMsgPtr & _msg);

    //TODO
    public: void test_relation();

    //TODO
    public: void open_files();

    //TODO
    public: void close_files();

    //TODO
    public: void loop();
};

int kbhit();

#endif
