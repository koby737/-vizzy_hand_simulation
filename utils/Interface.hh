/*!
    \file utils/Interface.hh
    \brief Grasp interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _INTERFACE_HH_
#define _INTERFACE_HH_

#include <vector>
#include <map>
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>

// my libraries
#include "my_lib/pid.h"
// Custom utilities
#include "utils.hh"
// Debug streams
#include "debug.hh"
// Grasp shape representation
#include "GraspShape.hh"
// Custom messages
#include "target.pb.h"
#include "hand.pb.h"
#include "my_msg.pb.h"
#include "contact_request.pb.h"

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

#define FREQUENCY 1000.0 // Hz
#define T 1.0/FREQUENCY // seconds
#define FINGERS 4
#define PHALANGES 3
#define THUMB 0
#define IND 1
#define MED 2
#define MIN 3

/// Topic monitored by hand plugin for incoming requests
#define HAND_PLUGIN_TOPIC "~/hand"

// Key for exiting program (Esc)
#define KEY_EXIT 27

/// \brief Hand interface
class Interface{

    typedef const boost::shared_ptr<const grasp::msgs::Contact> ContactPtr;
    /// Direct force/torque application
    public: static const grasp::msgs::Target_Type FORCE = grasp::msgs::Target::FORCE;
    /// Position controller target
    public: static const grasp::msgs::Target_Type POSITION = grasp::msgs::Target::POSITION;
    /// Velocity controller target
    public: static const grasp::msgs::Target_Type VELOCITY = grasp::msgs::Target::VELOCITY;

    /// Name of robot instance in Gazebo
    private: std::string robot_name;
    /// Gazebo communication node
    private: gazebo::transport::NodePtr node;
    /// Gazebo publisher
    private: gazebo::transport::PublisherPtr pub;
	     gazebo::transport::PublisherPtr pub_call;
    /// Gazebo publisher
    private: gazebo::transport::SubscriberPtr sub;
    /// Name of actuated robot joints
    private: std::vector<std::string> joints;
    /// Map with (name: value) states of each actuated joint
    private: std::map<std::string, double> state;
    /// Set of grasp shapes
    private: std::vector<GraspShape> grasps;
    /// Rigid transform from base link to gripper frame
    private: ignition::math::Matrix4d t_base_gripper;

	private:
		bool grasp_control = false;

		pid* fingers_array = (pid*)malloc(sizeof(pid) * FINGERS);

		bool contacts[FINGERS][PHALANGES]{};
		double force_measured[FINGERS][PHALANGES]{};

		double actuation_pose[FINGERS]{};

		bool warning = true;
		bool missing_contacts = true;
        std::mutex mtx;

        int mean_samples_ = 20;
        float saved_samples[FINGERS][PHALANGES][20]{};
        int index_ = 0;
        // to check if still in contact (all phalanges)
        int check_contact_index_ = 0;
        float mean_[FINGERS][PHALANGES];

        bool pose_noise = false;

        float expected_output[FINGERS - 1] = {1.0, 1.0, 1.0};

    /// \brief Constructor
    public: Interface();

    /// \brief Initalizes interface with config file
    /// \return True on success, false otherwise.
    public: bool init(const std::string & config_file, const std::string & robot);

    // Getters

    /// \brief Gets robot name
    /// \return Robot name string
    public: std::string getRobotName();

    /// \brief Get T base to gripper frame
    /// \return Rigid transform from base link to gripper frame
    public: ignition::math::Matrix4d getTransformBaseGripper();

    // API

    /// \brief Sets hand pose
    /// \details Sets hand pose using virtual joints
    /// \param pose New hand pose
    /// \param timeout Timer value
    void setPose(ignition::math::Pose3d pose, double timeout=-1);

    /// \brief Sends reset signal to hand plugin.
    public: void reset(void);

    /// \brief Releases fingers and opens hand.
    /// \param timeout Timer value
    /// \param set_position Set joint position to target value
    void openFingers(double timeout=-1, bool set_position=false);

    /// \brief Clenches fingers and closes hand.
    /// \param timeout Timer value
    /// \param apply_force Wether to bypass PIDs and apply force/torque directly
    void closeFingers(double timeout=-1, bool apply_force=false);

    /// \brief Raises hand.
    /// \param timeout Timer value
    void raiseHand(double timeout=-1);

    /// TODO
    public: void loop(void);

    /// TODO
    private: int processKeypress(char key);

    /// TODO
    private: void moveJoint(const char *joint, double value);

    /// TODO
    private: void moveFingers(double value);

    /// TODO
    private: void setJoints(std::vector<std::string> & joints, std::vector<double> & values);

	//TODO
	public: void read_values(ContactPtr & _msg);

	//TODO
	public: void control();

	//TODO
	public: void grasp_init();

    //TODO
    public: void test_relation_position();

    //TODO
    public: void test_relation_velocity();

    //TODO
    public: void test_relation_force();

    //TODO
    public: void turn_on_contact_sensors();

    //TODO
    public: void turn_off_contact_sensors();

    //TODO
    public: void test_force_control();

    //TODO
    public: void test_response_time();
};

#endif
