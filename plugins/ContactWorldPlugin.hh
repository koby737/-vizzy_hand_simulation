/*!
    \file plugins/ContactWorldPlugin.hh
    \brief Contact manager Gazebo world plugin

    \author João Borrego : jsbruglie
*/

#ifndef _CONTACT_WORLD_PLUGIN_HH_
#define _CONTACT_WORLD_PLUGIN_HH_

// Gazebo
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Custom messages
#include "contact_request.pb.h"
#include "my_msg.pb.h"

namespace ContactWorldPlugin {

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC   "~/grasp/contact"
}

namespace gazebo {

    /// Declaration for request message type
    typedef grasp::msgs::ContactRequest ContactRequest;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::ContactRequest>
        ContactRequestPtr;

    // Forward declaration of private data class
    class ContactWorldPluginPrivate;

    /// Contact manager world plugin
    class ContactWorldPlugin : public WorldPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<ContactWorldPluginPrivate> data_ptr;

        /// World pointer
        private: physics::WorldPtr world;

	private: bool msg_flag = false;

	private: 
		// get reference frame (body(link)) pose and subtract from it to get
		// relative force, torque, position and normal vectors
		ignition::math::Pose3d pose, frame_pose;
		ignition::math::Quaterniond rot, frame_rot;
		ignition::math::Vector3d pos, frame_pos;
		ignition::math::Vector3d position, normal;
		std::string message;

		std::string col = "vizzy_right_hand::r"; // only part with contact sensors are the fingers

	private: physics::ContactManager *manager;

        // Public methods

        /// \brief Constructs the object
        public: ContactWorldPlugin();

        /// \brief Destroys the object
        public: virtual ~ContactWorldPlugin();

        /// \brief Loads the plugin
        /// \param _world The world pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /// \brief Callback on contact message received event
        /// Stores message for later processing in update callback
        /// \param _msg Received message
        public: void onContact(ConstContactsPtr & _msg);

        /// \brief Callback function for handling incoming requests
        /// Subscribes to "~/physics/contacts" topic, which decreases performance.
        /// Incoming contact messages are monitored in onContact callback function.
        /// \param _msg  The message
        public: void onRequest(ContactRequestPtr & _msg);

    };
}

#endif
