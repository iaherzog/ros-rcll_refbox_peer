/***************************************************************************
 *  rcll_refbox_peer_node.cpp - RCLL Referee Box Peer as ROS node
 *
 *  Created: Fri May 27 21:38:32 2016
 *  Copyright  2016  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */


#include <ros/ros.h>

#include <protobuf_comm/peer.h>

#include <llsf_msgs/BeaconSignal.pb.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/RobotInfo.pb.h>

#include <rcll_ros_msgs/BeaconSignal.h>
#include <rcll_ros_msgs/Team.h>

#include <tf2/LinearMath/Quaternion.h>

#define GET_PRIV_PARAM(P)	  \
	{ \
		if (! ros::param::get("~" #P, cfg_ ## P)) { \
			ROS_ERROR("Failed to retrieve parameter " #P ", aborting"); \
			exit(-1); \
		} \
	}

using namespace protobuf_comm;
using namespace llsf_msgs;


ros::Publisher pub_beacon_;


void
handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
  ROS_WARN("Receive error from %s:%u: %s",
           endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}

void
handle_send_error(std::string msg)
{
  ROS_ERROR("Send error: %s", msg.c_str());
}

void
handle_message(boost::asio::ip::udp::endpoint &sender,
               uint16_t component_id, uint16_t msg_type,
               std::shared_ptr<google::protobuf::Message> msg)
{
	std::shared_ptr<BeaconSignal> b;
	if ((b = std::dynamic_pointer_cast<BeaconSignal>(msg))) {
		ROS_INFO("Received beacon signal from %s:%s", b->team_name().c_str(), b->peer_name().c_str());

		rcll_ros_msgs::BeaconSignal bs;
		bs.header.seq = b->seq();
		bs.header.stamp = ros::Time(b->time().sec(), b->time().nsec());
		bs.number = b->number();
		bs.team_name = b->team_name();
		bs.peer_name = b->peer_name();
		bs.team_color =
			(b->team_color() == llsf_msgs::CYAN) ? (int)rcll_ros_msgs::Team::CYAN : (int)rcll_ros_msgs::Team::MAGENTA;
		if (b->has_pose()) {
			bs.pose.header.frame_id = "/map";
			bs.pose.header.stamp = ros::Time(b->pose().timestamp().sec(), b->pose().timestamp().nsec());
			bs.pose.pose.position.x = b->pose().x();
			bs.pose.pose.position.y = b->pose().y();
			bs.pose.pose.position.z = 0.;
			tf2::Quaternion q;
			q.setRPY(0., 0., b->pose().ori());
			bs.pose.pose.orientation.x = q.x();
			bs.pose.pose.orientation.y = q.y();
			bs.pose.pose.orientation.z = q.z();
			bs.pose.pose.orientation.w = q.w();
		}
		pub_beacon_.publish(bs);
	}
}


int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rcll_refbox_peer");

	ros::NodeHandle n;

	// Parameter parsing	
	std::string  cfg_team_name;
	std::string  cfg_team_color;
	std::string  cfg_crypto_key;
	std::string  cfg_crypto_cipher;
	std::string  cfg_peer_address;
	bool         cfg_peer_public_local;
	int          cfg_peer_public_port;
	int          cfg_peer_public_send_port;
	int          cfg_peer_public_recv_port;
	bool         cfg_peer_private_local;
	int          cfg_peer_private_port;
	int          cfg_peer_private_send_port;
	int          cfg_peer_private_recv_port;

	GET_PRIV_PARAM(team_name);
	GET_PRIV_PARAM(team_color);

	if (cfg_team_color != "CYAN" && cfg_team_color != "MAGENTA") {
		ROS_ERROR("Invalid team color given, must be CYAN or MAGENTA");
		exit(-1);
	}

	GET_PRIV_PARAM(peer_address);
	
	if (ros::param::has("~peer_public_recv_port") && ros::param::has("~peer_public_send_port")) {
		cfg_peer_public_local = true;
		GET_PRIV_PARAM(peer_public_recv_port);
		GET_PRIV_PARAM(peer_public_send_port);
	} else {
		cfg_peer_public_local = false;
		GET_PRIV_PARAM(peer_public_port);
	}

	std::string cfg_pp_prefix =
		std::string("~peer_") + (cfg_team_color == "CYAN" ? "cyan" : "magenta") + "_";
	
	if (ros::param::has(cfg_pp_prefix+"recv_port") && ros::param::has(cfg_pp_prefix+"send_port")) {
		cfg_peer_private_local = true;
		if (! ros::param::get(cfg_pp_prefix+"recv_port", cfg_peer_private_recv_port)) {
			ROS_ERROR("Failed to retrieve parameter %s_recv_port, aborting", cfg_pp_prefix.c_str());
			exit(-1);
		}
		if (! ros::param::get(cfg_pp_prefix+"send_port", cfg_peer_private_send_port)) {
			ROS_ERROR("Failed to retrieve parameter %s_end_port, aborting", cfg_pp_prefix.c_str());
			exit(-1);
		}
	} else {
		cfg_peer_private_local = false;
		if (! ros::param::get(cfg_pp_prefix+"port", cfg_peer_private_port)) {
			ROS_ERROR("Failed to retrieve parameter %s_port, aborting", cfg_pp_prefix.c_str());
			exit(-1);
		}
	}

	GET_PRIV_PARAM(crypto_key);
	GET_PRIV_PARAM(crypto_cipher);

	// Setup ROS topics
	pub_beacon_ = n.advertise<rcll_ros_msgs::BeaconSignal>("rcll/beacon", 100);

	// Setup basic communication
  ProtobufBroadcastPeer *peer_public = NULL;
  ProtobufBroadcastPeer *peer_private = NULL;

  ROS_INFO("Creating public peer");
  if (cfg_peer_public_local) {
	  peer_public = new ProtobufBroadcastPeer(cfg_peer_address,
	                                          cfg_peer_public_send_port,
	                                          cfg_peer_public_recv_port);
  } else {
	  peer_public = new ProtobufBroadcastPeer(cfg_peer_address, cfg_peer_public_port);
  }

  MessageRegister & message_register = peer_public->message_register();
  message_register.add_message_type<llsf_msgs::BeaconSignal>();
  message_register.add_message_type<llsf_msgs::GameState>();
  message_register.add_message_type<llsf_msgs::MachineInfo>();
  message_register.add_message_type<llsf_msgs::RobotInfo>();

  ROS_INFO("Creating private peer");
  if (cfg_peer_private_local) {
	  peer_private = new ProtobufBroadcastPeer(cfg_peer_address,
	                                           cfg_peer_private_send_port,
	                                           cfg_peer_private_recv_port,
	                                           &message_register,
	                                           cfg_crypto_key, cfg_crypto_cipher);
  } else {
	  peer_private = new ProtobufBroadcastPeer(cfg_peer_address, cfg_peer_private_port,
	                                           &message_register,
	                                           cfg_crypto_key, cfg_crypto_cipher);
  }
  
  peer_public->signal_received().connect(handle_message);
  peer_public->signal_recv_error().connect(handle_recv_error);
  peer_public->signal_send_error().connect(handle_send_error);

  peer_private->signal_received().connect(handle_message);
  peer_private->signal_recv_error().connect(handle_recv_error);
  peer_private->signal_send_error().connect(handle_send_error);

  ros::spin();
	
	return 0;
}
