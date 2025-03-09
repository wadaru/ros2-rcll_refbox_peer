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
#include <llsf_msgs/VersionInfo.pb.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/RobotInfo.pb.h>
#include <llsf_msgs/ExplorationInfo.pb.h>
#include <llsf_msgs/MachineReport.pb.h>
#include <llsf_msgs/OrderInfo.pb.h>
#include <llsf_msgs/RingInfo.pb.h>
#include <llsf_msgs/MachineInstructions.pb.h>
#include <llsf_msgs/NavigationChallenge.pb.h>
#include <llsf_msgs/AgentTask.pb.h>
#include <llsf_msgs/ProductColor.pb.h>

#include <rcll_ros_msgs/BeaconSignal.h>
#include <rcll_ros_msgs/GameState.h>
#include <rcll_ros_msgs/Team.h>
#include <rcll_ros_msgs/MachineInfo.h>
#include <rcll_ros_msgs/ExplorationInfo.h>
#include <rcll_ros_msgs/MachineReportInfo.h>
#include <rcll_ros_msgs/OrderInfo.h>
#include <rcll_ros_msgs/RingInfo.h>
#include <rcll_ros_msgs/NavigationRoutes.h>
#include <rcll_ros_msgs/Route.h>
#include <rcll_ros_msgs/AgentTask.h>
#include <rcll_ros_msgs/ProductColor.h>

#include <rcll_ros_msgs/SendBeaconSignal.h>
#include <rcll_ros_msgs/SendMachineReport.h>
#include <rcll_ros_msgs/SendMachineReportBTR.h>
#include <rcll_ros_msgs/SendPrepareMachine.h>
#include <rcll_ros_msgs/SendAgentTask.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <mutex>
#include <condition_variable>
#include <chrono>

#define GET_PRIV_PARAM(P)	  \
	{ \
		if (! ros::param::get("~" #P, cfg_ ## P ## _)) { \
			ROS_ERROR("Failed to retrieve parameter " #P ", aborting"); \
			exit(-1); \
		} \
	}

using namespace protobuf_comm;
using namespace llsf_msgs;

std::string  cfg_team_name_;
std::string  cfg_robot_name_;
int          cfg_robot_number_;
int          cfg_team_color_;
std::string  cfg_crypto_key_;
std::string  cfg_crypto_cipher_;
std::string  cfg_peer_address_;
bool         cfg_peer_public_local_;
int          cfg_peer_public_port_;
int          cfg_peer_public_send_port_;
int          cfg_peer_public_recv_port_;
bool         cfg_peer_cyan_local_;
int          cfg_peer_cyan_port_;
int          cfg_peer_cyan_send_port_;
int          cfg_peer_cyan_recv_port_;
bool         cfg_peer_magenta_local_;
int          cfg_peer_magenta_port_;
int          cfg_peer_magenta_send_port_;
int          cfg_peer_magenta_recv_port_;

ros::Publisher pub_beacon_;
ros::Publisher pub_game_state_;
ros::Publisher pub_machine_info_;
ros::Publisher pub_exploration_info_;
ros::Publisher pub_machine_report_info_;
ros::Publisher pub_order_info_;
ros::Publisher pub_ring_info_;
ros::Publisher pub_navigation_routes_info_;
ros::Publisher pub_agent_task_;

ros::ServiceServer srv_send_beacon_;
ros::ServiceServer srv_send_machine_report_;
ros::ServiceServer srv_send_prepare_machine_;
ros::ServiceServer srv_send_agent_task_;

ProtobufBroadcastPeer *peer_public_ = NULL;
ProtobufBroadcastPeer *peer_private_ = NULL;

std::mutex mtx_machine_info_;
std::condition_variable cdv_machine_info_;
std::shared_ptr<MachineInfo> msg_machine_info_;

void setup_private_peer(llsf_msgs::Team team_color);

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


int
pb_to_ros_team_color(const llsf_msgs::Team &team_color)
{
	if (team_color == llsf_msgs::CYAN)
		return (int)rcll_ros_msgs::Team::CYAN;
	else
		return (int)rcll_ros_msgs::Team::MAGENTA;
}

llsf_msgs::Team
ros_to_pb_team_color(int team_color)
{
	if (team_color == rcll_ros_msgs::Team::CYAN)
		return llsf_msgs::CYAN;
	else
		return llsf_msgs::MAGENTA;
}

void
handle_message(boost::asio::ip::udp::endpoint &sender,
               uint16_t component_id, uint16_t msg_type,
               std::shared_ptr<google::protobuf::Message> msg)
{
	{
		// This is only sent right after starting the refbox and not
		// terribly interesting, hence only log
		static bool version_info_printed = false;
		std::shared_ptr<VersionInfo> v;
		if (! version_info_printed && (v = std::dynamic_pointer_cast<VersionInfo>(msg))) {
			ROS_INFO("Referee Box %s detected", v->version_string().c_str());
			version_info_printed = true;
		}
	}

	{
		std::shared_ptr<AgentTask> at;
		if ((at = std::dynamic_pointer_cast<AgentTask>(msg))){
			// ROS_INFO("Received agent task from %s:%s", at->team_name().c_str(), at->peer_name().c_str());
			rcll_ros_msgs::AgentTask rat;
			rat.team_color = pb_to_ros_team_color(at->team_color());
			rat.task_id = at->task_id();
			rat.robot_id = at->robot_id();
			if (at->has_move()) {
				rat.move.waypoint = at->move().waypoint();
				rat.move.machine_point = at->move().machine_point();
                        }
			if (at->has_retrieve()) {
				rat.retrieve.machine_id = at->retrieve().machine_id();
				rat.retrieve.machine_point = at->retrieve().machine_point();
			}
			if (at->has_deliver()) {
				rat.deliver.machine_id = at->deliver().machine_id();
				rat.deliver.machine_point =at->deliver().machine_point();
			}
			if (at->has_buffer()) {
				rat.buffer.machine_id = at->buffer().machine_id();
				rat.buffer.shelf_number = at->buffer().shelf_number();
			}
			if (at->has_explore_machine()){
				rat.explore_machine.machine_id = at->explore_machine().machine_id();
				rat.explore_machine.machine_point = at->explore_machine().machine_point();
				rat.explore_machine.waypoint = at->explore_machine().waypoint();
			}
			if (at->has_workpiece_description()){
				rat.workpiece_description.base_color = at->workpiece_description().base_color();
 				for (int i = 0; i < at->workpiece_description().ring_colors_size(); ++i) {
                                        rat.workpiece_description.ring_colors.push_back((int)at->workpiece_description().ring_colors(i));
                                }
				rat.workpiece_description.cap_color = at->workpiece_description().cap_color();
			}
			if (at->has_order_id())		rat.order_id = at->order_id();
			if (at->has_cancel_task())	rat.cancel_task = at->cancel_task();
			if (at->has_pause_task())	rat.pause_task = at->pause_task();
			if (at->has_successful())	rat.successful = at->successful();
			if (at->has_canceled())		rat.canceled = at->canceled();
			if (at->has_error_code())	rat.error_code = at->error_code();

			pub_agent_task_.publish(rat);
		}
	}

	{
		std::shared_ptr<BeaconSignal> b;
		if ((b = std::dynamic_pointer_cast<BeaconSignal>(msg))) {
			//ROS_INFO("Received beacon signal from %s:%s", b->team_name().c_str(), b->peer_name().c_str());

			rcll_ros_msgs::BeaconSignal rb;
			rb.header.seq = b->seq();
			rb.header.stamp = ros::Time(b->time().sec(), b->time().nsec());
			rb.number = b->number();
			rb.team_name = b->team_name();
			rb.peer_name = b->peer_name();
			rb.team_color = pb_to_ros_team_color(b->team_color());
			if (b->has_pose()) {
				rb.pose.header.frame_id = "/map";
				rb.pose.header.stamp = ros::Time(b->pose().timestamp().sec(), b->pose().timestamp().nsec());
				rb.pose.pose.position.x = b->pose().x();
				rb.pose.pose.position.y = b->pose().y();
				rb.pose.pose.position.z = 0.;
				tf2::Quaternion q;
				q.setRPY(0., 0., b->pose().ori());
				rb.pose.pose.orientation.x = q.x();
				rb.pose.pose.orientation.y = q.y();
				rb.pose.pose.orientation.z = q.z();
				rb.pose.pose.orientation.w = q.w();
			}
			pub_beacon_.publish(rb);
		}
	}

	{
		std::shared_ptr<GameState> gs;
		if ((gs = std::dynamic_pointer_cast<GameState>(msg))) {
			rcll_ros_msgs::GameState rgs;
			rgs.game_time.sec = gs->game_time().sec();
			rgs.game_time.nsec = gs->game_time().nsec();
			rgs.state = (int)gs->state();
			rgs.phase = (int)gs->phase();
			if (gs->has_points_cyan()) rgs.points_cyan = gs->points_cyan();
			if (gs->has_points_magenta()) rgs.points_magenta = gs->points_magenta();

			if (gs->has_team_cyan()) {
				if (gs->team_cyan() == cfg_team_name_ && cfg_team_color_ != (int)rcll_ros_msgs::Team::CYAN) {
					setup_private_peer(llsf_msgs::CYAN);
					cfg_team_color_ = (int)rcll_ros_msgs::Team::CYAN;
				}
				rgs.team_cyan = gs->team_cyan();
			}
			if (gs->has_team_magenta()) {
				if (gs->team_magenta() == cfg_team_name_ && cfg_team_color_ != (int)rcll_ros_msgs::Team::MAGENTA) {
					setup_private_peer(llsf_msgs::MAGENTA);
					cfg_team_color_ = (int)rcll_ros_msgs::Team::MAGENTA;
				}
				rgs.team_magenta = gs->team_magenta();
			}
			if (gs->has_field_height()) rgs.field_height = gs->field_height();
			if (gs->has_field_width()) rgs.field_width = gs->field_width();
			if (gs->has_field_mirrored()) rgs.field_mirrored = gs->field_mirrored();


			pub_game_state_.publish(rgs);
		}
	}

	{
		std::shared_ptr<MachineInfo> mi;
		if ((mi = std::dynamic_pointer_cast<MachineInfo>(msg))) {
			rcll_ros_msgs::MachineInfo rmi;
			for (int i = 0; i < mi->machines_size(); ++i) {
				const llsf_msgs::Machine &m = mi->machines(i);
				rcll_ros_msgs::Machine rm;
				rm.name = m.name();
				if (m.has_type())  rm.type = m.type();
				if (m.has_state()) rm.state = m.state();
				if (m.has_team_color()) pb_to_ros_team_color(m.team_color());
				if (m.has_zone()) rm.zone = (int)m.zone();
				if (m.has_rotation()) rm.rotation = (int)m.rotation();
				for (int j = 0; j < m.ring_colors_size(); ++j) {
					rm.rs_ring_colors.push_back((int)m.ring_colors(j));
				}
				rmi.machines.push_back(rm);
			}
			pub_machine_info_.publish(rmi);

			{ // Wake if anyone is waiting for this, i.e., the prepare svc callback
				std::unique_lock<std::mutex> lock(mtx_machine_info_);
				msg_machine_info_ = mi;
				cdv_machine_info_.notify_all();
			}			
		}
	}

	{
		std::shared_ptr<ExplorationInfo> ei;
		if ((ei = std::dynamic_pointer_cast<ExplorationInfo>(msg))) {
			rcll_ros_msgs::ExplorationInfo rei;
			for (int i = 0; i < ei->signals_size(); ++i) {
				const llsf_msgs::ExplorationSignal &es = ei->signals(i);
				rcll_ros_msgs::ExplorationSignal res;
				res.type = es.type();
				for (int j = 0; j < es.lights_size(); ++j) {
					const llsf_msgs::LightSpec &l = es.lights(j);
					rcll_ros_msgs::LightSpec rl;
					
					rl.light_color = (int)l.color();
					rl.light_state = (int)l.state();
					res.lights.push_back(rl);
				}
				rei.signals.push_back(res);
			}
			for (int i = 0; i < ei->zones_size(); ++i) {
				const llsf_msgs::ExplorationZone &ez = ei->zones(i);
				rcll_ros_msgs::ExplorationZone rez;
				rez.zone = (int)ez.zone();
				rez.team = pb_to_ros_team_color(ez.team_color());
				rei.zones.push_back(rez);
			}
			pub_exploration_info_.publish(rei);
		}
	}

	{
		std::shared_ptr<MachineReportInfo> mri;
		if ((mri = std::dynamic_pointer_cast<MachineReportInfo>(msg))) {
			rcll_ros_msgs::MachineReportInfo rmri;
			for (int i = 0; i < mri->reported_machines_size(); ++i) {
				rmri.reported_machines.push_back(mri->reported_machines(i));
			}
			rmri.team_color = pb_to_ros_team_color(mri->team_color());
			pub_machine_report_info_.publish(rmri);
		}
	}

	{
		std::shared_ptr<OrderInfo> oi;
		if ((oi = std::dynamic_pointer_cast<OrderInfo>(msg))) {
			rcll_ros_msgs::OrderInfo roi;
			for (int i = 0; i < oi->orders_size(); ++i) {
				const llsf_msgs::Order &o = oi->orders(i);
				rcll_ros_msgs::Order ro;
				ro.id = o.id();
				ro.complexity = (int)o.complexity();
				ro.base_color = (int)o.base_color();
				ro.cap_color  = (int)o.cap_color();
				for (int j = 0; j < o.ring_colors_size(); ++j) {
					ro.ring_colors.push_back((int)o.ring_colors(j));
				}
				ro.quantity_requested = o.quantity_requested();
				ro.quantity_delivered_cyan = o.quantity_delivered_cyan();
				ro.quantity_delivered_magenta = o.quantity_delivered_magenta();
				ro.delivery_period_begin = o.delivery_period_begin();
				ro.delivery_period_end = o.delivery_period_end();
				ro.delivery_gate = o.delivery_gate();
				roi.orders.push_back(ro);
				// printf("Order Info %d : %d\n", i, ro.id);
			}
			pub_order_info_.publish(roi);
		}
	}

	{
		std::shared_ptr<RingInfo> ri;
		if ((ri = std::dynamic_pointer_cast<RingInfo>(msg))) {
			rcll_ros_msgs::RingInfo rri;
			for (int i = 0; i < ri->rings_size(); ++i) {
				const llsf_msgs::Ring &r = ri->rings(i);
				rcll_ros_msgs::Ring rr;
				rr.ring_color = (int)r.ring_color();
				rr.raw_material = r.raw_material();
				rri.rings.push_back(rr);
			}
			pub_ring_info_.publish(rri);
		}
	}

	{
		std::shared_ptr<NavigationRoutes> nr;
		if ((nr = std::dynamic_pointer_cast<NavigationRoutes>(msg))) {
			rcll_ros_msgs::NavigationRoutes rnr;
			printf("NavigationRoutes received\n");
			printf("routes_size: %d\n", nr->routes_size());
			for (int i = 0; i < nr->routes_size(); ++i) {
				const llsf_msgs::Route &r = nr->routes(i);
				llsf_msgs::Zone rz;
				rcll_ros_msgs::Route rr;
				rr.id        = r.id();
				printf("route size: %d\n", r.route_size());
				printf("reached_size: %d\n", r.reached_size());
				printf("remaining_size: %d\n", r.remaining_size());
				/*for (int j = 0; j < r.route_size();    ++j) {
					rr.zone = r.route(j);
					rnr.route.push_back(rr);
				}*/
				/*for (int j = 0; j < r.reached_size();  ++j) {
					rr.zone = r.reached(j);
					rnr.route.push_back(rr);
				}*/
				// printf("remaining_size: %d\n", r.remaining_size());
				for (int j = 0; j < r.remaining_size(); ++j) {
				       	rr.zone = r.remaining(j);
					rnr.route.push_back(rr);
				}
			}
			pub_navigation_routes_info_.publish(rnr);
		}
	}
}


void
setup_private_peer(llsf_msgs::Team team_color)
{
	delete peer_private_;
	peer_private_ = NULL;
	
	if (team_color == llsf_msgs::CYAN) {
		ROS_INFO("Creating private peer for CYAN");
		
		if (cfg_peer_cyan_local_) {
			peer_private_ = new ProtobufBroadcastPeer(cfg_peer_address_,
			                                          cfg_peer_cyan_send_port_,
			                                          cfg_peer_cyan_recv_port_,
			                                          &peer_public_->message_register(),
			                                          cfg_crypto_key_, cfg_crypto_cipher_);
		} else {
			peer_private_ = new ProtobufBroadcastPeer(cfg_peer_address_, cfg_peer_cyan_port_,
			                                          &peer_public_->message_register(),
			                                          cfg_crypto_key_, cfg_crypto_cipher_);
		}
		
	} else {
		ROS_INFO("Creating private peer for MAGENTA");
		
		if (cfg_peer_magenta_local_) {
			peer_private_ = new ProtobufBroadcastPeer(cfg_peer_address_,
			                                          cfg_peer_magenta_send_port_,
			                                          cfg_peer_magenta_recv_port_,
			                                          &peer_public_->message_register(),
			                                          cfg_crypto_key_, cfg_crypto_cipher_);
		} else {
			peer_private_ = new ProtobufBroadcastPeer(cfg_peer_address_, cfg_peer_magenta_port_,
			                                          &peer_public_->message_register(),
			                                          cfg_crypto_key_, cfg_crypto_cipher_);
		}
	}

	peer_private_->signal_received().connect(handle_message);
	peer_private_->signal_recv_error().connect(handle_recv_error);
	peer_private_->signal_send_error().connect(handle_send_error);
}

bool
srv_cb_send_agent_task(rcll_ros_msgs::SendAgentTask::Request  &req,
		       rcll_ros_msgs::SendAgentTask::Response &res)
{
	if (! peer_private_) {
		res.ok = false;
		res.error_msg = "Cannot send agent task: private peer not setup, team not set in refbox?";
		return true;
	}

	llsf_msgs::AgentTask at;
	at.set_team_color(ros_to_pb_team_color(cfg_team_color_));
	at.set_task_id(req.task_id);
	at.set_robot_id(req.robot_id);

	// at.set_move(req.move);
	llsf_msgs::Move *atm = at.mutable_move();
	atm->set_waypoint(req.move.waypoint);
	atm->set_machine_point(req.move.machine_point);

	// at.set_retrieve(req.retrieve);
	llsf_msgs::Retrieve *atr = at.mutable_retrieve();
	atr->set_machine_id(req.retrieve.machine_id);
	atr->set_machine_point(req.retrieve.machine_point);

	// at.set_deliver(req.deliver)
	llsf_msgs::Deliver *atd = at.mutable_deliver();
	atd->set_machine_id(req.deliver.machine_id);
	atd->set_machine_point(req.deliver.machine_point);

	// at.set_bufferstation(req.buffer)
	llsf_msgs::BufferStation *atb = at.mutable_buffer();
	atb->set_machine_id(req.buffer.machine_id);
	atb->set_shelf_number(req.buffer.shelf_number);

	// at.set_explorewaypoint(req.explore_machine);
	llsf_msgs::ExploreWaypoint *ate = at.mutable_explore_machine();
	ate->set_machine_id(req.explore_machine.machine_id);
	ate->set_machine_point(req.explore_machine.machine_point);
	ate->set_waypoint(req.explore_machine.waypoint);

	// at.set_workpiecedescription(req.workpiece_description);
	llsf_msgs::WorkpieceDescription *atw = at.mutable_workpiece_description();
	atw->set_base_color((llsf_msgs::BaseColor)req.workpiece_description.base_color);
        for (size_t i = 0; i < req.workpiece_description.ring_colors.size(); ++i) {
		atw->add_ring_colors((llsf_msgs::RingColor)req.workpiece_description.ring_colors[i]);
        }
	atw->set_cap_color((llsf_msgs::CapColor)req.workpiece_description.cap_color);

	at.set_order_id(req.order_id);
	at.set_cancel_task(req.cancel_task);
	at.set_pause_task(req.pause_task);
	at.set_successful(req.successful);
	at.set_canceled(req.canceled);
	at.set_error_code(req.error_code);

        try {
                ROS_DEBUG("Sending agent Task %c:%c)", at.task_id(), at.robot_id());
                peer_private_->send(at);
                res.ok = true;
        } catch (std::runtime_error &e) {
                res.ok = false;
                res.error_msg = e.what();
        }

        return true;
}

bool
srv_cb_send_beacon(rcll_ros_msgs::SendBeaconSignal::Request  &req,
                   rcll_ros_msgs::SendBeaconSignal::Response &res)
{
	// printf("test\n");
	if (! peer_private_) {
		res.ok = false;
		res.error_msg = "Cannot send beacon signal: private peer not setup, team not set in refbox?";
		return true;
	}

	llsf_msgs::BeaconSignal b;
	b.mutable_time()->set_sec(req.header.stamp.sec);
	b.mutable_time()->set_nsec(req.header.stamp.nsec);
	b.set_seq(req.header.seq);
	b.set_number(cfg_robot_number_);
	b.set_team_name(cfg_team_name_);
	b.set_peer_name(cfg_robot_name_);
	b.set_team_color(ros_to_pb_team_color(cfg_team_color_));
	if (req.pose.pose.position.x != 0. || req.pose.pose.position.y != 0. ||
	    req.pose.pose.orientation.x != 0. || req.pose.pose.orientation.y != 0. ||
	    req.pose.pose.orientation.z != 0. || req.pose.pose.orientation.w != 0.)
	{
		if (req.pose.pose.position.z != 0.) {
			ROS_WARN("Poses must be 2.5D pose in ground support plane (x,y,ori)");
			res.ok = false;
			res.error_msg = "Poses must be 2.5D pose in ground support plane (x,y,ori)";
			return true;
		} else {
			b.mutable_pose()->mutable_timestamp()->set_sec(req.pose.header.stamp.sec);
			b.mutable_pose()->mutable_timestamp()->set_nsec(req.pose.header.stamp.nsec);
			b.mutable_pose()->set_x(req.pose.pose.position.x);
			b.mutable_pose()->set_y(req.pose.pose.position.y);
			tf2::Quaternion q(req.pose.pose.orientation.x, req.pose.pose.orientation.y,
			                  req.pose.pose.orientation.z, req.pose.pose.orientation.w);
			b.mutable_pose()->set_ori(tf2::getYaw(q) / 3.14159 * 180.0 * -1.0 - 90.0);
		}
	}

	printf("Sending beacon %s:%s (seq %lu) (%f, %f, %f)\n", b.team_name().c_str(), b.peer_name().c_str(), b.seq(), b.mutable_pose()->x(), b.mutable_pose()->y(), b.mutable_pose()->ori());
	try {
		ROS_DEBUG("Sending beacon %s:%s (seq %lu)", b.team_name().c_str(), b.peer_name().c_str(), b.seq());
	       	peer_private_->send(b);
	       	res.ok = true;
	} catch (std::runtime_error &e) {
		res.ok = false;
		res.error_msg = e.what();
	}

	return true;
}

bool
srv_cb_send_machine_report(rcll_ros_msgs::SendMachineReportBTR::Request  &req,
			   rcll_ros_msgs::SendMachineReportBTR::Response &res)
{
	if (! peer_private_) {
		res.ok = false;
		res.error_msg = "Cannot send machine report: private peer not setup, team not set in refbox?";
		return true;
	}

	llsf_msgs::MachineReport mr;
	mr.set_team_color(ros_to_pb_team_color(cfg_team_color_));

	std::string machines_sent;

	for (size_t i = 0; i < req.machines.size(); ++i) {
		const rcll_ros_msgs::MachineReportEntryBTR &rmre = req.machines[i];
		llsf_msgs::MachineReportEntry *mre = mr.add_machines();
		mre->set_name(rmre.name);
		// mre->set_type(rmre.type);

		if (! (Zone_IsValid(rmre.zone) || Zone_IsValid(-rmre.zone))) {
			res.ok = false;
			res.error_msg = std::string("Invalid zone value for machine") + rmre.name;
			printf("%s\n", res.error_msg.c_str());
			return true;
		}

		if(rmre.zone < 0) {
			mre->set_zone((llsf_msgs::Zone)(-rmre.zone + 1000));
		}else{
			mre->set_zone((llsf_msgs::Zone)rmre.zone);
		}
		if (rmre.rotation != 1000) {
			mre->set_rotation(((int)(rmre.rotation + 22.5) / 45) * 45);
		}

		if (! machines_sent.empty()) machines_sent += ", ";
		machines_sent += rmre.name;

	}
	
	try {
		ROS_DEBUG("Sending machine report for machines: %s", machines_sent.c_str());
		peer_private_->send(mr);
		res.ok = true;
	} catch (std::runtime_error &e) {
		res.ok = false;
		res.error_msg = e.what();
	}

	return true;
}

bool
srv_cb_send_prepare_machine(rcll_ros_msgs::SendPrepareMachine::Request  &req,
                            rcll_ros_msgs::SendPrepareMachine::Response &res)
{
	if (! peer_private_) {
		res.ok = false;
		res.error_msg = "private peer not setup, team not set in refbox?";
		return true;
	}

	if (req.machine.length() < 4) {
		res.ok = false;
		res.error_msg = "Invalid machine name '" + req.machine + "'";
		return true;
	}

	// parse machine name
	std::string machine_team = req.machine.substr(0, 1);
	std::string machine_type = req.machine.substr(2, 2);

	if (machine_team != "C" && machine_team != "M") {
		res.ok = false;
		res.error_msg = "Invalid team prefix '" + machine_team + "', must be C or M";
		return true;
	}
	if (machine_team == "C" && cfg_team_color_ == (int)rcll_ros_msgs::Team::MAGENTA) {
		res.ok = false;
		res.error_msg = "Invalid team prefix 'C', own team is MAGENTA";
		return true;
	}
	if (machine_team == "M" && cfg_team_color_ == (int)rcll_ros_msgs::Team::CYAN) {
		res.ok = false;
		res.error_msg = "Invalid team prefix 'M', own team is CYAN";
		return true;
	}
	if (machine_type != "BS" && machine_type != "DS" && machine_type != "CS" && machine_type != "RS") {
		res.ok = false;
		res.error_msg = "Invalid machine type '" + machine_type + "' in name";
		return true;
	}
	
	llsf_msgs::PrepareMachine pm;
	pm.set_team_color(ros_to_pb_team_color(cfg_team_color_));
	pm.set_machine(req.machine);

	printf("machine_type: %s\n", machine_type.c_str());
	if (machine_type == "BS") {
		if (! llsf_msgs::MachineSide_IsValid(req.bs_side)) {
			res.ok = false;
			res.error_msg = "Invalid BS machine side";
			return true;
		}
		if (! llsf_msgs::BaseColor_IsValid(req.bs_base_color)) {
			res.ok = false;
			res.error_msg = "Invalid BS base color";
			return true;
		}
		llsf_msgs::PrepareInstructionBS *bsi = pm.mutable_instruction_bs();
		bsi->set_side((llsf_msgs::MachineSide)req.bs_side);
		bsi->set_color((llsf_msgs::BaseColor)req.bs_base_color);
	} else if (machine_type == "DS") {
		llsf_msgs::PrepareInstructionDS *dsi = pm.mutable_instruction_ds();
		dsi->set_order_id(req.ds_order_id);
	} else if (machine_type == "CS") {
		printf("operation: %d\n", req.cs_operation);
		if (! llsf_msgs::CSOp_IsValid(req.cs_operation)) {
			res.ok = false;
			res.error_msg = "Invalid CS operation";
			return true;
		}
		llsf_msgs::PrepareInstructionCS *csi = pm.mutable_instruction_cs();
		csi->set_operation((llsf_msgs::CSOp)req.cs_operation);
	} else if (machine_type == "RS") {
		if (! llsf_msgs::RingColor_IsValid(req.rs_ring_color)) {
			res.ok = false;
			res.error_msg = "Invalid RS ring color";
			return true;
		}
		llsf_msgs::PrepareInstructionRS *rsi = pm.mutable_instruction_rs();
		rsi->set_ring_color((llsf_msgs::RingColor)req.rs_ring_color);
	}

	if (req.wait) {
		std::unique_lock<std::mutex> lock(mtx_machine_info_);

		bool need_to_wait = false;
		for (int i = 0; i < msg_machine_info_->machines_size(); ++i) {
			const llsf_msgs::Machine &m = msg_machine_info_->machines(i);
			if (m.name() == req.machine) {
				if (m.has_state() && m.state() != "IDLE") {
					ROS_INFO("Machine '%s' is not in IDLE state, waiting briefly", req.machine.c_str());
					need_to_wait = true;
				}
				break;
			}
		}
		if (need_to_wait) {
			int machine_idx = 0;
			cdv_machine_info_.wait_for(lock, std::chrono::seconds(10),
			                           [&req, &machine_idx]() -> bool {
				                           if (! msg_machine_info_) return false;
				                           rcll_ros_msgs::MachineInfo rmi;
				                           for (int i = 0; i < msg_machine_info_->machines_size(); ++i) {
					                           const llsf_msgs::Machine &m = msg_machine_info_->machines(i);
					                           if (m.name() == req.machine && m.has_state() && m.state() == "IDLE") {
						                           machine_idx = i;
						                           return true;
					                           }
				                           }
				                           return false;
			                           });

			const llsf_msgs::Machine &m = msg_machine_info_->machines(machine_idx);
			if (m.state() != "IDLE") {
				ROS_INFO("Machine '%s' went into '%s' state, not IDLE", req.machine.c_str(), m.state().c_str());
				res.ok = false;
				res.error_msg = "Machine '" + req.machine + "' went into '" + m.state() + "' state, not IDLE";
				return true;
			} else {
				ROS_INFO("Machine '%s' is now in IDLE state", req.machine.c_str());
			}
		}
	}
	
	try {
		ROS_DEBUG("Sending prepare machine instruction for machine: %s", req.machine.c_str());
		peer_private_->send(pm);
		res.ok = true;
	} catch (std::runtime_error &e) {
		res.ok = false;
		res.error_msg = e.what();
	}

	if (req.wait) {
		ROS_INFO("Waiting for machine '%s' state != IDLE", req.machine.c_str());
		std::unique_lock<std::mutex> lock(mtx_machine_info_);

		int machine_idx = 0;
		cdv_machine_info_.wait(lock,
		                       [&req, &machine_idx, &pm]() -> bool {
			                       if (! msg_machine_info_) return false;
			                       rcll_ros_msgs::MachineInfo rmi;
			                       for (int i = 0; i < msg_machine_info_->machines_size(); ++i) {
				                       const llsf_msgs::Machine &m = msg_machine_info_->machines(i);
				                       if (m.name() == req.machine && m.has_state() && m.state() != "IDLE") {
					                       machine_idx = i;
					                       return true;
				                       }
			                       }
			                       peer_private_->send(pm);
			                       return false;
		                       });

		const llsf_msgs::Machine &m = msg_machine_info_->machines(machine_idx);
		if (m.state() == "BROKEN") {
			ROS_WARN("Machine '%s' went into 'BROKEN' state", req.machine.c_str());
			res.ok = false;
			res.error_msg = "Machine '" + req.machine + "' went into '" + m.state() + "' state";
		} else {
			ROS_INFO("Machine '%s' went into '%s' state", req.machine.c_str(), m.state().c_str());
			res.ok = true;
		}
	}

	return true;
}

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rcll_refbox_peer");

	ros::NodeHandle n;

	ROS_INFO("%s starting up", ros::this_node::getName().c_str());

	// Parameter parsing	
	cfg_team_color_ = 0;
	
	GET_PRIV_PARAM(team_name);
	GET_PRIV_PARAM(robot_name);
	GET_PRIV_PARAM(robot_number);

	GET_PRIV_PARAM(peer_address);
	
	if (ros::param::has("~peer_public_recv_port") && ros::param::has("~peer_public_send_port")) {
		cfg_peer_public_local_ = true;
		GET_PRIV_PARAM(peer_public_recv_port);
		GET_PRIV_PARAM(peer_public_send_port);
	} else {
		cfg_peer_public_local_ = false;
		GET_PRIV_PARAM(peer_public_port);
	}

	if (ros::param::has("~peer_cyan_recv_port") && ros::param::has("~peer_cyan_send_port")) {
		cfg_peer_cyan_local_ = true;
		if (! ros::param::get("~peer_cyan_recv_port", cfg_peer_cyan_recv_port_)) {
			ROS_ERROR("Failed to retrieve parameter cyan_recv_port, aborting");
			exit(-1);
		}
		if (! ros::param::get("~peer_cyan_send_port", cfg_peer_cyan_send_port_)) {
			ROS_ERROR("Failed to retrieve parameter cyan_send_port, aborting");
			exit(-1);
		}
	} else {
		cfg_peer_cyan_local_ = false;
		if (! ros::param::get("~peer_cyan_port", cfg_peer_cyan_port_)) {
			ROS_ERROR("Failed to retrieve parameter cyan_port, aborting");
			exit(-1);
		}
	}
	if (ros::param::has("~peer_magenta_recv_port") && ros::param::has("~peer_magenta_send_port")) {
		cfg_peer_magenta_local_ = true;
		if (! ros::param::get("~peer_magenta_recv_port", cfg_peer_magenta_recv_port_)) {
			ROS_ERROR("Failed to retrieve parameter magenta_recv_port, aborting");
			exit(-1);
		}
		if (! ros::param::get("~peer_magenta_send_port", cfg_peer_magenta_send_port_)) {
			ROS_ERROR("Failed to retrieve parameter magenta_send_port, aborting");
			exit(-1);
		}
	} else {
		cfg_peer_magenta_local_ = false;
		if (! ros::param::get("~peer_magenta_port", cfg_peer_magenta_port_)) {
			ROS_ERROR("Failed to retrieve parameter magenta_port, aborting");
			exit(-1);
		}
	}

	GET_PRIV_PARAM(crypto_key);
	GET_PRIV_PARAM(crypto_cipher);

	// Setup ROS topics
	pub_agent_task_ = n.advertise<rcll_ros_msgs::AgentTask>("rcll/agent_task", 100);
	pub_beacon_ = n.advertise<rcll_ros_msgs::BeaconSignal>("rcll/beacon", 100);
	pub_game_state_ = n.advertise<rcll_ros_msgs::GameState>("rcll/game_state", 10);
	pub_machine_info_ = n.advertise<rcll_ros_msgs::MachineInfo>("rcll/machine_info", 10);
	pub_exploration_info_ = n.advertise<rcll_ros_msgs::ExplorationInfo>("rcll/exploration_info", 10);
	pub_machine_report_info_ = n.advertise<rcll_ros_msgs::MachineReportInfo>("rcll/machine_report_info", 10);
	pub_order_info_ = n.advertise<rcll_ros_msgs::OrderInfo>("rcll/order_info", 10);
	pub_ring_info_ = n.advertise<rcll_ros_msgs::RingInfo>("rcll/ring_info", 10);
	pub_navigation_routes_info_ = n.advertise<rcll_ros_msgs::NavigationRoutes>("rcll/routes_info", 10);

	// Setup basic communication
  if (cfg_peer_public_local_) {
	  ROS_WARN("Creating public peer: addr: %s  send: %u  recv: %u",
	           cfg_peer_address_.c_str(), cfg_peer_public_send_port_, cfg_peer_public_recv_port_);
	  peer_public_ = new ProtobufBroadcastPeer(cfg_peer_address_,
	                                           cfg_peer_public_send_port_,
	                                           cfg_peer_public_recv_port_);
  } else {
	  ROS_WARN("Creating public peer: addr: %s  port: %u",
	           cfg_peer_address_.c_str(), cfg_peer_public_port_);
	  peer_public_ = new ProtobufBroadcastPeer(cfg_peer_address_, cfg_peer_public_port_);
  }

  MessageRegister & message_register = peer_public_->message_register();
  message_register.add_message_type<llsf_msgs::VersionInfo>();
  message_register.add_message_type<llsf_msgs::AgentTask>();
  message_register.add_message_type<llsf_msgs::BeaconSignal>();
  message_register.add_message_type<llsf_msgs::GameState>();
  message_register.add_message_type<llsf_msgs::MachineInfo>();
  message_register.add_message_type<llsf_msgs::ExplorationInfo>();
  message_register.add_message_type<llsf_msgs::MachineReportInfo>();
  message_register.add_message_type<llsf_msgs::OrderInfo>();
  message_register.add_message_type<llsf_msgs::RingInfo>();
  message_register.add_message_type<llsf_msgs::RobotInfo>();
  message_register.add_message_type<llsf_msgs::PrepareMachine>();
  message_register.add_message_type<llsf_msgs::NavigationRoutes>();

  peer_public_->signal_received().connect(handle_message);
  peer_public_->signal_recv_error().connect(handle_recv_error);
  peer_public_->signal_send_error().connect(handle_send_error);

  // provide services
  srv_send_agent_task_ = n.advertiseService("/rcll/send_agent_task", srv_cb_send_agent_task);
  srv_send_beacon_ = n.advertiseService("rcll/send_beacon", srv_cb_send_beacon);
  srv_send_machine_report_ = n.advertiseService("rcll/send_machine_report", srv_cb_send_machine_report);
  srv_send_prepare_machine_ = n.advertiseService("rcll/send_prepare_machine", srv_cb_send_prepare_machine);

  ros::spin();
	
	return 0;
}
