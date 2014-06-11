/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2011 Stefan Osswald, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the University of Freiburg nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "NaoJointsAnalyzer.h"

#include <sstream>
#include <fstream>

PLUGINLIB_DECLARE_CLASS(nao_dashboard, NaoJointsAnalyzer,
		diagnostic_aggregator::NaoJointsAnalyzer,
		diagnostic_aggregator::Analyzer)

namespace diagnostic_aggregator {

/**
 * @brief Constructor.
 */
NaoJointsAnalyzer::NaoJointsAnalyzer() :
	m_path(""), m_niceName("Joints"), m_lastSeen(0) {
}

/**
 * @brief Destructor.
 */
NaoJointsAnalyzer::~NaoJointsAnalyzer() {
}

bool NaoJointsAnalyzer::init(const std::string base_name, const ros::NodeHandle &n) {
	if (!n.getParam("path", m_niceName))
	{
		ROS_ERROR("NaoJointsAnalyzer was not given parameter \"path\". Namespace: %s",
			  n.getNamespace().c_str());
		return false;
	}

	m_path = base_name;

	boost::shared_ptr<StatusItem> item(new StatusItem(m_niceName));
	m_jointsMasterItem = item;

	return true;
}

bool NaoJointsAnalyzer::match(const std::string name) {
	return name.find("nao_joint") == 0;
}

bool NaoJointsAnalyzer::analyze(const boost::shared_ptr<StatusItem> item) {
	if(item->getName().find("nao_joint") != 0)
		return false;

	JointData data;

	std::stringstream ssStiffness(item->getValue("Stiffness"));
	ssStiffness >> data.stiffness;

	std::stringstream ssTemperature(item->getValue("Temperature"));
	ssTemperature >> data.temperature;

	data.status = item;

	if(m_joints.find(item->getName()) == m_joints.end())
		m_joints.insert(make_pair(item->getName(), data));
	else
		m_joints.at(item->getName()) = data;

	m_lastSeen = ros::Time::now();

	return true;
}

bool NaoJointsAnalyzer::compareByTemperature(const NaoJointsAnalyzer::JointData& a, const NaoJointsAnalyzer::JointData& b) {
    return (a.temperature > b.temperature);
}

/**
 * @brief Helper function for adding key/value pairs to a diagnostic status message.
 * @tparam T Datatype of the value (must be convertible with std::ostream::operator<<).
 * @param joint_stat Diagnostic status message.
 * @param key Key.
 * @param value Value.
 */
template<typename T>
void NaoJointsAnalyzer::addValue(boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> joint_stat, const std::string& key, const T& value) const {
	std::stringstream ss;
	ss << value;
	diagnostic_msgs::KeyValue kv;
	kv.key = key;
	kv.value = ss.str();
	joint_stat->values.push_back(kv);
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > NaoJointsAnalyzer::report() {
    bool stale = (ros::Time::now()-m_lastSeen).toSec() > 5.0;
	boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> joint_stat =
			m_jointsMasterItem->toStatusMsg(m_path, stale);

	std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
	if(stale) {
		output.push_back(joint_stat);
		return output;
	}

	double maxTemperature = 0.0;
	double maxStiffness = 0.0;
	double minStiffness = 1.0;
	double minStiffnessWoHands = 1.0;

	std::vector<JointData> hotJoints;
	joint_stat->level = diagnostic_msgs::DiagnosticStatus::OK;

	for(JointsMapType::const_iterator it = m_joints.begin(); it != m_joints.end(); it++) {
		if(it->first.find("RHipYawPitch") == std::string::npos) {
			maxTemperature = std::max(maxTemperature, it->second.temperature);
			maxStiffness = std::max(maxStiffness, it->second.stiffness);
			minStiffness = std::min(minStiffness, it->second.stiffness);
			if(it->first.find("Hand") == std::string::npos)
				minStiffnessWoHands = std::min(minStiffnessWoHands, it->second.stiffness);
			if((int) it->second.status->getLevel() >= (int) diagnostic_msgs::DiagnosticStatus::WARN) {
			    hotJoints.push_back(it->second);
			}
			if(it->second.status->getLevel() > joint_stat->level)
			    joint_stat->level = it->second.status->getLevel();
		}
	}

	addValue(joint_stat, "Highest Temperature", maxTemperature);
	addValue(joint_stat, "Highest Stiffness", maxStiffness);
	addValue(joint_stat, "Lowest Stiffness", minStiffness);
	addValue(joint_stat, "Lowest Stiffness without Hands", minStiffnessWoHands);

    std::sort(hotJoints.begin(), hotJoints.end(), NaoJointsAnalyzer::compareByTemperature);
    std::stringstream hotJointsSS;
    for(size_t i = 0; i < hotJoints.size(); i++) {
        hotJointsSS << std::endl << removeLeadingNameChaff(hotJoints[i].status->getName(), "nao_joint") << ": " << hotJoints[i].temperature << "°C";
    }
    addValue(joint_stat, "Hot Joints", hotJointsSS.str());

	if (joint_stat->level == diagnostic_msgs::DiagnosticStatus::OK) {
		joint_stat->message = "OK";
	} else if (joint_stat->level == diagnostic_msgs::DiagnosticStatus::WARN) {
		joint_stat->message = "WARN";
	} else {
		joint_stat->message = "ERROR";
	}

	output.push_back(joint_stat);

	for(JointsMapType::const_iterator it = m_joints.begin(); it != m_joints.end(); it++) {
		boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> msg(it->second.status->toStatusMsg(m_path + "/" + m_niceName,
				(ros::Time::now() - it->second.status->getLastUpdateTime()).toSec() > 3.0));
		msg->name = m_path + "/" + m_niceName + "/" + removeLeadingNameChaff(it->second.status->getName(), "nao_joint: ");
		output.push_back(msg);
	}

	return output;
}
}
