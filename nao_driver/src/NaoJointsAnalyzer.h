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

#ifndef NAO_JOINTS_ANALYZER_H
#define NAO_JOINTS_ANALYZER_H

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <map>

namespace diagnostic_aggregator {

/**
 * @brief Analyzes diagnostic messages about Nao's joints from nao_diagnostic/nao_diagnostic_updater.
 */
class NaoJointsAnalyzer: public Analyzer {
public:
	NaoJointsAnalyzer();
	~NaoJointsAnalyzer();
	bool init(const std::string base_name, const ros::NodeHandle &n);
	bool match(const std::string name);
	bool analyze(const boost::shared_ptr<StatusItem> item);
	std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();
	std::string getPath() const {
		return m_path;
	}
	std::string getName() const {
		return m_niceName;
	}

private:
	std::string m_path, m_niceName;
	boost::shared_ptr<StatusItem> m_jointsMasterItem;
	boost::shared_ptr<StatusItem> m_statusItems;
	ros::Time m_lastSeen;

	struct JointData {
	    std::string name;
		double temperature, stiffness;
		boost::shared_ptr<diagnostic_aggregator::StatusItem> status;
	};
	typedef std::map<std::string, JointData> JointsMapType;
	JointsMapType m_joints;

	template<typename T> void addValue(boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> joint_stat, const std::string& key, const T& value) const;
	static bool compareByTemperature(const NaoJointsAnalyzer::JointData& a, const NaoJointsAnalyzer::JointData& b);
};

}
#endif //NAO_JOINTS_ANALYZER_H
