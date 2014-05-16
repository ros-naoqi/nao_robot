/**
 * @file NaoJointsAnalyzer.h
 * @author Stefan Osswald
 */

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
