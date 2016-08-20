/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Wim Meeussen, Dennis L. Wigand */

#ifndef KDLParser_H_
#define KDLParser_H_

#include <memory>
#include <kdl/tree.hpp>
#include <string>
#include <tinyxml.h>
//#include <boost/shared_ptr.hpp>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>

/**
 * This class acts just as some kind of struct to combine rtt component, associated model and gazebo caller.
 */
class KDLParser {
public:
	KDLParser();
	virtual ~KDLParser();
	bool treeFromFile(const std::string& file, KDL::Tree& tree);

	bool treeFromString(const std::string& xml, KDL::Tree& tree);

	bool treeFromXml(TiXmlDocument *xml_doc, KDL::Tree& tree);

	bool treeFromUrdfModel(boost::shared_ptr<urdf::ModelInterface> robot_model,
			KDL::Tree& tree);

	boost::shared_ptr<urdf::ModelInterface> parseURDF(
			const std::string &xml_string);

	boost::shared_ptr<urdf::ModelInterface> parseURDFFile(
			const std::string &path);

	bool initTreeAndChainFromURDFString(const std::string& urdfString,
				const std::string& root_link, const std::string& tip_link,
				KDL::Tree& kdl_tree, KDL::Chain& kdl_chain);

	bool loadURDFFileIntoString(const std::string &path, std::string& xmlOut);

private:
	KDL::Vector toKdl(urdf::Vector3 v);
	KDL::Rotation toKdl(urdf::Rotation r);
	KDL::Frame toKdl(urdf::Pose p);
	KDL::Joint toKdl(boost::shared_ptr<urdf::Joint> jnt);
	KDL::RigidBodyInertia toKdl(boost::shared_ptr<urdf::Inertial> i);
	bool addChildrenToTree(boost::shared_ptr<const urdf::Link> root,
			KDL::Tree& tree);
	bool parsePose(urdf::Pose &pose, TiXmlElement* xml);
	bool parseInertial(urdf::Inertial &i, TiXmlElement *config);
	bool parseLink(urdf::Link &link, TiXmlElement* config);
	bool parseJoint(urdf::Joint &joint, TiXmlElement* config);

	bool parseJointDynamics(urdf::JointDynamics &jd, TiXmlElement* config);
	bool parseJointSafety(urdf::JointSafety &js, TiXmlElement* config);
	bool parseJointCalibration(urdf::JointCalibration &jc,
			TiXmlElement* config);
	bool parseJointLimits(urdf::JointLimits &jl, TiXmlElement* config);
	bool parseJointMimic(urdf::JointMimic &jm, TiXmlElement* config);
};

#endif /* KDLParser_H_ */
