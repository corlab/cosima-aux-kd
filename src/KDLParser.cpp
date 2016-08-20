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

#include "KDLParser.hpp"
#include <kdl/frames_io.hpp>
#include <rtt/Logger.hpp>
#include <sstream>
#include <urdf_model/joint.h>
#include <boost/lexical_cast.hpp>
//#include <console_bridge/console.h>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>

using namespace std;
using namespace KDL;
using namespace RTT;
using namespace boost;

KDLParser::KDLParser() {

}

KDLParser::~KDLParser() {

}

// construct vector
Vector KDLParser::toKdl(urdf::Vector3 v) {
	return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation KDLParser::toKdl(urdf::Rotation r) {
	return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame KDLParser::toKdl(urdf::Pose p) {
	return Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
Joint KDLParser::toKdl(boost::shared_ptr<urdf::Joint> jnt) {
	Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

	switch (jnt->type) {
	case urdf::Joint::FIXED: {
		return Joint(jnt->name, Joint::None);
	}
	case urdf::Joint::REVOLUTE: {
		Vector axis = toKdl(jnt->axis);
		return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis,
				Joint::RotAxis);
	}
	case urdf::Joint::CONTINUOUS: {
		Vector axis = toKdl(jnt->axis);
		return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis,
				Joint::RotAxis);
	}
	case urdf::Joint::PRISMATIC: {
		Vector axis = toKdl(jnt->axis);
		return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis,
				Joint::TransAxis);
	}
	default: {
		log(Warning) << "[KDLParser] Converting unknown joint type of joint "
				<< jnt->name.c_str() << " into a fixed joint" << endlog();
		return Joint(jnt->name, Joint::None);
	}
	}
	return Joint();
}

// construct inertia
RigidBodyInertia KDLParser::toKdl(boost::shared_ptr<urdf::Inertial> i) {
	Frame origin = toKdl(i->origin);

	// the mass is frame indipendent
	double kdl_mass = i->mass;

	// kdl and urdf both specify the com position in the reference frame of the link
	Vector kdl_com = origin.p;

	// kdl specifies the inertia matrix in the reference frame of the link,
	// while the urdf specifies the inertia matrix in the inertia reference frame
	RotationalInertia urdf_inertia = RotationalInertia(i->ixx, i->iyy, i->izz,
			i->ixy, i->ixz, i->iyz);

	// Rotation operators are not defined for rotational inertia,
	// so we use the RigidBodyInertia operators (with com = 0) as a workaround
	RigidBodyInertia kdl_inertia_wrt_com_workaround = origin.M
			* RigidBodyInertia(0, Vector::Zero(), urdf_inertia);

	// Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
	// while the getRotationalInertia method returns the 3d inertia wrt the frame origin
	// (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
	RotationalInertia kdl_inertia_wrt_com =
			kdl_inertia_wrt_com_workaround.getRotationalInertia();

	return RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

// recursive function to walk through tree
bool KDLParser::addChildrenToTree(boost::shared_ptr<const urdf::Link> root,
		Tree& tree) {
	std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
	log(Debug) << "[KDLParser] Link " << root->name.c_str() << " had "
			<< (int) children.size() << " children" << endlog();

	// constructs the optional inertia
	RigidBodyInertia inert(0);
	if (root->inertial)
		inert = toKdl(root->inertial);

	// constructs the kdl joint
	Joint jnt = toKdl(root->parent_joint);

	// construct the kdl segment
	Segment sgm(root->name, jnt,
			toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

	// add segment to tree
	tree.addSegment(sgm, root->parent_joint->parent_link_name);

	// recurslively add all children
	for (size_t i = 0; i < children.size(); i++) {
		if (!addChildrenToTree(children[i], tree))
			return false;
	}
	return true;
}

bool KDLParser::parsePose(urdf::Pose &pose, TiXmlElement* xml) {
	pose.clear();
	if (xml) {
		const char* xyz_str = xml->Attribute("xyz");
		if (xyz_str != NULL) {
			try {
				pose.position.init(xyz_str);
			} catch (urdf::ParseError &e) {
//        CONSOLE_BRIDGE_logError(e.what());
				return false;
			}
		}

		const char* rpy_str = xml->Attribute("rpy");
		if (rpy_str != NULL) {
			try {
				pose.rotation.init(rpy_str);
			} catch (urdf::ParseError &e) {
//        CONSOLE_BRIDGE_logError(e.what());
				return false;
			}
		}
	}
	return true;
}

bool KDLParser::treeFromFile(const string& file, Tree& tree) {
//	TiXmlDocument urdf_xml;
//	urdf_xml.LoadFile(file);
//	return treeFromXml(&urdf_xml, tree);
//	boost::shared_ptr<urdf::ModelInterface> modelPtr = urdf::parseURDFFile(file);
//	return treeFromUrdfModel(*(modelPtr.get()), tree);
	return true;
}

bool KDLParser::treeFromString(const string& xml, Tree& tree) {
//	TiXmlDocument urdf_xml;
//	urdf_xml.Parse(xml.c_str());
//	return treeFromXml(&urdf_xml, tree);
	return false;
}

bool KDLParser::treeFromXml(TiXmlDocument *xml_doc, Tree& tree) {
//	urdf::ModelInterface robot_model;
//	if (!robot_model.initXml(xml_doc)) { // parse urdf/sdf DOM here
//		log(Error) << "Could not generate robot model" << endlog();
//		return false;
//	}
//	return treeFromUrdfModel(robot_model, tree);
	return false;
}

bool KDLParser::treeFromUrdfModel(
		boost::shared_ptr<urdf::ModelInterface> robot_model, Tree& tree) {
	tree = Tree(robot_model->getRoot()->name);

	// warn if root link has inertia. KDL does not support this
	if (robot_model->getRoot()->inertial)
		log(Warning) << "[KDLParser] The root link " << robot_model->getRoot()->name.c_str()
				<< " has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF."
				<< endlog();
	//  add all children
	for (size_t i = 0; i < robot_model->getRoot()->child_links.size(); i++)
		if (!addChildrenToTree(robot_model->getRoot()->child_links[i], tree))
			return false;

	return true;
}

bool KDLParser::parseInertial(urdf::Inertial &i, TiXmlElement *config) {
	i.clear();

	// Origin
	TiXmlElement *o = config->FirstChildElement("origin");
	if (o) {
		if (!KDLParser::parsePose(i.origin, o))
			return false;
	}

	TiXmlElement *mass_xml = config->FirstChildElement("mass");
	if (!mass_xml) {
//    CONSOLE_BRIDGE_logError("Inertial element must have a mass element");
		return false;
	}
	if (!mass_xml->Attribute("value")) {
//    CONSOLE_BRIDGE_logError("Inertial: mass element must have value attribute");
		return false;
	}

	try {
		i.mass = boost::lexical_cast<double>(mass_xml->Attribute("value"));
	} catch (boost::bad_lexical_cast &/*e*/) {
		std::stringstream stm;
		stm << "Inertial: mass [" << mass_xml->Attribute("value")
				<< "] is not a float";
//    CONSOLE_BRIDGE_logError(stm.str().c_str());
		return false;
	}

	TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
	if (!inertia_xml) {
//    CONSOLE_BRIDGE_logError("Inertial element must have inertia element");
		return false;
	}
	if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy")
			&& inertia_xml->Attribute("ixz") && inertia_xml->Attribute("iyy")
			&& inertia_xml->Attribute("iyz") && inertia_xml->Attribute("izz"))) {
//    CONSOLE_BRIDGE_logError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
		return false;
	}
	try {
		i.ixx = boost::lexical_cast<double>(inertia_xml->Attribute("ixx"));
		i.ixy = boost::lexical_cast<double>(inertia_xml->Attribute("ixy"));
		i.ixz = boost::lexical_cast<double>(inertia_xml->Attribute("ixz"));
		i.iyy = boost::lexical_cast<double>(inertia_xml->Attribute("iyy"));
		i.iyz = boost::lexical_cast<double>(inertia_xml->Attribute("iyz"));
		i.izz = boost::lexical_cast<double>(inertia_xml->Attribute("izz"));
	} catch (boost::bad_lexical_cast &/*e*/) {
		std::stringstream stm;
		stm << "Inertial: one of the inertia elements is not a valid double:"
				<< " ixx [" << inertia_xml->Attribute("ixx") << "]" << " ixy ["
				<< inertia_xml->Attribute("ixy") << "]" << " ixz ["
				<< inertia_xml->Attribute("ixz") << "]" << " iyy ["
				<< inertia_xml->Attribute("iyy") << "]" << " iyz ["
				<< inertia_xml->Attribute("iyz") << "]" << " izz ["
				<< inertia_xml->Attribute("izz") << "]";
//    CONSOLE_BRIDGE_logError(stm.str().c_str());
		return false;
	}
	return true;
}

bool KDLParser::parseLink(urdf::Link &link, TiXmlElement* config) {

	link.clear();

	const char *name_char = config->Attribute("name");
	if (!name_char) {
//    CONSOLE_BRIDGE_logError("No name given for the link.");
		return false;
	}
	link.name = std::string(name_char);

	// Inertial (optional)
	TiXmlElement *i = config->FirstChildElement("inertial");
	if (i) {
		link.inertial.reset(new urdf::Inertial());
		if (!parseInertial(*link.inertial, i)) {
//      CONSOLE_BRIDGE_logError("Could not parse inertial element for Link [%s]", link.name.c_str());
			return false;
		}
	}

//  // Multiple Visuals (optional)
//  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
//  {
//
//    VisualSharedPtr vis;
//    vis.reset(new Visual());
//    if (parseVisual(*vis, vis_xml))
//    {
//      link.visual_array.push_back(vis);
//    }
//    else
//    {
//      vis.reset();
//      CONSOLE_BRIDGE_logError("Could not parse visual element for Link [%s]", link.name.c_str());
//      return false;
//    }
//  }
//
//  // Visual (optional)
//  // Assign the first visual to the .visual ptr, if it exists
//  if (!link.visual_array.empty())
//    link.visual = link.visual_array[0];
//
//  // Multiple Collisions (optional)
//  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
//  {
//    CollisionSharedPtr col;
//    col.reset(new Collision());
//    if (parseCollision(*col, col_xml))
//    {
//      link.collision_array.push_back(col);
//    }
//    else
//    {
//      col.reset();
//      CONSOLE_BRIDGE_logError("Could not parse collision element for Link [%s]",  link.name.c_str());
//      return false;
//    }
//  }
//
//  // Collision (optional)
//  // Assign the first collision to the .collision ptr, if it exists
//  if (!link.collision_array.empty())
//    link.collision = link.collision_array[0];

	return true;
}

boost::shared_ptr<urdf::ModelInterface> KDLParser::parseURDF(
		const std::string &xml_string) {
	boost::shared_ptr<urdf::ModelInterface> model(new urdf::ModelInterface);
	model->clear();

	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error()) {
//    CONSOLE_BRIDGE_logError(xml_doc.ErrorDesc());
		xml_doc.ClearError();
		model.reset();
		return model;
	}

	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml) {
//    CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
		model.reset();
		return model;
	}

	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name) {
//    CONSOLE_BRIDGE_logError("No name given for the robot.");
		model.reset();
		return model;
	}
	model->name_ = std::string(name);

	// Get all Link elements
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link");
			link_xml; link_xml = link_xml->NextSiblingElement("link")) {
		boost::shared_ptr<urdf::Link> link;
		link.reset(new urdf::Link);

		try {
			parseLink(*link, link_xml);
			if (model->getLink(link->name)) {
//        CONSOLE_BRIDGE_logError("link '%s' is not unique.", link->name.c_str());
				model.reset();
				return model;
			} else {
				// set link visual material
//        CONSOLE_BRIDGE_logDebug("urdfdom: setting link '%s' material", link->name.c_str());
				if (link->visual) {
					if (!link->visual->material_name.empty()) {
						if (model->getMaterial(link->visual->material_name)) {
//              CONSOLE_BRIDGE_logDebug("urdfdom: setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
							link->visual->material = model->getMaterial(
									link->visual->material_name.c_str());
						} else {
							if (link->visual->material) {
//                CONSOLE_BRIDGE_logDebug("urdfdom: link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
								model->materials_.insert(
										make_pair(link->visual->material->name,
												link->visual->material));
							} else {
//                CONSOLE_BRIDGE_logError("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
								model.reset();
								return model;
							}
						}
					}
				}

				model->links_.insert(make_pair(link->name, link));
//        CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new link '%s'", link->name.c_str());
			}
		} catch (urdf::ParseError &/*e*/) {
//      CONSOLE_BRIDGE_logError("link xml is not initialized correctly");
			model.reset();
			return model;
		}
	}
	if (model->links_.empty()) {
//    CONSOLE_BRIDGE_logError("No link elements found in urdf file");
		model.reset();
		return model;
	}

	// Get all Joint elements
	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint");
			joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")) {
		boost::shared_ptr<urdf::Joint> joint;
		joint.reset(new urdf::Joint);

		if (parseJoint(*joint, joint_xml)) {
			if (model->getJoint(joint->name)) {
//        CONSOLE_BRIDGE_logError("joint '%s' is not unique.", joint->name.c_str());
				model.reset();
				return model;
			} else {
				model->joints_.insert(make_pair(joint->name, joint));
//        CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new joint '%s'", joint->name.c_str());
			}
		} else {
//      CONSOLE_BRIDGE_logError("joint xml is not initialized correctly");
			model.reset();
			return model;
		}
	}

	// every link has children links and joints, but no parents, so we create a
	// local convenience data structure for keeping child->parent relations
	std::map<std::string, std::string> parent_link_tree;
	parent_link_tree.clear();

	// building tree: name mapping
	try {
		model->initTree(parent_link_tree);
	} catch (urdf::ParseError &e) {
//    CONSOLE_BRIDGE_logError("Failed to build tree: %s", e.what());
		model.reset();
		return model;
	}

	// find the root link
	try {
		model->initRoot(parent_link_tree);
	} catch (urdf::ParseError &e) {
//    CONSOLE_BRIDGE_logError("Failed to find root link: %s", e.what());
		model.reset();
		return model;
	}

	return model;
}

boost::shared_ptr<urdf::ModelInterface> KDLParser::parseURDFFile(
		const std::string &path) {
	std::ifstream stream(path.c_str());
	if (!stream) {
		log(Info) << "[KDLParser] File " << path << " does not exist" << endlog();
		return boost::shared_ptr<urdf::ModelInterface>();
	}

	std::string xml_str((std::istreambuf_iterator<char>(stream)),
			std::istreambuf_iterator<char>());
	return parseURDF(xml_str);
}

bool KDLParser::loadURDFFileIntoString(const std::string &path,
		std::string& xmlOut) {
	std::ifstream stream(path.c_str());
	if (!stream) {
		log(Info) << "[KDLParser] File " << path << " does not exist" << endlog();
		return false;
	}

	std::string xml_str((std::istreambuf_iterator<char>(stream)),
			std::istreambuf_iterator<char>());
	xmlOut = xml_str;
	if (xmlOut != "") {
		return true;
	} else {
		return false;
	}
}

bool KDLParser::parseJointDynamics(urdf::JointDynamics &jd,
		TiXmlElement* config) {
	jd.clear();

	// Get joint damping
	const char* damping_str = config->Attribute("damping");
	if (damping_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_dynamics: no damping, defaults to 0");
		jd.damping = 0;
	} else {
		try {
			jd.damping = boost::lexical_cast<double>(damping_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("damping value (%s) is not a float: %s",damping_str, e.what());
			return false;
		}
	}

	// Get joint friction
	const char* friction_str = config->Attribute("friction");
	if (friction_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_dynamics: no friction, defaults to 0");
		jd.friction = 0;
	} else {
		try {
			jd.friction = boost::lexical_cast<double>(friction_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("friction value (%s) is not a float: %s",friction_str, e.what());
			return false;
		}
	}

	if (damping_str == NULL && friction_str == NULL) {
		//CONSOLE_BRIDGE_logError("joint dynamics element specified with no damping and no friction");
		return false;
	} else {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_dynamics: damping %f and friction %f", jd.damping, jd.friction);
		return true;
	}
}

bool KDLParser::parseJointLimits(urdf::JointLimits &jl, TiXmlElement* config) {
	jl.clear();

	// Get lower joint limit
	const char* lower_str = config->Attribute("lower");
	if (lower_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_limit: no lower, defaults to 0");
		jl.lower = 0;
	} else {
		try {
			jl.lower = boost::lexical_cast<double>(lower_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("lower value (%s) is not a float: %s", lower_str, e.what());
			return false;
		}
	}

	// Get upper joint limit
	const char* upper_str = config->Attribute("upper");
	if (upper_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_limit: no upper, , defaults to 0");
		jl.upper = 0;
	} else {
		try {
			jl.upper = boost::lexical_cast<double>(upper_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("upper value (%s) is not a float: %s",upper_str, e.what());
			return false;
		}
	}

	// Get joint effort limit
	const char* effort_str = config->Attribute("effort");
	if (effort_str == NULL) {
		//CONSOLE_BRIDGE_logError("joint limit: no effort");
		return false;
	} else {
		try {
			jl.effort = boost::lexical_cast<double>(effort_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("effort value (%s) is not a float: %s",effort_str, e.what());
			return false;
		}
	}

	// Get joint velocity limit
	const char* velocity_str = config->Attribute("velocity");
	if (velocity_str == NULL) {
		//CONSOLE_BRIDGE_logError("joint limit: no velocity");
		return false;
	} else {
		try {
			jl.velocity = boost::lexical_cast<double>(velocity_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("velocity value (%s) is not a float: %s",velocity_str, e.what());
			return false;
		}
	}

	return true;
}

bool KDLParser::parseJointSafety(urdf::JointSafety &js, TiXmlElement* config) {
	js.clear();

	// Get soft_lower_limit joint limit
	const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
	if (soft_lower_limit_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_safety: no soft_lower_limit, using default value");
		js.soft_lower_limit = 0;
	} else {
		try {
			js.soft_lower_limit = boost::lexical_cast<double>(
					soft_lower_limit_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("soft_lower_limit value (%s) is not a float: %s",soft_lower_limit_str, e.what());
			return false;
		}
	}

	// Get soft_upper_limit joint limit
	const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
	if (soft_upper_limit_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_safety: no soft_upper_limit, using default value");
		js.soft_upper_limit = 0;
	} else {
		try {
			js.soft_upper_limit = boost::lexical_cast<double>(
					soft_upper_limit_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("soft_upper_limit value (%s) is not a float: %s",soft_upper_limit_str, e.what());
			return false;
		}
	}

	// Get k_position_ safety "position" gain - not exactly position gain
	const char* k_position_str = config->Attribute("k_position");
	if (k_position_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_safety: no k_position, using default value");
		js.k_position = 0;
	} else {
		try {
			js.k_position = boost::lexical_cast<double>(k_position_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("k_position value (%s) is not a float: %s",k_position_str, e.what());
			return false;
		}
	}
	// Get k_velocity_ safety velocity gain
	const char* k_velocity_str = config->Attribute("k_velocity");
	if (k_velocity_str == NULL) {
		//CONSOLE_BRIDGE_logError("joint safety: no k_velocity");
		return false;
	} else {
		try {
			js.k_velocity = boost::lexical_cast<double>(k_velocity_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("k_velocity value (%s) is not a float: %s",k_velocity_str, e.what());
			return false;
		}
	}

	return true;
}

bool KDLParser::parseJointCalibration(urdf::JointCalibration &jc,
		TiXmlElement* config) {
	jc.clear();

	// Get rising edge position
	const char* rising_position_str = config->Attribute("rising");
	if (rising_position_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_calibration: no rising, using default value");
		jc.rising.reset();
	} else {
		try {
			jc.rising.reset(
					new double(
							boost::lexical_cast<double>(rising_position_str)));
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("risingvalue (%s) is not a float: %s",rising_position_str, e.what());
			return false;
		}
	}

	// Get falling edge position
	const char* falling_position_str = config->Attribute("falling");
	if (falling_position_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_calibration: no falling, using default value");
		jc.falling.reset();
	} else {
		try {
			jc.falling.reset(
					new double(
							boost::lexical_cast<double>(falling_position_str)));
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("fallingvalue (%s) is not a float: %s",falling_position_str, e.what());
			return false;
		}
	}

	return true;
}

bool KDLParser::parseJointMimic(urdf::JointMimic &jm, TiXmlElement* config) {
	jm.clear();

	// Get name of joint to mimic
	const char* joint_name_str = config->Attribute("joint");

	if (joint_name_str == NULL) {
		//CONSOLE_BRIDGE_logError("joint mimic: no mimic joint specified");
		return false;
	} else
		jm.joint_name = joint_name_str;

	// Get mimic multiplier
	const char* multiplier_str = config->Attribute("multiplier");

	if (multiplier_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_mimic: no multiplier, using default value of 1");
		jm.multiplier = 1;
	} else {
		try {
			jm.multiplier = boost::lexical_cast<double>(multiplier_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("multiplier value (%s) is not a float: %s",multiplier_str, e.what());
			return false;
		}
	}

	// Get mimic offset
	const char* offset_str = config->Attribute("offset");
	if (offset_str == NULL) {
		//CONSOLE_BRIDGE_logDebug("urdfdom.joint_mimic: no offset, using default value of 0");
		jm.offset = 0;
	} else {
		try {
			jm.offset = boost::lexical_cast<double>(offset_str);
		} catch (boost::bad_lexical_cast &e) {
			//CONSOLE_BRIDGE_logError("offset value (%s) is not a float: %s",offset_str, e.what());
			return false;
		}
	}

	return true;
}

bool KDLParser::parseJoint(urdf::Joint &joint, TiXmlElement* config) {
	joint.clear();

	// Get Joint Name
	const char *name = config->Attribute("name");
	if (!name) {
		//CONSOLE_BRIDGE_logError("unnamed joint found");
		return false;
	}
	joint.name = name;

	// Get transform from Parent Link to Joint Frame
	TiXmlElement *origin_xml = config->FirstChildElement("origin");
	if (!origin_xml) {
		//CONSOLE_BRIDGE_logDebug("urdfdom: Joint [%s] missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform).", joint.name.c_str());
		joint.parent_to_joint_origin_transform.clear();
	} else {
		if (!parsePose(joint.parent_to_joint_origin_transform, origin_xml)) {
			joint.parent_to_joint_origin_transform.clear();
			//CONSOLE_BRIDGE_logError("Malformed parent origin element for joint [%s]", joint.name.c_str());
			return false;
		}
	}

	// Get Parent Link
	TiXmlElement *parent_xml = config->FirstChildElement("parent");
	if (parent_xml) {
		const char *pname = parent_xml->Attribute("link");
		if (!pname) {
			//CONSOLE_BRIDGE_logInform("no parent link name specified for Joint link [%s]. this might be the root?", joint.name.c_str());
		} else {
			joint.parent_link_name = std::string(pname);
		}
	}

	// Get Child Link
	TiXmlElement *child_xml = config->FirstChildElement("child");
	if (child_xml) {
		const char *pname = child_xml->Attribute("link");
		if (!pname) {
			//CONSOLE_BRIDGE_logInform("no child link name specified for Joint link [%s].", joint.name.c_str());
		} else {
			joint.child_link_name = std::string(pname);
		}
	}

	// Get Joint type
	const char* type_char = config->Attribute("type");
	if (!type_char) {
		//CONSOLE_BRIDGE_logError("joint [%s] has no type, check to see if it's a reference.", joint.name.c_str());
		return false;
	}

	std::string type_str = type_char;
	if (type_str == "planar")
		joint.type = urdf::Joint::PLANAR;
	else if (type_str == "floating")
		joint.type = urdf::Joint::FLOATING;
	else if (type_str == "revolute")
		joint.type = urdf::Joint::REVOLUTE;
	else if (type_str == "continuous")
		joint.type = urdf::Joint::CONTINUOUS;
	else if (type_str == "prismatic")
		joint.type = urdf::Joint::PRISMATIC;
	else if (type_str == "fixed")
		joint.type = urdf::Joint::FIXED;
	else {
		//CONSOLE_BRIDGE_logError("Joint [%s] has no known type [%s]", joint.name.c_str(), type_str.c_str());
		return false;
	}

	// Get Joint Axis
	if (joint.type != urdf::Joint::FLOATING
			&& joint.type != urdf::Joint::FIXED) {
		// axis
		TiXmlElement *axis_xml = config->FirstChildElement("axis");
		if (!axis_xml) {
			//CONSOLE_BRIDGE_logDebug("urdfdom: no axis elemement for Joint link [%s], defaulting to (1,0,0) axis", joint.name.c_str());
			joint.axis = urdf::Vector3(1.0, 0.0, 0.0);
		} else {
			if (axis_xml->Attribute("xyz")) {
				try {
					joint.axis.init(axis_xml->Attribute("xyz"));
				} catch (urdf::ParseError &e) {
					joint.axis.clear();
					//CONSOLE_BRIDGE_logError("Malformed axis element for joint [%s]: %s", joint.name.c_str(), e.what());
					return false;
				}
			}
		}
	}

	// Get limit
	TiXmlElement *limit_xml = config->FirstChildElement("limit");
	if (limit_xml) {
		joint.limits.reset(new urdf::JointLimits());
		if (!parseJointLimits(*joint.limits, limit_xml)) {
			//CONSOLE_BRIDGE_logError("Could not parse limit element for joint [%s]", joint.name.c_str());
			joint.limits.reset();
			return false;
		}
	} else if (joint.type == urdf::Joint::REVOLUTE) {
		//CONSOLE_BRIDGE_logError("Joint [%s] is of type REVOLUTE but it does not specify limits", joint.name.c_str());
		return false;
	} else if (joint.type == urdf::Joint::PRISMATIC) {
		//CONSOLE_BRIDGE_logError("Joint [%s] is of type PRISMATIC without limits", joint.name.c_str());
		return false;
	}

	// Get safety
	TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
	if (safety_xml) {
		joint.safety.reset(new urdf::JointSafety());
		if (!parseJointSafety(*joint.safety, safety_xml)) {
			//CONSOLE_BRIDGE_logError("Could not parse safety element for joint [%s]", joint.name.c_str());
			joint.safety.reset();
			return false;
		}
	}

	// Get calibration
	TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
	if (calibration_xml) {
		joint.calibration.reset(new urdf::JointCalibration());
		if (!parseJointCalibration(*joint.calibration, calibration_xml)) {
			//CONSOLE_BRIDGE_logError("Could not parse calibration element for joint  [%s]", joint.name.c_str());
			joint.calibration.reset();
			return false;
		}
	}

	// Get Joint Mimic
	TiXmlElement *mimic_xml = config->FirstChildElement("mimic");
	if (mimic_xml) {
		joint.mimic.reset(new urdf::JointMimic());
		if (!parseJointMimic(*joint.mimic, mimic_xml)) {
			//CONSOLE_BRIDGE_logError("Could not parse mimic element for joint  [%s]", joint.name.c_str());
			joint.mimic.reset();
			return false;
		}
	}

	// Get Dynamics
	TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
	if (prop_xml) {
		joint.dynamics.reset(new urdf::JointDynamics());
		if (!parseJointDynamics(*joint.dynamics, prop_xml)) {
			//CONSOLE_BRIDGE_logError("Could not parse joint_dynamics element for joint [%s]", joint.name.c_str());
			joint.dynamics.reset();
			return false;
		}
	}

	return true;
}

bool KDLParser::initTreeAndChainFromURDFString(const std::string& urdfString,
		const std::string& root_link, const std::string& tip_link,
		KDL::Tree& kdl_tree, KDL::Chain& kdl_chain) {
	boost::shared_ptr<urdf::ModelInterface> urdf_modelPtr = parseURDF(
			urdfString);

	if (!urdf_modelPtr) {
		log(Error) << "[KDLParser] Could not parse URDF" << endlog();
		return false;
	}

	for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator joint =
			urdf_modelPtr->joints_.begin();
			joint != urdf_modelPtr->joints_.end(); ++joint) {
		if (joint->second->limits) {
			if (joint->second->limits->lower == joint->second->limits->upper) {
				// HACK: Setting pseudo fixed-joints to FIXED, so that KDL does not considers them.
				joint->second->type = urdf::Joint::FIXED;
				log(Info) << "[KDLParser] Removing fixed joint " << joint->second->name
						<< endlog();
			}
		}
	}

	if (!treeFromUrdfModel(urdf_modelPtr, kdl_tree)) {
		log(Error) << "[KDLParser] Failed to construct KDL tree" << endlog();
		return false;
	}

	if (!kdl_tree.getChain(root_link, tip_link, kdl_chain)) {
		log(Error) << "[KDLParser] Failed to construct KDL chain" << endlog();
		return false;
	}
	return true;
}
