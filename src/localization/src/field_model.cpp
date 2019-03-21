// Model for the soccer field including definition for field parameters
// dependencies:'field_model.h' header file containing declaration of funtions
//              'boost' from ubuntu for checking classes matching with templates in case of errors
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.04.06

#include "field_model.h"
#include <ros/node_handle.h>
#include <boost/concept_check.hpp>
#include <tf/tf.h>

namespace field_model
{

FieldModel* FieldModel::m_instance = 0;

// Define the three forms that World Objects have
WorldObject::WorldObject(WorldObject::Type type, const Eigen::Vector3d& pose)
 : m_type(type)
 , m_pos(pose)
{
}

WorldObject::WorldObject(WorldObject::Type type, double x, double y, double t)
 : m_type(type)
{
	m_pos << x, y, t;
}

WorldObject::WorldObject(WorldObject::Type type, const std::vector< Eigen::Vector2d >& points)
 : m_type(type)
 , m_pos(0.0, 0.0, 0.0)
 , m_points(points)
{
}

void WorldObject::setPose(const Eigen::Vector3d& pose)
{
	m_pos = pose;
}

// If object can be found on both left and right sides of the field, then mirror X
WorldObject WorldObject::mirrorX() const
{
	WorldObject ret;

	ret.m_type = m_type;
	ret.m_pos << -m_pos.x(), m_pos.y(), picut(M_PI - m_pos.z());

	for(size_t i = 0; i < m_points.size(); ++i)
	{
		ret.m_points.push_back(Eigen::Vector2d(-m_points[i].x(), m_points[i].y()));
	}

	return ret;
}

// If object can be found on both upper and lower sides of the field, then mirror Y
WorldObject WorldObject::mirrorY() const
{
	WorldObject ret;

	ret.m_type = m_type;
	ret.m_pos << m_pos.x(), -m_pos.y(), -m_pos.z();

	for(size_t i = 0; i < m_points.size(); ++i)
	{
		ret.m_points.push_back(Eigen::Vector2d(m_points[i].x(), -m_points[i].y()));
	}

	return ret;
}

// Field model with all the parameters defined
FieldModel::FieldModel()
{
	ros::NodeHandle nh;

	std::string field_type;
    // Get field type from parameter server - not used for stage 1
    // nh.param("/field_type", field_type, std::string("teensize"));

    // ROS_INFO("Loading field model for field type '%s'", field_type.c_str());
    field_type = "adultsize";
    if(field_type == "tsinghua")
	{
		m_length = 6.0;
		m_width = 4.0;
		m_centerCircleDiameter = 1.2;
        m_goalWidth = 2.7;
        m_goalAreaWidth = 3.0;
		m_goalAreaDepth = 0.6;
		m_penaltyMarkerDist = 1.8;
	}
	else if(field_type == "kidsize")
	{
        m_length = 9.0; //new as of 2014
        m_width = 6.0; //new as of 2014
        m_centerCircleDiameter = 1.5;  //new as of 2014
        m_goalWidth = 1.8; //new as of 2015
        m_goalAreaWidth = 3.45; //new as of 2014
		m_goalAreaDepth = 0.6;
		m_penaltyMarkerDist = 1.8;
	}
    else if(field_type == "adultsize")
    {
        m_length = 9.0;
        m_width = 6.0;
        m_centerCircleDiameter = 1.5;
        m_goalWidth = 2.6;//new as of 2015
        m_goalAreaWidth = 5.0; //new as of 2014
        m_goalAreaDepth = 1.0;
        m_penaltyMarkerDist = 2.1;
    }
	else
	{
		ROS_FATAL("Invalid field type '%s'. Check /field_type param!", field_type.c_str());
		abort();
	}

	double hw = m_width / 2.0;
	double hl = m_length / 2.0;
	std::vector<Eigen::Vector2d> points;

	// Goals & posts
	addObject(WorldObject::Type_Goal,     -hl, 0.0, 0.0, MirrorX);
	addObject(WorldObject::Type_GoalPost, -hl, m_goalWidth/2.0, 0.0, MirrorAll);

	// Center circle
	addObject(WorldObject::Type_Circle, 0.0, 0.0, 0.0, NoMirror);

	// Penalty markers
    addObject(WorldObject::Type_PenMarker, -hl + m_penaltyMarkerDist, 0.0, 0.0, MirrorX);

	// Center line
	points.clear();
	points.push_back(Eigen::Vector2d(0.0, hw));
	points.push_back(Eigen::Vector2d(0.0, -hw));
	addLine(points, NoMirror);

	// Side lines
	points.clear();
	points.push_back(Eigen::Vector2d(-hl, hw));
	points.push_back(Eigen::Vector2d(hl, hw));
	addLine(points, MirrorY);

	// Goal lines
	points.clear();
	points.push_back(Eigen::Vector2d(-hl, hw));
	points.push_back(Eigen::Vector2d(-hl, -hw));
	addLine(points, MirrorX);

	// Goal area lines (long line)
	points.clear();
	points.push_back(Eigen::Vector2d(-hl + m_goalAreaDepth, m_goalAreaWidth/2.0));
	points.push_back(Eigen::Vector2d(-hl + m_goalAreaDepth, -m_goalAreaWidth/2.0));
	addLine(points, MirrorX);

	// Goal area lines (small line)
	points.clear();
	points.push_back(Eigen::Vector2d(-hl, m_goalAreaWidth/2.0));
	points.push_back(Eigen::Vector2d(-hl + m_goalAreaDepth, m_goalAreaWidth/2.0));
	addLine(points, MirrorAll);

    // Corner L Crossings (theta is the half angle between the two lines)
    addObject(WorldObject::Type_LineCrossL, -hl, hw, -M_PI/4.0, MirrorAll);

    // Goal area L Line Crossings (theta is the half angle between the two lines)
    addObject(WorldObject::Type_LineCrossL, -hl + m_goalAreaDepth, m_goalAreaWidth/2.0, -3.0*M_PI/4.0, MirrorAll);

    // Goal area T Line Crossings (theta points along the central line)
    addObject(WorldObject::Type_LineCrossT, -hl, m_goalAreaWidth/2.0, 0.0, MirrorAll);
    addObject(WorldObject::Type_LineCrossT, 0.0, -hw, MirrorY);

    // Central X Line Crossings
    addObject(WorldObject::Type_LineCrossX, 0.0, m_centerCircleDiameter/2.0, 0.0, MirrorY);

    // Imu head yaw orientation

    // Don't use orientation for now
    //m_imuHeading = addObject(WorldObject::Type_ImuHeading, NAN, NAN, 0.0, NoMirror);


}


// Function to add a new object to our field model
WorldObject* FieldModel::addObject(WorldObject::Type type, double x, double y, double t, int flags)
{
	std::vector<WorldObject>* list = &m_objects[type];

	WorldObject obj(type, x, y, t);
	list->push_back(obj);

	WorldObject* ret = &list->back();

	if(flags & MirrorX)
		list->push_back(obj.mirrorX());

	if(flags & MirrorY)
		list->push_back(obj.mirrorY());

	if((flags & MirrorX) && (flags & MirrorY))
		list->push_back(obj.mirrorX().mirrorY());

	return ret;
}

// Function to add a new line to our field model
void FieldModel::addLine(const std::vector<Eigen::Vector2d>& points, int flags)
{
	std::vector<WorldObject>* list = &m_objects[WorldObject::Type_FieldLine];

	WorldObject obj(WorldObject::Type_FieldLine, points);
	list->push_back(obj);

	if(flags & MirrorX)
		list->push_back(obj.mirrorX());

	if(flags & MirrorY)
		list->push_back(obj.mirrorY());

	if((flags & MirrorX) && (flags & MirrorY))
		list->push_back(obj.mirrorX().mirrorY());
}

// Function to update the Imu head yaw heading - TODO: should be changed to be updated only with a callback function of a topic
void FieldModel::updateImuHeading(float heading)
{
    m_imuHeading->setPose(Eigen::Vector3d(NAN, NAN, heading));
}

double FieldModel::imuHeading() const
{
    // return theta-orientation from the m_imuHeading object, which is in the 3position of the Eigen::Vector3d m_pos
    return m_imuHeading->m_pos(2);
}

// Function to get new instance of our field model and its objects
FieldModel* FieldModel::getInstance()
{
	if(!m_instance)
		m_instance = new FieldModel;

	return m_instance;
}

}
