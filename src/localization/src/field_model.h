// Model for the soccer field
// dependencies: 'eigen3' from ubuntu (sudo apt-get install libeigen3-dev) usually already provided for matrix operations
//               'pf_functions.h' including useful mathematical functions and declarations
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.04.06

#ifndef FIELD_MODEL_H
#define FIELD_MODEL_H

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include "pf_functions.h"

namespace field_model
{

/**
 * Objects occuring in our soccer world.
 **/
class WorldObject
{
public:
	//! Object type
	enum Type
	{
        Type_Circle,      //< Center of central circle, 0
        Type_Goal,        //< Goal: center of two goalposts, 1
        Type_GoalPost,    //< A single goal post, 2
        Type_PenMarker,     //< One of the two penalty markers, 3
        Type_FieldLine,   //< Field line, 4
        Type_LineCrossT,   //< T-crossing of two lines, 5
        Type_LineCrossX,   //< X-crossing of two lines, 6
        Type_LineCrossL,   //< L-crossing of two lines, 7
        Type_ImuHeading, //< Imu heading-Yaw, 8
		NumTypes
	};

	WorldObject(Type type, const Eigen::Vector3d& pose);
	WorldObject(Type type, double x, double y, double t = 0.0);
	WorldObject(Type type, const std::vector<Eigen::Vector2d>& points);

	WorldObject mirrorX() const;
	WorldObject mirrorY() const;

    // Object pose (x, y, theta) specific for namespace field_model
	inline Eigen::Vector3d pose() const
	{ return m_pos; }

    // Points belonging to the object (e.g. line start and end)
	inline const std::vector<Eigen::Vector2d>& points() const
	{ return m_points; }

    // Object type
	inline Type type() const
	{ return m_type; }
protected:
	void setPose(const Eigen::Vector3d& pose);
private:
	friend class FieldModel;

	WorldObject() {}

	Type m_type;
	Eigen::Vector3d m_pos;
	std::vector<Eigen::Vector2d> m_points;
};

/**
 * @brief Model of the soccer field
 *
 * This adapts to the value of the `/field_type` parameter set inside the .cpp file.
 * Supported values are:
 *
 * <table>
 *   <tr>
 *      <td>'tsinghua'</td>
 *      <td>The soccer field in our lab</td>
 *   </tr>
 *   <tr>
 *      <td>'kidsize'/'adultsize'</td>
 *      <td>Official kidsize/adultsize playing field</td>
 *   </tr>
 * </table>
 *
 * All distances are given in meters.
 **/
class FieldModel
{
public:
	static FieldModel* getInstance();

    // Field width (inside the lines)
	inline double width() const
	{ return m_width; }

    // Field length (inside the lines)
	inline double length() const
	{ return m_length; }

    // Goal width
	inline double goalWidth() const
	{ return m_goalWidth; }

    // Width of the penalty area before each goal
	inline double goalAreaWidth() const
	{ return m_goalAreaWidth; }

    // Depth of the penalty area before each goal
	inline double goalAreaDepth() const
	{ return m_goalAreaDepth; }

    // Diameter of the central circle
	inline double centerCircleDiameter() const
	{ return m_centerCircleDiameter; }

    // Distance from the goal line to the penalty marker
	inline double penaltyMarkerDist() const
	{ return m_penaltyMarkerDist; }

    // Objects for a specific WorldObject::Type
	inline const std::vector<WorldObject>& objects(WorldObject::Type type) const
	{ return m_objects[type]; }

    // Functions for the Imu yaw provided by the gyro sensor

    // void setImuHeading(double heading); // not used anymore, being set only with callback function

    double imuHeading() const;
private:
	FieldModel();

	enum MirrorFlag
	{
		NoMirror = 0,
		MirrorX = (1 << 0),
		MirrorY = (1 << 1),
		MirrorAll = MirrorX | MirrorY
	};
	WorldObject* addObject(WorldObject::Type type, double x, double y, double t, int flags = MirrorX | MirrorY);
	void addLine(const std::vector<Eigen::Vector2d>& points, int flags = MirrorX | MirrorY);

    void updateImuHeading(float heading);


	static FieldModel* m_instance;

	double m_width;
	double m_length;
	double m_goalWidth;
	double m_goalAreaWidth;
	double m_goalAreaDepth;
	double m_centerCircleDiameter;
	double m_penaltyMarkerDist;

	std::vector<WorldObject> m_objects[WorldObject::NumTypes];

    // The Imu yaw heading will be set by the gyro sensor message after subscribing on the topic
    WorldObject* m_imuHeading;
};

}

#endif
