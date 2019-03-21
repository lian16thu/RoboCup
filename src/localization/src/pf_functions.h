// Math utilitites & typedefs for the particle filter
// dependencies: 'eigen3' from ubuntu (sudo apt-get install libeigen3-dev) usually already provided for matrix operations
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.03.05

#ifndef PF_MATH_H
#define PF_MATH_H

#include "eigen3/Eigen/Core"

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector2f Vec2f;

typedef Eigen::Vector2i Vec2i;

typedef Eigen::Matrix2f Mat22f;
typedef Eigen::Matrix3f Mat33f;


// Definition of 1D and 2D gaussians to be used for distributions
template<class T>
T gaussian(T x, T mean, T sigma)
{
    return 1.0 / (sigma * sqrt(2.0 * M_PI)) * exp(-0.5 * (x-mean)*(x-mean) / (sigma*sigma));
}

template<class T>
typename T::Scalar gaussian2d(const T& x, typename T::Scalar cx, typename T::Scalar cy, typename T::Scalar sigma)
{
    typename T::Scalar a = x.x() - cx;
    typename T::Scalar b = x.y() - cy;
    return 1.0 / (2.0 * M_PI * sigma*sigma) * exp(-(a*a+b*b) / 2.0 * sigma*sigma);
}

// Function to ensure orientation is within [-pi,pi]
inline double picut(double a)
{
	while(a > M_PI)
		a -= 2.0 * M_PI;
	while(a < -M_PI)
		a += 2.0 * M_PI;

	return a;
}

namespace PF
{

// Pose type including m_pose {x,y,t(theta)} and confidence of pose m_conf, along with useful functions
class Pose
{
public:
private:
	Vec3f m_pose;
	float m_conf;
public:
	Pose()
	{
		reset();
	};

	Pose(float x, float y, float t, float c = 0)
	 : m_pose(x, y, t)
	 , m_conf(c)
	{}

	inline void init(float x, float y, float t, float c = 0)
	{
		m_pose << x, y, t;
		m_conf = c;
	}

	inline void reset()
	{
		m_pose.setZero();
		m_conf = 0.0;
	}

	inline float& x() { return m_pose.x(); }
	inline float& y() { return m_pose.y(); }
	inline float& t() { return m_pose.z(); }
	inline float& conf() { return m_conf; }

	inline float x() const { return m_pose.x(); }
	inline float y() const { return m_pose.y(); }
	inline float t() const { return m_pose.z(); }
	inline float conf() const { return m_conf; }

	inline Vec2f position() const { return m_pose.head<2>(); }
};

}

#endif
