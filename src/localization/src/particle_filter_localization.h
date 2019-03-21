// Header file for Robot Self-Localization using Augmented-Adaptive Particle filter described in 'particle_filter.h' on the
// soccer field described in the 'field_model.h' and 'field_model.cpp'. Header file contains declarations of classes
// UniformParticleInitializer, PoseParticleInitializer, OdometryModel, LandmarkObservation, ObservationModel and LocalizationPF.
// Dependencies: 'particle_filter.h' including the particle filter
//               'field_model.h' including the model of the soccer field
//               'gsl' from ubuntu (sudo apt-get install libgsl0-dev) for random number generator and random distance
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.04.06

#ifndef LOCALIZATION_PF_H
#define LOCALIZATION_PF_H

#include "particle_filter.h"
#include "field_model.h"

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "localization/WorldObjects.h"

#include "decision/SerialReceived.h"

using namespace PF;

class LandmarkObservation;
class LocalizationPF;

typedef ParticleFilter<Vec3f, Vec3f, LandmarkObservation, Mat22f> ParticleFilterT;


// Creates uniformly distributed particles around center of field (0,0)
class UniformParticleInitializer : public ParticleFilterT::InitializerT
{
public:
    UniformParticleInitializer();
    virtual ~UniformParticleInitializer();

    void initParticle(ParticleT& particle);
private:
    gsl_rng* m_rng;
    field_model::FieldModel* m_field;
};

// Creates particles distributed around a mean pose with specific standard deviation
class PoseParticleInitializer : public ParticleFilterT::InitializerT
{
public:
    PoseParticleInitializer(const Vec3f& pose, const Vec3f& stddev);
    virtual ~PoseParticleInitializer();

    void initParticle(ParticleT& particle);
private:
    Vec3f m_pose;
    Vec3f m_stddev;
    gsl_rng* m_rng;
    field_model::FieldModel* m_field;
};

// Definition of Odometry Model for refreshing particles after control input
class OdometryModel : public ParticleFilterT::MotionModelT
{
public:
    OdometryModel();
    virtual ~OdometryModel();

    virtual Vec3f sampleState(const Vec3f& lastPose, const Vec3f& controlInput, float delta_t);
private:
    Eigen::Matrix3f m_A;
    gsl_rng* m_rng;
};

//
class LandmarkObservation : public field_model::WorldObject
{
public:
    LandmarkObservation(Type type, const Vec3f& pose);
    virtual ~LandmarkObservation();

    inline void setConfidence(float conf)
    { m_confidence = conf; }

    inline float confidence() const
    { return m_confidence; }

    void setAssociatedObject(const field_model::WorldObject* obj)
    { m_association = obj; }

    const field_model::WorldObject* associatedObject() const
    { return m_association; }

    void setMahalDist(double dist, double maxDist)
    { m_mahalDist = dist; m_maxMahalDist = maxDist; m_isMahalSet = true; }

    float mahalDist() const
    { return m_mahalDist; }

    float angle() const
    { return m_angle; }

    float distance() const
    { return m_distance; }

    void precalc();

    // Function converting mahalanobis distance to likelihood
    inline void
    mahal_dists_to_likelihoods(Likelihood& lik){
        lik.minLikelihood = m_alphauniform_X_max2rangeinv_X_pi2inv
            + m_alpha_exp
            * exp(-lik.min_mahal_dist);
        // if the mahalanobis distance is very big, set likelihood equal to min likelihood, else calculate
        if(lik.mahal_dist > INT_MAX/2)
            lik.likelihood = lik.minLikelihood;
        else
            lik.likelihood = m_alphauniform_X_max2rangeinv_X_pi2inv
            + m_alpha_exp
            * exp(-lik.mahal_dist);
    }

    inline bool isMahalSet() const
    { return m_isMahalSet; }

    // Function to get minimum and plain mahalanobis distance inside a likelihood structure
    inline Likelihood
    getMinAndLikelihood(){
        assert(m_isMahalSet);
        Likelihood mal;
        mal.mahal_dist     = m_mahalDist;
        mal.min_mahal_dist = m_maxMahalDist;
        return mal;
    }

private:
    friend class ObservationModel;

    float m_distance;
    float m_angle;
    float m_confidence;
    const field_model::WorldObject* m_association;
    double m_mahalDist;
    double m_maxMahalDist;

    bool m_isMahalSet;

    // Precalculated values
    double m_var_angle;
    double m_var_distance;
    double m_3var_distance;
    double m_3var_angle;
    double m_alpha_exp;
    double m_alphauniform_X_max2rangeinv_X_pi2inv;
};

class ObservationModel : public ParticleFilterT::ObservationModelT
{
public:
    explicit ObservationModel(LocalizationPF* pf);
    ~ObservationModel();

    virtual double likelihood(const ParticleT& particle, const LandmarkObservation& observation);
    virtual double minLikelihood(const ParticleT& particle, const LandmarkObservation& observation);
    virtual Likelihood minAndLikelihood(const ParticleT& particle, const LandmarkObservation& observation);
    virtual double likelihoodDiscount(const LandmarkObservation& observation);

    void precalculateVariances(LandmarkObservation& obs);
private:
    void updateVariances();

    LocalizationPF* m_pf;
    field_model::FieldModel* m_field;

    // Parameters that can be set by parameter server, but now we set them according to standard values.
    float m_std_angle;
    float m_std_distance;
    float m_std_imu_yaw;
    float m_alpha_uniform_base;

    float m_var_angle;
    float m_var_distance;
    float m_var_imu_yaw;
    float m_3var_imu_yaw;
};

class LocalizationPF : public ParticleFilterT
{
public:
    LocalizationPF(ros::NodeHandle& nh);
    virtual ~LocalizationPF();

    void newStep();

    virtual void establishDataAssociation(
        ParticleT& particle, std::vector<LandmarkObservation>& observations
    );
private:
    struct DataAssociationPair
    {
        DataAssociationPair(double _dist, const field_model::WorldObject* _object)
         : mahal_dist(_dist), object(_object)
        {}

        double mahal_dist;
        const field_model::WorldObject* object;
    };

    void handleDetections(const localization::WorldObjectsConstPtr& msg);
    void determineMatch(
        std::vector<DataAssociationPair>& matches,
        double minLikelihood,
        LandmarkObservation* obs
    );

    void meanShift(
        const Vec2f& startCartPos, float startOri, float maxCartDist2,
        float maxAngDist, Vec3f & mean, double & weightSumFraction
    );
    void determinePose();

    void publishDebug();
    void publishTransform();

    void handleInitPose(const geometry_msgs::PoseWithCovarianceStamped& pose);

    bool isOutOfField(const ParticleT& particle) const;

    void handleImu(const decision::SerialReceived::ConstPtr& serial_input_msg);

    ros::Time timestamp()
    {
        if(m_vision_detections)
            return m_vision_detections->header.stamp;

        return m_odomTimestamp;
    }

//    ros::NodeHandle& PFHandle;

    field_model::FieldModel* m_field;
    tf::TransformListener m_tf;
    tf::TransformBroadcaster m_pub_tf;
    Vec3f m_odomPose;
    ros::Time m_lastTime;
    ros::Subscriber m_sub_vision;

    localization::WorldObjectsConstPtr m_vision_detections;
    ros::Time m_odomTimestamp;

    ros::Publisher m_pub_particle_set;
    ros::Publisher m_pub_mean_pose_conf_stamped;
    ros::Subscriber m_sub_initial_pose;

    ros::Subscriber m_sub_imu;

    Vec3f m_mean;
    double m_confidence;
    double m_lastConfidence;

    float m_actualUniformReplacementProbability; // not necessarily /effective/ URP
    bool  m_usingActualReplacement;

    ros::Time is_vision_detection;

    // involving kidnapping
    double waist_yaw;
    bool is_lifted_before;
    bool is_lifted_now;
    float time_when_lifted;
    float time_when_settled;
    double m_yaw_when_lifted;
    double m_theta_when_lifted;
    bool is_kidnapped;
};

#endif
