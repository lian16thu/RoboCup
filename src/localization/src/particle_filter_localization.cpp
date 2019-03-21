// Robot Self-Localization using Augmented-Adaptive Particle filter described in 'particle_filter.h' on the soccer
// field described in the 'field_model.h' and 'field_model.cpp'. Code file contains Intitialization of the Particle Filter,
// the Motion Model, the Observation Model and the actual Localization code until the dominant pose is calculated.
// Dependencies: 'particle_filter_localization.h' the header file including declarations
//               'boost' from ubuntu for use of the foreach iterator
//               'particle_set.h' the header file for the header set ROS message
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.04.06

#include "particle_filter_localization.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/init.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/foreach.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>



#include <localization/ParticleSet.h>
#include <localization/MeanPoseConfStamped.h>



// Set up random number generator //çæéæºæ°
static gsl_rng* allocate_rng()
{
    gsl_rng_env_setup();
    const gsl_rng_type* T = gsl_rng_default;
    gsl_rng* rng = gsl_rng_alloc(T);
    gsl_rng_set(rng, (unsigned long)time(NULL));

    return rng;
}

// Creates uniformly distributed particles around center of field (0,0)
// Constructor
UniformParticleInitializer::UniformParticleInitializer()
{
    m_rng = allocate_rng();

    m_field = field_model::FieldModel::getInstance();
}

// Destructor
UniformParticleInitializer::~UniformParticleInitializer()
{
    gsl_rng_free(m_rng);
}

// Initialization function //åå§å
void UniformParticleInitializer::initParticle(
        ParticleFilterT::InitializerT::ParticleT& particle)
{
    float fl = m_field->length() + 1.0;
    float fw = m_field->width() + 1.0;

    // Set particle state in the middle according to the field's length and width. Also 0 angle
    particle.setState(Vec3f(
                          -fl/2.0 + fl*gsl_rng_uniform(m_rng),
                          -fw/2.0 + fw*gsl_rng_uniform(m_rng),
                          -M_PI + 2.0*M_PI*gsl_rng_uniform(m_rng)
                          ));

    // stores the orientation of the particle in rad
    particle.getUserData() = Eigen::Rotation2D<float>(particle.getState().z());
}

// Creates particles distributed around a mean pose with specific standard deviation
// Constructor
PoseParticleInitializer::PoseParticleInitializer(const Vec3f& pose, const Vec3f& stddev) : m_pose(pose) , m_stddev(stddev) , m_field(field_model::FieldModel::getInstance())
{
    m_rng = allocate_rng();
}

// Destructor
PoseParticleInitializer::~PoseParticleInitializer()
{
    gsl_rng_free(m_rng);
}

// Initialization function
void PoseParticleInitializer::initParticle(
        PoseParticleInitializer::ParticleT& particle)
{
    // Set particle state around the pose with gaussian distribution of the given standard deviation
    particle.setState(Vec3f(
                          m_pose.x() + gsl_ran_gaussian(m_rng, m_stddev.x()),
                          m_pose.y() + gsl_ran_gaussian(m_rng, m_stddev.y()),
                          picut(m_pose.z() + gsl_ran_gaussian(m_rng, m_stddev.z()))
                          ));

    // stores the orientation of the particle in rad
    particle.getUserData() = Eigen::Rotation2D<float>(particle.getState().z());
}

////////////////////////////////////////////////////////////////////////////////
// Motion model

// Constructor where the initial covariance-uncertainty of the control is defined
OdometryModel::OdometryModel()
{
    // Covariance matrix (x, y, theta) for the uncertainty of control. TODO: check numbers according to our uncertainty of walking
    // Set it small because we need to trust this
    m_A <<
           0.02, 0.02, 0.02 / M_PI,
            0.02, 0.02, 0.02 / M_PI,
            0.04, 0.04, 0.02f
            ;

    m_rng = allocate_rng();
}

// Destructor
OdometryModel::~OdometryModel()
{
}

// Function for updating the state of a particle given the previous pose and the control input
// after a specific time delta_t has passed
Vec3f OdometryModel::sampleState(const Vec3f& lastPose,
                                 const Vec3f& controlInput, float delta_t)
{
    //delta_t = 0.05; // every 0.05sec we get a new control input
    Vec3f mod_deltaT(delta_t, delta_t, 1.57 * delta_t); // numbers set empirically
    //                                    ^- TODO: what?

    // Set the input according to the message read from the control topic
    Vec3f abs_input = controlInput.array().abs();

    // Since we want the standard deviation to be greater than zero even
    // if we are not moving, use a sum of the elapsed time and the
    // supposedly covered distance.

    Vec3f std_dev = 0.5 * mod_deltaT + 0.1 * m_A * abs_input; //numbers set empirically
    //    Vec3f std_dev = 0.5 * mod_deltaT;
    Vec3f noisyControlInput = controlInput ; // check if it's better we don't add extra noise

    //    Vec3f noisyControlInput = controlInput + Vec3f(
    //        gsl_ran_gaussian(m_rng, std_dev.x()),
    //        gsl_ran_gaussian(m_rng, std_dev.y()),
    //        gsl_ran_gaussian(m_rng, std_dev.z())
    //    );

    //    Vec3f noisyControlInput = controlInput + Vec3f(std_dev.x(), std_dev.y(), std_dev.z()	);

    Vec2f dPose = noisyControlInput.head<2>();
    // multiplies the translational displacement with the vector of orientation
    //dPose = Eigen::Rotation2D<float>(lastPose.z()) * dPose;

    //ROS_INFO("current move %f %f %f",dPose.x(),dPose.y(),noisyControlInput.z());

    return Vec3f(
                lastPose.x() + dPose.x(),
                lastPose.y() + dPose.y(),		//picut(lastPose.z() + noisyControlInput.z())
                picut(lastPose.z() + noisyControlInput.z())
                );
}

////////////////////////////////////////////////////////////////////////////////
// Observation model

// Constructor
ObservationModel::ObservationModel(LocalizationPF* pf)
    : m_pf(pf)
    , m_field(field_model::FieldModel::getInstance())
{
    // Set some standard values for the standard deviations
    m_std_angle = 0.4;
    m_std_distance = 0.5;
    m_std_imu_yaw = 1/180* M_PI;  //standard deviation for yaw provided by imu, 5 degrees
    m_alpha_uniform_base = 0.2;
    updateVariances();
}

// Destructor
ObservationModel::~ObservationModel()
{
}

// Function to update variances according to standard deviations (for now variances are fixed to standard values)
void ObservationModel::updateVariances()
{
    m_var_angle = m_std_angle*m_std_angle;
    m_var_distance = m_std_distance*m_std_distance;
    m_var_imu_yaw = m_std_imu_yaw * m_std_imu_yaw;

    m_3var_imu_yaw = 9.0 * m_var_imu_yaw;
}

// Function to calculate variances of an observation
void ObservationModel::precalculateVariances(LandmarkObservation& obs)
{
    const double max_2range = 2.0 * 10.0;
    const double max_2range_inv = 1.0 / max_2range;
    const double pi2_inv = 0.5 / M_PI;

    double alpha_uniform = m_alpha_uniform_base + (1.0 - m_alpha_uniform_base) * (1.0 - obs.confidence());

    // 	double alpha_exp = 1.0 - alpha_uniform;

    double std_range = m_std_distance + 0.25 * obs.distance();
    obs.m_var_distance = std_range*std_range;
    obs.m_var_angle = m_var_angle;

    obs.m_3var_distance = 9.0 * obs.m_var_distance;
    obs.m_3var_angle = 9.0 * obs.m_var_angle;

    obs.m_alpha_exp = 1.0 - alpha_uniform;
    obs.m_alphauniform_X_max2rangeinv_X_pi2inv = alpha_uniform * max_2range_inv * pi2_inv;
}

// Function to calculate the minimum and plain mahalanobis distance given a particle and an observation
Likelihood ObservationModel::minAndLikelihood(
        const ObservationModel::ParticleT& particle, const LandmarkObservation& observation)
{
    Likelihood mal;

    mal.mahal_dist = likelihood(particle, observation);
    mal.min_mahal_dist = minLikelihood(particle, observation);

    return mal;
}


// Function that returns the mahalanobis distance, signifying the likelihood of an observation
double ObservationModel::likelihood(
        const ObservationModel::ParticleT& particle, const LandmarkObservation& observation)
{

    // Get the object associated with our observation
    const field_model::WorldObject* obj = observation.associatedObject();

    if(!obj)
    {
        //fprintf(stderr, "Warning: object without assoc\n");
        return INT_MAX; // if no association return maximum mahalanobis distance
    }

    double mahal_dist = 0.0;

    // Case for landmarks with a known position
    if(!isnan(obj->pose().x()) && !isnan(obj->pose().y())) //position of landmark (x,y) has to have numerical values
    {
        //ROS_INFO("known position");
        // Calculate the expected distance of particle to associated landmark
        Vec2f expCart = obj->pose().head<2>().cast<float>() - particle.getState().head<2>();

        // Multiply with the orientation of the particle in rad
        expCart = particle.getUserData().transpose() * expCart;

        // If the distance between the observed object and the particle is less that 4 meters (otherwise it's not worth calculating)
        // then calculate the mahalonobis distance by the angle deviation and the distance deviation from the expected ones, taking
        // the variances into consideration
        if(observation.distance() < 4.0)
        {
            float bearing_innov = picut(observation.angle() - atan2(expCart.y(), expCart.x()));

            mahal_dist += 0.5 * bearing_innov*bearing_innov / observation.m_var_angle;

            double range_innov = observation.distance() - expCart.norm();
            mahal_dist += 0.5 * range_innov*range_innov / observation.m_var_distance;
        }
    }

    // Case for landmarks with a known orientation
    if(!isnan(obj->pose().z()) && !isnan(observation.pose().z()))
    {
        //ROS_INFO("known orientation");
        // Calculate the expected angle of particle to associated landmark
        float expAngle = obj->pose().z() - particle.getState().z();

        // Add more to the mahalanobis distance according to the deviation from the expected angle difference
        float orient_innov = picut(observation.pose().z() - expAngle);
        mahal_dist += 0.5 * orient_innov*orient_innov / m_var_imu_yaw;
    }

    return mahal_dist;
}

// Define how much we trust the objects that were detected, by introducing a discount factor to multiply
// with the likelihood of actual observation, meaning the lower the factor, the less we trust the observation
double ObservationModel::likelihoodDiscount(const LandmarkObservation& observation)
{
    double start = 1;

    switch(observation.type())
    {
    case field_model::WorldObject::Type_PenMarker:
        return start * 1;
    case field_model::WorldObject::Type_GoalPost:
        return start * 1;

        // Trust L, T Xings less than X Xings. Much less because our code now is not so good
    case field_model::WorldObject::Type_LineCrossL:
        return start * 1;
    case field_model::WorldObject::Type_LineCrossT:
        return start * 1;
    case field_model::WorldObject::Type_LineCrossX:
        return start * 1;

    default:
        return start;
    }
}

// Function returning the minimum mahalanobis distance, associated with the minimum likelihood
// the calculation is based on the fixed variance values we determine in the beginning
double ObservationModel::minLikelihood(
        const ObservationModel::ParticleT& particle, const LandmarkObservation& observation)
{
    const field_model::WorldObject* obj = observation.associatedObject();

    if(!obj)
        return INT_MAX; // if no object is associated then the mahalanobis distance is set to maximum

    double min_mahal_dist = 0.0;

    if(!isnan(obj->pose().x()) && !isnan(obj->pose().y()))
    {
        min_mahal_dist += 0.5 * observation.m_3var_distance / observation.m_var_distance;
        min_mahal_dist += 0.5 * observation.m_3var_angle / observation.m_var_angle;
    }

    if(!isnan(obj->pose().z()) && !isnan(observation.pose().z()))
    {
        min_mahal_dist += 0.5 * m_3var_imu_yaw / m_var_imu_yaw;
    }

    return min_mahal_dist;
}


// Constructor of LandmarkObservation Class
LandmarkObservation::LandmarkObservation(Type type, const Vec3f& pose)
    : field_model::WorldObject(type, pose.cast<double>())
    , m_distance(pose.head<2>().norm())
    , m_angle(atan2(pose.y(), pose.x()))
    , m_confidence(1.0)
    , m_association(0)
    , m_isMahalSet(false)
{}

// Destructor
LandmarkObservation::~LandmarkObservation()
{
}

////////////////////////////////////////////////////////////////////////////////
// Particle filter Localization Class

// Constructor
LocalizationPF::LocalizationPF(ros::NodeHandle &nh)
    : ParticleFilter<Vec3f, Vec3f, LandmarkObservation, Mat22f>(
          new PoseParticleInitializer(		Vec3f(m_mean.x(), m_mean.y(), m_mean.z()),
                                            Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0) ),
          //new PoseParticleInitializer(Vec3f(0.0,0.0,0.0),Vec3f(1.0,1.0,1.0)),
          new OdometryModel(),
          new ObservationModel(this),
          80, // minimum number of particles
          150, // maximum number of particles
          0.1, // slow average likelihood learning rate
          0.5 // fast average likelihood learning rate
          )
    , m_field(field_model::FieldModel::getInstance())
{
    //    PFHandle = nh;
    //ROS_INFO("after nh");

    // store time of localization code initialization
    m_lastTime = ros::Time::now();

    // subscribe to topic for detected objects
    m_sub_vision = nh.subscribe("/localization/world_objects_detected", 10, &LocalizationPF::handleDetections, this);

    // subscribe to topic for the orientation heading from the IMU
    m_sub_imu = nh.subscribe("/decision/serial_receiver", 10, &LocalizationPF::handleImu, this);

    // subscribe to topic for initial pose given by behavior module
    m_sub_initial_pose = nh.subscribe("/behavior/initial_pose", 10, &LocalizationPF::handleInitPose, this);

    // advertise full particle set with poses and weights
    m_pub_particle_set = nh.advertise<localization::ParticleSet>("/localization/particle_set", 10);

    // advertise mean possition with confidence
    m_pub_mean_pose_conf_stamped = nh.advertise<localization::MeanPoseConfStamped>("/localization/mean_pose_conf_stamped", 10);


    m_odomPose.setZero();
    m_mean.setZero();
    m_confidence = 1.0;
    m_lastConfidence = 1.0;

    m_minEffectiveSampleSizeRatio = 0.5f; //0.5f;
    m_uniformReplacementProbability = 0.05f;
    m_actualUniformReplacementProbability = m_uniformReplacementProbability;
    m_usingActualReplacement = false;
    m_poseReplacementProbability = 0.05f;

    m_odomTimestamp = ros::Time(0);

    is_vision_detection = ros::Time::now();

    waist_yaw = 0;
    is_lifted_before = false;
    is_lifted_now = false;
    is_kidnapped = false;
}

// Destructor
LocalizationPF::~LocalizationPF()
{
}

// Callback function to handle detected world objects
void LocalizationPF::handleDetections(const localization::WorldObjectsConstPtr& msg)
{
    ROS_INFO("PF: %d new objects of time:%f",msg->objects.size(),msg->header.stamp.toSec());
    m_vision_detections = msg;
    is_vision_detection = ros::Time::now();
}

// Callback function to handle Imu readings
void LocalizationPF::handleImu(const decision::SerialReceived::ConstPtr& serial_input_msg)
{
    waist_yaw = serial_input_msg->received_data[10];
    is_lifted_before = is_lifted_now;
    is_lifted_now = std::fabs(serial_input_msg->received_data[11])>1e-6;
}

// Function to return true if a particle is outside the margins of the field
bool LocalizationPF::isOutOfField(const ParticleT& particle) const
{
    const double MARGIN = 0.5;
    double hw = m_field->width() / 2.0 + MARGIN;
    double hl = m_field->length() / 2.0 + MARGIN;

    Eigen::Array<float, 2, 1> abspos = particle.getState().head<2>().array().abs();

    return abspos.x() > hl || abspos.y() > hw;
}

// Function called every
void LocalizationPF::newStep()
{
    // Calculate uniform replacement probability according to fast/slow average likelihood, but always > threshold
    const float uniformReplacementTreshold = 0.02f; // scale between 0 and 1 in interval [UniformReplacementTreshold;1]
    float newUniformReplacementProbability = std::min( 1.f, std::max( 0.f, 1.f - ( float ) ( m_fastAverageLikelihood / m_slowAverageLikelihood ) - uniformReplacementTreshold) );
    newUniformReplacementProbability *= 1.f / (1.f-uniformReplacementTreshold);

    // finally the actual replacement probability changes only by 0.01 depending on the new rate and by 0.99 stays the same
    const float urp_alpha = 0.99;
    m_actualUniformReplacementProbability = urp_alpha * m_actualUniformReplacementProbability + (1.f-urp_alpha)*newUniformReplacementProbability;

    //ROS_INFO("Actual uniform replacement probability: %f", m_actualUniformReplacementProbability);

    if(!m_usingActualReplacement)
        // if it's the first time, set to a high value
        m_uniformReplacementProbability = m_actualUniformReplacementProbability = 0.05f;
    else {
        // minimum value for replacement probability is always 0.05
        m_uniformReplacementProbability = std::max(0.05f, m_actualUniformReplacementProbability);
    }
    m_usingActualReplacement = true;

    // m_uniformReplacementProbability = 0.00;  // test to have no replacement, NOPE, we need replacement, just around the current location


    // Calculate displacement for odometry model
    ros::Time now = ros::Time::now();

    tf::Stamped<tf::Pose> odom_pose;

    // Get pose at the head of the robot frame, '/base_link' refers to where the robot stands with the direction of the body
    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), ros::Time(0), "/base_link");

    // Get the pose of the '/base_link' in the '/odom' frame
    try
    {
        m_tf.transformPose("/odom", ident, odom_pose);
        m_odomTimestamp = odom_pose.stamp_;
    }
    catch(tf::TransformException& e)
    {
        fprintf(stderr, "pf loc:Failed to compute odometry pose, using identity\n");
        odom_pose = ident;
    }


    // Get the Euler angles of the odometry frame's rotation in regards to the body
    double yaw, pitch, roll;
    odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

    Vec3f current_odom(
                odom_pose.getOrigin().x(),
                odom_pose.getOrigin().y(),
                yaw
                );
    ROS_ERROR("PF: current odom %f %f %f",current_odom.x(),current_odom.y(),current_odom.z()/M_PI*180,m_odomPose.x(),m_odomPose.y(),m_odomPose.z()/M_PI*180);
    // Calculate the displacement delta by subtracting the current with the previous odometry pose, initially m_odomPose is 0
    Vec3f delta;
    delta.x() = (current_odom.x() - m_odomPose.x());
            //*cos(m_mean.z()) - (current_odom.y() - m_odomPose.y())*sin(m_mean.z());  // the use of this angle needs attention!!!
    delta.y() = (current_odom.y() - m_odomPose.y());
            //(current_odom.x() - m_odomPose.x())*sin(m_mean.z()) + (current_odom.y() - m_odomPose.y())*cos(m_mean.z());  // the use of this angle needs attention!!!
    delta.z() = (picut(current_odom.z() - m_odomPose.z())); // here z is the theta/yaw, we need the opposite, because in the loc, we use (-180,180], and in gait odom (180,-180]

    // If the odometry has been reset, then the difference will be large. Compare to a threshold and if larger, set to 0
    if ((fabs(delta.x()) > 0.5)  || (fabs(delta.y()) > 0.5) || (fabs(delta.z()/M_PI*180) > 50))
    {
        delta.x() = 0.0;
        delta.y() = 0.0;
        delta.z() = 0.0;
        ROS_INFO("PF: Too large delta, resetting odometry");
    }

    // update the odometry pose
    m_odomPose = current_odom;

    // if robot is lifted
    if (!is_lifted_before && is_lifted_now) {
        time_when_lifted = now.toSec();

        // record orientation when lifted 
        m_yaw_when_lifted = waist_yaw;
        m_theta_when_lifted = m_mean.z();
    }

    // if robot is kidnapped
    if (is_lifted_now && now.toSec()-time_when_lifted>4.0) {
        is_kidnapped = true;

        ParticleSetT &particles = m_particles[m_currParticleSetIdx];
        for (unsigned int p = 0; p < m_currNumParticles; ++p) {
            ParticleT &particle = particles[p];
            double prob = gsl_rng_uniform(m_rng);
            if (prob < 0.4) {
                PoseParticleInitializer poseParticleInitializer(
                        Vec3f(-2.1, 3, picut(m_theta_when_lifted+waist_yaw-m_yaw_when_lifted)),
                        Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0));
                poseParticleInitializer.initParticle(particle);
            }
            else if (prob < 0.8) {
                PoseParticleInitializer poseParticleInitializer(
                        Vec3f(2.1, 3, picut(m_theta_when_lifted+waist_yaw-m_yaw_when_lifted)),
                        Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0));
                poseParticleInitializer.initParticle(particle);
            }
            else if (prob < 1.0) {
                PoseParticleInitializer poseParticleInitializer(
                        Vec3f(0, 0, picut(m_theta_when_lifted+waist_yaw-m_yaw_when_lifted)),
                        Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0));
                poseParticleInitializer.initParticle(particle);
            }
            // else continue;
            particle.setWeight(0.1);
        }

        m_confidence = 0;
    }

    // if robot is settled
    if (is_kidnapped && !is_lifted_now) {
        is_kidnapped = false;
        time_when_settled = now.toSec();
    }

    //ROS_ERROR("move %f %f %f",delta.x(),delta.y(),delta.z());
    // use the function of the particle filter to update the particles' poses according to the displacement
    if (!is_kidnapped) sample(delta, (now - m_lastTime).toSec());

    // Constrain to soccer field
    for(size_t i = 0; i < m_currNumParticles; ++i)
    {
        ParticleT* particle = &m_particles[m_currParticleSetIdx][i];

        if(isOutOfField(*particle))
            particle->setWeight(-70);
    }

    ROS_INFO("After sampling");

    // 	bool debugPublished = false;
    double detections_time = ros::Time::now().toSec();
    int obs_push_back_counter = 0;


    std::vector<LandmarkObservation> observations;
    // for every detected object push an observation in the list of observations for this step
    if((m_vision_detections)&&(ros::Time::now()-is_vision_detection<ros::Duration(2.0)))
    {
        detections_time = m_vision_detections->header.stamp.toSec();

        for(size_t i = 0; i < m_vision_detections->objects.size(); ++i)
        {
            const localization::ObjectsDetected& obj = m_vision_detections->objects[i];

            Vec3f pose(obj.pose.x, obj.pose.y, NAN);

            //			if(obj.type >= field_model::WorldObject::NumTypes)
            //			{
            //				ROS_WARN("Invalid type in object detection: %d", obj.type);
            //				continue;
            //			}

            // TODO: needed?  If detected object is out of bounds then ignore it
            //			if(pose.x() < 0)
            //            {
            //                ROS_INFO("ignored");
            //                continue;
            //            }

            //fprintf(stderr, "Using %d at % 10.7lf, % 10.7lf\n", obj.type, pose.x(), pose.y());
            LandmarkObservation obs(
                        (field_model::WorldObject::Type)obj.type,
                        pose
                        );

            observations.push_back(obs);
            obs_push_back_counter++;
        }

        ROS_INFO("Pushed back %d detections of time:%f",obs_push_back_counter,is_vision_detection.toSec());
    }

    // if observations we inserted in the list, then for each of them calculate variances, find the new weights of the particles
    // and resample according to the new weights
    if(observations.size()>0)
    {
        BOOST_FOREACH(LandmarkObservation& ob, observations)
                ((ObservationModel*)m_observationModel)->precalculateVariances(ob);

        importance(observations);

        ROS_INFO("After associations");

        resample();

        PoseParticleInitializer poseParticleInitializer(
                    Vec3f(m_mean.x(), m_mean.y(), m_mean.z()),
                    Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0) // standard deviations for pose elements
                    );


        ParticleSetT & particles = m_particles[ m_currParticleSetIdx ];

        // Replace fraction at random around mean pose
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            ParticleT & particle = particles[ p ];
            if ( gsl_rng_uniform( m_rng ) < m_poseReplacementProbability ) {

                poseParticleInitializer.initParticle(particle );
            }
            particle.setWeight( 0 );
        }
        //fprintf(stderr,"Replaced particles around mean \n");
        ROS_INFO("After resampling");

        m_lastConfidence = m_confidence;
    }

    // 	if(!m_hasResampled)
    // Publish the debugging messages, determine the new dominant pose and publish the new transform
    publishDebug();
    determinePose();
    //publishTransform();

    if (is_kidnapped) m_confidence = 0;
    if (m_confidence>0.1 && observations.empty()) {
        m_confidence = m_lastConfidence * 0.999;
        m_lastConfidence = m_confidence;
    }

    m_lastTime = now;

    // Publish mean pose with confidence and timestamp

    localization::MeanPoseConfStamped mean_pose_msg;

    mean_pose_msg.header.frame_id = "/field";
    mean_pose_msg.header.stamp = timestamp();
    mean_pose_msg.robotPose.x = m_mean.x();
    mean_pose_msg.robotPose.y = m_mean.y();
    mean_pose_msg.robotPose.theta = m_mean.z();
    mean_pose_msg.robotPoseConfidence = m_confidence;

    ROS_ERROR("PF: Publishing new mean pose: (%f,%f,%f) & confidence: %f, of detection time:%f", m_mean.x(), m_mean.y(), m_mean.z(), m_confidence, detections_time);

    m_pub_mean_pose_conf_stamped.publish(mean_pose_msg);

    observations.clear();

}


// Function to establish if the observations made can be actually done from the pose of a given particle
void LocalizationPF::establishDataAssociation(
        ParticleT& particle, std::vector< LandmarkObservation >& observations)
{
    particle.getUserData() = Eigen::Rotation2D<float>(particle.getState().z());

    for(size_t i = 0; i < observations.size(); ++i)
    {
        LandmarkObservation* obs = &observations[i];

        if(obs->type() == field_model::WorldObject::Type_PenMarker
                || obs->type() == field_model::WorldObject::Type_GoalPost
                || obs->type() == field_model::WorldObject::Type_LineCrossL
                || obs->type() == field_model::WorldObject::Type_LineCrossT
                || obs->type() == field_model::WorldObject::Type_LineCrossX
                || obs->type() == field_model::WorldObject::Type_ImuHeading
                || obs->type() == field_model::WorldObject::Type_Goal
                )
        {
            std::vector<DataAssociationPair> matches;

            const std::vector<field_model::WorldObject>& objs = m_field->objects(obs->type());
            double minLikelihood = -1.0;

            for(size_t j = 0; j < objs.size(); ++j)
            {
                obs->setAssociatedObject(&objs[j]);

                double likelihood = m_observationModel->likelihood(particle, *obs);
                if(minLikelihood < 0)
                    minLikelihood = likelihood;

                matches.push_back(DataAssociationPair(
                                      likelihood, &objs[j]
                                      ));
            }


            //            if(obs->type() == field_model::WorldObject::Type_LineCrossL)
            //			{
            //                const std::vector<field_model::WorldObject>& objs = m_field->objects(field_model::WorldObject::Type_LineCrossT);

            //				for(size_t j = 0; j < objs.size(); ++j)
            //				{
            //					obs->setAssociatedObject(&objs[j]);

            //					double likelihood = m_observationModel->likelihood(particle, *obs);
            //					if(minLikelihood < 0)
            //						minLikelihood = likelihood;

            //					matches.push_back(DataAssociationPair(
            //						likelihood, &objs[j]
            //					));
            //				}
            //			}

            //            if(obs->type() == field_model::WorldObject::Type_LineCrossT)
            //			{
            //                const std::vector<field_model::WorldObject>& objs = m_field->objects(field_model::WorldObject::Type_LineCrossX);

            //				for(size_t j = 0; j < objs.size(); ++j)
            //				{
            //					obs->setAssociatedObject(&objs[j]);

            //					double likelihood = m_observationModel->likelihood(particle, *obs);
            //					if(minLikelihood < 0)
            //						minLikelihood = likelihood;

            //					matches.push_back(DataAssociationPair(
            //						likelihood, &objs[j]
            //					));
            //				}
            //			}

            determineMatch(matches, minLikelihood, obs);
        }
    }
}


// Function to determine if there is actually a match between an observation and the actual landmark supposedly observed by the pose
// of a given particle
void LocalizationPF::determineMatch(
        std::vector<DataAssociationPair>& matches, double minLikelihood,
        LandmarkObservation* obs)
{
    double minMahalDist = minLikelihood;
    const field_model::WorldObject* assoc = 0;

    for(size_t i = 0; i < matches.size(); ++i)
    {
        if(matches[i].mahal_dist <= minMahalDist)
        {
            minMahalDist = matches[i].mahal_dist;
            assoc = matches[i].object;
        }
    }

    if(assoc)
    {
        obs->setAssociatedObject(assoc);
        obs->setMahalDist(minMahalDist, minLikelihood);
    }
}


// Function for publishing the particle set here for debugging purposes, to make sure the new particles are published
void LocalizationPF::publishDebug()
{
    localization::ParticleSet msg;

    msg.header.frame_id = "/field";
    msg.header.stamp = timestamp();

    msg.particles.reserve(m_currNumParticles);

    float maxlh = -1e10;
    for(size_t p = 0; p < m_currNumParticles; p++ )
    {
        float lh = m_particles[m_currParticleSetIdx][p].getWeight();

        if(lh > maxlh)
            maxlh = lh;
    }
    //fprintf(stderr, "lh: % 10.7f (resampled: %d)\n", maxlh, m_hasResampled);

    for(size_t i = 0; i < m_currNumParticles; ++i)
    {
        const ParticleT& particle = m_particles[m_currParticleSetIdx][i];

        localization::Particle p;
        p.pose.x = particle.getState().x();
        p.pose.y = particle.getState().y();
        p.pose.theta = particle.getState().z();
        p.weight = exp(particle.getWeight() - maxlh);

        msg.particles.push_back(p);
    }

    //fprintf(stderr, "Publishing particle set \n");
    m_pub_particle_set.publish(msg);
}

// Function that takes the previous mean position and orientation and finds all the particles within the window near them
// Then it calculates the new mean position and orientation, and the sum of the particles' weight within that window
void LocalizationPF::meanShift(const Vec2f& startCartPos, float startOri, float maxCartDist2, float maxAngDist, Vec3f& mean, double& weightSumFraction)
{
    Vec2f lastCartMean( startCartPos );
    float lastOriMean( startOri );
    double lastWeightSumFraction = 0;
    const int maxNumIterations = 20;

    // for a set number of iterations shift the mean of the particles' position and orientation, until convergence
    for(int i = 0; i < maxNumIterations; ++i)
    {
        Vec2f windowCartMean( 0, 0 );
        Vec2f windowDirMean( 0, 0 );
        float windowOriMean = 0;
        double weightSum = 0;
        ParticleSetT & particles = m_particles[ m_currParticleSetIdx ];
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            const Vec3f & state = particles[ p ].getState();
            Vec2f cartPos = state.head<2>();
            if ( ( ( cartPos - lastCartMean ).squaredNorm() <= maxCartDist2 ) && ( fabs( picut( state.z() - lastOriMean ) ) <= maxAngDist ) ) {
                double weight = exp( ( double ) particles[ p ].getWeight( ) );
                // update the mean position and orientation within the window according to the weights of the particles
                windowCartMean += ( float ) weight * cartPos;
                windowDirMean += ( float ) weight * Vec2f( cos( state.z() ), sin( state.z() ) );
                weightSum += weight;
            }
        }
        if ( weightSum == 0 ) {
            windowCartMean = lastCartMean;
            windowOriMean = lastOriMean;
        }
        else {
            // divide by the weight sum to create the weighted mean
            windowCartMean /= ( float ) weightSum;
            windowOriMean = atan2( windowDirMean.y(), windowDirMean.x() );
        }
        // if the new calculated mean is very close to the old one, meaning our iterations' result is converging, then break
        if ( ( ( windowCartMean - lastCartMean ).squaredNorm() < 0.001f ) || ( fabs( picut( windowOriMean - lastOriMean ) ) < 0.001f ) ) {
            lastCartMean = windowCartMean;
            lastOriMean = windowOriMean;
            lastWeightSumFraction = weightSum;
            break;
        }
        // otherwise keep repeating the shift of the mean
        lastCartMean = windowCartMean;
        lastOriMean = windowOriMean;
    }
    mean = Vec3f( lastCartMean.x(), lastCartMean.y(), lastOriMean );
    weightSumFraction = lastWeightSumFraction;
}

// Function for determining dominant pose and orientation
void LocalizationPF::determinePose()
{
    ParticleT * bestParticle = NULL;
    float maxweight = -1000.f;
    double weightSum = 0;
    ParticleSetT & particles = m_particles[ m_currParticleSetIdx ];

    // for all particles find the one with the max weight
    for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
        ParticleT & particle = particles[ p ];
        if ( ( bestParticle == NULL ) || ( particle.getWeight( ) > maxweight ) ) {
            maxweight = particle.getWeight( );
            bestParticle = &particle;
        }
        weightSum += exp( ( double ) particle.getWeight( ) );
    }

    // Keep this simple way of finding the dominant one for debugging purposes
    //m_mean = bestParticle->getState();
    //m_cov.setEye();
    //m_confidence = 1;

    // set the mean area window, e.g. the max distance of the particles from the previous mean of the area
    // and the max angle difference from the previous orientation
    const float windowCartDist2 = 0.5f * 0.5f;
    const float windowAngDist = 0.3f;

    Vec3f lastMean;
    double lastMeanWeightFraction = 0;
    meanShift( m_mean.head<2>(), m_mean.z(), windowCartDist2, windowAngDist, lastMean, lastMeanWeightFraction );
    float lastMeanConfidence = 0;
    if ( weightSum != 0 ) {
        // determine confidence by how many particles are concentrated in the mean area
        lastMeanConfidence = ( float ) std::max( 0.f, std::min( 1.f, (float)(lastMeanWeightFraction / weightSum) ) );
    }

    m_mean = lastMean;
    m_confidence = lastMeanConfidence;

    if ( weightSum != 0 ) {
        Vec3f bestParticleMean;
        double bestParticleWeightFraction = 0;
        meanShift( bestParticle->getState( ).head<2>( ), bestParticle->getState( ).z(), windowCartDist2, windowAngDist, bestParticleMean, bestParticleWeightFraction );
        float bestParticleConfidence = ( float ) std::max( 0.f, std::min( 1.f, (float)(bestParticleWeightFraction / weightSum )) );
        // check to see if the best particle is better than the mean then replace
        if ( bestParticleConfidence > 4.5f * lastMeanConfidence ) {
            m_mean       = bestParticleMean;
            m_confidence = bestParticleConfidence;
        }
    }

    //ROS_INFO("Determining mean pose");
    // check to see if it's within boundaries of field
    m_mean.x() = std::max<float>(-m_field->length()/2.0, std::min<float>(m_field->length()/2.0, m_mean.x()));
    m_mean.y() = std::max<float>(-m_field->width()/2.0, std::min<float>(m_field->width()/2.0, m_mean.y()));
}


// Function to publish the new transform from '/field' to '/robot_head' frame after the new dominant pose calculated by the localization
void LocalizationPF::publishTransform()
{
    //fprintf(stderr,"Publishing new /field to /robot_head transform\n");
    tf::StampedTransform t;
    t.frame_id_ = "/field";
    t.child_frame_id_ = "/robot_head";
    //    t.stamp_ = timestamp() + ros::Duration(0.05); // time between step commands is 0.05 sec
    if (timestamp().toSec() != 0.0)
        t.stamp_ = timestamp() + ros::Duration(0.05); // time between step commands is 0.05 sec
    else
        t.stamp_ = ros::Time::now() + ros::Duration(0.05); // time between step commands is 0.05 sec if there are no detections

    t.setOrigin(tf::Vector3(m_mean.x(), m_mean.y(), 0.0));
    t.setRotation(tf::createQuaternionFromYaw(m_mean.z()));

    m_pub_tf.sendTransform(t);
}

// Function to handle an initial pose for our robot given by topic "/behavior/initial_pose"
void LocalizationPF::handleInitPose(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    PoseParticleInitializer poseParticleInitializer(
                Vec3f(pose.pose.pose.position.x, pose.pose.pose.position.y, tf::getYaw(pose.pose.pose.orientation)),
                Vec3f(0.2, 0.2, 10.0 * M_PI / 180.0) // standard deviations for pose elements
                );

    for(unsigned int i = 0; i < 2; ++i)
    {
        for( unsigned int p = 0; p < m_particles[i].size(); p++ ) {
            poseParticleInitializer.initParticle( m_particles[i][p] );
        }
    }
    fprintf(stderr,"particle filter localization: pf initialized \n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particle_filter_localization");

    ros::NodeHandle nh("~");
    //ROS_INFO("started particle filter localization");
    LocalizationPF pfl(nh);

    ros::Rate rate(10); // because step period is 1/20 = 0.05sec

    //ROS_INFO("Entering particle filter localization main loop");

    while(ros::ok())
    {
        ros::spinOnce();

        //fprintf(stderr, "Calling function for new step\n");
        pfl.newStep();
        rate.sleep();
    }

    return 0;
}
