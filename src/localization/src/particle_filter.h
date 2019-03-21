// Particle Filter code implementing Augmented_MCL with particle resampling, including classes Particle, ParticleSet,
// AbstractParticleInitializer, AbstractMotionModel and AbstractObservationModel. Inside the Main Class we have the
// functions Sample, Importance and Resample.
// dependencies: 'pf_functions.h' for some math funtions and tools
//               'gsl' from ubuntu (sudo apt-get install libgsl0-dev) for random number generator (rng)
//
// Maintainer: Sotirios Stasinopoulos email: sotstas@gmail.com
// Last edited: 2014.03.03

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <ros/ros.h>
#include <iostream>
#include <vector>

#include "pf_functions.h"

#include <gsl/gsl_rng.h>

#include <time.h>

#include <cfloat>

// * The entire ParticleFilter is within PF namespace * //
namespace PF {

// Definition of template for class Particle
template < class T_state, class T_userdata = void * >
class Particle
{
public:
    Particle( ) { }

    ~Particle( ) { }

    inline void
    setState( const T_state & state ) {m_state = state;}

    inline const T_state &
    getState( ) const {return m_state;}

    inline void
    setWeight( float weight ) {m_weight = weight;}

    inline float
    getWeight( ) const {return m_weight;}

    inline void
    increaseWeight( float inc ) {m_weight += inc;}

    inline void
    setUserData( const T_userdata & d ) {m_userdata = d;}

    inline T_userdata &
    getUserData( ) {return m_userdata;}

    inline const T_userdata &
    getUserData( ) const {return m_userdata;}

#define PARTICLE_DEBUGGING 0
#if PARTICLE_DEBUGGING
    std::vector < pair < std::wstring, Vec3f > > m_DebuggingVector;
#endif

protected:
    T_state m_state;
    float m_weight;
    T_userdata m_userdata;
};

// ************************************************************************************ //

// Definition of likelihood structure to be used
struct Likelihood {
    double likelihood;
    double minLikelihood;
    double mahal_dist;      // mahalanobis dist for likelihood
    double min_mahal_dist;  // mahal dist for minLikelihood
};

// ************************************************************************************ //

// Definition of template for class of Abstract Particle Initializer
template < class T_state, class T_userdata = void * >
class AbstractParticleInitializer
{
public:
    typedef Particle<T_state, T_userdata> ParticleT;

    AbstractParticleInitializer( ) { }

    virtual ~AbstractParticleInitializer( ) { }

    virtual void initParticle( ParticleT & particle ) = 0;

};

// ************************************************************************************ //

// Definition of template for class of set of Particles
template < class T_state, class T_userdata = void * >
class ParticleSet : public std::vector < Particle < T_state, T_userdata > >
{
public:
    ParticleSet( ) { }

    ~ParticleSet( ) { }

};

// ************************************************************************************ //

// Definition of template for class Abstract Motion Model
template < class T_state, class T_control >
class AbstractMotionModel
{
public:
    AbstractMotionModel( ) { }

    virtual ~AbstractMotionModel( ) { }

    virtual T_state sampleState( const T_state & lastPose, const T_control & controlInput, float delta_t ) = 0;

};

// ************************************************************************************ //

// Definition of template for class Abstract Obsrevation Model
template < class T_state, class T_obs, class T_userdata = void * >
class AbstractObservationModel
{
public:
    typedef Particle<T_state, T_userdata> ParticleT;

    AbstractObservationModel( ) { }

    virtual ~AbstractObservationModel( ) { }

    virtual double likelihood( const ParticleT& particle, const T_obs & observation ) = 0;

    float
    logLikelihood( const ParticleT& particle, const T_obs & observation ) {return ( float ) log( likelihood( particle, observation ) );}

    // p_0, or the lowest likelihood when associating
    virtual double minLikelihood( const ParticleT& particle, const T_obs & observation ) = 0;
    virtual Likelihood minAndLikelihood( const ParticleT& particle, const T_obs & observation ) = 0;

    virtual double likelihoodDiscount( const T_obs & observation ) = 0;

};

// ************************************************************************************ //

// * MAIN CLASS OF PARTICLE FILTER * //
template < class T_state, class T_control, class T_obs, class T_userdata = void * >
class ParticleFilter
{
public:
    typedef Particle<T_state, T_userdata> ParticleT;
    typedef ParticleSet<T_state, T_userdata> ParticleSetT;
    typedef AbstractParticleInitializer<T_state, T_userdata> InitializerT;
    typedef AbstractMotionModel<T_state, T_control> MotionModelT;
    typedef AbstractObservationModel<T_state, T_obs, T_userdata> ObservationModelT;

    ParticleFilter( AbstractParticleInitializer < T_state, T_userdata > * particleInitializer,
                    AbstractMotionModel < T_state, T_control > * motionModel,
                    AbstractObservationModel < T_state, T_obs, T_userdata > * observationModel,
                    unsigned int minNumParticles,
                    unsigned int maxNumParticles,
                    double slowAvgLikelihoodLearningRate,
                    double fastAvgLikelihoodLearningRate )
        : m_particleInitializer( particleInitializer )
        , m_motionModel( motionModel )
        , m_observationModel( observationModel )
        , m_minNumParticles( minNumParticles )
        , m_maxNumParticles( maxNumParticles )
        , m_currNumParticles( maxNumParticles )
        , m_prevNumParticles( 0 )
        , m_hasResampled( false )
        , m_minEffectiveSampleSizeRatio( 1.f )
        , m_slowAvgLikelihoodLearningRate( slowAvgLikelihoodLearningRate )
        , m_fastAvgLikelihoodLearningRate( fastAvgLikelihoodLearningRate )
    {

        m_effectiveSampleSize = ( float ) maxNumParticles;

        // * Init random number generator. * //sui ji shu
        const gsl_rng_type * T;
        gsl_rng_env_setup( );
        T = gsl_rng_default;
        m_rng = gsl_rng_alloc( T );
        gsl_rng_set( m_rng, 234523 ); // give seed to random number generator

        // * Init particles, both sets -old and new- with IDs 0 and 1 * //
        for ( unsigned int i = 0; i < 2; i++ ) {
            m_particles[ i ].clear( );
            m_particles[ i ].resize( maxNumParticles );
            for ( unsigned int p = 0; p < maxNumParticles; p++ ) {
                m_particleInitializer->initParticle( m_particles[ i ][ p ] );
                m_particles[ i ][ p ].setWeight( 0 );
            }
        }
        m_currParticleSetIdx = 0;

        m_slowAverageLikelihood = 1;
        m_fastAverageLikelihood = 0;

        fprintf(stderr, "after PF cons\n");
    }

    ~ParticleFilter( )
    {
        gsl_rng_free( m_rng );
        delete m_particleInitializer;
        delete m_motionModel;
        delete m_observationModel;
    }

    // per-particle data association
    // save association information within observations
    virtual void
    establishDataAssociation( Particle < T_state, T_userdata > & particle, std::vector < T_obs > & observations ) { }

    virtual void
    establishDataAssociation( Particle < T_state, T_userdata > & particle, std::vector < T_obs * > & observations ) { }

    // Create the sample function in order to update the state of all particles given a specific control input at a specific time
    virtual void
    sample( const T_control & controlInput, float delta_t )             //delta_t is delta time
    {
        ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            particle.setState( m_motionModel->sampleState( particle.getState( ), controlInput, delta_t ) );
        }
    }

    // Create importance function for pointer to or for plain observation set in order to calculate the new weights of the
    // particles given an observation set, to calculate the short-term (fast) and long-term (slow) averages of the
    // measurement likelihood and to normalize the particle weights
    void
    importance( std::vector < T_obs * > & observations )
    {
        //ROS_INFO("importance");
        double avgLikelihood = 0;

        ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            establishDataAssociation( particle, observations ); // for every particle, check if given the observations the robot
                                                                // has a high likelihood of actually being there
            for ( unsigned int o = 0; o < observations.size( ); o++ ) {
                T_obs * observation = observations[ o ];
                Likelihood mal;
                if(observation->isLikSet())
                    mal = observation->getMinAndLikelihood();
                else
                    mal = m_observationModel->minAndLikelihood( particle, *observation );
                observation->mahal_dists_to_likelihoods(mal);
                double obsLikelihood = std::max( mal.likelihood, mal.minLikelihood );
                avgLikelihood += obsLikelihood;
                particle.increaseWeight( ( float ) ( m_observationModel->likelihoodDiscount( *observation ) * log( obsLikelihood ) ) );
            }
        }
        // calculate short-term (fast) and long-term (slow) averages of the measurement likelihood
        if ( observations.size( ) > 0 ) {
            avgLikelihood /= ( ( double ) ( observations.size( ) * m_currNumParticles ) );
            m_slowAverageLikelihood += ( float ) ( m_slowAvgLikelihoodLearningRate * ( avgLikelihood - m_slowAverageLikelihood ) );
            m_fastAverageLikelihood += ( float ) ( m_fastAvgLikelihoodLearningRate * ( avgLikelihood - m_fastAverageLikelihood ) );
        }

        // get the maximum weight of all the particles
        float maxlogweight = -FLT_MAX;
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            if ( particle.getWeight( ) > maxlogweight ) {
                maxlogweight = particle.getWeight( );
            }
        }
        // normalize weights of particles by abstracting the maximum weight from all
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            particle.setWeight( particle.getWeight( ) - maxlogweight );
        }
        //ROS_INFO("end impartance");
    }

    void
    importance( std::vector < T_obs > & observations )
    {
        //ROS_INFO("importance");
        double avgLikelihood = 0;

        ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
        //ROS_INFO("ready");
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            establishDataAssociation( particle, observations );
            for ( unsigned int o = 0; o < observations.size( ); o++ ) {
                T_obs & observation = observations[ o ];
                Likelihood mal;
                if(observation.isMahalSet())
                    mal = observation.getMinAndLikelihood();
                else
                    mal = m_observationModel->minAndLikelihood( particle, observation );
                observation.mahal_dists_to_likelihoods(mal);
                double obsLikelihood = std::max( mal.likelihood, mal.minLikelihood );
                avgLikelihood += obsLikelihood;
                particle.increaseWeight( ( float ) ( m_observationModel->likelihoodDiscount( observation ) * log( obsLikelihood ) ) );
                assert( particle.getWeight( ) == particle.getWeight( ) );
            }
        }
       // ROS_INFO("second step");
        if ( observations.size( ) > 0 ) {
            avgLikelihood /= ( ( double ) ( observations.size( ) * m_currNumParticles ) );
            assert( avgLikelihood == avgLikelihood );
            m_slowAverageLikelihood += ( float ) ( m_slowAvgLikelihoodLearningRate * ( avgLikelihood - m_slowAverageLikelihood ) );
            m_fastAverageLikelihood += ( float ) ( m_fastAvgLikelihoodLearningRate * ( avgLikelihood - m_fastAverageLikelihood ) );
        }
        float maxlogweight = -FLT_MAX;
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            if ( particle.getWeight( ) > maxlogweight ) {
                maxlogweight = particle.getWeight( );
            }
        }
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            Particle < T_state, T_userdata > & particle = particles[ p ];
            particle.setWeight( particle.getWeight( ) - maxlogweight );
        }
        //ROS_INFO("end impartance");
    }

    // Create resample function in order to include observations
    void
    resample( )
    {
        // low-variance-sampler
        // sample with replacement
        ParticleSet < T_state, T_userdata > & particles = m_particles[ m_currParticleSetIdx ];
        ParticleSet < T_state, T_userdata > & tmp_particles = m_particles[ 1 - m_currParticleSetIdx ];


        // rescale weights for improved numerical stability (necessary???)
        float maxLogWeight = 0;
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            float logweight = particles[ p ].getWeight( );
            if ( logweight > maxLogWeight ) {
                maxLogWeight = logweight;
            }
        }
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            particles[ p ].setWeight( particles[ p ].getWeight( ) - maxLogWeight );
        }

        // calculate sum, min, max of new weights
        float sumWeight = 0;
        float minLogWeight = 0;
        maxLogWeight = 0;
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            float logweight = particles[ p ].getWeight( );
            sumWeight += ( float ) exp( ( double ) logweight );
            if ( logweight > maxLogWeight ) {
                maxLogWeight = logweight;
            }
            if ( logweight < minLogWeight ) {
                minLogWeight = logweight;
            }
        }
        float avgObsLikelihoodRatio = std::min( 1.f, std::max( 0.f, ( float ) ( m_fastAverageLikelihood / m_slowAverageLikelihood ) ) );

        // resample in any case if weights get numerically unstable
        // also resample for low avg obs likelihood to induce uniform particles
        if ( ( avgObsLikelihoodRatio > 0.8f ) && ( minLogWeight > -200.f ) && ( maxLogWeight < 200.f ) ) {

            // compute effective sample size, if necessary
            if ( m_minEffectiveSampleSizeRatio < 1.f ) {
                double sumSqrWeights = 0;
                for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
                    sumSqrWeights += exp( 2.0 * ( double ) particles[ p ].getWeight( ) );
                }
                if ( ( sumSqrWeights == 0 ) || ( sumWeight == 0 ) ) {
                    m_effectiveSampleSize = ( float ) m_currNumParticles;
                }
                else {
                    m_effectiveSampleSize = ( float ) ( ( sumWeight * sumWeight ) / sumSqrWeights );
                }
                if ( m_effectiveSampleSize >= ( m_minEffectiveSampleSizeRatio * ( ( float ) m_currNumParticles ) ) ) {
                    m_hasResampled = false;
                    //ROS_INFO("didn't resample");
                    return;
                }
            }
        }
        // 1. normalize weights of particles to resampling probabilities
        // if the sum of weights is 0 then set weight of all particles to uniform 1/M
        if ( sumWeight == 0 ) {
            const float psize_inv = 1.f / ( ( float ) m_currNumParticles );
            for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
                particles[ p ].setWeight( psize_inv );
            }
        }
        // else normalize the weight dividing by the sum of weights
        else {
            for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
                particles[ p ].setWeight( ( float ) ( exp( particles[ p ].getWeight( ) ) / sumWeight ) );
            }
        }
        // set new particle set size according to the average observation likelihood
        unsigned int newParticleSetSize = m_minNumParticles + ( unsigned int )( ( 1.f - std::max( 0.f, std::min( 1.f, ( avgObsLikelihoodRatio - 0.5f ) / 0.5f ) ) ) * ( m_maxNumParticles - m_minNumParticles ) );
        float n_inc = 1.f / ( ( float ) newParticleSetSize );

        // 2. resample
        // Get n=random number (within [0,1) ) x new uniform weight=1/new size, and as long as the sum
        // of weights is bigger than n accept old particles to the new set with new weight 0. If particles reach
        // the new size, break.
        float n = ( ( float ) gsl_rng_uniform( m_rng ) ) * n_inc;
        float p_sum = 0;
        unsigned int tmp_idx = 0;
        for ( unsigned int p = 0; p < m_currNumParticles; p++ ) {
            p_sum += particles[ p ].getWeight( );
            while ( p_sum > n ) {
                tmp_particles[ tmp_idx ] = particles[ p ];
                tmp_particles[ tmp_idx ].setWeight( 0 );
                n += n_inc;
                tmp_idx++;
                if ( tmp_idx >= newParticleSetSize ) {
                    break;
                }
            }
            if ( tmp_idx >= newParticleSetSize ) {
                break;
            }
        }

        // 3. replace fraction at random
//        for ( unsigned int p = 0; p < newParticleSetSize; p++ ) {
//            if ( gsl_rng_uniform( m_rng ) < m_uniformReplacementProbability ) {
//                // random particles inserted are initialized with 0 (center of field) , HERE see if we should
//                // initialize them in random way on the field, maybe with new function initRandomParticle
//                m_particleInitializer->initParticle( tmp_particles[ p ] );
//                //tmp_particles[p].setWeight( 0 );
//            }
//            tmp_particles[ p ].setWeight( 0 ); // maybe not needed
//        }
        // 4. Swap existing particle set with new one
        m_currParticleSetIdx = 1 - m_currParticleSetIdx; //swap ids all the time between 0 and 1,
        // total of two sets of particles in memory, new and old one in case we need to revert to old
        m_prevNumParticles = m_currNumParticles;
        m_currNumParticles = newParticleSetSize;
        m_hasResampled = true;
    }

    const ParticleSet < T_state, T_userdata > &
    getParticles( ) const {return m_particles[ m_currParticleSetIdx ];}

protected:

    // double buffered for improved performance
    ParticleSet < T_state, T_userdata > m_particles[ 2 ];
    unsigned int m_currParticleSetIdx;

    AbstractParticleInitializer < T_state, T_userdata > * m_particleInitializer;
    AbstractMotionModel < T_state, T_control > * m_motionModel;
    AbstractObservationModel < T_state, T_obs, T_userdata > * m_observationModel;

    float m_slowAverageLikelihood;
    float m_fastAverageLikelihood;

    unsigned int m_minNumParticles;
    unsigned int m_maxNumParticles;
    unsigned int m_currNumParticles, m_prevNumParticles;
    bool m_hasResampled;

    gsl_rng * m_rng;

    float m_minEffectiveSampleSizeRatio;
    float m_effectiveSampleSize;

    float m_uniformReplacementProbability;
        float m_poseReplacementProbability;

    double m_slowAvgLikelihoodLearningRate;
    double m_fastAvgLikelihoodLearningRate;
};

} // END of Namespace PF

#endif
