/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 1000;

	weights.resize(num_particles, 1.0);

	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

    for(int i=0; i<num_particles; ++i){

        Particle p = new Particle();
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;

        particles.push_back(p);
    }

    is_initialized = true;

    return;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for(int i=0; i<num_particles; ++i){

        Particle& p = particles[i];

        if (fabs(yaw_rate) < 0.001){
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        }
        else{
            p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta)));
            p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        }

        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);

	}

    return;
}

std::vector<LandmarkObs> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	std::vector<LandmarkObs> associated_landmarks;
	LandmarkObs closest;

	for(int i=0; i<observations.size(); ++i){

        double shortest = 1E10;

        for(int j=0; j<predicted.size(); ++j){

            double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            if (distance < shortest){
                shortest = distance;
                closest = predicted[j];
            }
        }

        associated_landmarks.push_back(closest);
	}

	return associated_landmarks;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];

    for(int i=0; i<particles.size(); ++i){

        Particle p = particles[i];

        // Transform from vehicle's coordinate system to map's coordinate system
        std::vector<LandmarkObs> transformed_observations;
        for(LandmarkObs observation: observations){

            LandmarkObs transformed_observation;

            transformed_observation.id = observation.id;
            transformed_observation.x = p.x + observation.x * cos(p.theta) - observation.y * sin(p.theta);
            transformed_observation.y = p.y + observation.x * sin(p.theta) + observation.y * cos(p.theta);

            transformed_observations.push_back(transformed_observation);
        }

        // List of landmarks that are within sight of the particle
        std::vector<LandmarkObs> landmarks_visible;
        for(auto landmark: map_landmarks.landmark_list){

            double distance = dist(p.x, p.y, landmark.x_f, landmark.y_f);

            if(distance <= sensor_range){
                LandmarkObs l;
                l.id = landmark.id_i;
                l.x = landmark.x_f;
                l.y = landmark.y_f;

                landmarks_visible.push_back(l);
            }
        }

        // Associate nearest landmark to every observation of the particle
        std::vector<LandmarkObs> associated_landmarks;
        associated_landmarks = dataAssociation(landmarks_visible, transformed_observations);

        double probability = 1;
        for(int j=0; j<associated_landmarks; ++j){
            double delta_x = transformed_observations[j].x - associated_landmarks[j].x;
            double delta_y = transformed_observations[j].y - associated_landmarks[j].y;
            probability *= 1.0/(2*M_PI*std_x*std_y) * exp(-(pow(delta_x,2)/(2*pow(std_x,2)) + pow(delta_y,2)/(2*pow(std_y,2))));
        }

        p.weight = probability;
        weights[i] = probability;
    }
    return;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Get max weight
	double max_weight = 0;
	for(Particle p:particles){
        if(p.weight > max_weight)
            max_weight = p.weight
	}

	// Generate new particles
	std::vector<Particle> new_particles(num_particles);
	default_random_engine gen();
	std::uniform_int_distribution<int> start(0, num_particles - 1);
	std::uniform_real_distribution<double> spin(0, 2.0 * max_weight);

	float beta = 0.0;
	int index = start(gen);

	for(int i=0; i<num_particles; ++i){
        beta +=  spin(gen);

        while(particles[index].weight < beta){
            beta -= particles[index].weight
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
