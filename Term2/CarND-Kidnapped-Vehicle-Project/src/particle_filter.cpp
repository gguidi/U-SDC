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


	// random engine used for sampling from normal distribution
	default_random_engine gen;
	// create random distribution 
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	// choose tje number of particles
	num_particles = 100;
	//initialize the weights to 1
	//initialize all particles to first position and add random noise
	weights[num_particles];
	particles[num_particles];
	for(int i=0; i<num_particles; i++){
		// sample the gaussian random distribution 
		double sample_x = dist_x(gen);
        double sample_y = dist_y(gen);
        double sample_theta = dist_theta(gen);
        //init weight
		weights[i]=1;
		//init particle i
		particles[i].id = i;
		particles[i].x = x + sample_x; 
		particles[i].y = y + sample_y;
		particles[i].theta = theta + sample_theta;
		particles[i].weight = 1;
		// initialize the association ???
	}
	


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// random engine used for sampling from normal distribution
	default_random_engine gen;
	// create random distribution 
	//normal_distribution<double> dist_x(x, std_pos[0]);
	//normal_distribution<double> dist_y(y, std_pos[1]);
	//normal_distribution<double> dist_theta(theta, std_pos[2]);

	for(int i=0; i<num_particles; i++){
		// sample the gaussian random distribution 
		//double sample_x = dist_x(gen);
        //double sample_y = dist_y(gen);
        //double sample_theta = dist_theta(gen);
        double a = velocity/yaw_rate;
		particles[i].x = particles[i].x + a*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta)); 
		particles[i].y = particles[i].y + a*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
		particles[i].theta = particles[i].theta + yaw_rate*delta_t;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	double min_dist = 50; 
	for(int i=0; i<predicted.size(); i++){
		for(int j=0; j<observations.size(); j++){
			double x1 = predicted[i].x;
			double y1 = predicted[i].y;
			double x2 = observations[j].x;
			double y2 = observations[j].y;

			double new_dist = dist(x1,y1,x2,y2);
			if (new_dist<min_dist){
				min_dist = new_dist;
				observations[j].id = predicted[i].id;
			}
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

		// don't forget to #include <math.h>
	sig_x= std_landmark[0];
	sig_y= std_landmark[1];
	// copy the obersations
	std::vector<LandmarkObs> predicted = observations
	// for each particle
	for (int j=0; j<particles.size(); j++){
		// re-initialize the predicted for a new particle 
		predicted = observations
		// transform the observation from the vehicle coordinate system to the map coordinate system 
		for(int i=0; i<predicted.size(); i++)
		{
			predicted.x = particles[j].x + cos(particles[j].theta * predicted.x) - sin(particles[j].theta * predicted.y);
			predicted.y = particles[j].y + sin(particles[j].theta * predicted.x) + cos(particles[j].theta * predicted.y);
		}
		// associate the observation with landmarks
		dataAssociation(predicted, map_landmarks);
		// calculate the final weight
		total_weight = 1;
		for(int i=0; i<predicted.size(); i++)
		{
			double x_obs=predicted[i].x;
			double y_obs=predicted[i].y;
			double x_mu=map_landmarks[predicted[i].id]
			double y_mu = 
			gauss_norm= (1/(2 * M_PI * sig_x * sig_y));
			exponent=(pow((x_obs - x_mu),2.0)/(2 * sig_x*sig_x) + (pow((y_obs - y_mu),2.0))/(2 * sig_y*sig_y);
			particles[i].weight = gauss_norm * exp(-exponent);
			total_weight *= particles[i].weight;
		}
		//update the weights vector
		weights[j] = total_weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
