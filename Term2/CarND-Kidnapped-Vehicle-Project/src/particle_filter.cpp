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
	// random engine used for sampling from normal distribution
	default_random_engine gen;
	// create random distribution 
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	// choose tje number of particles
	num_particles = 50;
	//initialize the weights to 1
	//initialize all particles to first position and add random noise

	for(int i=0; i<num_particles; i++){
		// sample the gaussian random distribution 
		double noise_x = dist_x(gen);
        double noise_y = dist_y(gen);
        double noise_theta = dist_theta(gen);
        //create a particule
        Particle p;
        //init weight
		weights.push_back(1.0);
		//init particle i
		p.id = i;
		p.x = noise_x; 
		p.y = noise_y;
		p.theta = noise_theta;
		p.weight = 1.0;
		// add the particule to the vector
		particles.push_back(p);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// random engine used for sampling from normal distribution
	default_random_engine gen;
	// create random distribution 
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for(int i=0; i<num_particles; i++){
		// sample the gaussian random distribution for the noise
		double noise_x = dist_x(gen);
        double noise_y = dist_y(gen);
        double noise_theta = dist_theta(gen);
        //do not divide by zero
        if(fabs(yaw_rate)>0.001){
        	double a = velocity/yaw_rate;
			particles[i].x +=  a*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta))+ noise_x; 
			particles[i].y +=  a*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t))+ noise_y;
			particles[i].theta += yaw_rate*delta_t + noise_theta;
        }
        else{
        	// yaw rate is zero so the yaw hasn't changed
        	particles[i].x += velocity*delta_t*cos(particles[i].theta)+ noise_x; 
			particles[i].y += velocity*delta_t*sin(particles[i].theta)+ noise_y; 
        }
	}
}

std::vector<LandmarkTrueToObs> ParticleFilter::dataAssociation(std::vector<LandmarkObs>& landmarks, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	std::vector<LandmarkTrueToObs> LandmarkAssociation; 
	for(int i=0; i<landmarks.size(); i++)
	{
		double min_dist = 1000000; 
		double x1 = landmarks[i].x;
		double y1 = landmarks[i].y;
		int id = landmarks[i].id;
		double x_obs=0;
		double y_obs=0;
		
		for(int j=0; j<observations.size(); j++)
		{
			double x2 = observations[j].x;
			double y2 = observations[j].y;

			double new_dist = dist(x1,y1,x2,y2);
			if (new_dist<min_dist){
				min_dist = new_dist;
				x_obs = x2;
				y_obs = y2;
			}
		}
		LandmarkAssociation.push_back({id, x1, y1, x_obs, y_obs});
	}
	return LandmarkAssociation;
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

	/*
	the sensor can only see in a certain range. 
	When we know the position of the particle, we can assume that the sensor will only see in a certain perimeter
	so we can only look at the landmarks that are in this perimeter for this particle.
	Once we have all the observations in the map coordinate system for a given particule, we can associate a landmark to 
	its closest observation. 
	I decided to have a structure holding this information. 
	With all that, we can calculate the new weights.
	*/

	// for each particle
	for (int j=0; j<particles.size(); j++){
		// list of all landmarks within range of this particle
		std::vector<LandmarkObs> landmarks_withinRange;
		// get particle coordinates
		double p_x = particles[j].x;
		double p_y = particles[j].y;
		double p_theta = particles[j].theta;
		// re-initialize the vector holding the observations in the map frame
		std::vector<LandmarkObs> observations_mapFrame;
		// transform the observation from the vehicle coordinate system to the map coordinate system 
		for(int i=0; i<observations.size(); i++)
		{
			double obsx_map = p_x + cos(p_theta) * observations[i].x - sin(p_theta)* observations[i].y;
			double obsy_map = p_y + sin(p_theta) * observations[i].x + cos(p_theta)* observations[i].y;
			int obsid_map = observations[i].id;
			observations_mapFrame.push_back(LandmarkObs{obsid_map, obsx_map, obsy_map});
		}
		// select only the landmarks around the particle
		for(int l=0; l<map_landmarks.landmark_list.size(); l++)
		{
			double l_x = map_landmarks.landmark_list[l].x_f;
			double l_y = map_landmarks.landmark_list[l].y_f;
			int l_id = map_landmarks.landmark_list[l].id_i;
			double d_x = l_x - p_x;
			double d_y = l_y - p_y;
			if(d_x*d_x + d_y*d_y <= sensor_range*sensor_range)
			{
				landmarks_withinRange.push_back(LandmarkObs{l_id, l_x, l_y});

			}
		}
		// associate the landmarks that are in range with the closest obervation
		std::vector<LandmarkTrueToObs> predicted_landmarks = dataAssociation(landmarks_withinRange, observations_mapFrame);
		// re-init the weights of each particle
		particles[j].weight= 1.;
		
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		double cov_x = sig_x*sig_x;
		double cov_y = sig_y*sig_y;
		double gauss_norm= (1/(2.0 * M_PI * sig_x * sig_y));
		if(predicted_landmarks.size()>0)
		{
			for(int i=0; i<predicted_landmarks.size(); i++)
			{
				double x_obs=predicted_landmarks[i].x_obs;
				double y_obs=predicted_landmarks[i].y_obs;
				double x_mu=predicted_landmarks[i].x_map;
				double y_mu = predicted_landmarks[i].y_map;
				double exponent=(pow((x_obs - x_mu),2.0)/(2*cov_x) + (pow((y_obs - y_mu),2.0))/(2*cov_y));
				particles[j].weight *= gauss_norm * exp(-exponent);
			}
		}
		else{ // couldn't find a landmark
			particles[j].weight = 0;
		}
		weights[j] = particles[j].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// generate random starting index for resampling wheel
	
	static default_random_engine gen;
	vector<Particle> new_particles;
	//randomly draw the index of a particle
	uniform_int_distribution<int> uniintdist(0, num_particles-1);
	int index = uniintdist(gen);

	// get max weight
	double max_weight = *max_element(weights.begin(), weights.end());

	// uniform random distribution [0.0, max_weight]
	uniform_real_distribution<double> unirealdist(0.0, 2*max_weight);

	double beta = 0.0;

	// spin the resample wheel!
	for (int i = 0; i < num_particles; i++) {
		beta += unirealdist(gen);
		while (beta > weights[index]) {
			beta -= weights[index];
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

bool ParticleFilter::point_inside_circle(float center_x, float center_y, double o_x, double o_y, double radius){
	double a = pow((center_x-o_x),2.0) + pow((center_y-o_y),2.0);
	double b = radius*radius;
	bool inside = false;
	if (a<b){
		inside = true;
	}
	return inside;
}