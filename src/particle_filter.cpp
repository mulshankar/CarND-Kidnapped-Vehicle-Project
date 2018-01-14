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
	
	num_particles=1;
	
	std:default_random_engine gen;
	
	std:normal_distribution<double> N_x(x,std[0]);
	std:normal_distribution<double> N_y(y,std[1]);
	std:normal_distribution<double> N_theta(theta,std[2]);
	
	for (int i=0; i<num_particles;i++) {	
		Particle particle;
		particle.id=i;
		particle.x=N_x(gen);
		particle.y=N_y(gen);
		particle.theta=N_theta(gen);
		particle.weight=1.0;		
		particles.push_back(particle);
		weights.push_back(particle.weight);
	}
	
	is_initialized=true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	std:default_random_engine gen;

	for (int i=0; i<num_particles;i++) {
		
		double new_x;
		double new_y;
		double new_theta;
		
		if (yaw_rate==0) {
			new_x=particles[i].x+delta_t*velocity*cos(particles[i].theta);
			new_y=particles[i].y+delta_t*velocity*sin(particles[i].theta);
			new_theta=particles[i].theta;
		}
		else {
			new_x=particles[i].x+ velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			new_y=particles[i].y+ velocity/yaw_rate*(-cos(particles[i].theta+yaw_rate*delta_t)+cos(particles[i].theta));
			new_theta=particles[i].theta+delta_t*yaw_rate;			
		}

		normal_distribution<double> N_x(new_x,std_pos[0]);
		normal_distribution<double> N_y(new_y,std_pos[1]);
		normal_distribution<double> N_theta(new_theta,std_pos[2]);
	
		particles[i].x=N_x(gen);
		particles[i].y=N_y(gen);
		particles[i].theta=N_theta(gen);
		
		cout<< "Particle id: "<<particles[i].id<<" Prediction particle x: " << particles[i].x << " Prediction particle y: " << particles[i].y << " Prediction particle theta: "<<particles[i].theta<<endl;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	//vector<int>associations;
	//vector<double>sense_x;
	//vector<double>sense_y;
	//vector<LandmarkObs> trans_observations;
	//LandmarkObs trans_obs;
	
	LandmarkObs obs;
		
	weights.clear(); // clear the old weights vector
				
	for (int p=0; p<num_particles;p++) { // for each particle
	
	double weight_upd=1.0; // init weight for each particle
	
		for (int i=0; i<observations.size();i++) { // transform from car co-ordinates to map co-ordinates
		
			obs=observations[i];		
			
			//STEP1: Transformation from car coordinates to map coordinates
			
			double x_obs=cos(particles[p].theta)*obs.x-sin(particles[p].theta)*obs.y+particles[p].x;
			double y_obs=cos(particles[p].theta)*obs.y+sin(particles[p].theta)*obs.x+particles[p].y;				
			//trans_observations.push_back(trans_obs); // all transformed observations in list				
			
			//STEP2: Associate observation to closest landmark via euclidean distance				
			
			LandmarkObs nearest_landmark;
			double min_dist=sensor_range;
			
			for (int l=0; l<map_landmarks.map.landmark_list.size();l++) { // calculate rho from various landmarks
				
				double x_l_map=map_landmarks.map.landmark_list[l].x_f;
				double y_l_map=map_landmarks.map.landmark_list[l].y_f;
				
				double x2_x1=(x_l_map-x_obs); // Calculate Euclidean distance, rho
				double y2_y1=(y_l_map-y_obs);
				double rho=sqrt(pow(x2_x1,2)+pow(y2_y1,2));
				
				if (rho<min_dist) { // if rho is above sensor range, ignore and find the nearest landmark
				
					nearest_landmark.id=map_landmarks.map.landmark_list[l].id;
					nearest_landmark.x=x_l_map;
					nearest_landmark.y=y_l_map;	
					min_dist=rho;							
				}				
			}		
			// STEP3: Update Weight
			
			double norm=1/(2*PI*std_landmark[0]*std_landmark[1]);
			
			double x_x_mu2=pow((x_obs-nearest_landmark.x),2);
			double y_y_mu2=pow((y_obs-nearest_landmark.y),2);
			double exp_term=(x_x_mu2/(2*std_landmark[0]*std_landmark[0])+y_y_mu2/(2*std_landmark[1]*std_landmark[1]));
			
			weight_upd= weight_upd*norm*exp(-exp_term);
		}
		
		particle[p].weight=weight_upd;
		weights.push_back(weight_upd);
	}
}

void ParticleFilter::resample() {

	default_random_engine gen;
	vector <Particle> resample_particles;	
	discrete_distribution<int> distribution(weights.begin(),weights.end());	
	
	for (int i=0; i<num_particles;i++) {	
		resample_particles.push_back(particles[distribution(gen)]);
	}
	
	particles=resample_particles;	
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
	
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
