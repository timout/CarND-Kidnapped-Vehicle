/*
 * particle_filter.cpp
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <map>

#include "particle_filter.h"
using namespace std;

using landmark_t=Map::single_landmark_s;

// Random Gaussian noise generator.
class NoiseGen 
{
private:
  normal_distribution<double> dist_x;
  normal_distribution<double> dist_y;
  normal_distribution<double> dist_theta;
  std::default_random_engine gen;
public:
  NoiseGen(double x, double y, double theta, double std_pos[]) :
    dist_x(x, std_pos[0]), dist_y(y, std_pos[1]), dist_theta(theta, std_pos[2]) {}

  double x() { return dist_x(gen); }  
  double y() { return dist_y(gen); } 
  double theta() { return dist_theta(gen); } 
};

void ParticleFilter::init(double x, double y, double theta, double std_pos[])
{
  // Sets the number of particles. Initializes all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Adds random Gaussian noise to each particle.
  const double init_weight = 1.0;
  NoiseGen noise(x, y, theta, std_pos);
  // Initializes particles - from the normal distributions set above
  for (int i = 0; i < num_particles; ++i) {
    particles[i] = {i, noise.x(), noise.y(), noise.theta(), init_weight};
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
  // Adds measurements for velocity and yaw_rate to each particle and adds random Gaussian noise.
  NoiseGen noise(0, 0, 0, std_pos);

  // Different equations based on if yaw_rate is zero or not
  for (Particle & p : particles)
  {
    if (fabs(yaw_rate) > 0.001)
    {
      // Add measurements to particles
      p.x += (velocity / yaw_rate) * (sin(p.theta + (yaw_rate * delta_t)) - sin(p.theta));
      p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + (yaw_rate * delta_t)));
      p.theta += yaw_rate * delta_t;
    }
    else
    {
      // Add measurements to particles
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
      // Theta will stay the same due to no yaw_rate
    }
    // Add noise to the particles
    p.x += noise.x();
    p.y += noise.y();
    p.theta += noise.theta();
  }
}

void ParticleFilter::updateWeights(
    double sensor_range,
    double std_landmark[],
    const std::vector<LandmarkObs> &observations,
    const Map &map_landmarks)
{
  // Updates the weights of each particle using a multi-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // First, when iterating through each particle, need to transform observation points to map coordinates.
  // Next, associate each observation to its nearest landmark. The distribution can then be calculated.

  // First term of multi-variate normal Gaussian distribution
  const double a = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

  // The denominators of the Gaussian distribution
  const double x_denom = 2 * std_landmark[0] * std_landmark[0];
  const double y_denom = 2 * std_landmark[1] * std_landmark[1];

  for (Particle & p : particles) {
    // Multi-variate Gaussian distribution
    double gd = 1.0;
    vector<LandmarkObs> trans_obs(observations.size());
    // Transform the observation point (from vehicle coordinates to map coordinates)
    transform(observations.begin(), observations.end(), trans_obs.begin(), [&](const LandmarkObs & obs) { 
      double obs_x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
      double obs_y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;
      LandmarkObs to = { obs.id, obs_x, obs_y}; 
      return to;
    });

    for (auto & o : trans_obs) {
      // use observations only in sensor range
      double obs_range = dist(p.x, p.y, o.x, o.y);
      if (obs_range > sensor_range) continue;
      auto landmarks = map_landmarks.landmark_list;
      landmark_t & nearest_landmark = *min_element(landmarks.begin(), landmarks.end(), [&](landmark_t l1, landmark_t l2) {
        double dist1 = dist(o.x, o.y, l1.x_f, l1.y_f);
        double dist2 = dist(o.x, o.y, l2.x_f, l2.y_f);
        return dist1 < dist2;
      });
      // Associate the observation point with its nearest landmark neighbor
      // and Calculate multi-variate Gaussian distribution
      double x_diff = o.x - nearest_landmark.x_f;
      double y_diff = o.y - nearest_landmark.y_f;
      double b = ((x_diff * x_diff) / x_denom) + ((y_diff * y_diff) / y_denom);
      gd *= a * exp(-b);
    }
    // Update particle weights with combined multi-variate Gaussian distribution
    p.weight = gd;
  }
}

void ParticleFilter::resample()
{
  // Resamples particles with replacement with probability proportional to their weight.
  // Vector for new particles
  vector<Particle> new_particles(num_particles);
  vector<double> w(num_particles);
  transform(particles.begin(), particles.end(), w.begin(), [](Particle &p) { return p.weight; });
  // Use discrete distribution to return particles by weight
  discrete_distribution<int> index(w.begin(), w.end());
  generate(new_particles.begin(), new_particles.end(), [&]() { return particles[index(gen)]; });
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(
    Particle & particle,
    const std::vector<int> &associations,
    const std::vector<double> &sense_x,
    const std::vector<double> &sense_y)
{
  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle & best)
{
  return join(best.associations.begin(), best.associations.end());
}

string ParticleFilter::getSenseX(Particle & best)
{
  return join(best.sense_x.begin(), best.sense_x.end());
}

string ParticleFilter::getSenseY(Particle & best)
{
  return join(best.sense_y.begin(), best.sense_y.end());
}
