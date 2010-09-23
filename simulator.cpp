#include <cmath>
#include <utility>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <tr1/random>
#include <boost/math/constants/constants.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/observation.hpp"
#include "slam/mcmc_slam.hpp"
#include "utilities/arraymap.hpp"
#include "utilities/random.hpp"

using planar_robot::pose;
using planar_robot::pose_dist;
using planar_robot::observation;
using planar_robot::observation_dist;
using planar_robot::landmark;

const double PI = boost::math::constants::pi<double>();
const double GRID_PITCH = 25;              // meters
const double RADIUS = 50;          // meters
const double CENTRE_X = RADIUS;            // meters
const double CENTRE_Y = 0;                 // meters

const double SPEED = 1;                    // meters/second
const double DELTA_T = 1.0;               // seconds
const double RUN_TIME = 1.1*2*PI*RADIUS/SPEED; // seconds

const double OBSERVATION_MAX_DIST = 30;    // meters
const double OBSERVATION_RANGE_SIGMA = 0.05;  // meters
const double OBSERVATION_BEARING_SIGMA = 0.02; // radians

const double ACTION_DIST_SIGMA = 0.1*DELTA_T;     // meters/second
const double ACTION_DIR_SIGMA = 0.02*DELTA_T;     // radians/second
const double ACTION_BEARING_SIGMA = 0.02*DELTA_T; // radians/second

static random_source rand_gen;

std::vector<observation> generate_circle_map () {
  std::vector<observation> features;
  for (double x = 0; x <= 2*RADIUS+GRID_PITCH; x += GRID_PITCH) {
    for (double y = -2*RADIUS; y <= 2*RADIUS+GRID_PITCH; y += GRID_PITCH) {
      features.push_back (landmark (x, y));
    }
  }
  return features;
}

pose get_pose (double time) {
  double theta = SPEED * time / RADIUS;
  double x = CENTRE_X + RADIUS*std::cos(theta);
  double y = CENTRE_Y + RADIUS*std::sin(theta);
  double heading = theta + PI/2;
  return pose (x, y, heading);
}

size_t add_observations (size_t time_step, const pose& position,
		       const std::vector<observation>& landmarks,
		       std::vector<arraymap<size_t, observation_dist> >& observations) {

  size_t num_observations = 0;

  for (size_t i = 0; i < landmarks.size(); ++i) {   
    observation_dist distribution (-position + landmarks[i],
				   OBSERVATION_RANGE_SIGMA, OBSERVATION_BEARING_SIGMA);
    observation measurement = distribution(rand_gen);
    if (measurement.range() < OBSERVATION_MAX_DIST) {
      observations[i][time_step] = observation_dist (measurement,
						     OBSERVATION_RANGE_SIGMA*10,
						     OBSERVATION_BEARING_SIGMA*5);
      ++num_observations;
    }
  }

  return num_observations;
}

void print_trajectory (std::ostream& out, pose p, const bitree<pose>& trajectory) {
  out << 0 << '\t' << p.x() << '\t' << p.y() << '\t' << p.bearing() << '\n';
  for (size_t i = 0; i < trajectory.size(); ++i) {
    p += trajectory.get(i);
    out << i+1 << '\t' << p.x() << '\t' << p.y() << '\t' << p.bearing() << '\n';
  }
}

void print_landmarks (std::ostream& out, pose p, const std::vector<observation>& map) {
  for (size_t i = 0; i < map.size(); ++i) {
    observation pos = p + map[i];
    out << pos.x() << '\t' << pos.y() << '\n';
  }
}

int main () {

  std::tr1::random_device sysrand;
  rand_gen.generator.seed(sysrand());

  std::vector<observation> map = generate_circle_map ();

  std::vector<pose_dist> actions;
  std::vector<arraymap<size_t, observation_dist> > observations (map.size());

  mcmc_slam<pose_dist, observation_dist> mcmc (actions, observations, rand_gen);
  bitree<pose> expected_trajectory, actual_trajectory;

  const pose initial_position = get_pose(0);

  size_t time_step = 0;
  while (time_step*DELTA_T <= RUN_TIME) {

    pose expected_position = initial_position + expected_trajectory.accumulate(time_step);
    pose actual_position = initial_position + actual_trajectory.accumulate(time_step);

    size_t num_observations
      = add_observations(time_step, actual_position, map, observations);

    std::cout << "Time step: " << time_step
	      << ", " << num_observations << " observations" <<  std::endl;

    mcmc.initialize();
    mcmc.update();
    ++time_step;

    pose expected_change = -expected_position + get_pose (time_step*DELTA_T);
    expected_trajectory.push_back (expected_change);

    pose_dist distribution (expected_change, ACTION_DIST_SIGMA,
			    ACTION_DIR_SIGMA, ACTION_BEARING_SIGMA);
    actions.push_back(distribution);

    pose actual_change = distribution(rand_gen);
    actual_trajectory.push_back(actual_change);

  }

  std::ofstream expected_file ("data/expected_trajectory.txt");
  print_trajectory (expected_file, initial_position, expected_trajectory);
  expected_file.close();

  std::ofstream actual_file ("data/actual_trajectory.txt");
  print_trajectory (actual_file, initial_position, actual_trajectory);
  actual_file.close();

  std::ofstream estimated_file ("data/estimated_trajectory.txt");
  print_trajectory (estimated_file, initial_position, mcmc.get_action_estimates());
  estimated_file.close();

  std::ofstream map_file ("data/landmarks.txt");
  print_landmarks (map_file, pose(), map);
  map_file.close();

  std::ofstream estimated_map_file ("data/estimated_map.txt");
  print_landmarks (estimated_map_file, initial_position, mcmc.get_feature_estimates());
  estimated_map_file.close();

  return 0;
}
