#include <map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <boost/math/constants/constants.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/observation.hpp"
#include "slam/mcmc_slam.hpp"
#include "slam/slam_data.hpp"
#include "utilities/random.hpp"

using planar_robot::pose;
using planar_robot::pose_dist;
using planar_robot::pose_dist_odometry;
using planar_robot::observation;
using planar_robot::observation_dist;
using planar_robot::landmark;

const double PI = boost::math::constants::pi<double>();
const double GRID_PITCH = 25; // meters
const double RADIUS = 200; // meters
const double CENTRE_X = RADIUS; // meters
const double CENTRE_Y = 0; // meters

const double SPEED = 3; // meters/second
const double DELTA_T = 0.25; // seconds
const double RUN_TIME = 2.1*2*PI*RADIUS/SPEED; // seconds

const double OBS_MAX_RANGE = 30; // meters
const double OBS_RANGE_SIGMA = 0.1; // meters
const double OBS_BEARING_SIGMA = 1.0*PI/180.0; // radians

//const pose_dist_odometry ODOMETRY_MODEL (0.05, 1.0*PI/180, 0.1, 0.0001*180/PI);
const pose_dist_odometry ODOMETRY_MODEL (0.05, 0.25*PI/180, 0.1, 0.0001*180/PI);

const int MCMC_STEPS = 100;
const double ACT_DIM = 3.0;
const double OBS_DIM = 2.0;

static random_source rand_gen;

std::map<size_t, observation> generate_circle_map () {
  std::map<size_t, observation> features;
  size_t feature_id = 0;
  for (double x = 0; x <= 2*RADIUS+GRID_PITCH; x += GRID_PITCH) {
    for (double y = -2*RADIUS; y <= 2*RADIUS+GRID_PITCH; y += GRID_PITCH) {
      features[feature_id++] = landmark (x, y);
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

size_t add_observations (const pose& position,
			 const std::map<size_t, observation>& landmarks,
			 slam_data<pose_dist, observation_dist>& data) {

  size_t num_observations = 0;

  std::map<size_t, observation>::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {

    observation_dist distribution (-position + i->second, OBS_RANGE_SIGMA, OBS_BEARING_SIGMA);
    observation measurement = distribution(rand_gen);

    if (measurement.range() < OBS_MAX_RANGE) {
      data.add_observation(i->first, observation_dist (measurement, OBS_RANGE_SIGMA, OBS_BEARING_SIGMA));
      ++num_observations;
    }
  }

  return num_observations;
}

void print_trajectory (std::ostream& out, pose p, const bitree<pose>& trajectory) {
  out << 0 << '\t' << p.x() << '\t' << p.y() << '\t' << p.bearing() << '\n';
  for (size_t i = 0; i < trajectory.size(); ++i) {
    p += trajectory.at(i);
    out << i+1 << '\t' << p.x() << '\t' << p.y() << '\t' << p.bearing() << '\n';
  }
}

void print_landmarks (std::ostream& out, const pose& p, const std::map<size_t, observation>& landmarks) {
  std::map<size_t, observation>::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {
    observation pos = p + i->second;
    out << i->first << '\t' << pos.x() << '\t' << pos.y() << '\n';
  }
}

double trajectory_error (const bitree<pose>& a, const bitree<pose>& b) {
  size_t points = std::min (a.size(), b.size());
  double error = 0;
  pose a_pos, b_pos;
  for (size_t i = 0; i < points; ++i) {
    a_pos += a.at(i);
    b_pos += b.at(i);
    double distance = (-a_pos + b_pos).distance();
    error += distance * distance;
  }
  return std::sqrt (error/points);
}

double landmark_error (const std::map<size_t, observation>& landmarks,
		       const pose& p, const std::map<size_t, observation>& estimates) {
  double error = 0;
  size_t num_landmarks = 0;
  std::map<size_t, observation>::const_iterator i = estimates.begin();
  for (; i != estimates.end(); ++i) {
    std::map<size_t, observation>::const_iterator j = landmarks.find(i->first);
    assert (j != landmarks.end());
    observation estimate = p + i->second;
    const observation& actual = j->second;
    double x = estimate.x() - actual.x();
    double y = estimate.y() - actual.y();
    error += x*x + y*y;
    ++num_landmarks;
  }
  return std::sqrt(error/num_landmarks);
}

int main () {

  const unsigned long seed = std::time(0);
  rand_gen.generator.seed(seed);

  const std::map<size_t, observation> landmarks = generate_circle_map ();

  typedef slam_data<pose_dist, observation_dist> slam_data_type;

  slam_data_type data;
  mcmc_slam<slam_data_type> mcmc (rand_gen, data, MCMC_STEPS, ACT_DIM, OBS_DIM);

  const pose initial_position = get_pose(0);
  bitree<pose> expected_trajectory, actual_trajectory;

  long time_step = 0;
  long num_steps = RUN_TIME/DELTA_T;
  long total_observations = 0;
  boost::progress_display progress (num_steps);
  boost::timer timer;

  while (time_step <= num_steps) {

    pose expected_position = initial_position + expected_trajectory.accumulate (time_step);
    pose actual_position = initial_position + actual_trajectory.accumulate (time_step);

    total_observations += add_observations (actual_position, landmarks, data);

    mcmc.update();
    ++time_step;
    ++progress;

    const pose_dist distribution = ODOMETRY_MODEL (-expected_position + get_pose (time_step*DELTA_T));

    expected_trajectory.push_back (distribution.mean());
    actual_trajectory.push_back(distribution(rand_gen));
    data.add_action(distribution);

  }

  const double elapsed_time = timer.elapsed();

  const std::map<size_t, observation> estimated_map = mcmc.get_feature_estimates();
  const bitree<pose>& estimated_trajectory = mcmc.get_action_estimates();

  std::ofstream expected_file ("data/expected_trajectory.txt");
  print_trajectory (expected_file, initial_position, expected_trajectory);
  expected_file.close();

  std::ofstream actual_file ("data/actual_trajectory.txt");
  print_trajectory (actual_file, initial_position, actual_trajectory);
  actual_file.close();

  std::ofstream estimated_file ("data/estimated_trajectory.txt");
  print_trajectory (estimated_file, initial_position, estimated_trajectory);
  estimated_file.close();

  std::ofstream map_file ("data/landmarks.txt");
  print_landmarks (map_file, pose(), landmarks);
  map_file.close();

  std::ofstream estimated_map_file ("data/estimated_map.txt");
  print_landmarks (estimated_map_file, initial_position, estimated_map);
  estimated_map_file.close();

  std::ofstream summary_file ("data/summary.txt");
  summary_file << "Random seed: " << seed << "\n"
	       << "MCMC iterations: " << MCMC_STEPS << "\n"
	       << "Grid spacing: " << GRID_PITCH << " meters\n"
	       << "Number of landmarks: " << landmarks.size() << "\n"
	       << "Circle radius: " << RADIUS << " meters \n"
	       << "Time steps: " << num_steps << "\n"
	       << "Average observations per step: " << double(total_observations)/num_steps << "\n"
	       << "Trajectory error: " << trajectory_error (actual_trajectory, estimated_trajectory) << "\n"
	       << "Landmark error: " << landmark_error (landmarks, initial_position, estimated_map) << "\n"
	       << "Elapsed time: " << elapsed_time << "\n";
  summary_file.close();

  return 0;
}
