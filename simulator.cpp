#include <map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <functional>

#include <boost/math/constants/constants.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>

#include "simulator.hpp"
#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/odometry_model.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "planar_robot/circle_controller.hpp"
#include "slam/mcmc_slam.hpp"
#include "slam/slam_data.hpp"
#include "utilities/random.hpp"

using planar_robot::pose;
using planar_robot::odometry_model;
using planar_robot::position;
using planar_robot::range_bearing_model;

const double PI = boost::math::constants::pi<double>();

// MAP AND TRAJECTORY PARAMETERS.
const double GRID_PITCH = 25; // meters; the spacing between landmarks on the grid.
const double RADIUS = 200; // meters; the radius of the circle driven by the robot.
const double CENTRE_X = RADIUS; // meters; the centre of the circle.
const double CENTRE_Y = 0; // meters; the centre of the circle.

// SIMULATION PARAMETERS
const double SPEED = 3; // meters/second; the speed of the robot.
const double DELTA_T = 0.25; // seconds; the interval between successive actions/observations.
const double REVS = 2.1; // the number of times to go around the circle

// ODOMETRY PARAMETERS
// see planar_robot/pose.hpp for an explanation of the four parameters of the odometry model.
//const odometry_model::builder ODOMETRY_MODEL (0.05, 1.0*PI/180, 0.1, 0.0001*180/PI);
const odometry_model::builder ODOMETRY_MODEL (0.05, 0.25*PI/180, 0.1, 0.0001*180/PI);

// OBSERVATION PARAMETERS
const range_bearing_model::builder SENSOR_MODEL (0.1, 1.0*PI/180.0); // standard deviation in range and bearing noise
const double OBS_MAX_RANGE = 30; // meters; the maximum range at which the sensors detect landmarks.

const int MCMC_STEPS = 100; // number of MCMC iterations per step.
const double ACT_DIM = 3.0; // the number of parameters estimated by each action edge.
const double OBS_DIM = 2.0; // the number of parameters estimated by each observation edge.


bool observation_in_range (const position& p) {
  return p.range() < OBS_MAX_RANGE;
}


/** Generates landmarks in an evenly spaced grid around the expected path of the robot. */
std::map<size_t, position> generate_circle_map () {
  std::map<size_t, position> features;
  size_t feature_id = 0;
  for (double x = 0; x <= 2*RADIUS+GRID_PITCH; x += GRID_PITCH) {
    for (double y = -2*RADIUS; y <= 2*RADIUS+GRID_PITCH; y += GRID_PITCH) {
      features[feature_id++] = position (x, y);
    }
  }
  return features;
}


/** Prints the given trajectory, as offset by the given pose, to ostream& out, with the format
    time_step  x_position  y_position  bearing */
void print_trajectory (std::ostream& out, pose p, const bitree<pose>& trajectory) {
  out << 0 << '\t' << p.x() << '\t' << p.y() << '\t' << p.bearing() << '\n';
  for (size_t i = 0; i < trajectory.size(); ++i) {
    p += trajectory.at(i);
    out << i+1 << '\t' << p.x() << '\t' << p.y() << '\t' << p.bearing() << '\n';
  }
}


/** Prints the given set of landmarks, as offset by the given pose, to ostream& out, with the format
    feature_id  x_position  y_position */
void print_landmarks (std::ostream& out, const std::map<size_t, position>& landmarks) {
  std::map<size_t, position>::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {
    const position& pos = i->second;
    out << i->first << '\t' << pos.x() << '\t' << pos.y() << '\n';
  }
}


/** Computes the cumulative root mean squared error between the two given trajectories. Ignores the
    bearing of the robot, but an incorrect bearing will drastically increase the error on future
    positions in the trajectory. */
double trajectory_error (const bitree<pose>& a, const bitree<pose>& b) {
  size_t points = std::min (a.size(), b.size());
  double error = 0;
  pose a_pos, b_pos;
  for (size_t i = 0; i < points; ++i) {
    a_pos += a.at(i);
    b_pos += b.at(i);
    error += (-a_pos + b_pos).distance_squared();
  }
  return std::sqrt (error/points);
}


/** Computes the root mean squared error between the given map and the given set of estimates. The
    estimates are assumed to be relative to the given pose p. */
double landmark_error (const std::map<size_t, position>& landmarks,
		       const std::map<size_t, position>& estimates) {
  double error = 0;
  size_t num_landmarks = 0;
  std::map<size_t, position>::const_iterator i = estimates.begin();
  for (; i != estimates.end(); ++i) {
    std::map<size_t, position>::const_iterator j = landmarks.find(i->first);
    assert (j != landmarks.end());
    const position& estimate = i->second;
    const position& actual = j->second;
    double x = estimate.x() - actual.x();
    double y = estimate.y() - actual.y();
    error += x*x + y*y;
    ++num_landmarks;
  }
  return std::sqrt(error/num_landmarks);
}


int main () {

  // The global random source. TODO: use two generators, one for the simulation itself and another for
  // the MCMC SLAM module.
  random_source random;

  // Seed the random generator from the current time. This is not ideal, but std::tr1::random_device
  // appears to be broken (it always returns the same value) so this is the best we can do.
  const unsigned long seed = std::time(0);
  random.generator.seed(seed);

  // Generate the list of landmarks.
  const std::map<size_t, position> landmarks = generate_circle_map ();

  // Use a circle controller
  planar_robot::circle_controller controller (CENTRE_X, CENTRE_Y, RADIUS, SPEED, REVS);

  simulator<odometry_model::builder, range_bearing_model::builder> sim
    (landmarks, std::tr1::ref(controller), controller.initial_pose(),
     ODOMETRY_MODEL, SENSOR_MODEL, observation_in_range,
     MCMC_STEPS, ACT_DIM, OBS_DIM);
     
  boost::timer timer;

  // Run the simulation
  while (!controller.finished()) {
    sim (random, DELTA_T);
    std::cout << sim.get_num_steps() << '\n';
  }

  // Store elapsed time. We don't want to include the time taken by computing errors and doing I/O.
  const double elapsed_time = timer.elapsed();

  // Get the estimated landmark positions generated by MCMC SLAM.
  const std::map<size_t, position> estimated_map = sim.get_feature_estimates();

  // Store the expected trajectory.
  std::ofstream expected_file ("data/expected_trajectory.txt");
  print_trajectory (expected_file, sim.get_initial_state(), sim.get_expected_state());
  expected_file.close();

  // Store the actual trajectory.
  std::ofstream actual_file ("data/actual_trajectory.txt");
  print_trajectory (actual_file, sim.get_initial_state(), sim.get_state());
  actual_file.close();

  // Store the estimated trajectory.
  std::ofstream estimated_file ("data/estimated_trajectory.txt");
  print_trajectory (estimated_file, sim.get_initial_state(), sim.get_estimated_state());
  estimated_file.close();

  // Store the actual landmark positions.
  std::ofstream map_file ("data/landmarks.txt");
  print_landmarks (map_file, landmarks);
  map_file.close();

  // Store the estimated landmark positions.
  std::ofstream estimated_map_file ("data/estimated_map.txt");
  print_landmarks (estimated_map_file, estimated_map);
  estimated_map_file.close();

  // Summarise key information about this simulation.
  std::ofstream summary_file ("data/summary.txt");
  summary_file << "Random seed: " << seed << "\n"
	       << "MCMC iterations: " << MCMC_STEPS << "\n"
	       << "Grid spacing: " << GRID_PITCH << " meters\n"
	       << "Number of landmarks: " << landmarks.size() << "\n"
	       << "Circle radius: " << RADIUS << " meters \n"
	       << "Time steps: " << sim.get_num_steps() << "\n"
	       << "Average observations per step: " << sim.get_observations_per_step() << "\n"
	       << "Trajectory error: " << trajectory_error (sim.get_state(), sim.get_estimated_state()) << "\n"
	       << "Landmark error: " << landmark_error (landmarks, estimated_map) << "\n"
	       << "Elapsed time: " << elapsed_time << "\n";
  summary_file.close();

  return 0;

}
