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
#include "planar_robot/position.hpp"
#include "planar_robot/odometry_model.hpp"
#include "planar_robot/range_bearing_model.hpp"
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
const double RUN_TIME = 2.1*2*PI*RADIUS/SPEED; // seconds; the total time for which the simulation runs.

// OBSERVATION PARAMETERS
const double OBS_MAX_RANGE = 30; // meters; the maximum range at which the sensors detect landmarks.
const double OBS_RANGE_SIGMA = 0.1; // meters; the standard deviation in the measured range to landmarks.
const double OBS_BEARING_SIGMA = 1.0*PI/180.0; // radians; the standard deviation in the measured bearing.

// ODOMETRY PARAMETERS
// see planar_robot/pose.hpp for an explanation of the four parameters of the odometry model.
//const odometry_model::builder ODOMETRY_MODEL (0.05, 1.0*PI/180, 0.1, 0.0001*180/PI);
const odometry_model::builder ODOMETRY_MODEL (0.05, 0.25*PI/180, 0.1, 0.0001*180/PI);

const int MCMC_STEPS = 100; // number of MCMC iterations per step.
const double ACT_DIM = 3.0; // the number of parameters estimated by each action edge.
const double OBS_DIM = 2.0; // the number of parameters estimated by each observation edge.

// The global random source. TODO: use two generators, one for the simulation itself and another for
// the MCMC SLAM module.
static random_source rand_gen;


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


/** Gets the expected pose of the robot at the given time. The robot expects to drive in a circle with
    the given radius and center and at the given speed. */
pose get_pose (double time) {
  double theta = SPEED * time / RADIUS;
  double x = CENTRE_X + RADIUS*std::cos(theta);
  double y = CENTRE_Y + RADIUS*std::sin(theta);
  double heading = theta + PI/2;
  return pose (x, y, heading);
}


/** Given the current position of the robot and the list of landmarks, generates observations for the 
    landmarks in range and adds them to the given slam_data object. */
size_t add_observations (const pose& sensor_pose, const std::map<size_t, position>& landmarks,
			 slam_data<odometry_model, range_bearing_model>& data) {

  size_t num_observations = 0;

  std::map<size_t, position>::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {

    // The measurement is a random sample from the distribution which has the true observation as mean.
    range_bearing_model distribution (-sensor_pose + i->second, OBS_RANGE_SIGMA, OBS_BEARING_SIGMA);
    position measurement = distribution(rand_gen);

    if (measurement.range() < OBS_MAX_RANGE) {
      // The measured observation is a distribution with the measurement as mean.
      data.add_observation(i->first, range_bearing_model (measurement, OBS_RANGE_SIGMA, OBS_BEARING_SIGMA));
      ++num_observations;
    }
  }

  // Returns the total number of observations added.
  return num_observations;
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
void print_landmarks (std::ostream& out, const pose& p, const std::map<size_t, position>& landmarks) {
  std::map<size_t, position>::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {
    position pos = p + i->second;
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
		       const pose& p, const std::map<size_t, position>& estimates) {
  double error = 0;
  size_t num_landmarks = 0;
  std::map<size_t, position>::const_iterator i = estimates.begin();
  for (; i != estimates.end(); ++i) {
    std::map<size_t, position>::const_iterator j = landmarks.find(i->first);
    assert (j != landmarks.end());
    position estimate = p + i->second;
    const position& actual = j->second;
    double x = estimate.x() - actual.x();
    double y = estimate.y() - actual.y();
    error += x*x + y*y;
    ++num_landmarks;
  }
  return std::sqrt(error/num_landmarks);
}


int main () {

  // Seed the random generator from the current time. This is not ideal, but std::tr1::random_device
  // appears to be broken (it always returns the same value) so this is the best we can do.
  const unsigned long seed = std::time(0);
  rand_gen.generator.seed(seed);

  // Generate the list of landmarks.
  const std::map<size_t, position> landmarks = generate_circle_map ();

  // A container to hold all the state changes and observations.
  typedef slam_data<odometry_model, range_bearing_model> slam_data_type;
  slam_data_type data;

  // Initialise the MCMC SLAM object.
  mcmc_slam<slam_data_type> mcmc (rand_gen, data, MCMC_STEPS, ACT_DIM, OBS_DIM);

  // The expected and actual trajectories and the initial pose relative to which they are measured.
  const pose initial_position = get_pose(0);
  bitree<pose> expected_trajectory, actual_trajectory;

  long time_step = 0;
  long num_steps = RUN_TIME/DELTA_T;
  long total_observations = 0;
  boost::progress_display progress (num_steps);
  boost::timer timer;

  while (time_step <= num_steps) {

    // The expected and actual positions of the robot at this time step.
    pose expected_position = initial_position + expected_trajectory.accumulate (time_step);
    pose actual_position = initial_position + actual_trajectory.accumulate (time_step);

    // Keep track of how many observations have been made throughout the simulation.
    total_observations += add_observations (actual_position, landmarks, data);

    mcmc.update(); // Perform MCMC updates
    ++time_step;
    ++progress;

    // The odometry reading is the difference between where the robot thinks it is now and where it
    // wants to be at the next time step. Therefore the state change distribution has this reading
    // as the mean.
    const odometry_model distribution = ODOMETRY_MODEL (-expected_position + get_pose (time_step*DELTA_T));

    // The expected trajectory (i.e. the maximum prior likelihood trajectory) just takes the mean of the
    // above distribution ...
    expected_trajectory.push_back (distribution.mean());

    // ... but the actual trajectory is a random sample from the distribution.
    actual_trajectory.push_back(distribution(rand_gen));

    // Store the distribution as the latest state change. The SLAM module must try to recover the random
    // sample that describes the actual trajectory of the robot.
    data.add_action(distribution);

  }

  // Store elapsed time. We don't want to include the time taken by computing errors and doing I/O.
  const double elapsed_time = timer.elapsed();

  // Get the estimated trajectory and landmark positions generated by MCMC SLAM.
  const std::map<size_t, position> estimated_map = mcmc.get_feature_estimates();
  const bitree<pose>& estimated_trajectory = mcmc.get_action_estimates();

  // Store the expected trajectory.
  std::ofstream expected_file ("data/expected_trajectory.txt");
  print_trajectory (expected_file, initial_position, expected_trajectory);
  expected_file.close();

  // Store the actual trajectory.
  std::ofstream actual_file ("data/actual_trajectory.txt");
  print_trajectory (actual_file, initial_position, actual_trajectory);
  actual_file.close();

  // Store the estimated trajectory.
  std::ofstream estimated_file ("data/estimated_trajectory.txt");
  print_trajectory (estimated_file, initial_position, estimated_trajectory);
  estimated_file.close();

  // Store the actual landmark positions.
  std::ofstream map_file ("data/landmarks.txt");
  print_landmarks (map_file, pose(), landmarks);
  map_file.close();

  // Store the estimated landmark positions.
  std::ofstream estimated_map_file ("data/estimated_map.txt");
  print_landmarks (estimated_map_file, initial_position, estimated_map);
  estimated_map_file.close();

  // Summarise key information about this simulation.
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
