#include <map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdio>

#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/nondet_random.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include "simulator.hpp"
#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/odometry_model.hpp"
#include "planar_robot/velocity_model.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "planar_robot/circle_controller.hpp"
#include "planar_robot/waypoint_controller.hpp"
#include "utilities/random.hpp"

using planar_robot::pose;
using planar_robot::position;

const double PI = boost::math::constants::pi<double>();

// FORWARD DECLARATIONS

std::map<size_t, position> generate_grid_map (double x, double y, double width, double height, double pitch);
std::vector<position> generate_circle_waypoints (double x, double y, double r, double points);
std::map<size_t, position> read_landmarks_from_file (const char* filename);
std::vector<position> read_waypoints_from_file (const char* filename);
void print_state (FILE* out, pose p, const bitree<pose>& state);
void print_landmarks (FILE* out, const std::map<size_t, position>& landmarks);
double state_error (const bitree<pose>& a, const bitree<pose>& b);
double landmark_error (const std::map<size_t, position>& landmarks, const std::map<size_t, position>& estimates);
FILE* fopen (const std::string& filename, const char* mode);


// -------------------------------------------------------------------------------------------------

// ROBOT PARAMETERS
const double SPEED = 3; // meters/second; the speed of the robot.
const double STEERING_MAX = 30*PI/180; // radians/second; maximum steering
const double STEERING_RATE = 20*PI/180; // radians/second^2; maximum steering change rate

// MAP AND TRAJECTORY PARAMETERS.
const double GRID_PITCH = 25; // meters; the spacing between landmarks on the grid.
const double RADIUS = 200; // meters; the radius of the circle driven by the robot.
const double CENTRE_X = RADIUS; // meters; the centre of the circle.
const double CENTRE_Y = 0; // meters; the centre of the circle.

// SIMULATION PARAMETERS
const double DELTA_T = 0.25; // seconds; the interval between successive actions/observations.
const double REPETITIONS = 1.1; // the number of times to go around the circle

// MCMC PARAMETERS
const int MCMC_STEPS = 10; // number of MCMC iterations per step.
const double STATE_PARAMS = 3.0; // the number of parameters estimated by each action edge.
const double OBSERVATION_PARAMS = 2.0; // the number of parameters estimated by each observation edge.

// -------------------------------------------------------------------------------------------------

// FEATURE GENERATION
const char* LANDMARKS_FILE = "input/example_webmap.landmarks.txt";
const std::map<size_t, position> LANDMARKS = read_landmarks_from_file (LANDMARKS_FILE);
//const std::map<size_t, position> LANDMARKS = generate_grid_map (0, -2*RADIUS, 2*RADIUS, 4*RADIUS, GRID_PITCH);

// WAYPOINT GENERATION
const char* WAYPOINTS_FILE = "input/example_webmap.waypoints.txt";
const std::vector<position> WAYPOINTS = read_waypoints_from_file (WAYPOINTS_FILE);
//const std::vector<position> WAYPOINTS = generate_circle_waypoints (CENTRE_X, CENTRE_Y, RADIUS, 20);

// -------------------------------------------------------------------------------------------------
/*

// ODOMETRY MODEL
const planar_robot::odometry_model::builder STATE_MODEL_BUILDER
(0.05, 1.0*PI/180, 0.1, 0.0001*180/PI);

// CIRCLE CONTROLLER
planar_robot::circle_controller CONTROLLER
(CENTRE_X, CENTRE_Y, RADIUS, SPEED, REPETITIONS);

*/
// -------------------------------------------------------------------------------------------------

// VELOCITY MODEL
const planar_robot::velocity_model::builder STATE_MODEL_BUILDER
(0.1, 1.0, 1.0*PI/180, 0.05*PI/180, 0.1*PI/180, 0.005*PI/180);

// WAYPOINT CONTROLLER
planar_robot::waypoint_controller CONTROLLER
(pose(), WAYPOINTS, 1.0, REPETITIONS, SPEED, STEERING_MAX, STEERING_RATE);

// -------------------------------------------------------------------------------------------------

// OBSERVATION MODEL

// range and bearing sigma
const planar_robot::range_bearing_model::builder OBSERVATION_MODEL_BUILDER (0.1, 1.0*PI/180.0);

// predicate to filter out observations
bool OBSERVATION_PREDICATE (const position& p) { return p.range() < 30; }

// -------------------------------------------------------------------------------------------------

template <class S, class O, class Controller>
void simulate (const S& state_model_builder,
	       const O& observation_model_builder,
	       Controller& controller,
	       const typename simulator<S, O>::map_type& landmarks,
	       const typename simulator<S, O>::observation_predicate_type& observation_predicate,
	       const double dt, const unsigned int mcmc_steps,
	       const double state_dim, const double obs_dim,
	       const boost::program_options::variables_map& options)
{

  // The global random source. TODO: use two generators, one for the simulation itself and another for
  // the MCMC SLAM module.
  random_source random;
  const unsigned int seed = options.count("seed")
    ? options["seed"].as<unsigned int>()
    : boost::random_device()();
  random.generator.seed (seed);

  simulator<S, O> sim (landmarks, std::tr1::ref(controller), controller.initial_pose(),
		       state_model_builder, observation_model_builder, observation_predicate,
		       mcmc_steps, state_dim, obs_dim);
     
  boost::timer timer;

  // Run the simulation
  while (!controller.finished()) {

    sim (random, dt);

    if (options.count ("verbose") && options.count ("output-prefix")) {
      
      const std::string& output_prefix = options["output-prefix"].as<std::string>();

      // Store the estimated state.
      FILE* state_estimated_file = fopen (output_prefix + "state.estimated."
					       + boost::lexical_cast<std::string>(sim.get_num_steps())
					       + ".txt", "w");
      if (state_estimated_file != NULL) {
	print_state (state_estimated_file, sim.get_initial_state(), sim.get_estimated_state());
	std::fclose (state_estimated_file);
      }

      // Store the estimated landmark positions.
      FILE* landmarks_estimated_file = fopen (output_prefix + "landmarks.estimated."
					      + boost::lexical_cast<std::string>(sim.get_num_steps())
					      + ".txt", "w");
      if (landmarks_estimated_file != NULL) {
	print_landmarks (landmarks_estimated_file, sim.get_feature_estimates());
	std::fclose (landmarks_estimated_file);
      }

    }
  }
  // Simulation is done

  // Store elapsed time. We don't want to include the time taken by computing errors and doing I/O.
  const double elapsed_time = timer.elapsed();

  // Get the estimated landmark positions generated by MCMC SLAM.
  const typename simulator<S, O>::map_type estimated_landmarks = sim.get_feature_estimates();

  FILE* summary_file = NULL;

  if (options.count ("output-prefix")) {

    const std::string& output_prefix = options["output-prefix"].as<std::string>();

    summary_file = fopen (output_prefix + "summary.txt", "w");

    // Store the actual state.
    FILE* state_file = fopen (output_prefix + "state.txt", "w");
    if (state_file != NULL) {
      print_state (state_file, sim.get_initial_state(), sim.get_state());
      std::fclose (state_file);
    }

    // Store the estimated state.
    FILE* state_estimated_file = fopen (output_prefix + "state.estimated.txt", "w");
    if (state_estimated_file != NULL) {
      print_state (state_estimated_file, sim.get_initial_state(), sim.get_estimated_state());
      std::fclose (state_estimated_file);
    }

    // Store the expected state.
    FILE* state_expected_file = fopen (output_prefix + "state.expected.txt", "w");
    if (state_expected_file != NULL) {
      print_state (state_expected_file, sim.get_initial_state(), sim.get_expected_state());
      std::fclose (state_expected_file);
    }

    // Store the actual landmark positions.
    FILE* landmarks_file = fopen (output_prefix + "landmarks.txt", "w");
    if (landmarks_file != NULL) {
      print_landmarks (landmarks_file, sim.get_landmarks());
      std::fclose (landmarks_file);
    }

    // Store the estimated landmark positions.
    FILE* landmarks_estimated_file = fopen (output_prefix + "landmarks.estimated.txt", "w");
    if (landmarks_estimated_file != NULL) {
      print_landmarks (landmarks_estimated_file, estimated_landmarks);
      std::fclose (landmarks_estimated_file);
    }
  }
  else {
    summary_file = stdout;
  }

  // Summarise key information about this simulation.
  if (summary_file != NULL) {
    std::fprintf (summary_file, "Random seed: %u\n", seed);
    std::fprintf (summary_file, "MCMC iterations: %u\n", mcmc_steps);
    std::fprintf (summary_file, "Landmarks: %zu\n", landmarks.size());
    std::fprintf (summary_file, "Observations per step: %f\n", sim.get_observations_per_step());
    std::fprintf (summary_file, "Time steps: %lu\n", sim.get_num_steps());
    std::fprintf (summary_file, "Time delta: %f\n", dt);
    std::fprintf (summary_file, "Simulated time: %f\n", sim.get_simulation_time());
    std::fprintf (summary_file, "Elapsed time: %f\n", elapsed_time);
    std::fprintf (summary_file, "Landmark error: %f\n", landmark_error (landmarks, estimated_landmarks));
    std::fprintf (summary_file, "State error: %f\n", state_error (sim.get_state(), sim.get_estimated_state()));
    std::fclose (summary_file);
  }

}



/** Generates landmarks in an evenly spaced grid around the expected path of the robot. */
std::map<size_t, position> generate_grid_map (double x, double y, double width, double height, double pitch) {
  std::map<size_t, position> features;
  size_t feature_id = 0;
  for (double dx = 0; dx <= width; dx += pitch) {
    for (double dy = 0; dy <= height; dy += pitch) {
      features[feature_id++] = position (x+dx, y+dy);
    }
  }
  return features;
}


std::vector<position> generate_circle_waypoints (double x, double y, double r, double points) { 
  const double TWO_PI = 2 * boost::math::constants::pi<double>();
  const double dtheta = TWO_PI / points;
  std::vector<position> waypoints;
  for (double theta = 0.0; theta < TWO_PI; theta += dtheta) {
    waypoints.push_back (position (x+r*std::cos(theta), y+r*std::sin(theta)));
  }
  return waypoints;
}


std::map<size_t, position> read_landmarks_from_file (const char* filename) {
  std::ifstream file (filename);
  std::map<size_t, position> features;
  size_t feature_id = 0;
  double x, y;
  while (file >> x >> y) {
    features[feature_id++] = position (x, y);
  }
  return features;
}


std::vector<position> read_waypoints_from_file (const char* filename) {
  std::ifstream file (filename);
  std::vector<position> waypoints;
  double x, y;
  while (file >> x >> y) {
    waypoints.push_back (position (x, y));
  }
  return waypoints;
}


/** Prints the given trajectory, as offset by the given pose, to ostream& out, with the format
    time_step  x_position  y_position  bearing */
void print_state (FILE* out, pose p, const bitree<pose>& state) {
  const char* fmt = "%f\t%f\t%f\n";
  std::fprintf (out, "# X\tY\tBEARING\n");
  std::fprintf (out, fmt, p.x(), p.y(), p.bearing());
  for (size_t i = 0; i < state.size(); ++i) {
    p += state[i];
    std::fprintf (out, fmt, p.x(), p.y(), p.bearing());
  }
}


/** Prints the given set of landmarks, as offset by the given pose, to ostream& out, with the format
    feature_id  x_position  y_position */
void print_landmarks (FILE* out, const std::map<size_t, position>& landmarks) {
  const char* fmt = "%2$f\t%3$f\t%1$zu\n";
  std::fprintf (out, "# X\tY\tID\n");
  std::map<size_t, position>::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {
    std::fprintf (out, fmt, i->first, i->second.x(), i->second.y());
  }
}


/** Computes the cumulative root mean squared error between the two given trajectories. Ignores the
    bearing of the robot, but an incorrect bearing will drastically increase the error on future
    positions in the trajectory. */
double state_error (const bitree<pose>& a, const bitree<pose>& b) {
  size_t points = std::min (a.size(), b.size());
  double error = 0;
  pose a_pos, b_pos;
  for (size_t i = 0; i < points; ++i) {
    a_pos += a[i];
    b_pos += b[i];
    error += (-a_pos + b_pos).distance_squared();
  }
  return error/points;
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
  return error/num_landmarks;
}


FILE* fopen (const std::string& filename, const char* mode) {
  return std::fopen (filename.c_str(), mode);
}


int main (int argc, char* argv[]) {

  namespace po = boost::program_options;

  po::options_description options ("Allowed options");
  options.add_options()
    ("help,h", "usage information")
    ("output-prefix,o", po::value<std::string>(), "prefix for simulation output files")
    ("verbose,v", "produce detailed simulation logs")
    ("seed,s", po::value<unsigned int>(), "seed for random number generator");

  po::variables_map values;
  po::store (po::parse_command_line (argc, argv, options), values);
  po::notify (values);

  if (values.count ("help")) {
    std::cout << options << "\n";
    return 0;
  }

  simulate (STATE_MODEL_BUILDER, OBSERVATION_MODEL_BUILDER, CONTROLLER, LANDMARKS, OBSERVATION_PREDICATE,
	    DELTA_T, MCMC_STEPS, STATE_PARAMS, OBSERVATION_PARAMS, values);
}
