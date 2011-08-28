#include <map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdio>

#include <boost/program_options.hpp>

#include "simulator.hpp"
#include "planar_robot/waypoint_controller.hpp"
#include "planar_robot/landmark_sensor.hpp"
#include "slam/mcmc_slam.hpp"
#include "utility/random.hpp"


int main (int argc, char* argv[]) {

	typedef planar_robot::waypoint_controller controller_type;
	typedef planar_robot::landmark_sensor sensor_type;

	typedef simulator<controller_type, sensor_type> simulator_type;
	typedef simulator_type::slam_data_type slam_data_type;

	typedef mcmc_slam<slam_data_type> mcmc_slam_type;

	namespace po = boost::program_options;

	po::options_description general_options ("General Options");
	general_options.add_options()
    		("help,h", "usage information")
    		("controller-help", "controller options")
    		("sensor-help", "sensor options")
    		("config-file,f", po::value<std::string>(), "configuration filename")
    		("output-prefix,o", po::value<std::string>(), "prefix for simulation output files")
    		("verbose,v", "produce detailed simulation logs")
    		("seed,s", po::value<unsigned int>(), "seed for random number generator")
    		("mcmc-steps,m", po::value<unsigned int>()->default_value(1), "mcmc steps per update");

	po::options_description controller_options = controller_type::program_options();
	po::options_description sensor_options = sensor_type::program_options();

	po::options_description all_options;
	all_options.add(general_options).add(controller_options).add(sensor_options);

	po::variables_map values;
	po::store (po::parse_command_line (argc, argv, all_options), values);
	po::notify (values);

	if (values.count ("help")) {
		std::cout << general_options << std::endl;
		return 0;
	}
	else if (values.count("controller-help")) {
		std::cout << controller_options << std::endl;
		return 0;
	}
	else if (values.count("sensor-help")) {
		std::cout << sensor_options << std::endl;
	}

	controller_type controller = controller_type::parse_options (values);
	sensor_type sensor = sensor_type::parse_options (values);

	random_source random;
	random_source mcmc_random;

	simulator_type sim (controller, sensor);
	mcmc_slam_type mcmc (sim.data, mcmc_random, 10 /* TODO: MCMC steps */ );

	sim(random);

	return 0;

}
