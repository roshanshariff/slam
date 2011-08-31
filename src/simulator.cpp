#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <tr1/random>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "simulator.hpp"
#include "planar_robot/waypoint_controller.hpp"
#include "planar_robot/landmark_sensor.hpp"
#include "slam/mcmc_slam.hpp"
#include "utility/random.hpp"


template <class Simulator>
boost::program_options::variables_map parse_options (int argc, char* argv[]);


int main (int argc, char* argv[]) {

	typedef planar_robot::waypoint_controller controller_type;
	typedef planar_robot::landmark_sensor sensor_type;

	typedef simulator<controller_type, sensor_type> simulator_type;
	typedef simulator_type::slam_data_type slam_data_type;

	typedef mcmc_slam<slam_data_type> mcmc_slam_type;

	boost::program_options::variables_map options =
			parse_options<simulator_type> (argc, argv);

	controller_type controller = controller_type::parse_options (options);
	sensor_type sensor = sensor_type::parse_options (options);

	std::tr1::random_device sys_rand;

	random_source random;
	const unsigned int random_seed = init_random (random, "sim-seed", options, sys_rand);

	random_source mcmc_random;
	const unsigned int mcmc_random_seed = init_random (mcmc_random, "mcmc-seed", options, random.generator);

	simulator_type sim (controller, sensor, random);

	mcmc_slam_type mcmc (sim.data, mcmc_random);
	mcmc.parse_options(options);

	boost::filesystem::path output_directory = "./output";
	if (options.count("output-prefix")) output_directory = options["output-prefix"].as<std::string>();

	boost::filesystem::path mcmc_incremental_output = output_directory / "mcmc_slam" / "incremental";
	print_incremental_info<slam_data_type, mcmc_slam_type> incremental_info_printer (
			sim.data, mcmc, controller.initial_state(), mcmc_incremental_output
	);
	if (options.count("verbose")) {
		boost::filesystem::create_directories (mcmc_incremental_output);
		incremental_info_printer.connect();
	}

	sim();

	return EXIT_SUCCESS;

}


template <class Simulator>
boost::program_options::variables_map parse_options (int argc, char* argv[]) {

	namespace po = boost::program_options;

	po::options_description command_line_options ("Command Line Options");
	command_line_options.add_options()
		("help,h", "usage information")
		("controller-options", "controller options")
		("sensor-options", "sensor options")
		("mcmc-slam-options", "MCMC-SLAM options")
		("config-file,f", po::value<std::vector<std::string> >()->composing(), "configuration files");

	po::options_description general_options ("General Options");
	general_options.add_options()
		("output-prefix,o", po::value<std::string>(), "prefix for simulation output files")
		("verbose,v", "produce detailed simulation logs")
		("seed", po::value<unsigned int>(), "seed for simulation random number generator")
		("mcmc-seed", po::value<unsigned int>(), "seed for MCMC-SLAM random number generator");

	po::options_description controller_options = Simulator::controller_type::program_options();
	po::options_description sensor_options = Simulator::sensor_type::program_options();
	po::options_description mcmc_options = mcmc_slam<typename Simulator::slam_data_type>::program_options();

	po::options_description config_options;
	config_options.add(general_options).add(controller_options).add(sensor_options).add(mcmc_options);

	po::options_description all_options;
	all_options.add(command_line_options).add(config_options);

	po::variables_map values;
	po::store (po::parse_command_line (argc, argv, all_options), values);
	po::notify (values);

	if (values.count ("help")) {
		po::options_description help_options;
		help_options.add(command_line_options).add(general_options);
		std::cout << help_options << std::endl;
		std::exit(EXIT_SUCCESS);
	}
	else if (values.count("controller-options")) {
		std::cout << controller_options << std::endl;
		std::exit(EXIT_SUCCESS);
	}
	else if (values.count("sensor-options")) {
		std::cout << sensor_options << std::endl;
		std::exit(EXIT_SUCCESS);
	}
	else if (values.count("mcmc-slam-options")) {
		std::cout << mcmc_options << std::endl;
		std::exit(EXIT_SUCCESS);
	}

	if (values.count("config-file")) {
		std::vector<std::string> files = values["config-file"].as<std::vector<std::string> >();
		for (std::vector<std::string>::const_iterator i = files.begin(); i != files.end(); ++i) {
			std::ifstream in (i->c_str());
			if (!in) {
				std::cerr << "Could not read configuration file: " << *i << std::endl;
				std::exit(EXIT_FAILURE);
			}
			po::store (po::parse_config_file (in, config_options, true), values);
			po::notify (values);
		}
	}

	return values;
}
