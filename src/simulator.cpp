#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <ctime>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "simulator.hpp"
#include "planar_robot/waypoint_controller.hpp"
#include "planar_robot/landmark_sensor.hpp"
#include "slam/mcmc_slam.hpp"
#include "utility/random.hpp"


template <class Simulator>
boost::program_options::variables_map parse_options (int argc, char* argv[]);

template <class RandGen>
random_source make_random_source (RandGen& rng, const char* key,
                                  const boost::program_options::variables_map& options,
                                  unsigned int* seed_ptr = 0);


int main (int argc, char* argv[]) {

    /* typedefs for common types */

	typedef planar_robot::waypoint_controller controller_type;
	typedef planar_robot::landmark_sensor sensor_type;

	typedef simulator<controller_type, sensor_type> simulator_type;
	typedef simulator_type::slam_data_type slam_data_type;

	typedef mcmc_slam<slam_data_type> mcmc_slam_type;

	/* parse program options and set up random number generators */
    
    boost::program_options::variables_map options = parse_options<simulator_type> (argc, argv);

	random_source random_generator;
    random_generator.seed(std::time(0));

	unsigned int random_seed;
	random_source random = make_random_source (random_generator, "seed", options, &random_seed);
    
    unsigned int sim_seed;
    random_source sim_random = make_random_source (random, "sim-seed", options, &sim_seed);

    unsigned int mcmc_slam_seed;
    random_source mcmc_slam_random = make_random_source (random, "mcmc-slam-seed", options, &mcmc_slam_seed);
    
    /* set up simulation objects */
    
	controller_type controller = controller_type::parse_options (options);
	sensor_type sensor = sensor_type::parse_options (options);
	
    simulator_type sim (controller, sensor, sim_random);

	mcmc_slam_type mcmc_slam = mcmc_slam_type::parse_options (sim.data, mcmc_slam_random, options);
    mcmc_slam.connect();
    
    /* set up simulation logging */
    
	boost::filesystem::path output_dir (options["output-dir"].as<std::string>());
    boost::filesystem::create_directories (output_dir);

	slam_logger<slam_data_type, mcmc_slam_type> mcmc_slam_logger
    (output_dir/"log"/"mcmc_slam", sim.data, mcmc_slam, controller.initial_state());
    
	if (options.count("log")) {
        mcmc_slam_logger.connect();
    }
    
    /* run simulation */
    
	sim();
    
    /* print output */
    
    slam_data_type::control_type initial_state = controller.initial_state();

    print_trajectory<slam_data_type> ((output_dir/"trajectory.txt").c_str(), sim, initial_state);
    print_map<slam_data_type> ((output_dir/"map.txt").c_str(), sim, initial_state);

    print_trajectory<slam_data_type> ((output_dir/"mcmc_slam.trajectory.txt").c_str(), mcmc_slam, initial_state);
    print_map<slam_data_type> ((output_dir/"mcmc_slam.map.txt").c_str(), mcmc_slam, initial_state);
    
	return EXIT_SUCCESS;

}


template <class Simulator>
boost::program_options::variables_map parse_options (int argc, char* argv[]) {

	namespace po = boost::program_options;

	po::options_description command_line_options ("Command Line Options");
	command_line_options.add_options()
		("help,h", "usage information")
		("controller-help", "controller options")
		("sensor-help", "sensor options")
		("mcmc-slam-help", "MCMC-SLAM options")
		("config-file,f", po::value<std::vector<std::string> >()->composing(), "configuration files");

	po::options_description general_options ("General Options");
	general_options.add_options()
		("output-dir,o", po::value<std::string>()->default_value("./output"),
         "directory for simulation output files")
        ("log", "produce detailed simulation logs")
        ("seed", po::value<unsigned int>(), "seed for global random number generator")
        ("sim-seed", po::value<unsigned int>(), "seed for control and sensor noise")
		("mcmc-slam-seed", po::value<unsigned int>(), "seed for MCMC-SLAM random number generator");

	po::options_description controller_options = Simulator::controller_type::program_options();
	po::options_description sensor_options = Simulator::sensor_type::program_options();
	po::options_description mcmc_options = mcmc_slam<typename Simulator::slam_data_type>::program_options();

	po::options_description config_options;
	config_options.add(general_options).add(controller_options).add(sensor_options).add(mcmc_options);

	po::options_description all_options;
	all_options.add(command_line_options).add(config_options);

	po::variables_map values;
	po::store (po::command_line_parser (argc, argv).options(all_options).allow_unregistered().run(), values);
	po::notify (values);
    
    po::options_description help_options;
    bool help_requested = false;

	if (values.count ("help")) {
		help_options.add(command_line_options).add(general_options);
        help_requested = true;
	}
    
	if (values.count("controller-help")) {
		help_options.add(controller_options);
        help_requested = true;
	}
    
	if (values.count("sensor-help")) {
		help_options.add(sensor_options);
        help_requested = true;
	}
    
	if (values.count("mcmc-slam-help")) {
		help_options.add(mcmc_options);
        help_requested = true;
	}
    
    if (help_requested) {
        std::cout << help_options << std::endl;
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


template <class RandGen>
random_source make_random_source (RandGen& rng, const char* key,
                                  const boost::program_options::variables_map& options,
                                  unsigned int* seed_ptr)
{
	unsigned int seed = rng();
	if (options.count(key)) seed = options[key].as<unsigned int>();
	if (seed_ptr) *seed_ptr = seed;
    
    return random_source (seed);
}


