#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <ctime>

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "planar_robot/waypoint_controller.hpp"
#include "planar_robot/landmark_sensor.hpp"
#include "slam/mcmc_slam.hpp"
#include "slam/fastslam.hpp"
#include "simulator/simulator.hpp"
#include "simulator/slam_plotter.hpp"
#include "simulator/likelihood_plotter.hpp"
#include "utility/random.hpp"
#include "utility/utility.hpp"


struct sim_types {
    
    typedef planar_robot::waypoint_controller controller_type;
    typedef planar_robot::landmark_sensor sensor_type;
    
    typedef simulator<controller_type, sensor_type> simulator_type;
    
    typedef simulator_type::control_model_type control_model_type;
    typedef simulator_type::observation_model_type observation_model_type;
    
    typedef mcmc_slam<control_model_type, observation_model_type> mcmc_slam_type;
    typedef fastslam<control_model_type, observation_model_type> fastslam_type;
    
    typedef likelihood_plotter<control_model_type, observation_model_type> likelihood_plotter_type;
    
};


boost::program_options::variables_map parse_options (int argc, char* argv[]);


int main (int argc, char* argv[]) {

	/* parse program options */
    
    boost::program_options::variables_map options = parse_options (argc, argv);

    /* set up simulation objects */
    
    random_source random (remember_option(options, "seed", (unsigned int)std::time(0)));
    
    unsigned int sim_seed = random();
    boost::shared_ptr<sim_types::simulator_type> sim
    = boost::make_shared<sim_types::simulator_type> (boost::ref(options), sim_seed);
    
    unsigned int mcmc_slam_seed = random();
    boost::shared_ptr<sim_types::mcmc_slam_type> mcmc_slam;
    if (options.count ("mcmc-slam")) {
        mcmc_slam = boost::make_shared<sim_types::mcmc_slam_type> (boost::ref(options), mcmc_slam_seed);
        mcmc_slam->connect (sim->get_slam_data());
    }
    
    unsigned int fastslam_seed = random();
    boost::shared_ptr<sim_types::fastslam_type> fastslam;
    if (options.count ("fastslam")) {
        fastslam = boost::make_shared<sim_types::fastslam_type> (boost::ref(options), fastslam_seed);
        fastslam->connect (sim->get_slam_data());
    }
    
    boost::shared_ptr<slam_plotter> slam_plot;
    if (options.count ("slam-plot")) {

        slam_plot = boost::make_shared<slam_plotter> (boost::ref(options), sim->get_initial_state());

        sim_types::simulator_type::timestep_slot_type timestep_slot (&slam_plotter::plot, slam_plot.get(), _1);
        sim->connect_timestep_listener (timestep_slot.track (slam_plot));
        
        slam_plot->add_data_source (sim, true, "Trajectory", "Landmarks",
                                    "lc rgbcolor 'red' pt 6 ps 1.5",
                                    "lc rgbcolor 'black' lw 5",
                                    "size 10,20,50 filled lc rgbcolor 'black'");
        
        if (fastslam) {
            slam_plot->add_data_source (fastslam, false, "FastSLAM 2.0", "",
                                        "lc rgbcolor 'blue' pt 3 ps 1",
                                        "lc rgbcolor 'blue' lw 2",
                                        "size 10,20,50 filled lc rgbcolor 'blue'");
        }

        if (mcmc_slam) {
            slam_plot->add_data_source (mcmc_slam, false, "MCMC-SLAM", "",
                                        "lc rgbcolor 'green' pt 3 ps 1",
                                        "lc rgbcolor 'green' lw 2",
                                        "size 10,20,50 filled lc rgbcolor 'green'");
        }
    }
    
    boost::shared_ptr<sim_types::likelihood_plotter_type> likelihood_plot;
    if (options.count ("plot-likelihood")) {
        likelihood_plot = boost::make_shared<sim_types::likelihood_plotter_type> (sim->get_slam_data(), mcmc_slam, fastslam);
        sim_types::simulator_type::timestep_slot_type timestep_slot (&sim_types::likelihood_plotter_type::plot, likelihood_plot.get());
        sim->connect_timestep_listener (timestep_slot.track (likelihood_plot));
    }
    
    /* set up simulation logging */
    
	boost::filesystem::path output_dir (options["output-dir"].as<std::string>());
    boost::filesystem::create_directories (output_dir);

	(*sim)();
    
	return EXIT_SUCCESS;

}


boost::program_options::variables_map parse_options (int argc, char* argv[]) {
    
	namespace po = boost::program_options;

	po::options_description command_line_options ("Command Line Options");
	command_line_options.add_options()
    ("help,h", "usage information")
    ("simulator-help", "simulator options")
    ("controller-help", "controller options")
    ("sensor-help", "sensor options")
    ("mcmc-slam-help", "MCMC-SLAM options")
    ("fastslam-help", "FastSLAM 2.0 options")
    ("slam-plot-help", "SLAM plotting options")
    ("config-file,f", po::value<std::vector<std::string> >()->composing(), "configuration files");

	po::options_description general_options ("General Options");
	general_options.add_options()
    ("output-dir,o", po::value<std::string>()->default_value("./output"),
     "directory for simulation output files")
    ("log", "produce detailed simulation logs")
    ("mcmc-slam", "enable MCMC-SLAM")
    ("fastslam", "enable FastSLAM 2.0")
    ("slam-plot", "produce SLAM gnuplot output")
    ("plot-likelihood", "produce plots of log-likelihood")
    ("seed", po::value<unsigned int>(), "seed for global random number generator");

    po::options_description simulator_options = sim_types::simulator_type::program_options();
	po::options_description controller_options = sim_types::controller_type::program_options();
	po::options_description sensor_options = sim_types::sensor_type::program_options();
	po::options_description mcmc_slam_options = sim_types::mcmc_slam_type::program_options();
    po::options_description fastslam_options = sim_types::fastslam_type::program_options();
    po::options_description slam_plot_options = slam_plotter::program_options();

	po::options_description config_options;
	config_options
    .add(general_options).add(simulator_options).add(controller_options).add(sensor_options)
    .add(mcmc_slam_options).add(fastslam_options).add(slam_plot_options);

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
    
    if (values.count("simulator-help")) {
        help_options.add(simulator_options);
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
		help_options.add(mcmc_slam_options);
        help_requested = true;
	}
    
	if (values.count("fastslam-help")) {
		help_options.add(fastslam_options);
        help_requested = true;
	}
    
	if (values.count("slam-plot-help")) {
		help_options.add(slam_plot_options);
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