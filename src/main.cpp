#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <random>
#include <utility>
#include <algorithm>
#include <cctype>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>

#include "planar_robot/rms_error.hpp"
#include "slam/interfaces.hpp"
#include "slam/slam_result_impl.hpp"
#include "slam/slam_initialiser.hpp"
#include "slam/mcmc_slam.hpp"
#include "slam/multi_mcmc.hpp"
#include "slam/g2o_slam.hpp"
#include "slam/g2o_clustering.hpp"
#include "slam/slam_likelihood.hpp"
#include "simulator/simulator.hpp"
#include "simulator/slam_plotter.hpp"
#include "simulator/time_series_plotter.hpp"
#include "utility/random.hpp"
#include "utility/utility.hpp"

#include "main.hpp"

using slam_result_type = slam::slam_result_of<control_model_type, observation_model_type>;
using slam_result_impl_type = slam::slam_result_of_impl<control_model_type, observation_model_type>;

using simulator_type = simulator<controller_type, sensor_type>;
using slam_initialiser_type = slam::slam_initialiser<control_model_type, observation_model_type>;
using mcmc_slam_type = slam::mcmc_slam<control_model_type, observation_model_type>;
using multi_mcmc_type = slam::multi_mcmc<control_model_type, observation_model_type>;
using g2o_slam_type = slam::g2o_slam<control_model_type, observation_model_type>;
using g2o_clustering_type = slam::g2o_clustering<control_model_type, observation_model_type>;

boost::program_options::variables_map parse_options (int argc, char* argv[]);


int main (int argc, char* argv[]) {
    
    /* parse program options */
    
    boost::program_options::variables_map options = parse_options (argc, argv);
    
    /* set up simulation objects */
    
    random_source random (remember_option(options, "seed", (unsigned int)std::random_device()()));
    
    unsigned int sim_seed = remember_option (options, "sim-seed", random());
    unsigned int init_seed = remember_option (options, "init-seed", random());
    unsigned int mcmc_slam_seed = remember_option (options, "mcmc-slam-seed", random());
    unsigned int multi_mcmc_seed = remember_option (options, "multi-mcmc-seed", random());
    
    auto sim = std::make_shared<simulator_type> (options, sim_seed);
    
    auto init = std::make_shared<slam_initialiser_type> (init_seed);
    sim->add_data_listener (init);
    
    std::shared_ptr<mcmc_slam_type> mcmc_slam;
    std::shared_ptr<mcmc_slam_type::updater> mcmc_slam_updater;
    if (options.count ("mcmc-slam")) {
        mcmc_slam = std::make_shared<mcmc_slam_type> (sim->get_slam_data(), mcmc_slam_seed);
        mcmc_slam->set_initialiser (init);
        mcmc_slam_updater = std::make_shared<mcmc_slam_type::updater>(mcmc_slam, options);
        sim->add_timestep_listener (mcmc_slam);
        sim->add_timestep_listener (mcmc_slam_updater);
    }
    
    std::shared_ptr<multi_mcmc_type> multi_mcmc;
    if (options.count ("multi-mcmc")) {
        multi_mcmc = std::make_shared<multi_mcmc_type> (sim->get_slam_data(),
                                                                   options, multi_mcmc_seed);
        sim->add_timestep_listener (multi_mcmc);
    }
    
    std::shared_ptr<g2o_slam_type> g2o_slam;
    std::shared_ptr<g2o_slam_type::updater> g2o_slam_updater;
    if (options.count ("g2o")) {
        g2o_slam = std::make_shared<g2o_slam_type> (init);
        g2o_slam_updater = std::make_shared<g2o_slam_type::updater>(g2o_slam, options);
        sim->add_data_listener (g2o_slam);
        sim->add_timestep_listener (g2o_slam_updater);
    }
    
    std::shared_ptr<g2o_clustering_type> g2o_clustering;
    g2o_clustering = std::make_shared<g2o_clustering_type>(sim->get_slam_data());
    sim->add_data_listener (g2o_clustering);
    
    std::shared_ptr<slam_plotter> slam_plot;
    if (options.count ("slam-plot")) {
        
        slam_plot = std::make_shared<slam_plotter> (options, sim->get_initial_state());
        
        slam_plot->add_data_source (sim, true, "Trajectory", "Landmarks",
                                    "lc rgbcolor 'black' pt 6 ps 1.5",
                                    "lc rgbcolor 'black' lw 5",
                                    "size 10,20,50 filled lc rgbcolor 'black'");
        
        if (mcmc_slam) {
            slam_plot->add_data_source (mcmc_slam, false, "MCMC-SLAM", "",
                                        "lc rgbcolor 'green' pt 3 ps 1",
                                        "lc rgbcolor 'green' lw 2",
                                        "size 10,20,50 filled lc rgbcolor 'green'");
        }
        
        if (g2o_slam) {
            slam_plot->add_data_source (g2o_slam, false, "G2O", "",
                                        "lc rgbcolor 'red' pt 3 ps 1",
                                        "lc rgbcolor 'red' lw 2",
                                        "size 10,20,50 filled lc rgbcolor 'red'");
        }
        
        sim->add_timestep_listener (slam_plot);
    }
    
    std::shared_ptr<time_series_plotter> likelihood_plot;
    if (options.count ("plot-stats")) {
        
        likelihood_plot = std::make_shared<time_series_plotter> (300);
        
        if (mcmc_slam) {

            likelihood_plot->add_data_source ([=](slam::timestep_type) {
                return slam::slam_log_likelihood (*sim->get_slam_data(), *mcmc_slam) - sim->get_log_likelihood();
            }, "MCMC-SLAM log likelihood", "lc rgbcolor 'green' lw 5");
        }
        
        sim->add_timestep_listener (likelihood_plot);
    }
    
    /* set up simulation logging */
    
    boost::filesystem::path output_dir (options["output-dir"].as<std::string>());
    boost::filesystem::create_directories (output_dir);
    
    {
        boost::timer::auto_cpu_timer timer (3, "CPU Time: %t seconds\n\n");
        (*sim)();
    }
    
    if (mcmc_slam) {
        std::cout
        << "MCMC-SLAM Trajectory RMSE: "
        << planar_robot::trajectory_rmse (sim->get_trajectory(), mcmc_slam->get_trajectory()) << '\n'
        << "MCMC-SLAM Map RMSE: "
        << planar_robot::map_rmse (sim->get_feature_map(), mcmc_slam->get_feature_map()) << '\n'
        << "MCMC-SLAM log likelihood ratio: "
        << mcmc_slam->get_log_likelihood() - sim->get_log_likelihood()
        << "\n\n";
    }
    
    if (multi_mcmc) {
        std::cout
        << "Multi-MCMC-SLAM Chains: "
        << multi_mcmc->num_chains() << '\n'
        << "Multi-MCMC-SLAM Trajectory RMSE: "
        << planar_robot::trajectory_rmse (sim->get_trajectory(), multi_mcmc->get_trajectory()) << '\n'
        << "Multi-MCMC-SLAM Map RMSE: "
        << planar_robot::map_rmse (sim->get_feature_map(), multi_mcmc->get_feature_map()) << '\n'
        << "Multi-MCMC-SLAM log likelihood ratio: "
        << multi_mcmc->get_log_likelihood() - sim->get_log_likelihood()
        << "\n\n";
        
        auto average = multi_mcmc->get_average();
        std::cout
        << "Multi-MCMC-SLAM Averaged Trajectory RMSE: "
        << planar_robot::trajectory_rmse (sim->get_trajectory(), average->get_trajectory()) << '\n'
        << "Multi-MCMC-SLAM Averaged Map RMSE: "
        << planar_robot::map_rmse (sim->get_feature_map(), average->get_feature_map()) << '\n'
        << "Multi-MCMC-SLAM log likelihood ratio: "
        << slam::slam_log_likelihood (*sim->get_slam_data(), *average) - sim->get_log_likelihood()
        << "\n\n";
        
    }
    
    if (g2o_slam) {
        std::cout
        << "G2O-SLAM Trajectory RMSE: "
        << planar_robot::trajectory_rmse (sim->get_trajectory(), g2o_slam->get_trajectory()) << '\n'
        << "G2O-SLAM Map RMSE: "
        << planar_robot::map_rmse (sim->get_feature_map(), g2o_slam->get_feature_map()) << '\n'
        << "G2O-SLAM log likelihood ratio: "
        << slam::slam_log_likelihood (*sim->get_slam_data(), *g2o_slam) - sim->get_log_likelihood()
        << "\n\n";
    }
    
//    if (mcmc_slam && g2o_clustering) {
//        for (int i = 0; i < 10; ++i) {
//            for (int j = 0; j < 1000; ++j) mcmc_slam->update();
//            std::cout << "Testing candidate cluster " << (i+1) << std::endl;
//            g2o_clustering->add (*mcmc_slam);
//        }
//        std::cout << "Number of clusters: "
//        << g2o_clustering->get_clusters().size()
//        << "\n\n";
//    }
    
    return EXIT_SUCCESS;
    
}


boost::program_options::variables_map parse_options (int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    
    po::options_description command_line_options ("Command Line Options");
    command_line_options.add_options()
    ("help,h", "usage information")
    ("config-file,f", po::value<std::vector<std::string> >()->composing(), "configuration files");
    
    po::options_description general_options ("General Options");
    general_options.add_options()
    ("output-dir,o", po::value<std::string>()->default_value("./output"),
     "directory for simulation output files")
    ("log", "produce detailed simulation logs")
    ("mcmc-slam", "enable MCMC-SLAM")
    ("multi-mcmc", "enable Multi-MCMC-SLAM")
    ("fastslam", "enable FastSLAM 2.0")
    ("g2o", "enable offline SLAM using G2O")
    ("mcmc-init", "initialise MCMC estimate from FastSLAM")
    ("slam-plot", "produce SLAM gnuplot output")
    ("plot-stats", "produce plots of various summary statistics")
    ("seed", po::value<unsigned int>(), "seed for global random number generator");
    
    const std::vector<std::pair<std::string, po::options_description>> module_options {
        { "Simulator", simulator_type::program_options() },
        { "Controller", controller_type::program_options() },
        { "Sensor", sensor_type::program_options() },
        { "MCMC-SLAM", mcmc_slam_type::program_options() },
        { "Multi-MCMC", multi_mcmc_type::program_options() },
        { "G2O-SLAM", g2o_slam_type::program_options() },
        { "SLAM plot", slam_plotter::program_options() }
    };
    
    auto module_option_desc = [](const std::string& name) -> std::pair<std::string, std::string> {
        std::string option_help (name);
        std::replace (option_help.begin(), option_help.end(), ' ', '-');
        std::transform (option_help.begin(), option_help.end(), option_help.begin(), (int (*)(int))std::tolower);
        return { option_help+"-help", name+" options" };
    };
    
    po::options_description config_options;
    config_options.add(general_options);
    
    for (const auto& module : module_options) {
        const auto& option_desc = module_option_desc (module.first);
        command_line_options.add_options()(option_desc.first.c_str(), option_desc.second.c_str());
        config_options.add (module.second);
    }
    
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
    
    for (const auto& module : module_options) {
        if (values.count (module_option_desc (module.first).first)) {
            help_options.add (module.second);
            help_requested = true;
        }
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
