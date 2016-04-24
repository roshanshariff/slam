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
#include "slam/hamiltonian.hpp"
#include "slam/slam_likelihood.hpp"
#include "simulator/simulator.hpp"
#include "simulator/slam_plotter.hpp"
#include "simulator/time_series_plotter.hpp"
#include "utility/random.hpp"
#include "utility/utility.hpp"

#include "main.hpp"
#include "dataset.hpp"

using slam_result_type = slam::slam_result_of<control_model_type, observation_model_type>;
using slam_result_impl_type = slam::slam_result_of_impl<control_model_type, observation_model_type>;

using simulator_type = simulator<control_model_type, observation_model_type, controller_type, sensor_type>;
using slam_data_type = slam::slam_data<control_model_type, observation_model_type>;
using slam_dataset_type = slam::dataset<control_model_type, observation_model_type>;
using slam_initialiser_type = slam::slam_initialiser<control_model_type, observation_model_type>;
using mcmc_slam_type = slam::mcmc_slam<control_model_type, observation_model_type>;
using multi_mcmc_type = slam::multi_mcmc<control_model_type, observation_model_type>;
using g2o_slam_type = slam::g2o_slam<control_model_type, observation_model_type>;
using g2o_clustering_type = slam::g2o_clustering<control_model_type, observation_model_type>;
using hmc_type = slam::reparam_hmc<control_model_type, observation_model_type>;


boost::program_options::variables_map parse_options (int argc, char* argv[]);


template <class T>
T remember_option (boost::program_options::variables_map& options, const std::string& key, T default_value) {
    if (options[key].empty()) {
        options.insert (std::make_pair (key, boost::program_options::variable_value (default_value, false)));
        return default_value;
    }
    else return options[key].as<T>();
}


/*
void learn_model (const slam_dataset_type& dataset, const slam_result_type& ground_truth,
                  const boost::program_options::variables_map& options);
*/


void dump_data (const slam_result_type& ground_truth, const slam_result_type& estimate,
                const std::string& prefix) {

  using planar_robot::pose;
  using planar_robot::position;

  pose origin = planar_robot::estimate_initial_pose(ground_truth.get_feature_map(),
                                                    estimate.get_feature_map());
  
  {
    std::ofstream traj_est (prefix+"traj_est.txt");
    for (slam::timestep_type t; t <= estimate.current_timestep(); ++t) {
      pose state = origin + estimate.get_state(t);
      traj_est << state.x() << ' ' << state.y() << '\n';
    }
  }

  {
    std::ofstream traj_gt (prefix+"traj_gt.txt");
    for (slam::timestep_type t; t <= ground_truth.current_timestep(); ++t) {
      pose state = ground_truth.get_state(t);
      traj_gt << state.x() << ' ' << state.y() << '\n';
    }
  }

  using namespace boost::adaptors;
  
  {
    std::ofstream map_est (prefix+"map_est.txt");
    const auto& feature_map = estimate.get_feature_map();
    for (const auto& feature : values(feature_map)) {
        position pos = origin + feature;
        map_est << pos.x() << ' ' << pos.y() << '\n';
    }
  }

  {
    std::ofstream map_gt (prefix+"map_gt.txt");
    const auto& feature_map = ground_truth.get_feature_map();
    for (const auto& feature : values(feature_map)) {
        position pos = feature;
        map_gt << pos.x() << ' ' << pos.y() << '\n';
    }
  }

}

int main (int argc, char* argv[]) {
    
    /* parse program options */
    
    boost::program_options::variables_map options = parse_options (argc, argv);
    
    /* set up simulation objects */
    
    random_source random (remember_option(options, "seed", (unsigned int)std::random_device()()));
    
    unsigned int sim_seed = remember_option (options, "sim-seed", (unsigned int)random());
    unsigned int init_seed = remember_option (options, "init-seed", (unsigned int)random());
    unsigned int mcmc_slam_seed = remember_option (options, "mcmc-slam-seed", (unsigned int)random());
    unsigned int hmc_seed = remember_option (options, "hmc-seed", (unsigned int)random());
    unsigned int fastslam_seed = remember_option (options, "fastslam-seed", (unsigned int)random());
    unsigned int multi_mcmc_seed = remember_option (options, "multi-mcmc-seed", (unsigned int)random());
    
    (void)fastslam_seed;
    
    const control_model_type::builder control_model_builder (options);
    const observation_model_type::builder observation_model_builder (options);
    
    auto data = std::make_shared<slam_data_type>();
    
    std::shared_ptr<slam_dataset_type> dataset;
    std::shared_ptr<slam_result_type> ground_truth;
    
    if (options.count ("dataset")) {
        std::tie(dataset, ground_truth) = read_range_only_data(options["dataset-dir"].as<std::string>(),
                                                               options["dataset"].as<std::string>());
    }
    else {
        controller_type controller (options);
        sensor_type sensor (options);
        auto simulator = std::make_shared<simulator_type> (options, sim_seed,
                                                           control_model_builder,
                                                           observation_model_builder,
                                                           controller, sensor);
        dataset = simulator;
        ground_truth = simulator;
    }
    assert (dataset);

    /*
    if (options.count("learn-model")) {
        learn_model (*dataset, *ground_truth, options);
        return 0;
    }
    */
    
    auto init = std::make_shared<slam_initialiser_type> (init_seed);
    data->add_listener (init);
    
    std::shared_ptr<mcmc_slam_type> mcmc_slam;
    std::shared_ptr<mcmc_slam_type::updater> mcmc_slam_updater;
    if (options.count ("mcmc-slam")) {
        mcmc_slam = std::make_shared<mcmc_slam_type> (data, mcmc_slam_seed);
        mcmc_slam->set_initialiser (init);
        mcmc_slam_updater = std::make_shared<mcmc_slam_type::updater>(mcmc_slam, options);
        data->add_timestep_listener (mcmc_slam);
        data->add_timestep_listener (mcmc_slam_updater);
    }
    
    std::shared_ptr<multi_mcmc_type> multi_mcmc;
    if (options.count ("multi-mcmc")) {
        multi_mcmc = std::make_shared<multi_mcmc_type> (data, options, multi_mcmc_seed);
        data->add_timestep_listener (multi_mcmc);
    }
    
    std::shared_ptr<g2o_slam_type> g2o_slam;
    std::shared_ptr<g2o_slam_type::updater> g2o_slam_updater;
    if (options.count ("g2o")) {
        g2o_slam = std::make_shared<g2o_slam_type> (init);
        g2o_slam_updater = std::make_shared<g2o_slam_type::updater>(g2o_slam, options);
        data->add_listener (g2o_slam);
        data->add_timestep_listener (g2o_slam_updater);
    }
    
    std::shared_ptr<g2o_clustering_type> g2o_clustering;
    if (options.count ("cluster")) {
        g2o_clustering = std::make_shared<g2o_clustering_type>(data, init);
        data->add_listener (g2o_clustering);
    }
    
    std::shared_ptr<time_series_plotter> likelihood_plot;
    if (options.count ("plot-stats")) {
        
        likelihood_plot = std::make_shared<time_series_plotter> (300);
        
        if (mcmc_slam) {

            likelihood_plot->add_data_source ([=](slam::timestep_type) {
                return slam::slam_log_likelihood (*data, *mcmc_slam) - slam::slam_log_likelihood (*data, *ground_truth);
            }, "MCMC-SLAM log likelihood", "lc rgbcolor 'green' lw 5");
        }
        
        data->add_timestep_listener (likelihood_plot);
    }
    
    /* set up simulation logging */
    
    boost::filesystem::path output_dir (options["output-dir"].as<std::string>());
    boost::filesystem::create_directories (output_dir);
    
    {
        boost::timer::auto_cpu_timer timer (3, "CPU Time: %t seconds\n\n");
        data->add_dataset (*dataset, control_model_builder, observation_model_builder);
    }
    
    std::shared_ptr<slam_plotter> slam_plot;
    if (options.count ("slam-plot")) {
        
        slam_plot = std::make_shared<slam_plotter> (options);
        
        slam_plot->set_ground_truth(ground_truth);
        slam_plot->add_data_source (ground_truth, true, "Trajectory", "Landmarks",
                                    "lc rgbcolor 'black' pt 6 ps 4",
                                    "lc rgbcolor 'gray' lw 5",
                                    "size 20,20,50 head filled lc rgbcolor 'gray'");
        
        if (mcmc_slam) {
            slam_plot->add_data_source (mcmc_slam, false, "MCMC-SLAM", "",
                                        "lc rgbcolor 'blue' pt 3 ps 3",
                                        "lc rgbcolor 'blue' lw 2",
                                        "size 20,20,50 head filled lc rgbcolor 'blue'");
        }
        
        if (g2o_slam) {
            slam_plot->add_data_source (g2o_slam, false, "G2O", "",
                                        "lc rgbcolor 'red' pt 3 ps 1",
                                        "lc rgbcolor 'red' lw 2",
                                        "size 10,20,50 filled lc rgbcolor 'red'");
        }
        
        data->add_timestep_listener (slam_plot);
    }
    
    const double dataset_log_likelihood = slam::slam_log_likelihood (*data, *ground_truth);
    
    if (mcmc_slam && g2o_slam) {
        g2o_slam->reinitialise (*mcmc_slam);
        g2o_slam_updater->completed();
        if (slam_plot) slam_plot->completed();
    }
    
    const auto print_rmse = [ground_truth](const slam_result_type& estimate, const char* name) {

        double map_rmse, traj_rmse;

        std::tie(map_rmse, traj_rmse) = planar_robot::map_traj_rmse_align_start (*ground_truth, estimate);
        std::cout << name << " Map RMSE: " << map_rmse << '\n' << name << " Trajectory RMSE: " << traj_rmse << '\n';

        std::tie(map_rmse, traj_rmse) = planar_robot::map_traj_rmse_align_best (*ground_truth, estimate);
        std::cout << name << " Aligned Map RMSE: " << map_rmse << '\n' << name << " Aligned Trajectory RMSE: " << traj_rmse << '\n';
    };

    if (mcmc_slam) {
        print_rmse (*mcmc_slam, "MCMC-SLAM");
        std::cout
        << "MCMC-SLAM log likelihood ratio: "
        << mcmc_slam->get_log_likelihood() - dataset_log_likelihood
        << "\n\n";
    }
    
    if (multi_mcmc) {
        
        std::cout
        << "Multi-MCMC-SLAM Chains: "
        << multi_mcmc->num_chains() << '\n';
        print_rmse (*mcmc_slam, "Multi-MCMC");
        std::cout
        << "Multi-MCMC-SLAM log likelihood ratio: "
        << multi_mcmc->get_log_likelihood() - dataset_log_likelihood
        << "\n\n";
        
        auto average = multi_mcmc->get_average();
        print_rmse (*mcmc_slam, "Averaged-Multi-MCMC");
        std::cout
        << "Averaged-Multi-MCMC log likelihood ratio: "
        << slam::slam_log_likelihood (*data, *average) - dataset_log_likelihood
        << "\n\n";        
    }
    
    if (g2o_slam) {
        print_rmse(*g2o_slam, "G2O-SLAM");
        std::cout
        << "G2O-SLAM log likelihood ratio: "
        << slam::slam_log_likelihood (*data, *g2o_slam) - dataset_log_likelihood
        << "\n\n";
    }
    
    if (mcmc_slam && g2o_clustering) {

        for (int i = 0; i < 100; ++i) {
            for (int j = 0; j < 10000; ++j) mcmc_slam->update();
            std::cout << "Testing candidate cluster " << (i+1) << std::endl;
            g2o_clustering->add (*mcmc_slam);
        }
        g2o_clustering->sort_clusters();
        std::cout << "Number of clusters: " << g2o_clustering->get_clusters().size() << "\n";
        
        std::vector<std::string> color_names = { "red", "green", "blue", "cyan", "magenta", "yellow" };
        
        std::vector<std::shared_ptr<slam_result_type>> top_clusters;
        for (const auto& cluster : g2o_clustering->get_clusters()) {
            if (top_clusters.size() >= color_names.size()) break;
            top_clusters.push_back (std::make_shared<slam_result_impl_type> (cluster.estimate));
            std::cout << top_clusters.size() << ": " << (cluster.log_likelihood - dataset_log_likelihood) << '\n';
        }
        
        auto cluster_plot = std::make_shared<slam_plotter>(options);
        cluster_plot->set_ground_truth(ground_truth);
        cluster_plot->add_data_source (ground_truth, true, "Trajectory", "Landmarks",
                                       "lc rgbcolor 'black' pt 6 ps 1.5",
                                       "lc rgbcolor 'black' lw 5",
                                       "size 10,20,50 filled lc rgbcolor 'black'");

        for (size_t i = 0; i < top_clusters.size(); ++i) {
            cluster_plot->add_data_source (top_clusters[i], false, "", "",
                                           "lc rgbcolor '"+color_names[i]+"' pt 3 ps 1",
                                           "lc rgbcolor '"+color_names[i]+"' lw 2",
                                           "size 10,20,50 filled lc rgbcolor '"+color_names[i]+"'");
        }

        cluster_plot->completed();
    }

    if (mcmc_slam && options.count("dump-data")) {
      const std::string prefix = options["dump-data"].as<std::string>();
      dump_data (*ground_truth, *mcmc_slam, prefix);
    }

    if (options.count("hmc")) {
        auto hmc = std::make_shared<hmc_type>(data, *init, hmc_seed);
        if (slam_plot) {
            slam_plot->add_data_source (hmc, false, "HMC", "",
                                        "lc rgbcolor 'red' pt 3 ps 3",
                                        "lc rgbcolor 'red' lw 2",
                                        "size 20,20,50 head filled lc rgbcolor 'red'");
        }
        if (slam_plot) slam_plot->completed();
        for (int i = 1; i < 10; ++i) {
            if (hmc->update(1e-10, 10)) {
                std::cout << "ACCEPTED\n";
            }
            else {
                std::cout << "REJECTED\n";
            }
            if (slam_plot) slam_plot->completed();
        }
    }
    
    return EXIT_SUCCESS;
    
}


boost::program_options::variables_map parse_options (int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    
    po::options_description command_line_options ("Command Line Options");
    command_line_options.add_options()
    ("help,h", "usage information")
    ("config-file,f", po::value<std::vector<std::string>>()->composing(), "configuration files");
    
    po::options_description general_options ("General Options");
    general_options.add_options()
    ("dataset", po::value<std::string>(), "name of dataset")
    ("dataset-dir", po::value<std::string>()->default_value("input"), "location of datasets")
    ("output-dir,o", po::value<std::string>()->default_value("output"),
     "directory for simulation output files")
    ("log", "produce detailed simulation logs")
    ("mcmc-slam", "enable MCMC-SLAM")
    ("multi-mcmc", "enable Multi-MCMC-SLAM")
    ("fastslam", "enable FastSLAM 2.0")
    ("g2o", "enable offline SLAM using G2O")
    ("hmc", "use Hamiltonian MCMC algorithm")
    ("cluster", "try to cluster MCMC-SLAM results")
    ("slam-plot", "produce SLAM gnuplot output")
    ("plot-stats", "produce plots of various summary statistics")
    ("learn-model", "learn observation and control models")
    ("learn-model-iterations", po::value<unsigned int>()->default_value(5),
     "number of iterations to use when learning control model")
    ("dump-data", po::value<std::string>()->default_value(""), "dump MCMC-SLAM results")
    ("seed", po::value<unsigned int>(), "seed for global random number generator");
    
    const std::vector<std::pair<std::string, po::options_description>> module_options {
        { "Simulator", simulator_type::program_options() },
        { "Controller", controller_type::program_options() },
        { "Sensor", sensor_type::program_options() },
        { "Control Model", control_model_type::builder::program_options() },
        { "Observation Model", observation_model_type::builder::program_options() },
        { "MCMC-SLAM", mcmc_slam_type::program_options() },
        { "Multi-MCMC", multi_mcmc_type::program_options() },
        { "G2O-SLAM", g2o_slam_type::program_options() },
            //{ "HMC", hmc_type::program_options() },
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

/*
void learn_model (const slam_dataset_type& dataset, const slam_result_type& ground_truth,
                  const boost::program_options::variables_map& options) {
    
    observation_model_type::builder observation_model_builder (options);
    //observation_model_builder = observation_model_type::learn_from_data (dataset, ground_truth);
    (void)ground_truth;
    
    slam_result_impl_type result;
    auto random_seed = (std::random_device())();

    control_model_type::builder control_model_builder (options);

    auto iterations = options["learn-model-iterations"].as<unsigned int>();
    while (iterations--) {

        auto data = std::make_shared<slam_data_type>();
        auto mcmc_slam = std::make_shared<mcmc_slam_type> (data, random_seed);
        auto mcmc_slam_updater = std::make_shared<mcmc_slam_type::updater>(mcmc_slam, options);
        data->add_timestep_listener (mcmc_slam);
        data->add_timestep_listener (mcmc_slam_updater);
        data->add_dataset (dataset, control_model_builder, observation_model_builder);
        
        control_model_builder = control_model_type::learn_from_data (dataset, *mcmc_slam);
    }
    
}
*/

