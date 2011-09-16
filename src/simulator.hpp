#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <string>
#include <cstdio>
#include <tr1/memory>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/signals2.hpp>

#include "slam/slam_data.hpp"
#include "slam/mcmc_slam.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"
#include "utility/file.hpp"


template <class Controller, class Sensor>
struct simulator {

	typedef Controller controller_type;
	typedef typename controller_type::model_type control_model_type;

	typedef Sensor sensor_type;
	typedef typename sensor_type::model_type observation_model_type;

	typedef slam_data<control_model_type, observation_model_type> slam_data_type;

    typedef typename slam_data_type::featureid_t featureid_t;
    typedef typename slam_data_type::timestep_t timestep_t;
    typedef typename slam_data_type::control_type control_type;
    typedef typename slam_data_type::observation_type observation_type;
    
	controller_type& controller;
	sensor_type& sensor;
	random_source& random;

	slam_data_type data;

	bitree<control_type> m_trajectory;

	simulator (controller_type& controller_, sensor_type& sensor_, random_source& random_)
	: controller(controller_), sensor(sensor_), random(random_) { }

	void operator() ();

	control_type current_state () const { return controller.initial_state() + m_trajectory.accumulate(); }
    
    const bitree<control_type>& trajectory () const { return m_trajectory; }
    
    template <class FeatureFunctor>
    void for_each_feature (FeatureFunctor f) const;
    
private:
    
    template <class FeatureFunctor>
    class for_each_feature_helper {

        FeatureFunctor f;
        control_type initial_state;

    public:

        for_each_feature_helper (const FeatureFunctor& f_, const control_type& initial_state_)
        : f(f_), initial_state(initial_state_) { }

        void operator() (featureid_t feature_id, const observation_type& obs) const {
            f (feature_id, -initial_state + obs);
        }

    };
    
};


template <class Controller, class Sensor>
void simulator<Controller, Sensor>::operator() () {

	sensor.sense(current_state(), boost::bind(&slam_data_type::add_observation, &data, _1, _2), random);

	while (!controller.finished()) {

		control_model_type control = controller.control(current_state());
		m_trajectory.push_back(control(random));
		data.add_control(control);

		sensor.sense(current_state(), boost::bind(&slam_data_type::add_observation, &data, _1, _2), random);

		data.timestep();
	}
}


template <class Controller, class Sensor>
template <class FeatureFunctor>
void simulator<Controller, Sensor>::for_each_feature (FeatureFunctor f) const {
    sensor.for_each_feature (for_each_feature_helper<FeatureFunctor> (f, controller.initial_state()));
}


template <class SlamData, class SlamImpl>
class print_map_helper {

	typedef typename SlamData::control_type control_type;
	typedef typename SlamData::observation_type observation_type;
	typedef typename SlamData::featureid_t featureid_t;

    std::tr1::shared_ptr<FILE> output;
	control_type initial_state;

public:
    
    print_map_helper (std::tr1::shared_ptr<FILE> output_, const control_type& initial_state_)
    : output(output_), initial_state(initial_state_) { }

	void operator() (featureid_t feature_id, observation_type obs) const {
		typename observation_type::vector_type v = (initial_state + obs).to_vector();
		std::fprintf (output.get(), "%zu", feature_id);
		for (int i = 0; i < v.size(); ++i) std::fprintf (output.get(), "\t%f", v(i));
		std::fprintf (output.get(), "\n");
	}

};


template <class SlamData, class SlamImpl>
void print_map (const char* filename, const SlamImpl& slam,
                const typename SlamData::control_type& initial_state)
{
    slam.for_each_feature (print_map_helper<SlamData, SlamImpl> (open_file (filename, "w"), initial_state));
}


template <class SlamData, class SlamImpl>
void print_trajectory (const char* filename, const SlamImpl& slam,
                       const typename SlamData::control_type& initial_state)
{
    typedef typename SlamData::control_type control_type;
    std::tr1::shared_ptr<FILE> output = open_file (filename, "w");
    if (output) {
        const bitree<control_type>& trajectory = slam.trajectory();
        for (size_t i = 0; i <= trajectory.size(); ++i) {
            typename control_type::vector_type v = (initial_state + trajectory.accumulate(i)).to_vector();
            std::fprintf (output.get(), "%zu", i);
            for (int j = 0; j < v.size(); ++j) std::fprintf (output.get(), "\t%f", v(j));
            std::fprintf(output.get(), "\n");
        }
    }
}


template <class SlamData, class SlamImpl>
class slam_logger {

    typedef typename SlamData::timestep_t timestep_t;
	typedef typename SlamData::control_type control_type;

	boost::filesystem::path path;

	const SlamData& data;
	const SlamImpl& slam;

	control_type initial_state;

	boost::signals2::scoped_connection conn;

public:

	slam_logger (boost::filesystem::path path_, const SlamData& data_, const SlamImpl& slam_,
                 control_type initial_state_)
	: path(path_), data(data_), slam(slam_), initial_state(initial_state_) { }

	void print (timestep_t timestep) const {
		boost::filesystem::create_directories (path);
        std::string seq = boost::lexical_cast<std::string>(timestep);
		print_trajectory<SlamData> ((path/("trajectory."+seq+".txt")).c_str(), slam, initial_state);
		print_map<SlamData> ((path/("map."+seq+".txt")).c_str(), slam, initial_state);
	}

	void connect () {
        conn = data.connect_timestep_listener(boost::bind(&slam_logger::print, this, _1));
    }

	void disconnect () {
        conn.release();
    }

};

#endif //_SIMULATOR_HPP
