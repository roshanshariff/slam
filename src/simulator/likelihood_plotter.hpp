//
//  likelihood_plotter.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-01.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_likelihood_plotter_hpp
#define slam_likelihood_plotter_hpp

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#include "slam/slam_data.hpp"
#include "slam/slam_result.hpp"
#include "simulator/gnuplot_process.hpp"

template <class ControlModel, class ObservationModel>
class likelihood_plotter {
    
    typedef typename ControlModel::result_type state_type;
    typedef typename ObservationModel::result_type feature_type;
    
    typedef slam_data<ControlModel, ObservationModel> slam_data_type;
    typedef typename slam_data_type::timestep_t timestep_t;
    typedef typename slam_data_type::featureid_t featureid_t;
    
    typedef slam_result<state_type, feature_type> slam_result_type;
    
    boost::shared_ptr<const slam_data_type> data;

    boost::shared_ptr<const slam_result_type> mcmcslam_result;
    boost::circular_buffer<float> mcmcslam_likelihood;

    boost::shared_ptr<const slam_result_type> fastslam_result;
    boost::circular_buffer<float> fastslam_likelihood;
    
    gnuplot_process gnuplot;
    
public:
    
    likelihood_plotter
    (boost::shared_ptr<const slam_data_type> data,
     boost::shared_ptr<const slam_result_type> mcmcslam_result,
     boost::shared_ptr<const slam_result_type> fastslam_result)
    : data(data),
    mcmcslam_result(mcmcslam_result), mcmcslam_likelihood(200),
    fastslam_result(fastslam_result), fastslam_likelihood(200) { }

    void plot ();
    
};

template <class ControlModel, class ObservationModel>
void likelihood_plotter<ControlModel, ObservationModel>::plot () {
    
    if (mcmcslam_result) {
        
        int xcoord = 0;
        
        mcmcslam_likelihood.push_back (data->log_likelihood (mcmcslam_result));
        
        size_t blanks = mcmcslam_likelihood.capacity() - mcmcslam_likelihood.size();
        for (size_t i = 0; i < blanks; ++i) gnuplot << /*(xcoord++) <<*/ 0;
        
        for (size_t i = 0; i < mcmcslam_likelihood.size(); ++i) {
            gnuplot << /*(xcoord++) <<*/ mcmcslam_likelihood[i];
        }
        
        gnuplot.plot(1);
        gnuplot.puts("title 'MCMC-SLAM' with lines");
    }
    
    if (fastslam_result) {
        
        int xcoord = 0;
    
        fastslam_likelihood.push_back (data->log_likelihood (fastslam_result));

        size_t blanks = fastslam_likelihood.capacity() - fastslam_likelihood.size();
        for (size_t i = 0; i < blanks; ++i) gnuplot << /*(xcoord++) <<*/ 0;
        
        for (size_t i = 0; i < fastslam_likelihood.size(); ++i) {
            gnuplot << /*(xcoord++) <<*/ fastslam_likelihood[i];
        }
        
        gnuplot.plot(1);
        gnuplot.puts("title 'FastSLAM 2.0' with lines");
    }
    
    gnuplot.plot();
}

#endif
