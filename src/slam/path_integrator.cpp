/*
 * path_integrator.hpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#ifndef PATH_INTEGRATOR_HPP_
#define PATH_INTEGRATOR_HPP_

#include <boost/bind.hpp>

#include "slam/slam_data.hpp"
#include "utility/bitree.hpp"

template <class SlamData>
class path_integrator {

	typedef SlamData slam_data_type;

	/** The types of action and feature identifiers, respectively. Usually same as size_t. */
	typedef typename slam_data_type::timestep_t timestep_t;

	/** The type of probability distributions for action and observation edges, respectively. */
	typedef typename slam_data_type::control_model_type control_model_type;
	typedef typename slam_data_type::observation_model_type  observation_model_type;

	typedef typename control_model_type::result_type control_type;
	typedef typename observation_model_type::result_type observation_type;

	slam_data_type& data;
	bitree<control_type> state_estimates;

	path_integrator (const slam_data_type& data_)
	: data(data_),
	  m_control_conn(data.connect_control_listener(boost::bind(&add_control, this, _1, _2)))
	{ }

	void add_control (timestep_t timestep, const control_model_type& control) {
		// This action must immediately follow the previously added action.
		assert (timestep == state_estimates.size());
		// Use the mean of the distribution as the initial estimate.
		state_estimates.push_back (control.mean());
	}

};


#endif /* PATH_INTEGRATOR_HPP_ */
