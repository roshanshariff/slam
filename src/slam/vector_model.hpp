/*
 * vector_model.hpp
 *
 *  Created on: 2011-08-23
 *      Author: roshan
 */

#ifndef VECTOR_MODEL_HPP_
#define VECTOR_MODEL_HPP_

#include "utility/random.hpp"


template <class VectorModel>
class vector_model_adapter {

	VectorModel model;

public:

	typedef VectorModel vector_model_type;
	typedef typename vector_model_type::vector_type vector_type;
	typedef typename vector_model_type::associated_type result_type;

	static const int vector_dim = vector_model_type::vector_dim;

	vector_model_adapter (const vector_model_type& m) : model(m) { }

	static result_type from_vector (const vector_type& v) { return vector_model_type::from_vector(v); }
	static vector_type to_vector (const result_type& x) { return vector_model_type::to_vector(x); }

	const vector_model_type& vector_model () const { return model; }
	vector_model_type& vector_model () { return model; }

	result_type mean () const { return from_vector(vector_model().mean()); }

	result_type operator() (random_source& rand) const { return from_vector(vector_model()(rand)); }

	double likelihood (const result_type& x) const { return vector_model().likelihood(to_vector(x)); }

	double log_likelihood (const result_type& x) const { return vector_model().log_likelihood(to_vector(x)); }

};


#endif /* VECTOR_MODEL_HPP_ */
