/*
 * vector_model.hpp
 *
 *  Created on: 2011-08-23
 *      Author: roshan
 */

#ifndef VECTOR_MODEL_HPP_
#define VECTOR_MODEL_HPP_

#include <functional>

#include "utility/random.hpp"


template <class VectorModel>
class vector_model_adapter {

	VectorModel model;

public:

	typedef VectorModel vector_model_type;
	typedef typename vector_model_type::vector_type vector_type;
	typedef typename vector_model_type::associated_type result_type;

	static const int vector_dim = vector_model_type::vector_dim;

	vector_model_adapter () { }

	vector_model_adapter (const vector_model_type& m) : model(m) { }

	static result_type from_vector (const vector_type& v) { return vector_model_type::from_vector(v); }
	static vector_type to_vector (const result_type& x) { return vector_model_type::to_vector(x); }
    static vector_type subtract (const vector_type& a, const vector_type& b) { return vector_model_type::subtract(a,b); }

	const vector_model_type& vector_model () const { return model; }
	vector_model_type& vector_model () { return model; }

	result_type mean () const { return from_vector(vector_model().mean()); }

	result_type operator() (random_source& rand) const { return from_vector(vector_model()(rand)); }

	double likelihood (const result_type& x) const { return vector_model().likelihood(to_vector(x)); }

	double log_likelihood (const result_type& x) const { return vector_model().log_likelihood(to_vector(x)); }

	struct builder : public std::unary_function<result_type, vector_model_adapter> {

		typedef typename vector_model_type::builder vector_builder_type;

		vector_builder_type vector_builder;
        
		builder (const vector_builder_type& vector_builder_) : vector_builder(vector_builder_) { }

		vector_model_adapter operator() (const result_type& x) const {
			return vector_model_adapter (vector_builder (to_vector (x)));
		}

	};

};


#endif /* VECTOR_MODEL_HPP_ */
