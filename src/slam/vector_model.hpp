/*
 * vector_model.hpp
 *
 *  Created on: 2011-08-23
 *      Author: roshan
 */

#ifndef VECTOR_MODEL_HPP_
#define VECTOR_MODEL_HPP_

#include <functional>

#include <boost/program_options.hpp>

#include "utility/random.hpp"


template <class VectorModel>
class vector_model_adapter {
    
    VectorModel model;
    
public:
    
    using vector_model_type = VectorModel;
    using vector_type = typename vector_model_type::vector_type;
    using matrix_type = typename vector_model_type::matrix_type;
    using result_type = typename vector_model_type::associated_type;
    
    const static int vector_dim = vector_model_type::vector_dim;
    
    vector_model_adapter () { }
    
    vector_model_adapter (const vector_model_type& m) : model(m) { }
    
    static auto from_vector (const vector_type& v) -> result_type { return vector_model_type::from_vector(v); }
    static auto to_vector (const result_type& x) -> vector_type { return vector_model_type::to_vector(x); }
    static auto subtract (const vector_type& a, const vector_type& b) -> vector_type { return vector_model_type::subtract(a,b); }
    
    auto vector_model () const -> const vector_model_type& { return model; }
    auto vector_model () -> vector_model_type& { return model; }
    
    auto mean () const -> result_type { return from_vector(vector_model().mean()); }
    
    auto operator() (random_source& rand) const -> result_type { return from_vector(vector_model()(rand)); }
    
    auto likelihood (const result_type& x) const -> double { return vector_model().likelihood(to_vector(x)); }
    
    auto log_likelihood (const result_type& x) const -> double { return vector_model().log_likelihood(to_vector(x)); }
    
    struct builder : public std::unary_function<result_type, vector_model_adapter> {
        
        using vector_builder_type = typename vector_model_type::builder;        
        vector_builder_type vector_builder;
        
        builder (const vector_builder_type& vector_builder_) : vector_builder(vector_builder_) { }
        
        builder (const boost::program_options::variables_map& options) : vector_builder(options) { }
        
        static auto program_options () -> boost::program_options::options_description {
            return vector_builder_type::program_options();
        }
        
        vector_model_adapter operator() (const result_type& x) const {
            return vector_model_adapter (vector_builder (to_vector (x)));
        }
        
    };
    
};


#endif /* VECTOR_MODEL_HPP_ */
