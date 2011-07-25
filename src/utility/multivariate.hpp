/*
 * multivariate.hpp
 *
 *  Created on: Jul 24, 2011
 *      Author: rshariff
 */

#ifndef MULTIVARIATE_HPP_
#define MULTIVARIATE_HPP_

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "utility/random.hpp"


template <class Scalar, int N>
class multivariate_normal_dist {

public:

    typedef Eigen::Matrix<Scalar, N, 1> vector_type;
    typedef Eigen::Matrix<Scalar, N, N> matrix_type;

    typedef vector_type result_type;

private:

    vector_type _mean;
    matrix_type _sqrt_cov;

public:

    template<class V, class M>
    multivariate_normal_dist (
	    const Eigen::MatrixBase<V>& mean,
	    const Eigen::MatrixBase<M>& cov
    ) : _mean(mean), _sqrt_cov(cov.llt().matrixL()) { }

    const vector_type& mean () const { return _mean; }
    vector_type& mean () { return _mean; }

    const matrix_type& sqrt_cov () const { return _sqrt_cov; }
    matrix_type& sqrt_cov () const { return _sqrt_cov; }

    matrix_type cov () const { return sqrt_cov() * sqrt_cov().transpose(); }

    vector_type operator() (const random_source& random) const {
	vector_type result;
	for (int i=0; i<result.size(); ++i) result[i] = Scalar(random.normal());
	result = sqrt_cov() * result;
	result += mean();
	return result;
    }

    template <class V>
    double likelihood (const Eigen::MatrixBase<V>& x) {

    }

};


#endif /* MULTIVARIATE_HPP_ */
