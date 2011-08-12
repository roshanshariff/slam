/*
 * multivariate.hpp
 *
 *  Created on: Jul 24, 2011
 *      Author: rshariff
 */

#ifndef MULTIVARIATE_HPP_
#define MULTIVARIATE_HPP_

#include <cmath>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "utility/random.hpp"


template <int N>
class multivariate_normal_dist {

public:

	static const int DIM = N;

	typedef Eigen::Matrix<double, N, 1> vector_type;
	typedef Eigen::Matrix<double, N, N> matrix_type;

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
		for (int i=0; i<result.size(); ++i) result[i] = random.normal();
		result = sqrt_cov().triangularView<Eigen::Lower>() * result;
		result += mean();
		return result;
	}

private:

	template <class V>
	double likelihood_exp (const Eigen::MatrixBase<V>& x) {
		vector_type v = x - mean();
		sqrt_cov().triangularView<Eigen::Lower>().solveInPlace(v);
		return -0.5 * v.transpose() * v;
	}

public:

	template <class V>
	double likelihood (const Eigen::MatrixBase<V>& x) {
		const double C = std::pow(boost::math::constants::root_two_pi<double>(), N)
							* sqrt_cov().triangularView<Eigen::Lower>().determinant();
		return std::exp(likelihood_exp(x)) / C;
	}

	template <class V>
	double log_likelihood (const Eigen::MatrixBase<V>& x) {
		const double C = 0.5*N*std::log(2*boost::math::constants::pi<double>())
							+ sqrt_cov().diagonal().array().log().sum();
		return likelihood_exp(x) - C;
	}

};


#endif /* MULTIVARIATE_HPP_ */
