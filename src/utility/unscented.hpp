/*
 * unscented.hpp
 *
 *  Created on: Jul 12, 2011
 *      Author: rshariff
 */

#ifndef UNSCENTED_HPP_
#define UNSCENTED_HPP_

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Cholesky>

#include "utility/multivariate.hpp"

template <int N>
class unscented_filter {

public:

	typedef multivariate_normal_dist<N> dist_type;
	typedef typename multivariate_normal_dist<N>::vector_type state_vector_type;

private:

	dist_type& state;
	StateDeltaFunc state_delta;

	const double lambda, eta, weight_first, weight_rest;


	/* Uses the unscented transform to apply the function f to the current state and produce the
	 * new distribution "result". delta should be a function such that delta(a,b) is roughly a - b,
	 * taking into account the topology of the space. Optionally, if cross_cov is not null, its
	 * referee is set to the cross-covariance between the state and its transformation by f.
	 */
	template <int M, class TransformFunc, class DeltaFunc>
	void apply_transform (
			TransformFunc f, DeltaFunc delta, multivariate_normal_dist<M>& result,
			Eigen::Matrix<double, N, M>* cross_cov = 0
	) {

		typedef Eigen::Matrix<double, M, 1> vector_type; // the result type of f

		const vector_type base = f(state.mean());

		Eigen::Matrix<double, M, 2*N> sigmapts;
		for (int i = 0; i < N; ++i) {
			const state_vector_type state_innov = eta * state.sqrt_cov().col(i);
			sigmapts.col(i) = delta(base, f(state.mean() - state_innov));
			sigmapts.col(N+i) = delta(base, f(state.mean() + state_innov));
		}

		const vector_type base_innov = weight_rest * sigmapts.rowwise().sum();
		for (int i = 0; i < 2*N; ++i) sigmapts.col(i) = delta(base_innov, sigmapts.col(i));

		result.mean() = delta(base, base_innov);

		result.sqrt_cov() = sigmapts.transpose().householderQr().matrixQR()
				.topLeftCorner<M,M>().triangularView<Eigen::Upper>().transpose();
		result.sqrt_cov() *= std::sqrt(weight_rest);

		cholesky_update(result.sqrt_cov(), base_innov, weight_first);

		if (cross_cov) cross_cov->noalias() = weight_rest * eta * state.sqrt_cov()
				* (sigmapts.rightCols<N>().transpose() - sigmapts.leftCols<N>().transpose());
	}


public:


	unscented_filter (dist_type& _state, double alpha, double beta, double kappa)
	: state(_state), lambda(alpha*alpha*(N+kappa) - N), eta(std::sqrt(N+lambda)),
	  weight_first(lambda/(N+lambda) + 1 - alpha*alpha + beta), weight_rest(0.5 / (N+lambda))
	{ }


	/* Modifies the state to take into account a transformation by f; i.e. the prediction step of
	 * the Kalman filter.
	 */
	template <class PredictFunc, class DeltaFunc = std::minus<Eigen::Matrix<double, N, 1> > >
	void predict (PredictFunc f = PredictFunc(), DeltaFunc delta = DeltaFunc()) {
		apply_transform(f, delta, state);
	}


	/* Modifies the state to take into account measurements under h.
	 * NOTE: the code below assumes that the observation has additive noise with zero mean and
	 * covariance equal to the covariance of the sensor noise (i.e. the covariance part of the
	 * observation parameter).
	 */
	template <int M, class ObsFunc, class DeltaFunc = std::minus<Eigen::Matrix<double, M, 1> > >
	void update (
			ObsFunc h = ObsFunc(), DeltaFunc delta = DeltaFunc(),
			const multivariate_normal_dist<M>& observation
	) {

		multivariate_normal_dist<M> predicted;
		Eigen::Matrix<double, N, M> cross_cov;

		apply_transform(h, delta, predicted, &cross_cov)

		for (int i = 0; i < M; ++i) cholesky_update(predicted.sqrt_cov(), observation.sqrt_cov().col(i));

		Eigen::Matrix<double, N, M> kalman_gain =
				predicted.sqrt_cov().triangularView<Eigen::Lower>().transpose().jacobiSvd().solve(
						predicted.sqrt_cov().triangularView<Eigen::Lower>().jacobiSvd().solve(
								cross_cov.transpose()
						)
				).transpose();

		state.mean().noalias() += kalman_gain * obs_delta(observation.mean(), predicted.mean());

		kalman_gain.noalias() *= predicted.sqrt_cov();
		for (int i = 0; i < M; ++i) cholesky_downdate(state.sqrt_cov(), kalman_gain.col(i));

	}


};


#endif /* UNSCENTED_HPP_ */
