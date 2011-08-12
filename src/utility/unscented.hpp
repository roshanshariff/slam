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

template <int N, class MinusFunc = std::minus>
class unscented_filter {

	typedef multivariate_normal_dist<N> dist_type;

	dist_type& state;

	DifferenceFunc difference;

	double alpha, kappa, beta;

public:

	unscented_filter (dist_type& _state) : state(_state) { }

	template <class PredictFunc>
	void predict (const PredictFunc f) {

		const double lambda = alpha*alpha*(N + kappa) - N;

		Eigen::Matrix<double, N, 2*N> sigmapts;
		sigmapts.leftCols<N>() = std::sqrt(N + lambda) * state.sqrt_cov();
		sigmapts.rightCols<N>() = -sigmapts.leftCols<N>();
		sigmapts += state.mean().rowwise().replicate<2*N>();

		const Eigen::Matrix<double, N, 1> base = f(state.mean());
		for (int i = 0; i < 2*N; ++i) sigmapts.col(i) = difference(base, f(sigmapts.col(i)));

		const Eigen::Matrix<double, N, 1> base_innov = sigmapts.rowwise().sum() / (2*(N + lambda));
		for (int i = 0; i < 2*N; ++i) sigmapts.col(i) = difference(base_innov, sigmapts.col(i));
		sigmapts /= std::sqrt(2*(N + lambda));

		state.mean() = difference(base, base_innov);
		state.sqrt_cov() = sigmapts.transpose().householderQr().matrixQR().topLeftCorner<N,N>().triangularView<Eigen::Upper>().transpose();

		// TODO Cholesky rank-one update
		// cholupdate(state.sqrt_cov(), std::sqrt(lambda/(L + lambda) + (1 - alpha*alpha + beta))*base_innov)

	}

};


struct unscented_params {
    double alpha, kappa, beta;
};

template <class Matrix, class Vector, class Function>
void unscented_transform (Vector& mean, Matrix& cov, Function f, const unscented_params& p) {

    const int n = mean.size();
    const double lambda = p.alpha * p.alpha * (n + p.kappa) - n;

    Matrix sqrtP = std::sqrt(n + lambda) * cov.llt().matrixU();

    std::vector<Vector> sigmapts (2*n+1);
    sigmapts[0] = f(mean);

    mean = lambda * sigmapts[0];

    for (int i = 1; i <= n; ++i) {
	sigmapts[2*i-1] = f(mean + sqrtP.col(i-1));
	sigmapts[2*i] = f(mean - sqrtP.col(i-1));
	mean += (sigmapts[2*i-1] + sigmapts[2*i]) / 2;
    }

    mean /= n + lambda;

    sigmapts[0] -= mean;
    cov = sigmapts[0] * sigmapts[0].transpose() * (lambda + (n + lambda)*(1 - p.alpha*p.alpha + p.beta));

    for (int i = 1; i <=2*n+1; ++i) {
	sigmapts[i] -= mean;
	cov += sigmapts[i] * sigmapts[i].transpose() / 2;
    }

    cov /= n + lambda;

}


#endif /* UNSCENTED_HPP_ */
