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

#include "utility/cholesky.hpp"
#include "utility/random.hpp"

/* See "The unscented particle filter", van de Merwe, Doucet, de Freitas, et al,
 * (2001) for details
 * on the algorithm and the meanings of the parameters below. */

template <int N>
class unscented_params {

  double m_eta, m_weight_first, m_weight_rest;

public:
  double eta() const { return m_eta; }
  double weight_first() const { return m_weight_first; }
  double weight_rest() const { return m_weight_rest; }

  unscented_params(double alpha, double beta, double kappa) {
    const double lambda = alpha * alpha * (N + kappa) - N;
    m_eta = std::sqrt(N + lambda);
    m_weight_first = lambda / (N + lambda) + 1 - alpha * alpha + beta;
    m_weight_rest = 0.5 / (N + lambda);
  }
};

/* Uses the unscented transform to apply the function f to the given state and
 * produce the
 * new distribution "result". Optionally, if cross_cov is not null, its referee
 * is set to the
 * cross-covariance between the state and its transformation by f.
 */
template <class TransformFunc, int N, class StateDist, int M, class ResultDist,
          class NoiseMatrix>
void unscented_transform(const unscented_params<N>& params, TransformFunc f,
                         const multivariate_normal_base<N, StateDist>& state,
                         multivariate_normal_dense_base<M, ResultDist>& result,
                         const Eigen::MatrixBase<NoiseMatrix>& noise_chol_cov,
                         Eigen::Matrix<double, N, M>* cross_cov = 0) {

  const typename ResultDist::vector_type base = f(state.mean());
  Eigen::Matrix<double, M, 2 * N + M> sigmapts;

  const typename StateDist::matrix_type& state_chol_cov =
      state.derived().chol_cov();
  for (int i = 0; i < N; ++i) {
    const typename StateDist::vector_type state_innov =
        params.eta() * state_chol_cov.col(i);
    sigmapts.col(i) = ResultDist::subtract(base, f(state.mean() - state_innov));
    sigmapts.col(N + i) =
        ResultDist::subtract(base, f(state.mean() + state_innov));
  }

  const typename ResultDist::vector_type base_innov =
      params.weight_rest()
      * sigmapts.template leftCols<2 * N>().rowwise().sum();

  for (int i = 0; i < 2 * N; ++i)
    sigmapts.col(i) = ResultDist::subtract(base_innov, sigmapts.col(i));

  result.mean() = ResultDist::subtract(base, base_innov);

  sigmapts.template leftCols<2 * N>() *= std::sqrt(params.weight_rest());
  sigmapts.template rightCols<M>() = noise_chol_cov;

  result.chol_cov() = sigmapts.transpose()
                          .householderQr()
                          .matrixQR()
                          .template topLeftCorner<M, M>()
                          .template triangularView<Eigen::Upper>()
                          .transpose();

  cholesky_update<M>(result.chol_cov(), base_innov, params.weight_first());

  if (cross_cov)
    cross_cov->noalias() = std::sqrt(params.weight_rest()) * params.eta()
                           * state_chol_cov
                           * (sigmapts.template block<M, N>(0, N)
                              - sigmapts.template leftCols<N>())
                                 .transpose();
}

template <class TransformFunc, int N, class StateDist, int M, class ResultDist>
void unscented_transform(const unscented_params<N>& params, TransformFunc f,
                         const multivariate_normal_base<N, StateDist>& state,
                         multivariate_normal_dense_base<M, ResultDist>& result,
                         Eigen::Matrix<double, N, M>* cross_cov = nullptr) {
  unscented_transform(params, f, state, result, ResultDist::matrix_type::Zero(),
                      cross_cov);
}

/* Modifies the state to take into account measurements under h.
 * NOTE: the code below assumes that the observation has additive noise with
 * zero mean and
 * covariance equal to the covariance of the sensor noise (i.e. the covariance
 * part of the
 * observation parameter).
 */
template <int N, class StateDist, int M, class ObsDist, class ObsFunc>
void unscented_update(const unscented_params<N>& params, ObsFunc h,
                      multivariate_normal_dense_base<N, StateDist>& state,
                      const multivariate_normal_base<M, ObsDist>& obs) {

  multivariate_normal_adapter<ObsDist> predicted;
  Eigen::Matrix<double, N, M> kalman_gain;

  unscented_transform(params, h, state, predicted, obs.derived().chol_cov(),
                      &kalman_gain);

  const typename ObsDist::matrix_type& p = predicted.chol_cov();
  kalman_gain = p.template triangularView<Eigen::Lower>()
                    .transpose()
                    .template solve<Eigen::OnTheRight>(kalman_gain);
  kalman_gain = p.template triangularView<Eigen::Lower>()
                    .template solve<Eigen::OnTheRight>(kalman_gain);

  state.mean().noalias() +=
      kalman_gain * ObsDist::subtract(obs.mean(), predicted.mean());

  kalman_gain *= p;

  for (int i = 0; i < M; ++i)
    cholesky_downdate<N>(state.chol_cov(), kalman_gain.col(i));
}

#endif /* UNSCENTED_HPP_ */
