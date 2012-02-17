/*
 * cholesky.hpp
 *
 *  Created on: 2011-08-11
 *      Author: roshan
 */

#ifndef CHOLESKY_HPP_
#define CHOLESKY_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Jacobi>


/* NOTE: This function is a hack; in particular the first two parameters are modified even though
 * they are passed by const reference. See
 * 	http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing
 * for why this hack is required.
 *
 * Computes [a b]' := rot * [a b]' in-place, where a and b are column vectors and rot is a Jacobi
 * rotation, i.e. rot = |  cos(x)  sin(x) |
 * 	                    | -sin(x)  cos(x) |
 */
template <class VectorTypeA, class VectorTypeB, class ScalarType>
void apply_jacobi_rotation (
		const Eigen::MatrixBase<VectorTypeA>& a,
		const Eigen::MatrixBase<VectorTypeB>& b,
		const Eigen::JacobiRotation<ScalarType>& rot)
{
	Eigen::internal::apply_rotation_in_the_plane
    (const_cast<Eigen::MatrixBase<VectorTypeA>&>(a),
     const_cast<Eigen::MatrixBase<VectorTypeB>&>(b),
     rot);
}


/* See M. Seeger, "Low Rank Updates for the Cholesky Decomposition", 2008 at
 * 	http://lapmal.epfl.ch/papers/cholupdate.pdf
 * for more an explanation of the algorithm used here.
 */
template <int N>
void cholesky_update (Eigen::Matrix<double, N, N>& L, Eigen::Matrix<double, N, 1> v) {

	Eigen::JacobiRotation<double> rot;

	for (int i = 0; i < N; ++i) {
		rot.makeGivens(L(i,i), -v(i), &L(i,i)), v(i) = 0;
		if (i < N-1) apply_jacobi_rotation(L.col(i).tail(N-i-1), v.tail(N-i-1), rot);
	}
}


/* See M. Seeger, "Low Rank Updates for the Cholesky Decomposition", 2008 at
 * 	http://lapmal.epfl.ch/papers/cholupdate.pdf
 * for more an explanation of the algorithm used here.
 */
template <int N>
void cholesky_downdate (Eigen::Matrix<double, N, N>& L, Eigen::Matrix<double, N, 1> p) {

	L.template triangularView<Eigen::Lower>().solveInPlace(p);

	assert(p.squaredNorm() < 1); // otherwise the downdate would destroy positive definiteness.
	double rho = std::sqrt (1 - p.squaredNorm());

	Eigen::JacobiRotation<double> rot;
	Eigen::Matrix<double, N, 1> temp;
	temp.setZero();

	for (int i = N-1; i >= 0; --i) {
		rot.makeGivens(rho, p(i), &rho), p(i) = 0;
		apply_jacobi_rotation(temp, L.col(i), rot);
	}
}


template <int N>
void cholesky_update (Eigen::Matrix<double, N, N>& L, const Eigen::Matrix<double, N, 1>& v, double k) {
	if (k > 0) cholesky_update<N> (L, std::sqrt(k)*v);
	else if (k < 0) cholesky_downdate<N> (L, std::sqrt(-k)*v);
}


#endif /* CHOLESKY_HPP_ */
