#ifndef _UTILITY_RANDOM_HPP
#define _UTILITY_RANDOM_HPP

#include <boost/random.hpp>
#include <boost/math/constants/constants.hpp>


struct random_source {

  typedef boost::mt19937 engine_type;
  typedef boost::variate_generator<engine_type&, boost::uniform_01<double> > uniform_type;
  typedef boost::variate_generator<uniform_type&, boost::normal_distribution<double> > normal_type;

  engine_type generator;
  uniform_type uniform;
  normal_type normal;

  random_source ()
    : generator(), uniform(generator, boost::uniform_01<double>()),
      normal(uniform, boost::normal_distribution<double>()) { }

};


template <int N, class Derived>
struct multivariate_normal_base {

	const static int vector_dim = N;

	typedef Eigen::Matrix<double, N, 1> vector_type;
	typedef Eigen::Matrix<double, N, N> matrix_type;

	typedef vector_type result_type;

private:

	vector_type m_mean;

protected:

	multivariate_normal_base (const vector_type& v) : m_mean(v) { }

public:

	const Derived& derived() const { return static_cast<const Derived&>(*this); }
	Derived& derived() const { return static_cast<Derived&>(*this); }

	const vector_type& mean () const { return m_mean; }
	vector_type& mean () { return m_mean; }

	matrix_type chol_cov () const { derived().chol_cov(); };

	void chol_cov_multiply (vector_type& v) const { derived().chol_cov_multiply(v); }

	void chol_cov_solve (vector_type& v) const { derived().chol_cov_solve(v); }

	double chol_cov_det () const { derived().chol_cov_det(); }

	double chol_cov_log_det () const { derived().chol_cov_log_det(); }

	static vector_type subtract (const vector_type& a, const vector_type& b) { return Derived::subtract(a, b); }

	vector_type operator() (random_source& random) const {
		vector_type result;
		for (int i=0; i<vector_dim; ++i) result(i) = random.normal();
		chol_cov_multiply(result);
		result += mean();
		return result;
	}

	double likelihood_exponent (vector_type x) {
		x = subtract(x, mean());
		chol_cov_solve(x);
		return -0.5 * x.squaredNorm();
	}

	double likelihood (const vector_type& x) {
		const double root_two_pi = boost::math::constants::root_two_pi<double>();
		return std::exp(likelihood_exponent(x)) * std::pow(root_two_pi, -vector_dim) / chol_cov_det();
	}

	double log_likelihood (const vector_type& x) {
		const double log_root_two_pi = 0.5 * std::log(2*boost::math::constants::pi<double>());
		return likelihood_exponent(x) - vector_dim*log_root_two_pi - chol_cov_log_det();
	}

};


template <int N, class Derived>
class multivariate_normal_dense_base : public multivariate_normal_base<N, Derived> {

	matrix_type m_chol_cov;

protected:

	multivariate_normal_dense_base (const vector_type& mean, const matrix_type& cov)
	: multivariate_normal_base(mean), m_chol_cov(cov.llt().matrixL()) { }

public:

	const matrix_type& chol_cov () const { return m_chol_cov; }
	matrix_type& chol_cov () { return m_chol_cov; }

	void chol_cov_multiply (vector_type& v) const {
		v = m_chol_cov.triangularView<Eigen::Lower>() * v;
	}

	void chol_cov_solve (vector_type& v) const {
		m_chol_cov.triangularView<Eigen::Lower>().solveInPlace(v);
	}

	double chol_cov_det () const {
		return m_chol_cov.diagonal().array().product();
	}

	double chol_cov_log_det () const {
		return m_chol_cov.diagonal().array().log().sum();
	}

};


template <int N>
struct multivariate_normal_dist : public multivariate_normal_base<N, multivariate_normal_dist> {

	static vector_type subtract (const vector_type& a, const vector_type& b) const { return a - b; }

};


template <int N, class Derived>
class independent_normal_base : public multivariate_normal_base<N, Derived> {

	vector_type m_stddev;

protected:

	independent_normal_base (const vector_type& mean, const vector_type& std_dev)
	: multivariate_normal_base(mean), m_stddev(std_dev) { }

public:

	matrix_type chol_cov () const { return m_std_dev.asDiagonal(); }

	void chol_cov_multiply (vector_type& v) const { v *= m_std_dev.array(); }

	void chol_cov_solve (vector_type& v) const { v /= m_std_dev.array(); }

	double chol_cov_det () const { return m_std_dev.array().product(); }

	double chol_cov_log_det () const { return m_std_dev.array().log().sum(); }

	const vector_type& chol_cov_diag () const { return m_stddev; }
	vector_type& chol_cov_diag () { return m_stddev; }

};


#endif //_UTILITY_RANDOM_HPP
