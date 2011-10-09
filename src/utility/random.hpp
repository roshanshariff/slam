#ifndef _UTILITY_RANDOM_HPP
#define _UTILITY_RANDOM_HPP

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/normal_distribution.hpp>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>

class random_source {
    
public:
    
    typedef boost::random::mt19937 engine_type;
    typedef engine_type::result_type result_type;
    
private:
    
    engine_type engine;
    boost::random::uniform_01<> uniform_dist;
    boost::random::normal_distribution<> normal_dist;

public:
            
    random_source () { }    
    random_source (result_type seed) : engine(seed) { }
    
    result_type operator()() { return engine(); }
    void seed (result_type seed) { engine.seed(seed); uniform_dist.reset(); normal_dist.reset(); }
        
    static result_type min() { return engine_type::min(); }
    static result_type max() { return engine_type::max(); }

    bool operator== (const random_source& r) const { return *this == r; }
    
    double uniform () { return uniform_dist(*this); }
    double normal () { return normal_dist(*this); }
    
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

	multivariate_normal_base () { }

	multivariate_normal_base (const vector_type& v) : m_mean(v) { }

public:

	const Derived& derived() const { return static_cast<const Derived&>(*this); }
	Derived& derived() { return static_cast<Derived&>(*this); }

	const vector_type& mean () const { return m_mean; }
	vector_type& mean () { return m_mean; }

	matrix_type chol_cov () const { derived().chol_cov(); };

	void chol_cov_multiply (vector_type& v) const { derived().chol_cov_multiply(v); }

	void chol_cov_solve (vector_type& v) const { derived().chol_cov_solve(v); }

	double chol_cov_det () const { return derived().chol_cov_det(); }

	double chol_cov_log_det () const { return derived().chol_cov_log_det(); }

	static vector_type subtract (const vector_type& a, const vector_type& b) { return Derived::subtract(a, b); }

	vector_type operator() (random_source& random) const {
		vector_type result;
		for (int i=0; i<vector_dim; ++i) result(i) = random.normal();
		chol_cov_multiply(result);
		result += mean();
		return result;
	}

	double likelihood_exponent (vector_type x) const {
		x = subtract(x, mean());
		chol_cov_solve(x);
		return -0.5 * x.squaredNorm();
	}

	double likelihood (const vector_type& x) const {
		const double root_two_pi = boost::math::constants::root_two_pi<double>();
		return std::exp(likelihood_exponent(x)) * std::pow(root_two_pi, -vector_dim) / chol_cov_det();
	}

	double log_likelihood (const vector_type& x) const {
		const double log_root_two_pi = 0.5 * std::log(2*boost::math::constants::pi<double>());
		return likelihood_exponent(x) - vector_dim*log_root_two_pi - chol_cov_log_det();
	}

};


template <int N, class Derived>
class multivariate_normal_dense_base : public multivariate_normal_base<N, Derived> {

	typedef multivariate_normal_base<N, Derived> base_type;
	typedef typename base_type::vector_type vector_type;
	typedef typename base_type::matrix_type matrix_type;

	matrix_type m_chol_cov;

protected:

	multivariate_normal_dense_base () { }

	multivariate_normal_dense_base (const vector_type& mean, const matrix_type& cov)
	: base_type(mean), m_chol_cov(cov.llt().matrixL()) { }

public:

	const matrix_type& chol_cov () const { return m_chol_cov; }
	matrix_type& chol_cov () { return m_chol_cov; }

	void chol_cov_multiply (vector_type& v) const {
		v = m_chol_cov.template triangularView<Eigen::Lower>() * v;
	}

	void chol_cov_solve (vector_type& v) const {
		m_chol_cov.template triangularView<Eigen::Lower>().solveInPlace(v);
	}

	double chol_cov_det () const {
		return m_chol_cov.diagonal().array().product();
	}

	double chol_cov_log_det () const {
		return m_chol_cov.diagonal().array().log().sum();
	}

};


template <int N>
struct multivariate_normal_dist : public multivariate_normal_dense_base<N, multivariate_normal_dist<N> > {

	typedef multivariate_normal_dense_base<N, multivariate_normal_dist> base_type;
	typedef typename base_type::vector_type vector_type;
	typedef typename base_type::matrix_type matrix_type;

	multivariate_normal_dist () { }

	multivariate_normal_dist (const vector_type& mean, const matrix_type& cov)
	: base_type(mean, cov) { }

	static vector_type subtract (const vector_type& a, const vector_type& b) { return a - b; }

};


template <int N, class Derived>
struct independent_normal_base : public multivariate_normal_base<N, Derived> {

	typedef multivariate_normal_base<N, Derived> base_type;
	typedef typename base_type::vector_type vector_type;
	typedef typename base_type::matrix_type matrix_type;

private:

	vector_type m_stddev;

protected:

	independent_normal_base () { }

	independent_normal_base (const vector_type& mean, const vector_type& std_dev)
	: base_type(mean), m_stddev(std_dev) { }

public:

	matrix_type chol_cov () const { return m_stddev.asDiagonal(); }

	void chol_cov_multiply (vector_type& v) const { v.array() *= m_stddev.array(); }

	void chol_cov_solve (vector_type& v) const { v.array() /= m_stddev.array(); }

	double chol_cov_det () const { return m_stddev.array().prod(); }

	double chol_cov_log_det () const { return m_stddev.array().log().sum(); }

	const vector_type& chol_cov_diag () const { return m_stddev; }
	vector_type& chol_cov_diag () { return m_stddev; }

};


#endif //_UTILITY_RANDOM_HPP
