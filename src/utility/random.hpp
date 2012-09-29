#ifndef _UTILITY_RANDOM_HPP
#define _UTILITY_RANDOM_HPP

#include <cmath>
#include <random>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>

class random_source {
    
public:
    
    using engine_type = std::mt19937;
    using result_type = engine_type::result_type;
    
private:
    
    engine_type engine;
    std::uniform_real_distribution<> uniform_dist;
    std::normal_distribution<> normal_dist;
    
public:
    
    random_source () { }
    random_source (result_type seed) : engine(seed) { }
    
    auto operator()() -> result_type { return engine(); }
    void seed (result_type seed) { engine.seed(seed); uniform_dist.reset(); normal_dist.reset(); }
    
    static auto min() -> result_type { return engine_type::min(); }
    static auto max() -> result_type { return engine_type::max(); }
    
    auto operator== (const random_source& r) const -> bool { return engine == r.engine; }
    
    double uniform () { return uniform_dist(*this); }
    double normal () { return normal_dist(*this); }
    
};


template <int N, class Derived>
struct multivariate_normal_base {
    
    const static int vector_dim = N;
    
    using vector_type = Eigen::Matrix<double, N, 1>;
    using matrix_type = Eigen::Matrix<double, N, N>;
    
private:
    
    vector_type m_mean;
    
protected:
    
    multivariate_normal_base () { }
    
    multivariate_normal_base (const vector_type& v) : m_mean(v) { }
    
public:
    
    auto derived () const -> const Derived& { return static_cast<const Derived&>(*this); }
    auto derived () -> Derived& { return static_cast<Derived&>(*this); }
    
    auto mean () const -> const vector_type& { return m_mean; }
    auto mean () -> vector_type& { return m_mean; }
    
    auto chol_cov () const -> matrix_type { derived().chol_cov(); };
    
    void chol_cov_multiply (vector_type& v) const { derived().chol_cov_multiply(v); }
    
    void chol_cov_solve (vector_type& v) const { derived().chol_cov_solve(v); }
    
    auto chol_cov_det () const -> double { return derived().chol_cov_det(); }
    
    auto chol_cov_log_det () const -> double { return derived().chol_cov_log_det(); }
    
    static auto subtract (const vector_type& a, const vector_type& b) -> vector_type { return Derived::subtract(a, b); }
    
    auto operator() (random_source& random) const -> vector_type {
        vector_type result;
        for (int i=0; i<vector_dim; ++i) result(i) = random.normal();
            chol_cov_multiply(result);
            result += mean();
            return result;
    }
    
private:
    
    auto likelihood_exponent (vector_type x) const -> double {
        x = subtract(x, mean());
        chol_cov_solve(x);
        return -0.5 * x.squaredNorm();
    }
    
public:
    
    auto likelihood (const vector_type& x) const -> double {
        const double root_two_pi = boost::math::constants::root_two_pi<double>();
        return std::exp(likelihood_exponent(x)) * std::pow(root_two_pi, -vector_dim) / chol_cov_det();
    }
    
    auto log_likelihood (const vector_type& x) const -> double {
        const double log_root_two_pi = 0.5 * std::log(2*boost::math::constants::pi<double>());
        return likelihood_exponent(x) - vector_dim*log_root_two_pi - chol_cov_log_det();
    }
    
};


template <int N, class Derived>
struct multivariate_normal_dense_base : public multivariate_normal_base<N, Derived> {
    
    using base_type = multivariate_normal_base<N, Derived>;
    using typename base_type::matrix_type;
    using typename base_type::vector_type;
    
private:
    
    matrix_type m_chol_cov;
    
protected:
    
    multivariate_normal_dense_base () { }
    
    multivariate_normal_dense_base (const vector_type& mean, const matrix_type& chol_cov)
    : base_type(mean), m_chol_cov(chol_cov) { }
    
public:
    
    auto chol_cov () const -> const matrix_type& { return m_chol_cov; }
    auto chol_cov () -> matrix_type& { return m_chol_cov; }
    
    void chol_cov_multiply (vector_type& v) const {
        v = m_chol_cov.template triangularView<Eigen::Lower>() * v;
    }
    
    void chol_cov_solve (vector_type& v) const {
        m_chol_cov.template triangularView<Eigen::Lower>().solveInPlace(v);
    }
    
    auto chol_cov_det () const -> double {
        return m_chol_cov.diagonal().array().product();
    }
    
    auto chol_cov_log_det () const -> double {
        return m_chol_cov.diagonal().array().log().sum();
    }
    
};


template <int N>
struct multivariate_normal_dist : public multivariate_normal_dense_base<N, multivariate_normal_dist<N> > {
    
    using base_type = multivariate_normal_dense_base<N, multivariate_normal_dist>;
    using typename base_type::matrix_type;
    using typename base_type::vector_type;
    
    multivariate_normal_dist () { }
    
    multivariate_normal_dist (const vector_type& mean, const matrix_type& chol_cov)
    : base_type(mean, chol_cov) { }
    
    static auto subtract (const vector_type& a, const vector_type& b) -> vector_type { return a - b; }
    
};


template <class Adapted>
struct multivariate_normal_adapter
: public multivariate_normal_dense_base<Adapted::vector_dim, multivariate_normal_adapter<Adapted> > {
    
    using base_type = multivariate_normal_dense_base<Adapted::vector_dim, multivariate_normal_adapter>;
    using typename base_type::matrix_type;
    using typename base_type::vector_type;
    
    using associated_type = Adapted;
    
    multivariate_normal_adapter () { }
    
    multivariate_normal_adapter (const vector_type& mean, const matrix_type& chol_cov) : base_type(mean, chol_cov) { }
    
    static auto subtract (const vector_type& a, const vector_type& b) -> vector_type {
        return associated_type::subtract (a, b);
    }
    
    static auto to_vector (const associated_type& value) -> vector_type {
        return value.to_vector();
    }
    
    static auto from_vector (const vector_type& v) -> associated_type {
        return associated_type::from_vector (v);
    }
    
};


template <int N, class Derived>
struct independent_normal_base : public multivariate_normal_base<N, Derived> {
    
    using base_type = multivariate_normal_base<N, Derived>;
    using typename base_type::matrix_type;
    using typename base_type::vector_type;
    
private:
    
    vector_type m_stddev;
    
protected:
    
    independent_normal_base () { }
    
    independent_normal_base (const vector_type& mean, const vector_type& std_dev)
    : base_type(mean), m_stddev(std_dev) { }
    
public:
    
    auto stddev () const -> const vector_type& { return m_stddev; }
    auto stddev () -> vector_type&  { return m_stddev; }
    
    auto chol_cov () const -> matrix_type { return m_stddev.asDiagonal(); }
    
    void chol_cov_multiply (vector_type& v) const { v.array() *= m_stddev.array(); }
    
    void chol_cov_solve (vector_type& v) const { v.array() /= m_stddev.array(); }
    
    auto chol_cov_det () const -> double { return m_stddev.array().prod(); }
    
    auto chol_cov_log_det () const -> double { return m_stddev.array().log().sum(); }
    
    auto chol_cov_diag () const -> const vector_type& { return m_stddev; }
    auto chol_cov_diag () -> vector_type& { return m_stddev; }
    
};


#endif //_UTILITY_RANDOM_HPP
