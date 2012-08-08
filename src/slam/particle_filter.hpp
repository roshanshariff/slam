//
//  particle_filter.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-23.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_particle_filter_hpp
#define slam_particle_filter_hpp

#include <cassert>
#include <utility>
#include <algorithm>
#include <iterator>

#include <boost/container/vector.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

#include "utility/random.hpp"

template <class Particle>
class particle_filter {
    
    struct particle : public Particle {
        double weight;
        particle (double w = 1.0) : weight(w) { }
        particle (const Particle& p, double w = 1.0) : Particle(p), weight(w) { }
    };
    
    struct compare_particle_weight {
        bool operator() (const particle& a, const particle& b) const { return a.weight > b.weight; }
    };
    
    boost::container::vector<particle> particles;
    const particle* max_weight;

    double weight_sum;
    double squared_weight_sum;
    
public:

    particle_filter ()
    : particles(1), max_weight(&particles.front()), weight_sum(1.0), squared_weight_sum(1.0) { }

    size_t size () const { return particles.size(); }
    double effective_size () const {
        return squared_weight_sum > 0 ? weight_sum*weight_sum/squared_weight_sum : 0;
    }
    
    template <class Updater> void update (Updater);
    
    void resample (random_source& random, size_t new_size);
    void resample (random_source& random) { resample (random, size()); }
    
    template <class Initializer> void reinitialize (size_t new_size, Initializer);
    
    const Particle& operator[] (size_t index) const { return particles[index]; }
    Particle& operator[] (size_t index) { return particles[index]; }
    
    const Particle& max_weight_particle () const { return *max_weight; }
    
};

template <class Particle>
template <class Updater>
void particle_filter<Particle>::update (Updater f) {
    
    weight_sum = 0;
    squared_weight_sum = 0;
    max_weight = &particles.front();
    
    for (auto& particle : particles) {
        particle.weight *= f (static_cast<Particle&>(particle));
        if (!boost::math::isfinite(particle.weight) || particle.weight < 0) particle.weight = 0;
        weight_sum += particle.weight;
        squared_weight_sum += particle.weight * particle.weight;
        if (compare_particle_weight()(particle, *max_weight)) max_weight = &particle;
    }
}

template <class Particle>
void particle_filter<Particle>::resample (random_source& random, size_t new_size) {
    
    std::sort (particles.begin(), particles.end(), compare_particle_weight());

    boost::container::vector<particle> new_particles;
    new_particles.reserve (new_size);
    
    const double offset = random.uniform();
    double weight = 0;

    for (const auto& particle : particles) {
        weight += particle.weight;
        while (weight_sum * (offset + new_particles.size()) < weight * new_size) {
            new_particles.emplace_back (particle, 1.0);
        }
    }
    
    assert (new_particles.size() == new_size);
    particles.swap (new_particles);
    
    max_weight = &particles.front();
    weight_sum = squared_weight_sum = new_size;
}

template <class Particle>
template <class Initializer>
void particle_filter<Particle>::reinitialize (size_t new_size, Initializer init) {
    particles.clear();
    particles.reserve(new_size);
    std::generate_n (std::back_inserter (particles), new_size, init);
    max_weight = &particles.front();
    weight_sum = squared_weight_sum = new_size;
}

#endif
