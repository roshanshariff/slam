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

#include <boost/container/vector.hpp>

#include "utility/random.hpp"

template <class T>
class particle_filter {
    
    struct particle {
        double weight;
        T data;
        particle () : weight(1.0) { }
        particle (const T& data) : weight(1.0), data(data) { }
        bool operator< (const particle& p) const { return weight < p.weight; }
    };
    
    boost::container::vector<particle> particles;
    const particle* max_weight;

    double weight_sum;
    double squared_weight_sum;
    
public:
    
    particle_filter ()
    : particles(1), max_weight(particles.begin()), weight_sum(1.0), squared_weight_sum(1.0) { }
    
    size_t size () const { return particles.size(); }
    double effective_size () const { return weight_sum * weight_sum / squared_weight_sum; }
    
    void resample (random_source& random, size_t new_size);
    void resample (random_source& random) { resample (random, size()); }
    
    template <class Updater> void update (Updater);
    
    const T& operator[] (size_t index) const { return particles[index].data; }
    T& operator[] (size_t index) { return particles[index].data; }
    
    const T& max_weight_particle () const { return max_weight->data; }
    
};

template <class Particle>
void particle_filter<Particle>::resample (random_source& random, size_t new_size) {
    
    std::sort (particles.rbegin(), particles.rend());

    boost::container::vector<particle> new_particles;
    new_particles.reserve (new_size);
    
    const double offset = random.uniform();
    double weight = 0;

    for (const auto& particle : particles) {
        weight += particle.weight;
        while (weight_sum * (offset + new_particles.size()) < weight * new_size) {
            new_particles.emplace_back (particle.data);
        }
    }
    
    assert (new_particles.size() == new_size)
    particles.swap (new_particles);
    
    max_weight = &particles.front();
    weight_sum = squared_weight_sum = new_size;
}

template <class Particle>
template <class Updater>
void particle_filter<Particle>::update (Updater f) {

    weight_sum = 0;
    squared_weight_sum = 0;
    max_weight = particles.begin();
    
    for (auto& particle : particles) {
        particle.weight *= f (particle.data);
        weight_sum += particle.weight;
        squared_weight_sum += particle.weight * particle.weight;
        if (particle.weight > max_weight->weight) max_weight = &particle;
    }
}


#endif
