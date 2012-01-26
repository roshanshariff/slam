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

    double weight_sum;
    double squared_weight_sum;
    
public:
    
    particle_filter () : particles(1), weight_sum(1.0), squared_weight_sum(1.0) { }
    
    size_t size () const { return particles.size(); }
    double effective_size () const { return weight_sum * weight_sum / squared_weight_sum; }
    
    void resample (random_source& random, size_t new_size);
    void resample (random_source& random) { resample (random, size()); }
    
    template <class Updater> void update (Updater);
    
    const T& operator[] (size_t index) const { return particles[index].data; }
    T& operator[] (size_t index) { return particles[index].data; }
    
    const T& max_weight_particle () const {
        return std::max_element (particles.begin(), particles.end())->data;
    }
    
};

template <class Particle>
void particle_filter<Particle>::resample (random_source& random, size_t new_size) {
    
    assert (!particles.empty());
    assert (weight_sum > 0);

    std::vector<particle> new_particles;
    new_particles.reserve (new_size);
    
    const double U = random.uniform();
    
    size_t i = 0;
    double t = particles.front().first;
    for (size_t j = 0; j < new_size; ++j) {
        while (t * new_size <= (j+U)*weight_sum && i < particles.size() - 1) {
            t += particles[++i].first;
        }
        new_particles.emplace_back (particles[i].second);
    }
    
    particles.swap (new_particles);
    weight_sum = squared_weight_sum = new_size;
}

template <class Particle>
template <class Updater>
void particle_filter<Particle>::update (Updater f) {

    weight_sum = 0;
    squared_weight_sum = 0;

    for (size_t i = 0; i < size(); ++i) {
        particles[i].weight *= f (particles[i].data);
        weight_sum += particles[i].weight;
        squared_weight_sum += particles[i].weight * particles[i].weight;
    }
}


#endif
