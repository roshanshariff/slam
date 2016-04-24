#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <cassert>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/sub_range.hpp>

#include "slam/interfaces.hpp"
#include "utility/flat_map.hpp"
#include "utility/listeners.hpp"
#include "utility/utility.hpp"

#include "main.hpp"

namespace slam {

/** This class stores a record of all state changes and observations,
 as probability distributions.  It is also responsible for notifying
 listeners when new state changes and observations are added.
 ActionModel and ObservationModel are the types of distributions over
 state changes and observations, respectively. */
template <class ControlModel, class ObservationModel>
class slam_data : public data_source {

public:
  using feature_observations =
      utility::flat_map<timestep_type, ObservationModel>;

private:
  using feature_collection = std::map<featureid_type, feature_observations>;

public:
  using feature_iterator = typename feature_collection::const_iterator;

  class observation_info {

    feature_iterator f;
    std::size_t i;

  public:
    observation_info(feature_iterator f, std::size_t i) : f(f), i(i) {}

    auto iterator() const -> feature_iterator { return f; }
    auto index() const -> std::size_t { return i; }

    auto id() const -> featureid_type { return f->first; }
    auto observation() const -> const ObservationModel& {
      return (f->second.begin() + i)->second;
    }
  };

  struct listener : public virtual timestep_listener {
    virtual void control(timestep_type, const ControlModel&) = 0;
    virtual void observation(timestep_type, const observation_info&) = 0;
  };

private:
  using observation_collection =
      utility::flat_multimap<timestep_type, observation_info>;
  using observation_range = boost::sub_range<const observation_collection>;

  feature_collection m_features;
  observation_collection m_observations;

  std::vector<ControlModel> m_controls;
  utility::listeners<timestep_listener> m_timestep_listeners;
  utility::listeners<listener> m_listeners;

public:
  slam_data() {}

  /** Noncopyable */
  slam_data(const slam_data&) = delete;
  slam_data& operator=(const slam_data&) = delete;

  /** Member functions from data_source */

  virtual timestep_type current_timestep() const override {
    return timestep_type(m_controls.size());
  }

  /** Member functions from timestep_listener. Forward signals to listeners */

  virtual void timestep(timestep_type timestep) override {
    assert(timestep == current_timestep());
    using namespace std::placeholders;
    m_timestep_listeners.for_each(std::bind(&listener::timestep, _1, timestep));
  }

  virtual void completed() override {
    using namespace std::placeholders;
    m_timestep_listeners.for_each(std::bind(&listener::completed, _1));
  }

  /** Retrieve controls. */

  const ControlModel& control(timestep_type timestep) const {
    return m_controls.at(timestep);
  }

  /** Retrieve observations. */

  observation_range observations() const { return m_observations; }

  observation_range observations_at(timestep_type t) const {
    return m_observations.equal_range(t);
  }

  /** Retrieve features. */

  bool feature_observed(featureid_type f) const {
    return m_features.find(f) != m_features.end();
  }

  feature_iterator get_feature_iterator(featureid_type featureid) const {
    return m_features.find(featureid);
  }

  const feature_observations& get_observations(featureid_type featureid) const {
    return m_features.at(featureid);
  }

  /** Add new data */

  void add_control(const ControlModel&);
  void add_observation(featureid_type, const ObservationModel&);

  void add_dataset(const dataset<ControlModel, ObservationModel>&,
                   const typename ControlModel::builder&,
                   const typename ObservationModel::builder&);

  /** Add new listener */

  void add_timestep_listener(const std::shared_ptr<timestep_listener>& l) {
    m_timestep_listeners.add(l);
  }

  void add_listener(const std::shared_ptr<listener>& l) {
    add_timestep_listener(l);
    m_listeners.add(l);
  }
};

} // namespace slam

template <class ControlModel, class ObservationModel>
void slam::slam_data<ControlModel, ObservationModel>::add_control(
    const ControlModel& control) {

  timestep_type t = current_timestep();
  m_controls.push_back(control);

  using namespace std::placeholders;
  m_listeners.for_each(
      std::bind(&listener::control, _1, t, std::cref(control)));
}

template <class ControlModel, class ObservationModel>
void slam::slam_data<ControlModel, ObservationModel>::add_observation(
    featureid_type id, const ObservationModel& obs) {

  const timestep_type t = current_timestep();

  auto feature_iter = m_features
                          .emplace(std::piecewise_construct,
                                   std::make_tuple(id), std::make_tuple())
                          .first;
  auto& feature_obs = feature_iter->second;

  assert(feature_obs.lower_bound(t) == feature_obs.end());
  auto obs_iter = feature_obs.emplace_hint(feature_obs.end(), t, obs);

  const std::size_t obs_index = obs_iter - feature_obs.begin();

  assert(m_observations.upper_bound(t) == m_observations.end());
  auto obs_info_iter = m_observations.emplace_hint(
      m_observations.end(), t, observation_info(feature_iter, obs_index));

  using namespace std::placeholders;
  m_listeners.for_each(std::bind(&listener::observation, _1, t,
                                 std::cref(obs_info_iter->second)));
}

template <class ControlModel, class ObservationModel>
void slam::slam_data<ControlModel, ObservationModel>::add_dataset(
    const dataset<ControlModel, ObservationModel>& data,
    const typename ControlModel::builder& control_model_builder,
    const typename ObservationModel::builder& obs_model_builder) {

  using namespace boost::adaptors;

  auto add_observations = [&](timestep_type t) {
    for (const auto& obs : values(data.observations_at(t))) {
      add_observation(obs.id, obs_model_builder(obs.observation));
    }
  };

  add_observations(current_timestep());
  timestep(current_timestep());

  while (current_timestep() < data.current_timestep()) {
    add_control(control_model_builder(data.control(current_timestep()),
                                      data.timedelta(current_timestep())));
    add_observations(current_timestep());
    timestep(current_timestep());
  }

  completed();
}

extern template class slam::slam_data<control_model_type,
                                      observation_model_type>;

#endif //_SLAM_SLAM_DATA_HPP
