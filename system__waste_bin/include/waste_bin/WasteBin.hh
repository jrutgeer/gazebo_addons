/*
 * Copyright (C) 2023 Rock Diamond
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GZ_SIM_SYSTEMS_WASTE_BIN_HH_
#define GZ_SIM_SYSTEMS_WASTE_BIN_HH_

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class WasteBinPrivate;

  /// \brief Plugin that deletes all models that touch the model to
  /// which this plugin is attached to.
  ///
  /// It requires that a contact sensor is placed in at least one link on the
  /// model to which this plugin is attached.
  ///
  /// Note: the wastebin model does not discriminate, so it should not make contact
  /// with the ground plane, as the ground plane would also be deleted.
  ///
  ///
  /// Optional parameter:
  ///
  /// - `<delay>` Required contact time in seconds before the contacting model is deleted.
  ///             In case of loss of all contacts, the timer is reset.
  ///             In case of multiple contacting models, the delay starts counting at
  ///             the first contact and all models that either make of made contact are
  ///             deleted if the delay lapses (i.e. it is not checked if some models have
  ///             lost contact in the mean time).
  ///
  ///
  class WasteBin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: WasteBin();

    /// \brief Destructor
    public: ~WasteBin() override = default;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<WasteBinPrivate> dataPtr;
  };
  }
}
}
}

#endif
