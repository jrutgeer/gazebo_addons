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

#include "waste_bin/WasteBin.hh"

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>  

#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::WasteBinPrivate
{
  /// \brief The model this system is attached to.
  public: Model model{kNullEntity};

  /// \brief The name of the model this system is attached to.
  public: std::string modelName;

  /// \brief SdfEntityCreator, used to _delete_ entities.
  public: std::unique_ptr<SdfEntityCreator> creator{nullptr};

  /// \brief Collision entities of the model this plugin is attached to,
  /// and that have been designated as contact sensors. Models that are
  /// in contact with these entities will be removed.
  public: std::vector<Entity> wasteBinEntities;

  /// \brief Set of collision entities that make or made contact during the delay period.
  /// This set is cleared only if all contacts are lost before the delay lapses.
  public: std::unordered_set<Entity> contactingEntities;

  /// \brief Set of model entities to remove (i.e. the top level model entities
  ///  corresponding to the collision entities in 'contactingEntities').
  public: std::unordered_set<Entity> modelEntities;

  /// \brief std::chrono::duration type used throught this plugin.
  public: using DurationType = std::chrono::duration<double>;

  /// \brief Required continuous contact time before the contacting models are removed.
  public: DurationType delayDuration{0};

  /// \brief Time of first contact.
  public: DurationType firstContactTime{0};

  /// \brief Set to true if the Configure() step has completed successfully.
  public: bool configured{false};

  /// \brief Set to true if the collisions are initialized in the postUpdate() step.
  public: bool collisionsInitialized{false};

  /// \brief Skip contact checking during one iteration after removing entities.
  public: bool skipOne{false};
};


//////////////////////////////////////////////////
WasteBin::WasteBin()
: System(), dataPtr(std::make_unique<WasteBinPrivate>())
{
}


//////////////////////////////////////////////////
void WasteBin::Configure(const Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
                         EntityComponentManager & _ecm, EventManager &_eventManager)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "WasteBin plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);
  this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  // Read delay duration if specified in the SDF
  if (_sdf->HasElement("delay"))
  {
    this->dataPtr->delayDuration = WasteBinPrivate::DurationType(_sdf->Get<double>("delay"));
  }

  // Configure() was completed successfully
  this->dataPtr->configured = true;
}


//////////////////////////////////////////////////
void WasteBin::PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("WasteBin::PreUpdate");

  if (_info.paused) return;

  // Remove the models that were added to the set
  if (this->dataPtr->modelEntities.size() > 0)
  {
    for (auto model : this->dataPtr->modelEntities)
    {
      this->dataPtr->creator->RequestRemoveEntity(model);

      gzdbg << "Removal of model " << Model(model).Name(_ecm) << " requested at time "
            << std::chrono::duration_cast<WasteBinPrivate::DurationType>(_info.simTime).count()
            << "s." << std::endl;
    }

    // Reset entity and time variables
    this->dataPtr->contactingEntities.clear();
    this->dataPtr->modelEntities.clear();
    this->dataPtr->firstContactTime = WasteBinPrivate::DurationType::zero();

    // Skip contact checking during one iteration to allow for entity removal
    this->dataPtr->skipOne = true;
  }  
}


//////////////////////////////////////////////////
void WasteBin::PostUpdate(const UpdateInfo & _info,
                          const EntityComponentManager & _ecm)
{
  GZ_PROFILE("WasteBin::PostUpdate");

  if (_info.paused) return;
  if (!this->dataPtr->configured) return;

  // This conceptually rather belongs to the Configure() step, but in the Configure() step
  // the ContactSensorData components are not yet created.
  if (!this->dataPtr->collisionsInitialized)
  {
    // Create a list of collision entities of this model, that are specified in
    // the contact sensors configuration (in the SDF).
    // These collision entities are identified by their ContactSensorData component.
    auto modelLinks = _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), components::Link());
  
    for (const Entity linkEntity : modelLinks)
    {
      auto linkCollisions = _ecm.ChildrenByComponents(linkEntity, components::Collision());
      for (const Entity collisionEntity : linkCollisions)
      {
        if (_ecm.EntityHasComponentType(collisionEntity, components::ContactSensorData::typeId))
        {
          this->dataPtr->wasteBinEntities.push_back(collisionEntity);
        }
      }
      // Sort so that we can do binary search later on.
      std::sort(this->dataPtr->wasteBinEntities.begin(), this->dataPtr->wasteBinEntities.end());

    }

    if (0 == this->dataPtr->wasteBinEntities.size())
      gzerr << "No collision entities with contact sensor found for model "
            << this->dataPtr->modelName << std::endl;
    
    this->dataPtr->collisionsInitialized = true;
  }

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  bool touching{false};
  if(!this->dataPtr->skipOne)
  {
    // Iterate through the wastebin collision entities and check if there is a contact
    for (const Entity collisionEntity : this->dataPtr->wasteBinEntities)
    {
      auto * contacts = _ecm.Component<components::ContactSensorData>(collisionEntity);
      if (contacts->Data().contact().size() > 0)
      {
        // Insert the contacting entity into the contactingEntities set:
        // An std::unordered_set only holds unique values so entities making multiple
        // contacts will still be only one time in the set.
        for (const auto & contact : contacts->Data().contact())
        {
          if (std::binary_search(this->dataPtr->wasteBinEntities.begin(),
                                this->dataPtr->wasteBinEntities.end(),
                                contact.collision1().id()))
          {
            // If contact.collision1() is a waste bin entity, then contact.collision2()
            // is the contacting entity.
            this->dataPtr->contactingEntities.insert(contact.collision2().id());
          }
          else
          {
            // else contact.collision1() is the contacting entity.
            this->dataPtr->contactingEntities.insert(contact.collision1().id());
          }
        }
        touching = true;
      }
    }
  }
  else
  {
    // Above code block has been skipped so set to false.
    this->dataPtr->skipOne = false;
  }

  
  if (!touching) {
    if (WasteBinPrivate::DurationType::zero() != this->dataPtr->firstContactTime)
    {
      gzdbg << "Model " << this->dataPtr->modelName
            << " lost contact at "
            << std::chrono::duration_cast<WasteBinPrivate::DurationType>(_info.simTime).count()
            << "s." << std::endl;

      this->dataPtr->contactingEntities.clear();
      this->dataPtr->firstContactTime = WasteBinPrivate::DurationType::zero();
    }
    return;
  }

  // Save first contact time if it was not set yet
  if (WasteBinPrivate::DurationType::zero() == this->dataPtr->firstContactTime)
  {
    this->dataPtr->firstContactTime = std::chrono::duration_cast<WasteBinPrivate::DurationType>(
      _info.simTime);

    gzdbg << "Model " << this->dataPtr->modelName << " in contact at "
          << this->dataPtr->firstContactTime.count() << "s."
          << std::endl;
  }

  // Check if the delay has lapsed
  bool delayLapsed = (std::chrono::duration_cast<WasteBinPrivate::DurationType>(_info.simTime) -
    this->dataPtr->firstContactTime) > this->dataPtr->delayDuration;

  if (delayLapsed)
  {
    Entity topModel;
    // For each contacting entity, look up its top level model entity.
    for (const auto & entity : this->dataPtr->contactingEntities)
    {
      topModel = topLevelModel(entity, _ecm);
      if (kNullEntity != topModel)
      {
        // Insert the top level model entity into the modelEntities set.
        // An std::unordered_set only holds unique values, so even if multiple
        // contacting entities belong to the same model, the set will still
        // hold only one entry for the top level model.
        this->dataPtr->modelEntities.insert(topModel);
      }
    }

    // Entities added to 'modelEntities' as well as their child entities, are deleted
    // in the preUpdate() step as deleting here leads to a segmentation fault.
    // See corresponding issue: https://github.com/gazebosim/gz-sim/issues/2073
  }
}



GZ_ADD_PLUGIN(
  WasteBin,
  System,
  WasteBin::ISystemConfigure,
  WasteBin::ISystemPreUpdate,
  WasteBin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(WasteBin, "gz::sim::systems::WasteBin")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(WasteBin, "ignition::gazebo::systems::WasteBin")
