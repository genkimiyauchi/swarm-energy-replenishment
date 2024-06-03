/**
 * @file <utility/battery_discharge_model.h>
 *
 * @author Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 * 
 * Defines custom battery discharge models.
 */

#ifndef CUSTOM_BATTERY_DISCHARGE_MODEL_H
#define CUSTOM_BATTERY_DISCHARGE_MODEL_H

#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {

  /****************************************/
  /****************************************/

  /**
  * A battery discharge model in which the charge decreases with time, motion, and task performing activity.
  *
  * In this model, the charge is calculated as follows:
  *
  * new charge = old charge - time_delta - if_moving_or_rotating * pos_delta
  */
  class CBatteryDischargeModelFixedTimeMotion : public CBatteryDischargeModel {
    
  public:
    
    CBatteryDischargeModelFixedTimeMotion() :
        m_psAnchor(NULL),
        m_fDeltaTime(1e-5),
        m_fDeltaPos(1e-3) {};

    CBatteryDischargeModelFixedTimeMotion(Real f_delta_time, Real f_delta_pos) :
        m_psAnchor(NULL),
        m_fDeltaTime(f_delta_time),
        m_fDeltaPos(f_delta_pos) {};

    virtual void Init(TConfigurationNode& t_tree);
    
    /* Idle energy usage */
    void SetDeltaTime(Real f_delta) {
        m_fDeltaTime = f_delta;
    }
    
    /* Moving energy usage */
    void SetDeltaPos(Real f_delta) {
        m_fDeltaPos = f_delta;
    }
    
    virtual void SetBattery(CBatteryEquippedEntity* pc_battery);
    
    virtual void operator()();
    
  protected:
    
    const SAnchor* m_psAnchor;
    CVector3 m_cOldPosition;
    CQuaternion m_cOldOrientation;
    Real m_fDeltaTime;
    Real m_fDeltaPos;
  };

  /****************************************/
  /****************************************/

}

#endif