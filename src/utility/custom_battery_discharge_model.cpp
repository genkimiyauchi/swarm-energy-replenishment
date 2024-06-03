/**
 * @file <utility/battery_discharge_model.cpp>
 *
 * @author Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 * 
 * Defines custom battery discharge models.
 */


#include "custom_battery_discharge_model.h"
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

    /****************************************/
    /****************************************/

    void CBatteryDischargeModelFixedTimeMotion::Init(TConfigurationNode& t_tree) {
        GetNodeAttributeOrDefault(t_tree, "delta_time", m_fDeltaTime, m_fDeltaTime);
        GetNodeAttributeOrDefault(t_tree, "delta_pos", m_fDeltaPos, m_fDeltaPos);
    }

    /****************************************/
    /****************************************/

    void CBatteryDischargeModelFixedTimeMotion::SetBattery(CBatteryEquippedEntity* pc_battery) {
        try {
            /* Execute default logic */
            CBatteryDischargeModel::SetBattery(pc_battery);
            /* Get a hold of the body and anchor of the entity that contains the battery */
            CEntity* pcRoot = &pc_battery->GetRootEntity();
            auto* cComp = dynamic_cast<CComposableEntity*>(pcRoot);
            if(cComp != nullptr) {
            auto& cBody = cComp->GetComponent<CEmbodiedEntity>("body");
            m_psAnchor = &cBody.GetOriginAnchor();
            m_cOldPosition = m_psAnchor->Position;
            }
            else {
            THROW_ARGOSEXCEPTION("Root entity is not composable");
            }
        }
        catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"time_motion_work\"", ex);
        }
    }

    /****************************************/
    /****************************************/
        
    void CBatteryDischargeModelFixedTimeMotion::operator()() {
        if(m_pcBattery->GetAvailableCharge() > 0.0) {

            Real isMoving = 0;

            /* Calculate delta position */
            Real fDeltaPos = Distance(m_psAnchor->Position,
                                      m_cOldPosition);
            if(fDeltaPos > 0)
                isMoving = 1;
                        
            /* Calculate delta orientation */
            CQuaternion cDeltaOrient =
                m_cOldOrientation.Inverse() *
                m_psAnchor->Orientation;
            CRadians cDeltaAngle;
            CVector3 cDeltaAxis;
            cDeltaOrient.ToAngleAxis(cDeltaAngle, cDeltaAxis);
            if(cDeltaAngle.GetValue() > 0)
                isMoving = 1;

            // LOG << m_psAnchor->Body.GetParent().GetId() << " moved: " << fDeltaPos << ", rot: " << cDeltaAngle.GetValue() << std::endl;

            /* Calculate new level */
            m_pcBattery->SetAvailableCharge(
            Max<Real>(
                0.0,
                m_pcBattery->GetAvailableCharge() -
                m_fDeltaTime -
                isMoving * m_fDeltaPos
                ));

            /* Save position for next step */
            m_cOldPosition = m_psAnchor->Position;
            m_cOldOrientation = m_psAnchor->Orientation;
        }
    }

    /****************************************/
    /****************************************/

    REGISTER_BATTERY_DISCHARGE_MODEL(CBatteryDischargeModelFixedTimeMotion, "time_motion_work");

    /****************************************/
    /****************************************/

}

