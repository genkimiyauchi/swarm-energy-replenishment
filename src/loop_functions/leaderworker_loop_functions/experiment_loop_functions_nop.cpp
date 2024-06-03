#include "experiment_loop_functions_nop.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/e-puck_charger/simulator/epuckcharger_entity.h>
#include <controllers/worker/worker.h>
#include <controllers/charger/charger.h>
#include <argos3/plugins/simulator/entities/circle_task_entity.h>
#include <argos3/plugins/simulator/entities/rectangle_task_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

#include <utility/robot_message.h>
#include <utility/custom_battery_discharge_model.h>

#include <filesystem>
#include <fstream>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <protos/generated/time_step.pb.h>

namespace fs = std::filesystem;

/****************************************/
/****************************************/

static const Real        EP_RADIUS        = 0.035f;
static const Real        EP_AREA          = ARGOS_PI * Square(0.035f);
static const Real        EP_RAB_RANGE     = 0.5333333333f; // 1.6 * (1/3)
static const std::string WO_CONTROLLER    = "worker";
static const std::string WOMC_CONTROLLER  = "worker_mc";
static const std::string CH_CONTROLLER    = "charger";
static const UInt32      MAX_PLACE_TRIALS = 100;
static const UInt32      MAX_ROBOT_TRIALS = 20;

static const std::string BINARY_FILENAME   = "log_data.pb";
static const std::string SUMMARY_FILENAME  = "summary.csv";
static const std::string COMMAND_FILENAME  = "commands.csv";

/****************************************/
/****************************************/
// Reference: https://www.geeksforgeeks.org/check-if-a-point-lies-on-or-inside-a-rectangle-set-2/
// function to find if given point
// lies inside a given rectangle or not.
bool PointIsInside(Real x1, Real y1, Real x2, Real y2, Real x, Real y)
{
    if (x >= x1 and x <= x2 and y >= y1 and y <= y2)
        return true;
 
    return false;
}

/****************************************/
/****************************************/

CExperimentLoopFunctionsNop::CExperimentLoopFunctionsNop() :
    m_pcFloor(NULL),
    m_pcRNG(NULL),
    m_bTaskExists(false),
    m_bTaskComplete(true),
    finishDelay(0) {
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::Init(TConfigurationNode& t_node) {
    
    LOG << "[LOG] Init experiment loop function" << std::endl;

    LOG << "[LOG] SEED: " << (int)CSimulator::GetInstance().GetRandomSeed() << std::endl;

    config = t_node;

    try {
        /*
        * Parse the configuration file
        */
        TConfigurationNode& tChainFormation = GetNode(config, "output");
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        /* Get the output file name from XML */
        GetNodeAttributeOrDefault(tChainFormation, "logging", m_bLogging, false);
        GetNodeAttributeOrDefault(tChainFormation, "out_path", m_strOutput, std::string("results/default/"));
        GetNodeAttributeOrDefault(tChainFormation, "run_number", m_strRunNumber, std::string(""));
        /* Set the frame grabbing settings */
        GetNodeAttributeOrDefault(tChainFormation, "frame_grabbing", m_bFrameGrabbing, false);
        GetNodeAttributeOrDefault(tChainFormation, "camera_index", m_unCameraIndex, (UInt32)0);

        TConfigurationNode& tDraw = GetNode(config, "draw");
        GetNodeAttributeOrDefault(tDraw, "robot_label", m_bDrawRobotLabel, true);

        // /* Get charging area settings */
        // if(NodeExists(config, "charging_area")) { // Check if charging area is defined
        //     TConfigurationNode& tChargingArea = GetNode(config, "charging_area");
        //     GetNodeAttributeOrDefault(tChargingArea, "position", m_cChargingPos, CVector2(0.0f, 0.0f));
        //     GetNodeAttributeOrDefault(tChargingArea, "radius", m_fChargingRadius, 0.5);
        // } else {
        //     m_cChargingPos = CVector2(0.0f, 0.0f);
        //     m_fChargingRadius = 0.0;
        // }

        /* Get battery discharge model settings */
        TConfigurationNode& tBattery = GetNode(config, "battery_model");
        GetNodeAttributeOrDefault(tBattery, "discharge_model", m_strBatteryDischargeModel, std::string("time_motion_work"));
        GetNodeAttributeOrDefault(tBattery, "full_charge_worker", m_fFullChargeWorker, 100.0);
        GetNodeAttributeOrDefault(tBattery, "full_charge_charger", m_fFullChargeCharger, 100.0);

        std::string strStartChargeWorker;
        GetNodeAttributeOrDefault(tBattery, "start_charge_worker", strStartChargeWorker, std::string("50.0,100.0"));
        // split strStartChargeWorker by ',' and store them to m_fStartChargeWorker as a pair
        std::stringstream ss(strStartChargeWorker);
        std::string str;
        std::vector<Real> vecStartCharge;
        while(std::getline(ss, str, ','))
            vecStartCharge.push_back(std::stof(str));
        m_fStartChargeWorker = std::make_pair(vecStartCharge[0], vecStartCharge[1]);

        std::string strStartChargeCharger;
        GetNodeAttributeOrDefault(tBattery, "start_charge_charger", strStartChargeCharger, std::string("50.0,100.0"));
        // split strStartChargeCharger by ',' and store them to m_fStartChargeCharger as a pair
        vecStartCharge.clear();
        str = "";
        ss = std::stringstream(strStartChargeCharger);
        while(std::getline(ss, str, ','))
            vecStartCharge.push_back(std::stof(str));
        m_fStartChargeCharger = std::make_pair(vecStartCharge[0], vecStartCharge[1]);

        LOG << "m_fStartChargeWorker: " << m_fStartChargeWorker.first << " " << m_fStartChargeWorker.second << std::endl;
        LOG << "m_fStartChargeCharger: " << m_fStartChargeCharger.first << " " << m_fStartChargeCharger.second << std::endl;

        GetNodeAttributeOrDefault(tBattery, "delta_time", m_fDeltaTime, 0.005);
        GetNodeAttributeOrDefault(tBattery, "delta_pos_worker", m_fDeltaPosWorker, 0.025);
        GetNodeAttributeOrDefault(tBattery, "delta_pos_charger", m_fDeltaPosCharger, 0.025);
        TConfigurationNode& tExtraBatteryInfo = GetNode(config, "extra_battery_info");
        GetNodeAttributeOrDefault(tExtraBatteryInfo, "delta_work", m_fDeltaWork, 0.055);
        GetNodeAttributeOrDefault(tExtraBatteryInfo, "delta_recharge", m_fDeltaRecharge, 100.0);
        GetNodeAttributeOrDefault(tExtraBatteryInfo, "delta_transfer_loss", m_fDeltaTransferLoss, 0.0);

        /* ############################# */
        m_fEnergyShared = 0;
        m_fEnergyLost = 0;
        m_bNoDemandTasks = true;
        /* Energy-aware swarm */
        m_cFixedChargePos = CVector2(-0.65, 0);
        m_fFixedChargeAreaSideX = 0.3;
        m_fFixedChargeAreaSideY = 1.6;
        
        m_cMobileChargePos = CVector2(0.35, 0);
        m_fMobileChargeAreaSideX = 0.3;
        m_fMobileChargeAreaSideY = 1.6;

        m_cTaskPos = CVector2(0.65, 0);
        m_fTaskAreaSideX = 0.3;
        m_fTaskAreaSideY = 1.6;
        CVector2 cCenter = CVector2(0.65, 0);
        
        InitRobots();
        InitTask();
        // InitChargers();
        if(m_bLogging) {
            InitLoggingEnergy();
        }
        /* ############################# */

        // InitRobots();
        // m_bNoDemandTasks = false;
        // InitTasks();

        if(m_bLogging) {
            /* Log arena information */
            m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
            // m_cOutput << "ARENA_RADIUS," << m_fArenaRadius << "\n";
            // m_cOutput << "DEPLOY_RADIUS," << m_fDeploymentRadius << "\n";
            m_cOutput << "DELTA_WORK," << m_fDeltaWork << "\n";
            // m_cOutput << "DELTA_POS_CHARGER," << m_fDeltaPosCharger << "\n";
            m_cOutput.close();
        }
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::Reset() {
    std::cout << "RESET called" << std::endl;
    
    /* Delete existing robot and task entities from the simulation */
    for(const auto& id : m_vecEntityID) {
        RemoveEntity(id);
    }

    m_vecEntityID.clear();

    InitRobots();
    // InitTasksCircular();
    InitTask();
    // AssignTasks();
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::Destroy() {
    int final_time = GetSpace().GetSimulationClock();
    LOG << "[LOG] Final Timestep: " << final_time << std::endl;
    
    // if(m_bLogging) {
    //     m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
    //     m_cOutput << "\n";
    //     m_cOutput << "FINISH_TIME," << final_time << "\n";
    //     if(m_bTaskComplete) {
    //         m_cOutput << "TASK_STATUS,FINISHED" << "\n";
    //         std::cout << "[LOG] Task Status: FINISHED" << std::endl;
    //     } else {
    //         m_cOutput << "TASK_STATUS,UNFINISHED" << "\n";
    //         std::cout << "[LOG] Task Status: UNFINISHED" << std::endl;
    //     }
    //     m_cOutput.close();
    // }
    
    LOG << "[LOG] DESTROY called" << std::endl;
    CSimulator::GetInstance().Terminate();

}

/****************************************/
/****************************************/

CColor CExperimentLoopFunctionsNop::GetFloorColor(const CVector2& c_position_on_plane) {
    
    /* Charging area */
    if(PointIsInside(m_cFixedChargePos.GetX() - m_fFixedChargeAreaSideX/2 - 0.025f,
                    m_cFixedChargePos.GetY() - m_fFixedChargeAreaSideY/2 - 0.025f,
                    m_cFixedChargePos.GetX() + m_fFixedChargeAreaSideX/2,
                    m_cFixedChargePos.GetY() + m_fFixedChargeAreaSideY/2 + 0.025f,
                    c_position_on_plane.GetX(),         
                    c_position_on_plane.GetY())) {
        return CColor(191,255,191);
    }

    /* Energy Sharing area */
    if(m_unTotalChargers > 0) {
        if(PointIsInside(m_cMobileChargePos.GetX() - m_fMobileChargeAreaSideX/2,
                        m_cMobileChargePos.GetY() - m_fMobileChargeAreaSideY/2 - 0.025f,
                        m_cMobileChargePos.GetX() + m_fMobileChargeAreaSideX/2,
                        m_cMobileChargePos.GetY() + m_fMobileChargeAreaSideY/2 + 0.025f,
                        c_position_on_plane.GetX(),         
                        c_position_on_plane.GetY())) {
            return CColor(191,191,255);
        }
    }

    CSpace::TMapPerType* cCTasks;
    if(m_bNoDemandTasks) {
        cCTasks = &GetSpace().GetEntitiesByType("rectangle_task_no_demand");
    } else {
        cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
    }

    for(CSpace::TMapPerType::iterator it = cCTasks->begin();
       it != cCTasks->end();
       ++it) {

        if(m_bNoDemandTasks) {
            CRectangleTaskNoDemandEntity& cCTask = *any_cast<CRectangleTaskNoDemandEntity*>(it->second);
            if(PointIsInside(cCTask.GetPosition().GetX() - cCTask.GetWidthX()/2,
                            cCTask.GetPosition().GetY() - cCTask.GetWidthY()/2 - 0.025f,
                            cCTask.GetPosition().GetX() + cCTask.GetWidthX()/2 + 0.025f,
                            cCTask.GetPosition().GetY() + cCTask.GetWidthY()/2 + 0.025f,
                            c_position_on_plane.GetX(),         
                            c_position_on_plane.GetY())) {
                return CColor(255,191,191);
            }
        } else {
            CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(it->second);
            if(PointIsInside(cCTask.GetPosition().GetX() - cCTask.GetWidthX()/2,
                            cCTask.GetPosition().GetY() - cCTask.GetWidthY()/2 - 0.025f,
                            cCTask.GetPosition().GetX() + cCTask.GetWidthX()/2 + 0.025f,
                            cCTask.GetPosition().GetY() + cCTask.GetWidthY()/2 + 0.025f,
                            c_position_on_plane.GetX(),         
                            c_position_on_plane.GetY())) {
                if(cCTask.GetDemand() > 0) {
                    return CColor(255,191,191);
                } else {
                    return CColor(255,250,250);
                }
            }
        }
    }
    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PreStep() {

    LOG << "TIME: " << GetSpace().GetSimulationClock() << std::endl;

    UInt32 unConnectors = 0;
    m_mapRobotPerTask.clear(); // Used to store the number of e-pucks that have worked on each task in the previous timestep

    /* Add existing task id to the map */
    CSpace::TMapPerType* cCTasks;

    if(m_bTaskExists) {
        if(m_bNoDemandTasks) {
            cCTasks = &GetSpace().GetEntitiesByType("rectangle_task_no_demand");
        } else {
            // cCTasks = &GetSpace().GetEntitiesByType("circle_task");
            cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
        }

        for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
            itTask != cCTasks->end();
            ++itTask) {
            
            /* Initialize each task with zero e-pucks working on it */
            if(m_bNoDemandTasks) {
                CRectangleTaskNoDemandEntity& cCTask = *any_cast<CRectangleTaskNoDemandEntity*>(itTask->second);
                m_mapRobotPerTask[cCTask.GetId()] = 0;
            } else {
                CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                m_mapRobotPerTask[cCTask.GetId()] = 0;
            }

        }
    }

    /* List of followers ordered in ascending energy levels */
    std::vector<CEPuckEntity> vecFollowerEnergy;

    /* Loop workers */
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
        itEpuck != m_cEPucks.end();
        ++itEpuck) {

        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        try {
            CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());

            /* Count how many e-pucks are in each state */
            if( cController.GetRobotState() == RobotState::WORKER ) {
                // Count flock state

                /* 
                * Check whether the e-puck is working on a task
                */            

                /* Current location */
                CVector2 cPos = CVector2(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                         cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

                if(m_bTaskExists) {

                    for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
                        itTask != cCTasks->end();
                        ++itTask) {

                        /* Task location */
                        if(m_bNoDemandTasks) {
                            CRectangleTaskNoDemandEntity& cCTask = *any_cast<CRectangleTaskNoDemandEntity*>(itTask->second);

                            CVector2 cTaskPos = cCTask.GetPosition();

                            /* Check if robot is working on a task */
                            if(cController.IsWorking()) {
                                if(cCTask.InArea(cPos)) {
                                    
                                    m_mapRobotPerTask[cCTask.GetId()]++; // Increment robot working on this task
                                    m_mapRobotTaskStatus[cEPuck.GetId()] = true;
                                    break;
                                }
                            }
                        } else {
                            CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                            // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                            CVector2 cTaskPos = cCTask.GetPosition();

                            /* Check if robot is working on a task */
                            if(cController.IsWorking()) {
                                /* Check e-puck and its leader is within the range of a task */
                                // if((cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2) &&
                                // (cLeaderPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                                    
                                //     m_mapRobotPerTask[cCTask.GetId()]++; // Increment robot working on this task
                                //     break;
                                // }
                                if(cCTask.InArea(cPos)) {
                                    
                                    m_mapRobotPerTask[cCTask.GetId()]++; // Increment robot working on this task
                                    m_mapRobotTaskStatus[cEPuck.GetId()] = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                /* Store entity */
                vecFollowerEnergy.push_back(cEPuck);
            }

        } catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
            
        } catch(const std::bad_cast& e) {
            std::cout << e.what() << " in PreStep" << '\n';

        }
    }

    std::set<std::string> setChargersSharingEnergyTo;

    /* Loop chargers */
    if(m_unTotalChargers > 0) {
        CSpace::TMapPerType& m_cEPuckChargers = GetSpace().GetEntitiesByType("e-puck_charger");
        for(CSpace::TMapPerType::iterator itEpuck = m_cEPuckChargers.begin(); itEpuck != m_cEPuckChargers.end(); ++itEpuck) {

            /* Get handle to e-puck entity and controller */
            CEPuckChargerEntity& cEPuck = *any_cast<CEPuckChargerEntity*>(itEpuck->second);
            try {
                CCharger& cController = dynamic_cast<CCharger&>(cEPuck.GetControllableEntity().GetController());

                /* Store the robot this charger intends to share energy to */
                if(!cController.GetEnergyTo().empty() )
                    setChargersSharingEnergyTo.insert(cController.GetEnergyTo());

            } catch(CARGoSException& ex) {
                THROW_ARGOSEXCEPTION_NESTED("While casting robot as a charger", ex);
            }
        }
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PostStep() {
    Real total_demand = 0;

    /* Add existing task id to the map */
    CSpace::TMapPerType* cCTasks;

    if(m_bTaskExists) {
        if(m_bNoDemandTasks) {
            cCTasks = &GetSpace().GetEntitiesByType("rectangle_task_no_demand");
        } else {
            // cCTasks = &GetSpace().GetEntitiesByType("circle_task");
            cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
        }

        for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
            itTask != cCTasks->end();
            ++itTask) {
            
            if(m_bNoDemandTasks) {
                CRectangleTaskNoDemandEntity& cCTask = *any_cast<CRectangleTaskNoDemandEntity*>(itTask->second);
                total_demand += (int)cCTask.GetWorkPerformed();
            } else {
                CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                total_demand += (int)cCTask.GetDemand();
            }
        }
    }

    /* Update task demands */
    if(m_bTaskExists) {

        for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
            itTask != cCTasks->end();
            ++itTask) {

            if(m_bNoDemandTasks) {
                CRectangleTaskNoDemandEntity& cCTask = *any_cast<CRectangleTaskNoDemandEntity*>(itTask->second);
                UInt32 currentWorkPerformed = cCTask.GetWorkPerformed();

                cCTask.SetCurrentRobotNum(m_mapRobotPerTask[cCTask.GetId()]);

                /* Check if there is enough robots working on the task */
                if(m_mapRobotPerTask[cCTask.GetId()] >= cCTask.GetMinRobotNum()) {

                    /* Update task demand */
                    cCTask.SetWorkPerformed(currentWorkPerformed + m_mapRobotPerTask[cCTask.GetId()]);
                    m_unPointsObtained += m_mapRobotPerTask[cCTask.GetId()];
                }
            } else {
                CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                UInt32 currentDemand = cCTask.GetDemand();

                cCTask.SetCurrentRobotNum(m_mapRobotPerTask[cCTask.GetId()]);

                if(currentDemand == 0)
                    continue; // Skip completed tasks

                /* Check if there is enough robots working on the task */
                if(m_mapRobotPerTask[cCTask.GetId()] >= cCTask.GetMinRobotNum()) {

                    /* Update task demand */
                    if(currentDemand <= m_mapRobotPerTask[cCTask.GetId()]) {
                        cCTask.SetDemand(0);
                        /* The floor texture must be updated */
                        m_pcFloor->SetChanged();
                    } else {
                        cCTask.SetDemand(currentDemand - m_mapRobotPerTask[cCTask.GetId()]);
                    }
                }
            }
        }
    }

    // /* For logging energy consumption from moving or performing work */
    // if(m_bLogging) {
    //     // loop workers
    //     for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin(); itEpuck != m_cEPucks.end(); ++itEpuck) {

    //         /* Get handle to e-puck entity and controller */
    //         CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
    //         try {
    //             CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());
    //             CBatteryEquippedEntity& cBattery = cEPuck.GetBatterySensorEquippedEntity();                

    //             /* consuption */
    //             m_mapEnergyConsumed[cEPuck.GetId()] += cBattery;

    //         } catch(CARGoSException& ex) {
    //             THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
    //         }
    //     }
    // }

    /* Loop workers to find robots that wishes to transfer energy */
    std::unordered_map<std::string,CEPuckEntity> mapTravelerProviders;
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin(); itEpuck != m_cEPucks.end(); ++itEpuck) {

        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        try {
            CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());
            /* Check if it is trying to share energy */
            if( !cController.GetEnergyTo().empty() ) {
                /* Check if it has started to share energy */
                if(cController.IsSharingEnergy()) {
                    // LOG << cEPuck.GetId() << " sharing energy target = " << cController.GetEnergyTo() << std::endl;
                    mapTravelerProviders[cController.GetEnergyTo()] = cEPuck;
                }
            }
        } catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
        }
    }
    /* Loop chargers to find robots that wishes to transfer energy */
    std::unordered_map<std::string,CEPuckChargerEntity> mapChargerProviders;
    if(m_unTotalChargers > 0) {
        CSpace::TMapPerType& m_cEPuckChargers = GetSpace().GetEntitiesByType("e-puck_charger");
        for(CSpace::TMapPerType::iterator itEpuck = m_cEPuckChargers.begin(); itEpuck != m_cEPuckChargers.end(); ++itEpuck) {

            /* Get handle to e-puck entity and controller */
            CEPuckChargerEntity& cEPuck = *any_cast<CEPuckChargerEntity*>(itEpuck->second);
            try {
                CCharger& cController = dynamic_cast<CCharger&>(cEPuck.GetControllableEntity().GetController());
                /* Check if it is trying to share energy */
                if( !cController.GetEnergyTo().empty() ) {
                    /* Check if it has started to share energy */
                    if(cController.IsSharingEnergy()) {
                        LOG << cEPuck.GetId() << " sharing energy target = " << cController.GetEnergyTo() << std::endl;
                        mapChargerProviders[cController.GetEnergyTo()] = cEPuck;
                    }
                }

            } catch(CARGoSException& ex) {
                THROW_ARGOSEXCEPTION_NESTED("While casting robot as a charger", ex);
            }
        }
    }

    // Loop provider keys
    for(const auto& [key, value] : mapTravelerProviders) {
        LOG << "Energy receiver: " << key << std::endl;
    }
    for(const auto& [key, value] : mapChargerProviders) {
        LOG << "Energy receiver: " << key << std::endl;
    }

    /* Loop workers */
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin(); itEpuck != m_cEPucks.end(); ++itEpuck) {

        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        try {
            CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());
            CBatteryEquippedEntity& cBattery = cEPuck.GetBatterySensorEquippedEntity();

            CVector2 cPos = CVector2(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                     cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

            /* Energy update */

            /* Idle */
            Real newIdleCharge = cBattery.GetAvailableCharge() - m_fDeltaTime;
            if(newIdleCharge < 0)
                cBattery.SetAvailableCharge(0);
            else
                cBattery.SetAvailableCharge(newIdleCharge);
            m_mapEnergyConsumed[cController.GetId()] += m_fDeltaTime;

            if(cController.IsWorking() && m_mapRobotTaskStatus[cEPuck.GetId()]) { // Worker inside the task
                /* Working */
                Real newCharge = cBattery.GetAvailableCharge() - m_fDeltaWork;
                Real remainingCharge = 0;
                if(newCharge < 0) {
                    cBattery.SetAvailableCharge(0);
                    remainingCharge = cBattery.GetAvailableCharge();
                } else
                    cBattery.SetAvailableCharge(newCharge);

                if(remainingCharge > 0) {
                    m_mapEnergyConsumedToWork[cController.GetId()] += remainingCharge;
                    m_mapEnergyConsumed[cController.GetId()] += remainingCharge;
                } else {
                    m_mapEnergyConsumedToWork[cController.GetId()] += m_fDeltaWork;
                    m_mapEnergyConsumed[cController.GetId()] += m_fDeltaWork;
                }
            }

            if(cController.IsCharging() &&
                PointIsInside(m_cFixedChargePos.GetX() - m_fFixedChargeAreaSideX/2,
                            m_cFixedChargePos.GetY() - m_fFixedChargeAreaSideY/2,
                            m_cFixedChargePos.GetX() + m_fFixedChargeAreaSideX/2,
                            m_cFixedChargePos.GetY() + m_fFixedChargeAreaSideY/2,
                            cPos.GetX(),         
                            cPos.GetY())) {

                /* Charging */
                Real newCharge = cBattery.GetAvailableCharge() + m_fDeltaRecharge;
                if(newCharge > cBattery.GetFullCharge())
                    cBattery.SetAvailableCharge(cBattery.GetFullCharge());
                else
                    cBattery.SetAvailableCharge(newCharge);
            }                                
            
            if(cController.IsMoving()) {
                /* Moving */
                Real newCharge = cBattery.GetAvailableCharge() - m_fDeltaPosWorker;
                Real remainingCharge = 0;
                if(newCharge < 0) {
                    cBattery.SetAvailableCharge(0);
                    remainingCharge = cBattery.GetAvailableCharge();
                }
                else
                    cBattery.SetAvailableCharge(newCharge);

                if(remainingCharge > 0) {
                    m_mapEnergyConsumed[cController.GetId()] += remainingCharge;
                } else {
                    m_mapEnergyConsumed[cController.GetId()] += m_fDeltaPosWorker;
                }
            } 

            /* Depleted energy */
            if(cBattery.GetAvailableCharge() == 0.0f) {
                // if(cController.GetRobotState() == RobotState::CONNECTOR) {
                //     m_setDepletedConnectors.insert(cEPuck.GetId());
                // } else {
                    m_setDepletedWorkers.insert(cEPuck.GetId());
                // }
            }

            /* Transfering energy */
            if(cController.IsCharging()) {
                Real fTransferRatePerStep = m_fDeltaRecharge;
                Real fTransferEfficiency = m_fDeltaTransferLoss;
                // if(mapTravelerProviders.count(cEPuck.GetId())) {
                //     CEPuckEntity& cProvider = mapTravelerProviders[cEPuck.GetId()];
                //     CBatteryEquippedEntity& cProviderBattery = cProvider.GetBatterySensorEquippedEntity();

                //     /* Check the distance between the two robots and whether the provider has started sharing energy */
                //     Real fDistPair = Distance(cProvider.GetEmbodiedEntity().GetOriginAnchor().Position, cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position);
                    
                //     // LOG << "Check if can transfer... " << cProvider.GetId() << " -> " << cEPuck.GetId() << " (dist = " << fDistPair << ")" << std::endl;

                //     if(fDistPair < cController.GetDistToShareEnergy()) {
                //         LOG << "Transfering energy! " << cProvider.GetId() << " -> " << cEPuck.GetId() << std::endl;
                //         Real energyDelta = fTransferRatePerStep * fTransferEfficiency;
                //         Real newChargeProvider, newChargeReceiver;
                //         Real excessEnergy = 0;

                //         /* Increase receiver energy */
                //         newChargeReceiver = cBattery.GetAvailableCharge() + energyDelta;
                //         if(newChargeReceiver > cBattery.GetFullCharge()) {
                //             cBattery.SetAvailableCharge(cBattery.GetFullCharge());
                //             excessEnergy = newChargeReceiver - cBattery.GetFullCharge();
                //             m_fEnergyShared += energyDelta - excessEnergy;
                //         } else {
                //             cBattery.SetAvailableCharge(newChargeReceiver);
                //             m_fEnergyShared += energyDelta;
                //         }

                //         /* Reduce provider energy */
                //         if(excessEnergy > 0) {
                //             newChargeProvider = cProviderBattery.GetAvailableCharge() - energyDelta + excessEnergy;
                //         } else
                //             newChargeProvider = cProviderBattery.GetAvailableCharge() - energyDelta;

                //         if(newChargeProvider <= 0)
                //             cProviderBattery.SetAvailableCharge(0);
                //         else
                //             cProviderBattery.SetAvailableCharge(newChargeProvider);

                //     }
                // } 
                // else 
                if(mapChargerProviders.count(cEPuck.GetId())) {
                    CEPuckChargerEntity& cProvider = mapChargerProviders[cEPuck.GetId()];
                    CBatteryEquippedEntity& cProviderBattery = cProvider.GetBatterySensorEquippedEntity();
                    CCharger& cProviderController = dynamic_cast<CCharger&>(cProvider.GetControllableEntity().GetController());
                    /* Check the distance between the two robots and whether the provider has started sharing energy */
                    Real fDistPair = 100 * Distance(cProvider.GetEmbodiedEntity().GetOriginAnchor().Position, cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position);
                    
                    // LOG << "Check if can transfer... " << cProvider.GetId() << " -> " << cEPuck.GetId() << " (dist = " << fDistPair << " < " << cProviderController.GetDistToShareEnergy() << ")" << std::endl;

                    if(fDistPair < cProviderController.GetDistToShareEnergy()) {
                        LOG << "Transfering energy! " << cProvider.GetId() << " -> " << cEPuck.GetId() << std::endl;
                        Real energyDelta = fTransferRatePerStep * (1 - fTransferEfficiency);
                        // LOG << "Energy delta: " << energyDelta << std::endl;
                        // LOG << "fTransferRatePerStep" << fTransferRatePerStep << std::endl;
                        Real newChargeProvider, newChargeReceiver;
                        Real excessEnergy = 0;

                        /* Increase receiver energy */
                        newChargeReceiver = cBattery.GetAvailableCharge() + energyDelta;
                        if(newChargeReceiver > cBattery.GetFullCharge()) {
                            cBattery.SetAvailableCharge(cBattery.GetFullCharge());
                            excessEnergy = newChargeReceiver - cBattery.GetFullCharge();
                            m_fEnergyShared += energyDelta - excessEnergy;
                            m_fEnergyLost += fTransferRatePerStep * fTransferEfficiency;
                        } else {
                            cBattery.SetAvailableCharge(newChargeReceiver);
                            m_fEnergyShared += energyDelta;
                            m_fEnergyLost += fTransferRatePerStep * fTransferEfficiency;
                        }

                        /* Reduce provider energy */
                        if(excessEnergy > 0) {
                            newChargeProvider = cProviderBattery.GetAvailableCharge() - fTransferRatePerStep + excessEnergy;
                        } else
                            newChargeProvider = cProviderBattery.GetAvailableCharge() - fTransferRatePerStep;

                        if(newChargeProvider <= 0)
                            cProviderBattery.SetAvailableCharge(0);
                        else
                            cProviderBattery.SetAvailableCharge(newChargeProvider);

                        // cController.SetLED(CColor::MAGENTA);
                        // cProviderController.SetLED(CColor::MAGENTA);
                    } else {
                        // cController.SetLED(CColor::GREEN);
                        // cProviderController.SetLED(CColor::BLUE);
                    }
                }
            }
            
        } catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
        }
    }

    /* Loop chargers */
    if(m_unTotalChargers) {
        CSpace::TMapPerType& m_cEPuckChargers = GetSpace().GetEntitiesByType("e-puck_charger");
        for(CSpace::TMapPerType::iterator itEpuck = m_cEPuckChargers.begin(); itEpuck != m_cEPuckChargers.end(); ++itEpuck) {

            /* Get handle to e-puck entity and controller */
            CEPuckChargerEntity& cEPuck = *any_cast<CEPuckChargerEntity*>(itEpuck->second);
            try {
                CCharger& cController = dynamic_cast<CCharger&>(cEPuck.GetControllableEntity().GetController());
                CBatteryEquippedEntity& cBattery = cEPuck.GetBatterySensorEquippedEntity();

                CVector2 cPos = CVector2(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                        cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());    

                /* Energy update */

                /* Idle */
                Real newIdleCharge = cBattery.GetAvailableCharge() - m_fDeltaTime;
                if(newIdleCharge < 0)
                    cBattery.SetAvailableCharge(0);
                else
                    cBattery.SetAvailableCharge(newIdleCharge);
                m_mapEnergyConsumed[cController.GetId()] += m_fDeltaTime;

                if(cController.IsCharging() && 
                    PointIsInside(m_cFixedChargePos.GetX() - m_fFixedChargeAreaSideX/2,
                                m_cFixedChargePos.GetY() - m_fFixedChargeAreaSideY/2,
                                m_cFixedChargePos.GetX() + m_fFixedChargeAreaSideX/2,
                                m_cFixedChargePos.GetY() + m_fFixedChargeAreaSideY/2,
                                cPos.GetX(),         
                                cPos.GetY())) {

                    /* Charging */
                    Real newCharge = cBattery.GetAvailableCharge() + m_fDeltaRecharge;
                    if(newCharge > cBattery.GetFullCharge())
                        cBattery.SetAvailableCharge(cBattery.GetFullCharge());
                    else
                        cBattery.SetAvailableCharge(newCharge);
                }         
                
                if(cController.IsMoving()) {
                    /* Moving */
                    Real newCharge = cBattery.GetAvailableCharge() - m_fDeltaPosWorker;
                    Real remainingCharge = 0;
                    if(newCharge < 0) {
                        cBattery.SetAvailableCharge(0);
                        remainingCharge = cBattery.GetAvailableCharge();
                    }
                    else
                        cBattery.SetAvailableCharge(newCharge);

                    if(remainingCharge > 0) {
                        m_mapEnergyConsumed[cController.GetId()] += remainingCharge;
                    } else {
                        m_mapEnergyConsumed[cController.GetId()] += m_fDeltaPosWorker;
                    }
                } 

                /* Depleted energy */
                if(cBattery.GetAvailableCharge() == 0.0f) {
                    m_setDepletedChargers.insert(cEPuck.GetId());
                }

            } catch(CARGoSException& ex) {
                THROW_ARGOSEXCEPTION_NESTED("While casting robot as a charger", ex);
            }
        }
    }

    // /* Update current energy for logging */
    // // loop workers
    // for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin(); itEpuck != m_cEPucks.end(); ++itEpuck) {

    //     /* Get handle to e-puck entity and controller */
    //     CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
    //     try {
    //         CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());
    //         CBatteryEquippedEntity& cBattery = cEPuck.GetBatterySensorEquippedEntity();

    //         Real consumption = m_mapCurrentEnergy[cEPuck.GetId()] - cBattery.GetAvailableCharge();
    //         if(consumption > 0) {
    //             m_mapEnergyConsumed[cEPuck.GetId()] += consumption;
    //         }
    //         m_mapCurrentEnergy[cEPuck.GetId()] = cBattery.GetAvailableCharge();

    //     } catch(CARGoSException& ex) {
    //         THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
    //     }
    // }
    // // loop chargers
    // if(m_unTotalChargers) {
    //     CSpace::TMapPerType& m_cEPuckChargers = GetSpace().GetEntitiesByType("e-puck_charger");
    //     for(CSpace::TMapPerType::iterator itEpuck = m_cEPuckChargers.begin(); itEpuck != m_cEPuckChargers.end(); ++itEpuck) {

    //         /* Get handle to e-puck entity and controller */
    //         CEPuckChargerEntity& cEPuck = *any_cast<CEPuckChargerEntity*>(itEpuck->second);
    //         try {
    //             CCharger& cController = dynamic_cast<CCharger&>(cEPuck.GetControllableEntity().GetController());
    //             CBatteryEquippedEntity& cBattery = cEPuck.GetBatterySensorEquippedEntity();

    //             Real consumption = m_mapCurrentEnergy[cEPuck.GetId()] - cBattery.GetAvailableCharge();
    //             if(consumption > 0) {
    //                 m_mapEnergyConsumed[cEPuck.GetId()] += consumption;
    //             }
    //             m_mapCurrentEnergy[cEPuck.GetId()] = cBattery.GetAvailableCharge();

    //         } catch(CARGoSException& ex) {
    //             THROW_ARGOSEXCEPTION_NESTED("While casting robot as a charger", ex);
    //         }
    //     }
    // }

    /* 
    * Output stuff to file 
    */

    if(m_bLogging) {
        /* Create new node for this timestep */
        TimeStep tData;
        tData.set_time(GetSpace().GetSimulationClock());
        tData.set_points(m_unPointsObtained);

        /* Log energy info */

        // loop m_mapEnergyConsumed and add total
        Real totalEnergyConsumed = 0;
        for(const auto& [key, value] : m_mapEnergyConsumed) {
            totalEnergyConsumed += value;
        }
        tData.set_totalenergy(totalEnergyConsumed);

        // loop m_mapEnergyConsumedToWork and add total
        Real totalEnergyConsumedToWork = 0;
        for(const auto& [key, value] : m_mapEnergyConsumedToWork) {
            totalEnergyConsumedToWork += value;
        }
        tData.set_workenergy(totalEnergyConsumedToWork);

        // add total m_setDepletedWorkersAndChargers
        tData.set_workersdepleted(m_setDepletedWorkers.size());
        tData.set_chargersdepleted(m_setDepletedChargers.size());

        tData.set_energyshared(m_fEnergyShared);
        tData.set_energylost(m_fEnergyLost);

        /* Output worker info */
        for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
            itEpuck != m_cEPucks.end();
            ++itEpuck) {

            CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
            CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());

            Robot* robot = tData.add_robots();
            robot->set_name(cEPuck.GetId());
            switch(cController.GetRobotState()) {
                case RobotState::WORKER:
                    robot->set_state(Robot_State_WORKER);
                    break;
                default:
                    std::cerr << "Tried to log unknown state " << (int)cController.GetRobotState() << std::endl;
                    break;
            }
            if( !cController.GetLastAction().empty() )
                robot->set_action(cController.GetLastAction());
            // robot->mutable_position()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
            // robot->mutable_position()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            // robot->mutable_orientation()->set_w(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW());
            // robot->mutable_orientation()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX());
            // robot->mutable_orientation()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY());
            // robot->mutable_orientation()->set_z(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ());

            // if(cController.GetRobotState() == RobotState::CONNECTOR) {
            //     for(const auto& [team, hop] : cController.GetHops()) {
            //         HopCount* hopCount = robot->add_hopcount();
            //         hopCount->set_teamid(team);
            //         hopCount->set_count(hop.count);
            //         hopCount->set_neighbor(hop.ID);
            //     }
            //     for(const auto& [team, hop] : cController.GetPrevHops()) {
            //         HopCount* prevHop = robot->add_prevhops();
            //         prevHop->set_teamid(team);
            //         prevHop->set_count(hop.count);
            //         prevHop->set_neighbor(hop.ID);
            //     }
            // }

            robot->set_energylevel(cEPuck.GetBatterySensorEquippedEntity().GetAvailableCharge());

            std::string moveType = cController.GetMoveType();
            if(moveType == "MOVE_TO_WORK") {
                robot->set_movetype(Robot_MoveType_MOVE_TO_WORK);
            } else if(moveType == "MOVE_TO_CHARGE") {
                robot->set_movetype(Robot_MoveType_MOVE_TO_CHARGE);
            } else {
                LOGERR << "Unknown move type: " << moveType << " for " << cEPuck.GetId() << std::endl;
            }
            robot->set_isworking(cController.IsWorking());
            robot->set_ischarging(cController.IsCharging());
            robot->set_ismoving(cController.IsMoving());
        }

        /* Output charger info */
        if(m_unTotalChargers > 0) {
            CSpace::TMapPerType& m_cEPuckChargers = GetSpace().GetEntitiesByType("e-puck_charger");
            for(CSpace::TMapPerType::iterator itEpuck = m_cEPuckChargers.begin();
                itEpuck != m_cEPuckChargers.end();
                ++itEpuck) {

                CEPuckChargerEntity& cEPuck = *any_cast<CEPuckChargerEntity*>(itEpuck->second);
                CCharger& cController = dynamic_cast<CCharger&>(cEPuck.GetControllableEntity().GetController());

                Robot* robot = tData.add_robots();
                robot->set_name(cEPuck.GetId());
                robot->set_state(Robot_State_CHARGER);
                if( !cController.GetLastAction().empty() )
                    robot->set_action(cController.GetLastAction());
                // robot->mutable_position()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
                // robot->mutable_position()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
                // robot->mutable_orientation()->set_w(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW());
                // robot->mutable_orientation()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX());
                // robot->mutable_orientation()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY());
                // robot->mutable_orientation()->set_z(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ());
                robot->set_energylevel(cEPuck.GetBatterySensorEquippedEntity().GetAvailableCharge());

                std::string moveType = cController.GetMoveType();
                if(moveType == "MOVE_TO_WORK") {
                    robot->set_movetype(Robot_MoveType_MOVE_TO_WORK);
                } else if(moveType == "MOVE_TO_CHARGE") {
                    robot->set_movetype(Robot_MoveType_MOVE_TO_CHARGE);
                } else {
                    LOGERR << "Unknown move type: " << moveType << " for " << cEPuck.GetId() << std::endl;
                }
                robot->set_ischarging(cController.IsCharging());
                robot->set_ismoving(cController.IsMoving());
            }
        }

        if(m_bTaskExists) {

            /* Output task info */
            for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
                itTask != cCTasks->end();
                ++itTask) {

                if(m_bNoDemandTasks) {
                    CRectangleTaskNoDemandEntity& cCTask = *any_cast<CRectangleTaskNoDemandEntity*>(itTask->second);

                    /* Log tasks that are inside the arena */
                    if(cCTask.GetPosition().GetX() < 500) {
                        Task* task = tData.add_tasks();
                        task->set_name(cCTask.GetId());
                        task->set_demand(cCTask.GetWorkPerformed());
                        // task->set_requiredrobots(cCTask.GetMinRobotNum());
                        // task->set_currentrobots(cCTask.GetCurrentRobotNum());
                        // task->mutable_position()->set_x(cCTask.GetPosition().GetX());
                        // task->mutable_position()->set_y(cCTask.GetPosition().GetY());
                        // task->set_radius(cCTask.GetRadius());
                    }
                } else {
                    CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                    // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                    /* Log tasks that are inside the arena */
                    if(cCTask.GetPosition().GetX() < 500) {
                        Task* task = tData.add_tasks();
                        task->set_name(cCTask.GetId());
                        task->set_demand(cCTask.GetDemand());
                        // task->set_requiredrobots(cCTask.GetMinRobotNum());
                        // task->set_currentrobots(cCTask.GetCurrentRobotNum());
                        // task->mutable_position()->set_x(cCTask.GetPosition().GetX());
                        // task->mutable_position()->set_y(cCTask.GetPosition().GetY());
                        // task->set_radius(cCTask.GetRadius());
                    }
                }
            }
        }

        /* Write to file */
        m_cOutput.open(m_strBinaryFilePath.c_str(), std::ios::app | std::ios::binary);
        google::protobuf::util::SerializeDelimitedToOstream(tData, &m_cOutput);
        m_cOutput.close();
    }

    /* Grab frame */
    if(m_bFrameGrabbing) {
        CQTOpenGLRender& render = dynamic_cast<CQTOpenGLRender&>(GetSimulator().GetVisualization());
        CQTOpenGLWidget& widget = render.GetMainWindow().GetOpenGLWidget();
        widget.SetCamera(m_unCameraIndex);
        widget.SetGrabFrame(m_bFrameGrabbing);
    }

    /* Terminate simulation time limit is reached */
    if(m_bTaskExists) {
        if(m_bNoDemandTasks) {
            if (GetSpace().GetSimulationClock() == CSimulator::GetInstance().GetMaxSimulationClock()) {
                int final_time = GetSpace().GetSimulationClock();
                m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
                m_cOutput << "\n";
                m_cOutput << "FINISH_TIME," << final_time << "\n";
                m_cOutput << "POINTS SCORED," << (int)m_unPointsObtained << "\n";
                m_cOutput << "DEPLETED WORKERS," << (int)m_setDepletedWorkers.size() << "\n";
                m_cOutput << "DEPLETED CHARGERS," << (int)m_setDepletedChargers.size() << "\n";
                m_cOutput << "ENERGY LOST, " << m_fEnergyLost << "\n";
                // m_cOutput << "TASK_STATUS,FINISHED" << "\n";
                m_cOutput.close();
                std::cout << "[LOG] Reached time limit!" << std::endl;
                std::cout << "[LOG] Score: " << (int)m_unPointsObtained << std::endl;
                // std::cout << "[LOG] Mission time: " << final_time << std::endl;
                // std::cout << "[LOG] All tasks completed" << std::endl;
                std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
                this->Destroy();
            }
        } else {
            if(total_demand == 0) {
                int final_time = GetSpace().GetSimulationClock();
                m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
                m_cOutput << "\n";
                m_cOutput << "FINISH_TIME," << final_time << "\n";
                m_cOutput << "POINTS SCORED," << (int)m_unPointsObtained << "\n";
                // m_cOutput << "TASK_STATUS,FINISHED" << "\n";
                m_cOutput.close();
                std::cout << "[LOG] Tasks completed!" << std::endl;
                std::cout << "[LOG] Score: " << (int)m_unPointsObtained << std::endl;
                // std::cout << "[LOG] Mission time: " << final_time << std::endl;
                // std::cout << "[LOG] All tasks completed" << std::endl;
                std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
                this->Destroy();
            }
        }
    }
    else if (GetSpace().GetSimulationClock() == CSimulator::GetInstance().GetMaxSimulationClock()) {

        int final_time = GetSpace().GetSimulationClock();
        m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
        m_cOutput << "\n";
        m_cOutput << "FINISH_TIME," << final_time << "\n";
        m_cOutput << "POINTS SCORED," << (int)m_unPointsObtained << "\n";
        // m_cOutput << "TASK_STATUS,FINISHED" << "\n";
        m_cOutput.close();
        std::cout << "[LOG] Reached time limit!" << std::endl;
        std::cout << "[LOG] Score: " << (int)m_unPointsObtained << std::endl;
        // std::cout << "[LOG] Mission time: " << final_time << std::endl;
        // std::cout << "[LOG] All tasks completed" << std::endl;
        std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
        this->Destroy();
    }
}

// std::vector<CVector2> CExperimentLoopFunctionsNop::GetArenaSize() const {
//     return m_vecArenaSize;
// }

bool CExperimentLoopFunctionsNop::IsDrawRobotLabel() const {
    return m_bDrawRobotLabel;
}

/****************************************/
/****************************************/

bool CExperimentLoopFunctionsNop::IsLogging() {
    return m_bLogging;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitLogging() {
    
    /* 
    * Create new directory to store the logs
    */ 

    std::string dir_name = m_strOutput;

    /* Get the experiment directory name */
    if(dir_name[dir_name.size() - 1] == '/') {
        dir_name.pop_back(); // If last char is /, drop it
    }

    std::stringstream ss(dir_name);
    std::string segment;
    std::vector<std::string> dir_path;

    while(std::getline(ss, segment, '/')) {
        dir_path.push_back(segment);
    }
    dir_name = dir_path[dir_path.size() - 1];

    /* Get the parent directory name */
    dir_path.pop_back();
    std::ostringstream oss;
    oss.str("");
    for(auto& segment : dir_path) {
        oss << segment << "/";
    }
    std::string dir_parent_name = oss.str();
    
    /* Loop directory to see what experiment number to append to dir_name */
    oss.str("");
    oss << dir_parent_name << dir_name << "/";
    m_strDirPath = oss.str();
    std::vector<std::string> r;

    if(fs::exists(m_strDirPath)) {
        for(auto& p : fs::recursive_directory_iterator(m_strDirPath)) {
            if (p.is_directory()) {
                if(p.path().string().find(dir_name) != std::string::npos)
                    r.push_back(p.path().string()); // Count
            }
        }
    }
    
    std::string new_dir_name = dir_name;

    // /* Append energy-aware strategy */
    // if(m_strWorkerType == "worker") {
    //     new_dir_name.append("(travel)");
    // } else if(m_strWorkerType == "worker_mc") {
    //     new_dir_name.append("(charger)");
    // } else {
    //     LOGERR << "Unknown worker type: " << m_strWorkerType << std::endl;
    // }

    /* Append experiment config (team_num, team_size) */
    oss.str("");
    // oss << "_" << m_unTeams << "T_" << m_unWorkerPerTeam << "R";
    new_dir_name.append(oss.str());

    /* Append experiment number */
    int run_number;
    if( !m_strRunNumber.empty() ) {
        run_number = stoi(m_strRunNumber);
    } else {
        run_number = r.size() + 1;
    }

    if(run_number < 10) {
        new_dir_name.append("_00");
    } else if(run_number < 100) {
        new_dir_name.append("_0");
    } else {
        new_dir_name.append("_");
    }
    new_dir_name.append(std::to_string(run_number));

    /* Create directory */
    oss.str("");
    oss << dir_parent_name << dir_name << "/" << new_dir_name << "/";
    m_strDirPath = oss.str();
    fs::create_directories(m_strDirPath);
    LOG << "Created " << m_strDirPath << std::endl;

    /* Set output file names */
    oss.str("");
    oss << m_strDirPath << BINARY_FILENAME;
    m_strBinaryFilePath = oss.str();

    oss.str("");
    oss << m_strDirPath << SUMMARY_FILENAME;
    m_strSummaryFilePath = oss.str();

    oss.str("");
    oss << m_strDirPath << COMMAND_FILENAME;
    m_strCommandFilePath = oss.str();

    LOG << m_strBinaryFilePath << std::endl;
    LOG << m_strCommandFilePath << std::endl;

    /* 
    * Log experiment summary data
    */

    /* Write to file */
    m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
    m_cOutput << "SCENARIO_NAME," << new_dir_name << "\n";
    m_cOutput << "SEED," << (int)CSimulator::GetInstance().GetRandomSeed() << "\n";
    m_cOutput << "MAX_SIMULATION_CLOCK," << (int)CSimulator::GetInstance().GetMaxSimulationClock() << "\n";

    // m_cOutput << "TOTAL_LEADERS," << (int)m_unTeams << "\n";
    m_cOutput << "TOTAL_WORKERS," << (int)m_unTotalWorkers << "\n";
    m_cOutput << "TOTAL_CHARGERS," << (int)m_unTotalChargers << "\n";
    // m_cOutput << "TOTAL_ROBOTS," << (int)(m_unTeams + m_unWorkerPerTeam) << "\n";

    // m_cOutput << "TOTAL_TASKS," << (int)m_unTotalTasks << "\n";
    // m_cOutput << "TASK_DEMAND," << (int)m_unTaskDemand << "\n";

    m_cOutput.close();
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitLoggingEnergy() {
    
    // /* 
    // * Create new directory to store the logs
    // */ 

    // std::string dir_name = m_strOutput;

    // /* Get the experiment directory name */
    // if(dir_name[dir_name.size() - 1] == '/') {
    //     dir_name.pop_back(); // If last char is /, drop it
    // }

    // std::stringstream ss(dir_name);
    // std::string segment;
    // std::vector<std::string> dir_path;

    // while(std::getline(ss, segment, '/')) {
    //     dir_path.push_back(segment);
    // }
    // dir_name = dir_path[dir_path.size() - 1];

    // /* Get the parent directory name */
    // dir_path.pop_back();
    // std::ostringstream oss;
    // oss.str("");
    // for(auto& segment : dir_path) {
    //     oss << segment << "/";
    // }
    // std::string dir_parent_name = oss.str();
    
    // /* Loop directory to see what experiment number to append to dir_name */
    // oss.str("");
    // oss << dir_parent_name << dir_name << "/";
    // m_strDirPath = oss.str();
    // std::vector<std::string> r;

    // if(fs::exists(m_strDirPath)) {
    //     for(auto& p : fs::recursive_directory_iterator(m_strDirPath)) {
    //         if (p.is_directory()) {
    //             if(p.path().string().find(dir_name) != std::string::npos)
    //                 r.push_back(p.path().string()); // Count
    //         }
    //     }
    // }
    
    // std::string new_dir_name = dir_name;

    // /* Append energy-aware strategy */
    // m_strWorkerType = "worker"; // TEMP: add to config
    // if(m_strWorkerType == "worker") {
    //     new_dir_name.append("(travel)");
    // } else if(m_strWorkerType == "worker_mc") {
    //     new_dir_name.append("(charger)");
    // } else {
    //     LOGERR << "Unknown worker type: " << m_strWorkerType << std::endl;
    // }

    // /* Append experiment config (team_num, team_size) */
    // oss.str("");
    // // oss << "_" << m_unTeams << "T_" << m_unWorkerPerTeam << "R_" << m_unChargersPerTeam << "C";
    // new_dir_name.append(oss.str());

    // /* Append experiment number */
    // int run_number;
    // if( !m_strRunNumber.empty() ) {
    //     run_number = stoi(m_strRunNumber);
    // } else {
    //     run_number = r.size() + 1;
    // }

    // if(run_number < 10) {
    //     new_dir_name.append("_00");
    // } else if(run_number < 100) {
    //     new_dir_name.append("_0");
    // } else {
    //     new_dir_name.append("_");
    // }
    // new_dir_name.append(std::to_string(run_number));

    // /* Create directory */
    // oss.str("");
    // oss << dir_parent_name << dir_name << "/" << new_dir_name << "/";
    // m_strDirPath = oss.str();
    // fs::create_directories(m_strDirPath);
    // LOG << "Created " << m_strDirPath << std::endl;

    m_strDirPath = m_strOutput;

    /* Set output file names */
    std::ostringstream oss;
    oss.str("");
    oss << m_strDirPath << BINARY_FILENAME;
    m_strBinaryFilePath = oss.str();

    oss.str("");
    oss << m_strDirPath << SUMMARY_FILENAME;
    m_strSummaryFilePath = oss.str();

    // oss.str("");
    // oss << m_strDirPath << COMMAND_FILENAME;
    // m_strCommandFilePath = oss.str();

    LOG << m_strBinaryFilePath << std::endl;
    // LOG << m_strCommandFilePath << std::endl;

    /* 
    * Log experiment summary data
    */

    /* Write to file */
    m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
    LOG << "m_strOutput " << m_strOutput << std::endl;
    
    // m_cOutput << "SCENARIO_NAME," << new_dir_name << "\n";
    m_cOutput << "SEED," << (int)CSimulator::GetInstance().GetRandomSeed() << "\n";
    m_cOutput << "MAX_SIMULATION_CLOCK," << (int)CSimulator::GetInstance().GetMaxSimulationClock() << "\n";

    // m_cOutput << "TOTAL_LEADERS," << (int)m_unTeams << "\n";
    m_cOutput << "TOTAL_WORKERS," << (int)m_unTotalWorkers << "\n";
    m_cOutput << "TOTAL_CHARGERS," << (int)m_unTotalChargers << "\n";
    // m_cOutput << "TOTAL_ROBOTS," << (int)(m_unTeams + m_unWorkerPerTeam) << "\n";

    // m_cOutput << "TOTAL_TASKS," << (int)m_unTotalTasks << "\n";
    // m_cOutput << "TASK_DEMAND," << (int)m_unTaskDemand << "\n";

    m_cOutput.close();
}

/****************************************/
/****************************************/

std::string CExperimentLoopFunctionsNop::GetCommandFilePath() {
    return m_strCommandFilePath;
}

/****************************************/
/****************************************/

std::unordered_map<std::string, UInt32> CExperimentLoopFunctionsNop::GetRobotPerTask() {
    return m_mapRobotPerTask;
}

/****************************************/
/****************************************/

UInt32 CExperimentLoopFunctionsNop::GetCurrentPoints() {
    return m_unPointsObtained;
}

/****************************************/
/****************************************/

std::string CExperimentLoopFunctionsNop::GetWorkerType() const {
    return m_strWorkerType;
}

std::string CExperimentLoopFunctionsNop::GetTaskType() const {
    // if(m_bNoDemandTasks) {
    //     return "circle_task_no_demand";
    // } else {
    //     return "circle_task";
    // }
    if(m_bNoDemandTasks) {
        return "rectangle_task_no_demand";
    } else {
        return "rectangle_task";
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitRobots() {
    /*
    * Distribute robots
    */

    LOG << "[LOG] Adding robots..." << std::endl;

    /* ID counts */
    UInt32 unNextRobotId = 1;
    UInt32 unNextChargerId = 1;
    /* Get the teams node */
    TConfigurationNode& et_tree = GetNode(config, "distribute");
    /* Go through the nodes */
    TConfigurationNodeIterator itDistr;
    m_unTotalWorkers = 0;
    m_unTotalChargers = 0;

    /* Get the needed nodes */
    // TConfigurationNode cPositionNode;
    // cPositionNode = GetNode(et_tree, "position");
    TConfigurationNode cEPuckNode;
    cEPuckNode = GetNode(et_tree, "e-puck");
    TConfigurationNode cEPuckChargerNode;
    cEPuckChargerNode = GetNode(et_tree, "e-puck_charger");

    CVector2 cMin, cMax;
    // GetNodeAttributeOrDefault(cPositionNode, "min", cMin, CVector2(-0.8,-0.5));
    // GetNodeAttributeOrDefault(cPositionNode, "max", cMax, CVector2(-0.5,0.5));

    /* Place e-puck */
    UInt32 unNumWorkers;
    GetNodeAttributeOrDefault(cEPuckNode, "controller", m_strWorkerType, ToString("worker"));
    GetNodeAttributeOrDefault(cEPuckNode, "num_robots", unNumWorkers, (UInt32)10);

    // if(m_strWorkerType == "worker") {
        cMin = CVector2(m_cFixedChargePos.GetX() - m_fFixedChargeAreaSideX/2,
                        m_cFixedChargePos.GetY() - m_fFixedChargeAreaSideY/2);
        cMax = CVector2(m_cFixedChargePos.GetX() + m_fFixedChargeAreaSideX/2,
                        m_cFixedChargePos.GetY() + m_fFixedChargeAreaSideY/2);
    // } else if(m_strWorkerType == "worker_mc") {
    //     LOG << "minX " << m_cTaskPos.GetX() << std::endl;
    //     LOG << "sideX " << m_fTaskAreaSideX << std::endl;
    //     cMin = CVector2(m_cTaskPos.GetX() - m_fTaskAreaSideX/2.0,
    //                     m_cTaskPos.GetY() - m_fTaskAreaSideY/2.0);
    //     cMax = CVector2(m_cTaskPos.GetX() + m_fTaskAreaSideX/2.0,
    //                     m_cTaskPos.GetY() + m_fTaskAreaSideY/2.0);
    // } else {
    //     LOGERR << "Unknown worker type: " << m_strWorkerType << std::endl;
    // }

    LOG << "cMin " << cMin << std::endl;
    LOG << "cMax " << cMax << std::endl;
    LOG << "controller " << m_strWorkerType << std::endl;
    LOG << "unNumWorkers " << unNumWorkers << std::endl;

    PlaceRobots(cMin, cMax, unNumWorkers, m_strWorkerType);
    m_unTotalWorkers = unNumWorkers;

    /* Place e-puck_charger */
    UInt32 unNumChargers;
    GetNodeAttributeOrDefault(cEPuckChargerNode, "controller", m_strChargerType, ToString("charger"));
    GetNodeAttributeOrDefault(cEPuckChargerNode, "num_robots", unNumChargers, (UInt32)0);
    LOG << "controller " << m_strChargerType << std::endl;
    LOG << "unNumChargers " << unNumChargers << std::endl;

    PlaceRobots(cMin, cMax, unNumChargers, m_strChargerType);
    m_unTotalChargers = unNumChargers;

    LOG << "[LOG] Added robots" << std::endl;

}

// /****************************************/
// /****************************************/

// void CExperimentLoopFunctionsNop::InitChargers() {
//     /*
//     * Distribute chargers
//     */

//     LOG << "[LOG] Adding chargers..." << std::endl;
//     UInt32 unNextChargerId = 1;

//     /* Add chargers to each team */
//     for(auto& team : m_vecTeamCenters) {
//         CVector2 cCenter = team.second;
//         UInt32 unChargers = m_vecNumChargers[team.first];
//         Real fDensity = m_vecTeamDensity[team.first];
//         m_unTotalChargers += unChargers;
//         PlaceMobileChargers(cCenter, unChargers, team.first, fDensity, unNextChargerId);
//         unNextChargerId += unChargers;
//     }

//     m_unChargersPerTeam = m_unTotalChargers / m_unTeams;

//     LOG << "[LOG] Added chargers" << std::endl;
// }

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitTasks() {
    /*
    * Initialize tasks
    */

    LOG << "[LOG] Adding tasks..." << std::endl;

    /* ID counts */
    UInt32 m_unNextTaskId = 1;
    /* Meta data */
    size_t m_unTotalTasks = 0;
    UInt32 m_unTaskDemand = 0; 
    /* Get the teams node */
    TConfigurationNode& ts_tree = GetNode(config, "tasks");
    /* Go through the nodes (tasks) */
    TConfigurationNodeIterator itDistr;
    for(itDistr = itDistr.begin(&ts_tree);
        itDistr != itDistr.end();
        ++itDistr) {

        m_bTaskExists = true;
        m_bTaskComplete = false;

        /* Get current node (task) */
        TConfigurationNode& tDistr = *itDistr;
        /* Task center */
        CVector2 cCenter;
        GetNodeAttribute(tDistr, "position", cCenter);
        /* Task radius */
        Real fRadius;
        GetNodeAttribute(tDistr, "radius", fRadius);
        /* Task Height */
        Real fHeight;
        GetNodeAttribute(tDistr, "height", fHeight);
        /* Task demand */
        UInt32 unDemand;
        GetNodeAttribute(tDistr, "task_demand", unDemand);
        /* Minimum robot constraint */
        UInt32 unMinRobotNum;
        GetNodeAttribute(tDistr, "minimum_robot_num", unMinRobotNum);
        /* Maximum robot constraint */
        UInt32 unMaxRobotNum;
        GetNodeAttribute(tDistr, "maximum_robot_num", unMaxRobotNum);
        
        /* Place Tasks */
        // PlaceTask(cCenter, fRadius, unDemand, unMinRobotNum, unMaxRobotNum, m_unNextTaskId);
        PlaceCircleTask(cCenter, fRadius, fHeight, unDemand, unMinRobotNum, unMaxRobotNum, m_unNextTaskId);

        /* Update task count */
        m_unNextTaskId++;

        m_unTotalTasks++;
        m_unTaskDemand += unDemand;
    }

    if(m_bLogging) {
        /* Write to file */
        m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
        m_cOutput << "TOTAL_TASKS," << (int)m_unTotalTasks << "\n";
        m_cOutput << "TASK_DEMAND," << (int)m_unTaskDemand << "\n";
        m_cOutput.close();
    }

    LOG << "[LOG] Added tasks" << std::endl;
}

/****************************************/
/****************************************/

// void CExperimentLoopFunctionsNop::InitTasksCircular() {
//     /*
//     * Initialize tasks
//     */

//     LOG << "[LOG] Adding tasks..." << std::endl;

//     /* Set the type of the task */
//     // m_bNoDemandTasks = true;
//     // m_bNoDemandTasks = false;

//     /* ID counts */
//     m_unNextTaskId = 1;
//     /* Meta data */
//     UInt32 unTotalTasks = 1;
//     UInt32 unInitTasks = 1;
//     m_unTotalTasks = 0;
//     m_unTaskDemand = 0; 
//     m_unPointsObtained = 0;

//     m_bTaskExists = true;
//     m_bTaskComplete = false;

//     // TODO: Check if task exists at all in argos file

//     for(UInt32 i = 1; i <= unTotalTasks; ++i) {

//         /* Task dimensions */
//         Real density = 0.05;
//         // Real fRadius = Sqrt(EP_AREA * (m_unWorkerPerTeam + 1) / (density * ARGOS_PI));
//         Real fRadius = 1;
//         Real fHeight = 0.3;
//         /* Task demand */
//         UInt32 unDemand = 2000; // FIXED
//         if(m_bNoDemandTasks)
//             unDemand = 400000;
//         /* Min and Max robot constraint */
//         UInt32 unMinRobotNum = 1; // m_unWorkerPerTeam / 2;
//         UInt32 unMaxRobotNum = 100;

//         CVector2 cCenter = CVector2();
//         if(i <= unInitTasks) {

//             /* Check whether this position overlaps with existing tasks */
//             for(UInt32 j = 0; j < 100; ++j) {

//                 LOG << "--------------" << std::endl;

//                 // // Pick random length
//                 // // Make sure it fits in the circle arena and avoids the deployment area
//                 CRange<Real> cLengthRange = CRange<Real>(0, 1); 

//                 //#########################################################################
//                 // RANDOMLY DISTRIBUTE TASKS ANYWHERE
//                 CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);
//                 // Uniformly choose a point based on https://stackoverflow.com/a/50746409
//                 Real radius = (m_fArenaRadius - fRadius) * Sqrt(m_pcRNG->Uniform(cLengthRange));
//                 CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
//                 //#########################################################################

//                 LOG << "radius: " << radius << ", angle: " << angle << std::endl;
//                 cCenter = CVector2(radius, angle);

//                 LOG << "Trying " << cCenter.GetX() << ", " << cCenter.GetY() << std::endl;

//                 bool bInvalidTaskPos = false;

//                 /* Check whether the chosen position does not overlap with the deployment area */
//                 // LOG << "cCenter.Length(): " << cCenter.Length() << ", m_fDeploymentRadius + fRadius: " << (m_fDeploymentRadius + fRadius) << ", m_fArenaRadius - fRadius: " << (m_fArenaRadius - fRadius) << std::endl;
//                 if(cCenter.Length() < (m_fDeploymentRadius + fRadius)) {
//                     bInvalidTaskPos = true;
//                 }

//                 /* Check whether the chosen position is not too close to existing tasks (2*comm_range) */
//                 for(auto& [task_id, task] : m_mapTaskPos) {

//                     if(task.first.Length() > 500)
//                         continue; // Skip tasks outside of arena

//                     if((task.first - cCenter).Length() < EP_RAB_RANGE || (task.first - cCenter).Length() < 2*fRadius) {
//                         bInvalidTaskPos = true;
//                     }
//                 }

//                 if(bInvalidTaskPos) {
//                     /* Position is invalid. Try again */
//                     LOG << "invalid position" << std::endl;
//                     continue;
//                 } else {
//                     /* Position is valid. Place task at the chosen position */
//                     LOG << "VALID position" << std::endl;
//                     break;
//                 }
//             }

//             m_unTotalTasks++;
//             m_unTaskDemand += unDemand;
//         } else {
//             // Place it out of sight
//             cCenter = CVector2(1000, 1000);
//         }

//         // LOG << "Arena radius " << m_fArenaRadius << std::endl;
//         // LOG << "Task pos " << cCenter.GetX() << ", " << cCenter.GetY() << std::endl;

//         // PlaceTask
//         PlaceCircleTask(cCenter, fRadius, fHeight, unDemand, unMinRobotNum, unMaxRobotNum, m_unNextTaskId);

//         m_mapTaskPos[m_unNextTaskId] = {cCenter, fRadius};

//         /* Update task count */
//         m_unNextTaskId++;
//     }

//     LOG << "[LOG] Added tasks" << std::endl;
// }


void CExperimentLoopFunctionsNop::InitTask() {
    /*
    * Initialize tasks
    */

    LOG << "[LOG] Adding task..." << std::endl;

    /* ID counts */
    m_unNextTaskId = 1;
    /* Meta data */
    UInt32 unTotalTasks = 1;
    UInt32 unInitTasks = 1;
    m_unTotalTasks = unTotalTasks;
    m_unTaskDemand = 0; 
    m_unPointsObtained = 0;

    m_bTaskExists = true;
    m_bTaskComplete = false;

    /* Task dimensions */
    Real fHeight = 0;
    // CVector2 cCenter = CVector2(0.65, 0);

    /* Task demand */
    UInt32 unDemand = 2000;
    if(m_bNoDemandTasks)
        unDemand = 400000;
    // /* Min and Max robot constraint */
    // UInt32 unMinRobotNum = 1; // m_unWorkerPerTeam / 2;
    // UInt32 unMaxRobotNum = 100;

    m_unTotalTasks++;
    m_unTaskDemand += unDemand;

    // // LOG << "Arena radius " << m_fArenaRadius << std::endl;
    LOG << "Task pos " << m_cTaskPos.GetX() << ", " << m_cTaskPos.GetY() << std::endl;

    // // PlaceTask
    PlaceRectangleTask(m_cTaskPos, m_fTaskAreaSideX, m_fTaskAreaSideY, fHeight, unDemand, m_unNextTaskId);

    // m_mapTaskPos[m_unNextTaskId] = {cCenter, fRadius};

    /* Update task count */
    m_unNextTaskId++;

    LOG << "[LOG] Added task" << std::endl;
}

/****************************************/
/****************************************/

// void CExperimentLoopFunctionsNop::AssignTasks() {
       
//     /* Sort tasks according to their angle */
//     std::vector<std::pair<UInt32, CVector2>> v;
//     for(const auto& [id, pair] : m_mapTaskPos) {
//         v.push_back({id, pair.first});
//     }

//     std::sort(v.begin(), v.end(), [](auto &left, auto &right) {
//         Real lv = left.second.Angle().GetValue();
//         Real rv = right.second.Angle().GetValue();
//         if(lv < 0)
//             lv += 2*ARGOS_PI;
//         if(rv < 0)
//             rv += 2*ARGOS_PI;
//         return lv < rv;
//     });

//     /* Create a sequence of leader names to use to assign tasks */
//     std::vector<std::string> leaders;
//     std::ostringstream cId;
    
//     size_t startLeaderId;
//     if(m_bNoDemandTasks) {
//         startLeaderId = 2;
//     } else {
//         startLeaderId = 1;
//     }

//     for(size_t i = startLeaderId; i <= m_unTotalLeaders; i++) {
//         cId.str("");
//         cId << "L" << i;
//         leaders.push_back(cId.str());
//     }

//     // for(const auto& t : v) {
//     //     LOG << t.first << ", " << t.second.Angle() << std::endl;
//     // }

//     /* Find the task with the smallest angle for L1 from the arena centre */
//     CEntity& cEntity = GetSpace().GetEntity(leaders[0]);
//     CEPuckLeaderEntity* cEPuckLeader = dynamic_cast<CEPuckLeaderEntity*>(&cEntity);
//     CVector2 L1Angle = CVector2(cEPuckLeader->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
//                                 cEPuckLeader->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
//     Real firstTask = (v.front().second.Angle() - L1Angle.Angle()).GetAbsoluteValue(); // first task in the vector
//     Real lastTask = (v.back().second.Angle() - L1Angle.Angle()).GetAbsoluteValue(); // last task in the vector
//     // std::cout << firstTask << " VS " << lastTask << std::endl;
//     if(lastTask < firstTask) {
//         // Put the last task to first
//         auto tmp = v.back();
//         v.pop_back();
//         v.insert(v.begin(), tmp);
//         LOG << "Moved last task in the vector to the front" << std::endl;
//     }

//     for(const auto& t : v) {
//         LOG << t.first << ", " << t.second.Angle() << std::endl;
//     }

//     UInt32 count = 0;
// }

// /****************************************/
// /****************************************/

void CExperimentLoopFunctionsNop::PlaceRobots(const CVector2& c_min,
                                            const CVector2& c_max,
                                            UInt32 un_robots,
                                            std::string str_controller_type) {

    try {

        /* Position */
        CRange<Real> cXRange = CRange<Real>(c_min.GetX(), c_max.GetX());
        CRange<Real> cYRange = CRange<Real>(c_min.GetY(), c_max.GetY());
        /* Orientation */
        CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);

        /* Place robots */
        UInt32 unTrials;

        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;

        UInt32 number_of_trials = 0;

        /* Determine controller */
        std::string strController;
        if(str_controller_type == "worker") {
            strController = WO_CONTROLLER;
        } else if(str_controller_type == "worker_mc") {
            strController = WOMC_CONTROLLER;
        } else if(str_controller_type == "charger") {
            strController = CH_CONTROLLER;
        } else {
            LOG << "[WARNING] Unknown worker type. Using default type: 'worker'" << std::endl;
            strController = WO_CONTROLLER;
        }

        if(str_controller_type == "worker" || str_controller_type == "worker_mc") {
            CEPuckEntity* pcEP;
            /* For each robot worker */
            for(size_t i = 0; i < un_robots; ++i) {
                /* Make the id */
                cEPId.str("");
                cEPId << "F" << (i + 1);
                /* Create the robot in the origin and add it to ARGoS space */
                pcEP = new CEPuckEntity(cEPId.str(),
                                        strController,
                                        CVector3(),
                                        CQuaternion(),
                                        EP_RAB_RANGE,
                                        MESSAGE_BYTE_SIZE,
                                        "");
                AddEntity(*pcEP);
                m_vecEntityID.push_back(cEPId.str());

                /* Set custom battery model */
                CBatteryEquippedEntity& cBattery = pcEP->GetBatterySensorEquippedEntity();
                cBattery.SetFullCharge(m_fFullChargeWorker);
                // Find a random number between the two values in m_fStartChargeWorker first and second
                Real fEnergyVariation = m_pcRNG->Uniform(CRange<Real>(m_fStartChargeWorker.first, m_fStartChargeWorker.second));
                cBattery.SetAvailableCharge(fEnergyVariation);
                // if(m_strBatteryDischargeModel == "fixed_time_motion") {
                //     CBatteryDischargeModelFixedTimeMotion* pcDischargeModel = new CBatteryDischargeModelFixedTimeMotion(m_fDeltaTime, m_fDeltaPosWorker);
                //     cBattery.SetDischargeModel(pcDischargeModel);
                // } else {
                //     LOG << "[WARNING] Unknown battery discharge model. Using default." << std::endl;
                // }
                
                m_mapEnergyConsumed[cEPId.str()] = 0.0f;
                m_mapEnergyConsumedToWork[cEPId.str()] = 0.0f;

                CWorker* cfController = dynamic_cast<CWorker*>(&pcEP->GetControllableEntity().GetController());
                cfController->SetMoveDischargeRate(m_fDeltaPosWorker, m_fFullChargeWorker);

                /* Try to place it in the arena */
                unTrials = 0;
                bool bDone;
                do {
                    /* Choose a random position */
                    ++unTrials;
                    cEPPos.Set(m_pcRNG->Uniform(cXRange),
                            m_pcRNG->Uniform(cYRange),
                            0.0f);      
                    cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CVector3::Z);
                    bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);

                } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
                if(!bDone) {
                    THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
                }
            }
        } else if(str_controller_type == "charger") {
            CEPuckChargerEntity* pcEP;

            /* For each charger */
            for(size_t i = 0; i < un_robots; ++i) {
                /* Make the id */
                cEPId.str("");
                cEPId << "C" << (i + 1);
                /* Create the robot in the origin and add it to ARGoS space */
                pcEP = new CEPuckChargerEntity(cEPId.str(),
                                        strController,
                                        CVector3(),
                                        CQuaternion(),
                                        EP_RAB_RANGE,
                                        MESSAGE_BYTE_SIZE,
                                        "");
                AddEntity(*pcEP);
                m_vecEntityID.push_back(cEPId.str());

                /* Set custom battery model */
                CBatteryEquippedEntity& cBattery = pcEP->GetBatterySensorEquippedEntity();
                cBattery.SetFullCharge(m_fFullChargeCharger);
                // Find a random number between the two values in m_fStartChargeCharger first and second
                Real fEnergyVariation = m_pcRNG->Uniform(CRange<Real>(m_fStartChargeCharger.first, m_fStartChargeCharger.second));
                cBattery.SetAvailableCharge(fEnergyVariation);
                // if(m_strBatteryDischargeModel == "fixed_time_motion") {
                //     CBatteryDischargeModelFixedTimeMotion* pcDischargeModel = new CBatteryDischargeModelFixedTimeMotion(m_fDeltaTime, m_fDeltaPosCharger);
                //     cBattery.SetDischargeModel(pcDischargeModel);
                // } else {
                //     LOG << "[WARNING] Unknown battery discharge model. Using default." << std::endl;
                // }
                
                m_mapEnergyConsumed[cEPId.str()] = 0.0f;
                m_mapEnergyConsumedToWork[cEPId.str()] = 0.0f;

                CCharger* cfController = dynamic_cast<CCharger*>(&pcEP->GetControllableEntity().GetController());
                cfController->SetMoveDischargeRate(m_fDeltaPosCharger, m_fFullChargeWorker, m_fFullChargeCharger);

                /* Try to place it in the arena */
                unTrials = 0;
                bool bDone;
                do {
                    /* Choose a random position */
                    ++unTrials;
                    cEPPos.Set(m_pcRNG->Uniform(cXRange),
                            m_pcRNG->Uniform(cYRange),
                            0.0f);      
                    cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                        CVector3::Z);
                    bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);

                } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
                if(!bDone) {
                    THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
                }
            }
        }

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
    }
}

// /****************************************/
// /****************************************/

// void CExperimentLoopFunctionsNop::PlaceCluster(const CVector2& c_center,
//                                             UInt32 un_leaders,
//                                             UInt32 un_robots,
//                                             UInt32 un_chargers,
//                                             Real f_density,
//                                             std::string str_worker_type,
//                                             UInt32 un_leader_id_start,
//                                             UInt32 un_robot_id_start) {

//     try {
//         /* Calculate side of the region in which the robots are scattered (square) */
//         // Real fHalfSide = Sqrt((EP_AREA * (un_leaders + un_robots)) / f_density) / 2.0f;
//         // CRange<Real> cAreaRange(-fHalfSide, fHalfSide);
//         // std::cout << "half: " << fHalfSide << std::endl;

//         /* Calculate radius of the region in which the robots are scattered (circle) */
//         // Real fRadius = Sqrt((EP_AREA * (un_leaders + un_robots + un_chargers)) / (f_density * ARGOS_PI));
//         // m_fTeamDeployRadius = fRadius;

//         // CRange<Real> cLengthRange(0, fRadius);
//         // CRange<Real> cAngleRange = CRange<Real>(-ARGOS_PI, ARGOS_PI);
//         // std::cout << "radius: " << fRadius << std::endl;

//         CRange<Real> cLengthRange = CRange<Real>(0, 1); 
//         CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);

//         /* Place robots */
//         UInt32 unTrials;
//         CEPuckLeaderEntity* pcEPL;
//         CEPuckEntity* pcEP;
//         CEPuckChargerEntity* pcEPC;
//         std::ostringstream cEPId;
//         CVector3 cEPPos;
//         CQuaternion cEPRot;
//         std::string leaderID = "";

//         UInt32 number_of_trials = 0;

//         /* Determine worker type */
//         std::string strController;
//         if(str_worker_type == "worker") {
//             strController = WO_CONTROLLER;
//         } else if(str_worker_type == "worker_mc") {
//             strController = WOMC_CONTROLLER;
//         } else {
//             LOG << "[WARNING] Unknown worker type. Using default type: 'worker'" << std::endl;
//             strController = WO_CONTROLLER;
//         }

//         /* For each robot worker */
//         for(size_t i = 0; i < un_robots; ++i) {
//             /* Make the id */
//             cEPId.str("");
//             cEPId << "F" << (i + un_robot_id_start);
//             /* Create the robot in the origin and add it to ARGoS space */
//             pcEP = new CEPuckEntity(cEPId.str(),
//                                     strController,
//                                     CVector3(),
//                                     CQuaternion(),
//                                     EP_RAB_RANGE,
//                                     MESSAGE_BYTE_SIZE,
//                                     "");
//             AddEntity(*pcEP);
//             m_vecEntityID.push_back(cEPId.str());

//             /* Set custom battery model */
//             CBatteryEquippedEntity& cBattery = pcEP->GetBatterySensorEquippedEntity();
//             cBattery.SetFullCharge(m_fFullChargeWorker);
//             // Find a random number between the two values in m_fStartChargeWorker first and second
//             Real fEnergyVariation = m_pcRNG->Uniform(CRange<Real>(m_fStartChargeWorker.first, m_fStartChargeWorker.second));
//             cBattery.SetAvailableCharge(fEnergyVariation);
//             if(m_strBatteryDischargeModel == "fixed_time_motion") {
//                 CBatteryDischargeModelFixedTimeMotion* pcDischargeModel = new CBatteryDischargeModelFixedTimeMotion(m_fDeltaTime, m_fDeltaPosWorker);
//                 cBattery.SetDischargeModel(pcDischargeModel);
//             } else {
//                 LOG << "[WARNING] Unknown battery discharge model. Using default." << std::endl;
//             }
            
//             m_mapEnergyConsumed[cEPId.str()] = 0.0f;
//             m_mapEnergyConsumedToWork[cEPId.str()] = 0.0f;
//             m_mapEnergyConsumedByConnectors[cEPId.str()] = 0.0f;

//             CWorker* cfController = dynamic_cast<CWorker*>(&pcEP->GetControllableEntity().GetController());
//             cfController->SetDischargeRate(m_fDeltaTime, m_fDeltaPosWorker, m_fFullChargeWorker);

//             /* Try to place it in the arena */
//             unTrials = 0;
//             bool bDone;
//             do {
//                 /* Choose a random position */
//                 ++unTrials;
//                 // cEPPos.Set(m_pcRNG->Uniform(cAreaRange) + c_center.GetX(),
//                 //            m_pcRNG->Uniform(cAreaRange) + c_center.GetY(),
//                 //            0.0f);
//                 // CVector2 posInPlane;
//                 // if(i + un_robot_id_start == 1) {
//                 //     posInPlane = CVector2(m_pcRNG->Uniform(cLengthRangeInitConnector),
//                 //                           CRadians(m_pcRNG->Uniform(cAngleRange)));
//                 // } else {
//                 //     posInPlane = CVector2(m_pcRNG->Uniform(cLengthRange),
//                 //                           CRadians(m_pcRNG->Uniform(cAngleRange)));
//                 // }
//                 // cEPPos.Set(posInPlane.GetX() + c_center.GetX(),
//                 //            posInPlane.GetY() + c_center.GetY(),
//                 //            0.0f);    
//                 Real radius = m_fTeamDeployRadius * Sqrt(m_pcRNG->Uniform(cLengthRange));
//                 CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
//                 CVector2 randPos = CVector2(radius, angle);      
//                 cEPPos.Set(randPos.GetX() + c_center.GetX(),
//                            randPos.GetY() + c_center.GetY(),
//                            0.0f);      
//                 cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
//                                      CVector3::Z);
//                 bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);
//                                 // LOG << "trials " << unTrials << std::endl;

//             } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
//             if(!bDone) {
//                 THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
//             }
//         }

//         /* Get entity with the name "F1" and cast it as an epuckentity */
//         if(m_bNoDemandTasks) {
//             if(un_leader_id_start == 2) {
//                 LOG << "Moving F1, next leader " << un_leader_id_start << std::endl;
//                 CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity(std::string("F1")));
//                 // move to 0.15,0,0
//                 cEPPos.Set(0.15, 0, 0);
//                 // cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE), CVector3::Z);
//                 MoveEntity(cEntity.GetEmbodiedEntity(), cEPPos, cEPRot);
//             }
//         } else {
//             if(un_leader_id_start == 1) {
//                 LOG << "Moving F1, next leader " << un_leader_id_start << std::endl;
//                 CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity(std::string("F1")));
//                 // move to 0.15,0,0
//                 cEPPos.Set(0, 0, 0);
//                 // cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE), CVector3::Z);
//                 MoveEntity(cEntity.GetEmbodiedEntity(), cEPPos, cEPRot);
//             }
//         }

//     } catch(CARGoSException& ex) {
//         THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
//     }
// }

// /****************************************/
// /****************************************/

// void CExperimentLoopFunctionsNop::PlaceMobileCharger(const CVector2& c_center,
//                                                      UInt32 un_charger_id_start) {

//     try {
//         /* Place robots */
//         UInt32 unTrials;
//         CEPuckChargerEntity* pcEPC;
//         std::ostringstream cEPId;
//         CVector3 cEPPos;
//         CQuaternion cEPRot;

//         /* Make the id */
//         cEPId.str("");
//         cEPId << "C" << un_charger_id_start;
//         /* Create the charger and add it to ARGoS space */
//         pcEPC = new CEPuckChargerEntity(cEPId.str(),
//                                         CH_CONTROLLER,
//                                         CVector3(0,0,0),
//                                         CQuaternion(),
//                                         EP_RAB_RANGE,
//                                         MESSAGE_BYTE_SIZE,
//                                         "");
//         AddEntity(*pcEPC);
//         m_vecEntityID.push_back(cEPId.str());

//         /* Assign initial number of followers */
//         CCharger* cfController = dynamic_cast<CCharger*>(&pcEPC->GetControllableEntity().GetController());
//         // TEMP hard-coded teamID
//         cfController->SetDischargeRate(m_fDeltaTime, m_fDeltaPosCharger, m_fFullChargeWorker, m_fFullChargeWorker);

//         /* Set custom battery model */
//         CBatteryEquippedEntity& cBattery = pcEPC->GetBatterySensorEquippedEntity();
//         cBattery.SetFullCharge(m_fFullChargeCharger);
//         Real fEnergyVariation = m_pcRNG->Uniform(CRange<Real>(m_fStartChargeCharger.first, m_fStartChargeCharger.second));
//         cBattery.SetAvailableCharge(fEnergyVariation);
//         if(m_strBatteryDischargeModel == "fixed_time_motion") {
//             CBatteryDischargeModelFixedTimeMotion* pcDischargeModel = new CBatteryDischargeModelFixedTimeMotion(m_fDeltaTime, m_fDeltaPosCharger);
//             cBattery.SetDischargeModel(pcDischargeModel);
//         } else {
//             LOG << "[WARNING] Unknown battery discharge model. Using default." << std::endl;
//         }

//         /* Try to place it in the arena */
//         bool bDone = false;
//         /* Place on specified position */
//         cEPPos.Set(c_center.GetX(),
//                     c_center.GetY(),
//                     0.0f);
//         cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
//                                 CVector3::Z);
//         bDone = MoveEntity(pcEPC->GetEmbodiedEntity(), cEPPos, cEPRot);

//         LOG << "Successfully placed mobile charger? = " << bDone << std::endl;

//     } catch(CARGoSException& ex) {
//         THROW_ARGOSEXCEPTION_NESTED("While placing a mobile charger", ex);
//     }
// }

// /****************************************/
// /****************************************/

// void CExperimentLoopFunctionsNop::PlaceMobileChargers(const CVector2& c_center,
//                                                     UInt32 un_chargers,
//                                                     UInt32 un_leader_id,
//                                                     Real f_density,
//                                                     UInt32 un_charger_id_start) {

//     try {

//         CRange<Real> cLengthRange = CRange<Real>(0, 1); 
//         CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);

//         /* F1 range (to deploy F1 near the center) */
//         // CRange<Real> cLengthRangeInitConnector(0, 0.1);

//         /* Place robots */
//         UInt32 unTrials;
//         CEPuckLeaderEntity* pcEPL;
//         CEPuckEntity* pcEP;
//         CEPuckChargerEntity* pcEPC;
//         std::ostringstream cEPId;
//         CVector3 cEPPos;
//         CQuaternion cEPRot;
//         std::string leaderID = "";

//         UInt32 number_of_trials = 0;

//         /* For each charger */
//         for(size_t i = 0; i < un_chargers; ++i) {
//             /* Make the id */
//             cEPId.str("");
//             cEPId << "C" << (i + un_charger_id_start);
//             /* Create the robot in the origin and add it to ARGoS space */
//             pcEPC = new CEPuckChargerEntity(cEPId.str(),
//                                             CH_CONTROLLER,
//                                             CVector3(),
//                                             CQuaternion(),
//                                             EP_RAB_RANGE,
//                                             MESSAGE_BYTE_SIZE,
//                                             "");
//             AddEntity(*pcEPC);
//             m_vecEntityID.push_back(cEPId.str());

//             /* Set custom battery model */
//             CBatteryEquippedEntity& cBattery = pcEPC->GetBatterySensorEquippedEntity();
//             // Adjust charger capacity
//             cBattery.SetFullCharge(m_fFullChargeCharger);
//             // Find a random number between the two values in m_fStartChargeCharger first and second
//             Real fEnergyVariation = m_pcRNG->Uniform(CRange<Real>(m_fStartChargeCharger.first, m_fStartChargeCharger.second));
//             cBattery.SetAvailableCharge(fEnergyVariation);
//             if(m_strBatteryDischargeModel == "fixed_time_motion") {
//                 CBatteryDischargeModelFixedTimeMotion* pcDischargeModel = new CBatteryDischargeModelFixedTimeMotion(m_fDeltaTime, m_fDeltaPosCharger);
//                 cBattery.SetDischargeModel(pcDischargeModel);
//             } else {
//                 LOG << "[WARNING] Unknown battery discharge model. Using default." << std::endl;
//             }
            
//             m_mapEnergyConsumed[cEPId.str()] = 0.0f;
//             m_mapEnergyConsumedToWork[cEPId.str()] = 0.0f;

//             CCharger* cfController = dynamic_cast<CCharger*>(&pcEPC->GetControllableEntity().GetController());
//             cfController->SetDischargeRate(m_fDeltaTime, m_fDeltaPosCharger, m_fFullChargeWorker, m_fFullChargeCharger);

//             /* Try to place it in the arena */
//             unTrials = 0;
//             bool bDone;
//             do {
//                 /* Choose a random position */
//                 ++unTrials;

//                 Real radius = m_fTeamDeployRadius * Sqrt(m_pcRNG->Uniform(cLengthRange));
//                 CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
//                 CVector2 randPos = CVector2(radius, angle);      
//                 cEPPos.Set(randPos.GetX() + c_center.GetX(),
//                            randPos.GetY() + c_center.GetY(),
//                            0.0f);      
//                 cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
//                                      CVector3::Z);
//                 bDone = MoveEntity(pcEPC->GetEmbodiedEntity(), cEPPos, cEPRot);
//                                 // LOG << "trials " << unTrials << std::endl;

//             } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
//             if(!bDone) {
//                 THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
//             }
//         }
//     } catch(CARGoSException& ex) {
//         THROW_ARGOSEXCEPTION_NESTED("While placing chargers in a cluster", ex);
//     }
// }

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceTask(const CVector2& c_center,
                                         Real f_radius,
                                         UInt32 un_demand,
                                         UInt32 un_min_robot_num,
                                         UInt32 un_max_robot_num,
                                         UInt32 un_task_id_start) {

    try {
        CCircleTaskEntity* pcCTS;
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        pcCTS = new CCircleTaskEntity(cTSId.str(),
                                      c_center,
                                      f_radius,
                                      un_demand,
                                      un_min_robot_num,
                                      un_max_robot_num);
        AddEntity(*pcCTS);
        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceCircleTask(const CVector2& c_center,
                                                  Real f_radius,
                                                  Real f_height,
                                                  UInt32 un_demand,
                                                  UInt32 un_min_robot_num,
                                                  UInt32 un_max_robot_num,
                                                  UInt32 un_task_id_start) {

    try {
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        if(m_bNoDemandTasks) {
            CCircleTaskNoDemandEntity* pcCTS;
            pcCTS = new CCircleTaskNoDemandEntity(cTSId.str(),
                                        c_center,
                                        f_radius,
                                        f_height);
            AddEntity(*pcCTS);

        } else {
            CCircleTaskEntity* pcCTS;
            pcCTS = new CCircleTaskEntity(cTSId.str(),
                                        c_center,
                                        f_radius,
                                        f_height,
                                        un_demand,
                                        un_min_robot_num,
                                        un_max_robot_num);
            AddEntity(*pcCTS);
        }

        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceRectangleTask(const CVector2& c_center,
                                                    Real f_width_x,
                                                    Real f_width_y,
                                                    Real f_height,
                                                    UInt32 un_demand,
                                                    UInt32 un_task_id_start) {

    try {
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        if(m_bNoDemandTasks) {
            CRectangleTaskNoDemandEntity* pcCTS;
            pcCTS = new CRectangleTaskNoDemandEntity(cTSId.str(),
                                        c_center,
                                        f_width_x,
                                        f_width_y,
                                        f_height);
            AddEntity(*pcCTS);

        } else {
            CRectangleTaskEntity* pcCTS;
            pcCTS = new CRectangleTaskEntity(cTSId.str(),
                                        c_center,
                                        f_width_x,
                                        f_width_y,
                                        f_height);
            AddEntity(*pcCTS);
        }

        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CExperimentLoopFunctionsNop, "experiment_loop_functions_nop")
