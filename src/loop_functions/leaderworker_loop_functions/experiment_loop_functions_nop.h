#ifndef EXPERIMENT_LOOP_FUNCTIONS_NOP_H
#define EXPERIMENT_LOOP_FUNCTIONS_NOP_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <argos3/plugins/simulator/entities/rectangle_task_entity.h>

#include <unordered_map>
#include <unordered_set>

using namespace argos;

class CExperimentLoopFunctionsNop : public CLoopFunctions {

public:

   CExperimentLoopFunctionsNop();
   virtual ~CExperimentLoopFunctionsNop() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual void PostStep();
   // virtual std::vector<CVector2> GetArenaSize() const;
   virtual bool IsDrawRobotLabel() const;
   virtual bool IsLogging();
   virtual std::string GetCommandFilePath();
   virtual std::unordered_map<std::string, UInt32> GetRobotPerTask();
   virtual UInt32 GetCurrentPoints();
   virtual std::string GetWorkerType() const;
   virtual std::string GetTaskType() const;

private:

   TConfigurationNode config;
   std::vector<std::string> m_vecEntityID;

   std::vector<CVector2> m_vecWaypointPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;
   UInt32 m_unTotalLeaders;
   UInt32 m_unTotalWorkers;
   UInt32 m_unTotalChargers;
   // std::vector<CVector2> m_vecArenaSize;
   bool m_bTaskExists;
   bool m_bTaskComplete;
   UInt32 m_unNextTaskId;
   UInt32 m_unTotalTasks;
   UInt32 m_unTaskDemand;
   UInt32 m_unPointsObtained;
   bool m_bNoDemandTasks;

   /* Energy */
   CVector2 m_cFixedChargePos, m_cMobileChargePos, m_cTaskPos;
   Real m_fFixedChargeAreaSideX, m_fFixedChargeAreaSideY;
   Real m_fMobileChargeAreaSideX, m_fMobileChargeAreaSideY;
   Real m_fTaskAreaSideX, m_fTaskAreaSideY;

   Real m_fEnergyConsumed;
   Real m_fEnergyConsumedToWork;
   Real m_fEnergyShared;
   Real m_fEnergyLost;
   std::unordered_set<std::string> m_setDepletedWorkers;
   std::unordered_set<std::string> m_setDepletedChargers;

   std::unordered_map<std::string,Real> m_mapCurrentEnergy;
   std::unordered_map<std::string,Real> m_mapEnergyConsumed;
   std::unordered_map<std::string,Real> m_mapEnergyConsumedToWork;

   std::string m_strWorkerType;
   std::string m_strChargerType;

   /* Charging area position */
   // CVector2 m_cChargingPos;
   // /* Charging area radius */
   // Real m_fChargingRadius;

   /* Battery model */
   std::string m_strBatteryDischargeModel;
   Real m_fFullChargeWorker;
   Real m_fFullChargeCharger;
   std::pair<Real,Real> m_fStartChargeWorker;
   std::pair<Real,Real> m_fStartChargeCharger;
   Real m_fDeltaTime;
   Real m_fDeltaPosWorker;
   Real m_fDeltaPosCharger;
   Real m_fDeltaWork;
   Real m_fDeltaRecharge;
   Real m_fDeltaTransferLoss;

   /* Temp stored variables to init chargers */
   std::map<UInt32,CVector2> m_vecTeamCenters;   // key = team_id, value = team_center
   std::map<UInt32,UInt32> m_vecNumChargers;     // key = team_id, value = num_chargers
   std::map<UInt32,Real> m_vecTeamDensity;       // key = team_id, value = team_density

   // std::vector<std::unordered_map<std::string,UInt32>> m_vecTaskDemand;
   std::map<UInt32, std::pair<CVector2, Real>> m_mapTaskPos; // position & radius
   /* Number of robots working on each task in the current timestep */
   std::unordered_map<std::string,UInt32> m_mapRobotPerTask;
   /* Map of whether a robot is performing a task in the current timestep */
   std::unordered_map<std::string,bool> m_mapRobotTaskStatus;

   CRange<Real> cArenaSideX = CRange<Real>(-1.45f, 1.45f);
   CRange<Real> cArenaSideY = CRange<Real>(-1.45f, 1.45f);

   /* Output file */
   bool m_bLogging;
   std::string m_strOutput;
   std::string m_strRunNumber;
   std::string m_strDirPath;
   std::string m_strBinaryFilePath;
   std::string m_strSummaryFilePath;
   std::string m_strCommandFilePath;
   std::fstream m_cOutput;

   /* Draw configurations */
   bool m_bDrawRobotLabel;

   /* Frame Grabbing */
   bool m_bFrameGrabbing;
   UInt32 m_unCameraIndex;

   /* robots that have collided */
   std::unordered_set<std::string> robotsCollided;
   UInt32 finishDelay;

   /* Init logging */
   void InitLogging();
   void InitLoggingEnergy();

   /* Init robots */
   void InitRobots();
   // void InitChargers();

   /* Init tasks */
   void InitTasks(); // Testing
   void InitTasksCircular(); // Scalability analysis experiment
   void InitTask();

   /* Assign tasks */
   // void AssignTasks();

   // /* Distribute a leader-robot team */
   // void PlaceCluster(const CVector2& c_center,
   //                   UInt32 un_leaders,
   //                   UInt32 un_robots,
   //                   UInt32 un_chargers,
   //                   Real f_density,
   //                   std::string str_worker_type,
   //                   UInt32 un_leader_id_start,
   //                   UInt32 un_robot_id_start);

   // void PlaceMobileCharger(const CVector2& c_center,
   //                         UInt32 un_charger_id_start);

   // void PlaceMobileChargers(const CVector2& c_center,
   //                          UInt32 un_chargers,
   //                          UInt32 un_team,
   //                          Real f_density,
   //                          UInt32 un_charger_id_start);

   void PlaceRobots(const CVector2& c_min,
                     const CVector2& c_max,
                     UInt32 un_robots,
                     std::string str_worker_type);

   /* Place a task */
   void PlaceTask(const CVector2& c_center,
                  Real f_radius,
                  UInt32 un_demand,
                  UInt32 un_min_robot_num,
                  UInt32 un_max_robot_num,
                  UInt32 un_task_id_start);

   /* Place a task */
   void PlaceCircleTask(const CVector2& c_center,
                        Real f_radius,
                        Real f_height,
                        UInt32 un_demand,
                        UInt32 un_min_robot_num,
                        UInt32 un_max_robot_num,
                        UInt32 un_task_id_start);

   /* Place a task */
   void PlaceRectangleTask(const CVector2& c_center,
                        Real f_width_x,
                        Real f_width_y,
                        Real f_height,
                        UInt32 un_demand,
                        UInt32 un_task_id_start);
};

#endif
