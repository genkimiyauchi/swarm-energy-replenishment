/*
 * AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 *
 * An example controller for running SCT with the e-puck.
 *
 * The controller uses the supervisors generated in Nadzoru to 
 * determine its next action in each timestep.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/leader_worker.argos
 * 
 * This example has been modified from the following examples provided in argos3-examples: https://github.com/ilpincy/argos3-examples/
 *   - argos3-examples/controllers/epuck_obstacleavoidance/
 *   - argos3-examples/controllers/footbot_flocking/
 */

#ifndef WORKER_H
#define WORKER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the ground sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
/* Definition of the batter sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/* SCT generator player */
#include <utility/sct.h>
/* Message structure */
#include <utility/robot_message.h>

#include <set>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CWorker : public CCI_Controller {

public:

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
    struct SWheelTurningParams {
        /*
        * The turning mechanism.
        * The robot can be in three different turning states.
        */
        enum ETurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
        * Angular thresholds to change turning state.
        */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><charger_controller><parameters><team_flocking>
    * section.
    */
    struct SFlockingInteractionParams {
        /* Target robot-robot distance in cm */
        Real TargetDistance;
        /* Gain of the Lennard-Jones potential */
        Real Gain;
        /* Exponent of the Lennard-Jones potential */
        Real Exponent;

        void Init(TConfigurationNode& t_node);
        // Real GeneralizedLennardJones(Real f_distance);
        Real GeneralizedLennardJonesRepulsion(Real f_distance);
    };

    /* List of move types available to the robot */
    enum class MoveType {
        MOVE_TO_WORK = 0,
        MOVE_TO_CHARGE,   
    } currentMoveType;

public:

    /* Class constructor. */
    CWorker();

    /* Class destructor. */
    virtual ~CWorker();

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /*
    * Set LED.
    */
    virtual void SetLED(const CColor& c_color);

    /*
    * Get robot state.
    */
    virtual RobotState GetRobotState() const;

    /*
    * Get move type.
    */
    virtual std::string GetMoveType() const;

    /*
    * Return whether the robot is working on a task.
    */
    virtual bool IsWorking();

    /*
    * Returns the last action.
    */
    virtual std::string GetLastAction() const;

    /*
    * Get the current robot energy
    */
    virtual Real GetCurrentEnergy() const;

    /*
    * Returns true if the robot is moving
    */
    virtual bool IsMoving() const;

    /*
    * Returns true if the robot is activating is charging capability
    */
    virtual bool IsCharging() const;

    /*
    * Returns true if the robot is sharing energy
    */
    virtual bool IsSharingEnergy() const;

    /*
    * Get the ID of the robot it is sharing energy to
    */
    virtual std::string GetEnergyTo() const;

    /*
    * Get the distance the robot can share energy to
    */
    virtual Real GetDistToShareEnergy() const;

    /* 
    * Set energy discharge rates
    */
    virtual void SetMoveDischargeRate(Real fDeltaPos, Real fMaxCapacity);

protected:

    /*
    * Reset variables
    */
    virtual void ResetVariables();

    /* 
    * Receive messages from neighboring robots.
    */
    virtual void GetMessages();

    /* 
    * Update sensor readings.
    */
    virtual void Update();

    /* 
    * Get a repulsion vector between itself and all other robots.
    */
    virtual CVector2 GetRobotRepulsionVector(std::vector<Message>& msgs);

    /*
    * Get a repulsion vector from obstacles.
    */
    virtual CVector2 GetObstacleRepulsionVector();

    /*
    * Move wheels to travel
    */
    virtual void Travel();

    /*
    * Get a vector to travel along the chain.
    */
    virtual CVector2 GetTravelVector();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
    virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Print robot id.
    */
    virtual void PrintName();

    /* Callback functions */
    virtual void Callback_MoveToWork(void* data);
    virtual void Callback_MoveToCharge(void* data);
    virtual void Callback_Work(void* data);
    virtual void Callback_Charge(void* data);

    virtual unsigned char Check_AtWork(void* data);
    virtual unsigned char Check_NotAtWork(void* data);
    virtual unsigned char Check_AtCharger(void* data);
    virtual unsigned char Check_NotAtCharger(void* data);
    virtual unsigned char Check_LowEnergy(void* data);
    virtual unsigned char Check_HighEnergy(void* data);

protected:

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the e-puck proximity sensor */
    CCI_ProximitySensor* m_pcProximity;
    /* Pointer to the range-and-bearing actuator */
    CCI_RangeAndBearingActuator* m_pcRABAct;
    /* Pointer to the range-and-bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABSens;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;
    /* Pointer to the ground sensor */
    CCI_GroundSensor* m_pcGround;
    /* Pointer to the battery sensor */
    CCI_BatterySensor* m_pcBattery;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;

    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;
    /* The flocking interaction parameters between teammates. */
    SFlockingInteractionParams m_sTeamFlockingParams;

    /* Weights using for the motion in each role */
    Real travelerAttraction, travelerRepulsion, travelerObstacle;

    /* Controller */
    std::unique_ptr<SCT> sct;

    /* Last controllable action */
    std::string lastControllableAction;

    /* Current robot state */
    RobotState currentState;

    /* Outgoing message */
    CByteArray cbyte_msg;

    /* Messages received from nearby robots */
    std::vector<Message> workerMsgs;
    std::vector<Message> chargerMsgs;

    /* Flag to indicate whether this robot is working on a task */
    bool bPerformingTask;

    /* Timer to count the timesteps for the initial communication to occur at the beginning of the simulation */
    size_t initStepTimer;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */

    /* SCT yaml path */
    std::string m_strSCTPath;

    /* 
    * Energy related variables
    */
    Real fEnergy;
    Real fEnergyLowThres;
    Real fEnergyHighThres;
    bool bMoving;
    bool bCharging;
    // static constexpr UInt8 chargeAreaID = 1; // Team ID of the charging area (hard-coded to travel to team 1)

    /* Sharing energy */
    bool bSharingEnergy;
    bool bRequestingEnergy;
    std::string strEnergyFrom; // used in the CONNECTOR state
    bool bOtherLowEnergy; // used in the TRAVELER state
    bool bAgreedToShareEnergy; // used in the TRAVELER state
    std::string strEnergyTo; // used in the TRAVELER state
    Real fDistSE; // Distance to the robot it will share energy to (used in the TRAVELER state) in cm
    Real fTargetDistSE; // in cm

    /* Energy discharge rates */
    Real m_fDeltaPos;
    Real m_fDistToMC; // Distance to the mobile charger (used in the follower state) in cm
    Real m_fDistToCharger;
    Real m_fDesiredAngleOffset;
};

/* Create CWorkerMC which extends from CWorker */
class CWorkerMC : public CWorker {
public:
    CWorkerMC() : CWorker() {}
    virtual ~CWorkerMC() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

protected:

    /* 
    * Update sensor readings.
    */
    virtual void Update();

    /*
    * Move wheels to travel
    */
    virtual void Travel();

    /*
    * Get a vector to travel along the chain.
    */
    virtual CVector2 GetTravelVector();

    /*
    * Get a vector that attracts itself to the connector.
    */
    virtual CVector2 GetApproachToShareEnergyVector(Message& msg);

    /*
    * Check whether any robot is approaching to share energy
    */
    virtual void CheckEnergyProvider();

protected:

    /* Callback functions */
    virtual void Callback_MoveToWork(void* data);
    virtual void Callback_MoveToCharge(void* data);
    virtual void Callback_Work(void* data);
    virtual void Callback_Charge(void* data);

    virtual unsigned char Check_AtWork(void* data);
    virtual unsigned char Check_NotAtWork(void* data);
    virtual unsigned char Check_AtCharger(void* data);
    virtual unsigned char Check_NotAtCharger(void* data);
    virtual unsigned char Check_LowEnergy(void* data);
    // virtual unsigned char Check_MidEnergy(void* data);
    virtual unsigned char Check_HighEnergy(void* data);
    // virtual unsigned char Check_FoundCharger(void* data);
    // virtual unsigned char Check_NotFoundCharger(void* data);

};

#endif
