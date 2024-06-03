/*
* AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
* 
* Define the message structure used to communicate between the robots.
*/

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/*
 * Include some necessary headers.
 */

/* Definition of the CVector2 datatype */
#include <argos3/core/utility/math/vector2.h>

/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <map>
#include <unordered_map>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/* List of states */
enum class RobotState {
    WORKER = 0,
    CHARGER
};

/* Structure to store request/response message to receive/send energy between robots */
struct EnergyMsg {
    bool requestingEnergy = false;  // Whether the robot is requesting for energy
    char energyLevel = 'M';         // L (low) or M (medium) or H (high) or D (dead) energy level
    // std::string owner = "";         // The ID of the robot sending the message
    // RobotState state;               // The state of the robot sending the message
    std::string from = "";          // The ID of the robot that it is receiving the energy from
    std::string to = "";            // The ID of the robot that it is sending the energy to
};

/*
* Communication buffer size
*/
static const UInt32 MESSAGE_BYTE_SIZE = 8;

/* 
* Structure to store incoming data received from other robots 
*/
class Message {

    public:

        /* Class constructor */
        Message();

        Message(CCI_RangeAndBearingSensor::SPacket packet);

        virtual ~Message();

        virtual CByteArray GetCByteArray();

        virtual bool Empty();

        virtual void Print() const;

    public:

        /* Core */
        CVector2 direction = CVector2();
        RobotState state;
        std::string ID; // Only store numberhere. Together with robot type, it must have the structure with a char, follower by a number (e.g. F1, L2)

        /* Energy Message */
        EnergyMsg emsg;

};

#endif