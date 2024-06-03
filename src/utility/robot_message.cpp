#include "robot_message.h"

/****************************************/
/****************************************/

Message::Message() {

}

/****************************************/
/****************************************/

Message::Message(CCI_RangeAndBearingSensor::SPacket packet) {

    size_t index = 0;

    /* Core */
    direction = CVector2(packet.Range, packet.HorizontalBearing);
    state = static_cast<RobotState>(packet.Data[index++]);
    ID = std::to_string(packet.Data[index++]); // Only stores number part of the id here

    /* Request Energy */
    if(packet.Data[index] != 255) {
        emsg.requestingEnergy = (bool)packet.Data[index++];
        emsg.energyLevel = (char)packet.Data[index++]; 
    } else
        index += 2;

    // if(packet.Data[index] != 255) {
    //     std::string robotID;
    //     robotID += (char)packet.Data[index++];            // First char of ID
    //     robotID += std::to_string(packet.Data[index++]);  // ID number
    //     emsg.owner = robotID;
    // } else
    //     index += 2;

    // if(packet.Data[index] != 255)
    //     emsg.state = static_cast<RobotState>(packet.Data[index++]);
    // else
    //     index++;

    if(packet.Data[index] != 255) {
        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        emsg.from = robotID;
    } else
        index += 2;

    if(packet.Data[index] != 255) {
        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        emsg.to = robotID;
    } else
        index += 2;

}

/****************************************/
/****************************************/

Message::~Message() {}

/****************************************/
/****************************************/

CByteArray Message::GetCByteArray() {
    // std::cout << "---start--- \t\t\tstate: " << int(state) << " ID: " << ID << std::endl;

    CByteArray arr = CByteArray(MESSAGE_BYTE_SIZE, 255);
    size_t index = 0;

    /* Sender State */
    arr[index++] = static_cast<UInt8>(state);

    /* Sender ID */
    arr[index++] = stoi(ID.substr(1));

    /* Request Energy */
    arr[index++] = (UInt8)emsg.requestingEnergy;
    arr[index++] = (UInt8)emsg.energyLevel;

    // if( emsg.owner.empty() )
    //     index += 2; // Skip
    // else {
    //     arr[index++] = emsg.owner[0];              // ID
    //     arr[index++] = stoi(emsg.owner.substr(1)); // ID
    // }

    // arr[index++] = static_cast<UInt8>(emsg.state);

    if( emsg.from.empty() )
        index += 2; // Skip
    else {
        arr[index++] = emsg.from[0];              // ID
        arr[index++] = stoi(emsg.from.substr(1)); // ID
    }

    if( emsg.to.empty() )
        index += 2; // Skip
    else {
        arr[index++] = emsg.to[0];              // ID
        arr[index++] = stoi(emsg.to.substr(1)); // ID
    }

    return arr;
}

/****************************************/
/****************************************/

/* 
* Checks whether the Message is empty or not by checking the direction it was received from
*/
bool Message::Empty() {
    return direction.Length() == 0.0f;
}

/****************************************/
/****************************************/

void Message::Print() const {

    std::cout << "\n##########" << std::endl;
    
    switch(state) {
        case RobotState::WORKER:
            std::cout << "state: WORKER" << std::endl;
            break;
        case RobotState::CHARGER:
            std::cout << "state: CHARGER" << std::endl;
            break;
        default:
            /* The message is not initialised */
            if(int(state) != 255) {
                std::cerr << "Unknown state " << int(state) << " in " << ID << std::endl;
            }
            break;
    }

    std::cout << "ID: " << ID << std::endl;

    std::cout << "requestingEnergy: " << emsg.requestingEnergy << std::endl;
    std::cout << "energyLevel: " << emsg.energyLevel << std::endl;
    // std::cout << "owner: " << emsg.owner << std::endl;
    // if( !emsg.owner.empty() ) {
    //     switch(emsg.state) {
    //         case RobotState::WORKER:
    //             std::cout << "state: WORKER" << std::endl;
    //             break;
    //         case RobotState::CHARGER:
    //             std::cout << "state: CHARGER" << std::endl;
    //             break;
    //         default:
    //             /* The message is not initialised */
    //             if(int(state) != 255) {
    //                 std::cerr << "Unknown state " << int(state) << " in " << ID << std::endl;
    //             }
    //             break;
    //     }
    // } else
    //     std::cout << "state: " << std::endl;
    std::cout << "energyFrom: " << emsg.from << std::endl;
    std::cout << "energyTo: " << emsg.to << std::endl;

    std::cout << std::endl;


}
