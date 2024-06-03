/* Include the controller definition */
#include "worker.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>

/****************************************/
/****************************************/

static const std::vector<CRadians> PROX_ANGLE {
                                                CRadians::PI / 10.5884f,
                                                CRadians::PI / 3.5999f,
                                                CRadians::PI_OVER_TWO,  // side sensor
                                                CRadians::PI / 1.2f,    // back sensor
                                                CRadians::PI / 0.8571f, // back sensor
                                                CRadians::PI / 0.6667f, // side sensor
                                                CRadians::PI / 0.5806f,
                                                CRadians::PI / 0.5247f
                                              };


/****************************************/
/****************************************/

void CWorker::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CWorker::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CWorker::SFlockingInteractionParams::GeneralizedLennardJonesRepulsion(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

CWorker::CWorker() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL) {}

/****************************************/
/****************************************/

CWorker::~CWorker() {}

/****************************************/
/****************************************/

void CWorker::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcGround    = GetSensor  <CCI_GroundSensor                >("ground"               );
    m_pcBattery   = GetSensor  <CCI_BatterySensor               >("battery"              );
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

        /* Flocking-related */
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));

        /* Weights using for the motion*/
        GetNodeAttribute(GetNode(t_node, "traveler"), "attract",  travelerAttraction);
        GetNodeAttribute(GetNode(t_node, "traveler"), "repulse",  travelerRepulsion);
        GetNodeAttribute(GetNode(t_node, "traveler"), "obstacle", travelerObstacle);

        /* SCT Model */
        GetNodeAttribute(GetNode(t_node, "SCT"), "path", m_strSCTPath);

        /* Energy */
        GetNodeAttribute(GetNode(t_node, "energy"), "low_thres", fEnergyLowThres);
        GetNodeAttribute(GetNode(t_node, "energy"), "high_thres", fEnergyHighThres);
        GetNodeAttribute(GetNode(t_node, "energy"), "share_dist", fTargetDistSE);

    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Initialization */
    initStepTimer = 0;
    currentState = RobotState::WORKER;
    currentMoveType = MoveType::MOVE_TO_CHARGE;
    bPerformingTask = false;
    bMoving = false;
    bCharging = false;
    bSharingEnergy = false;
    bRequestingEnergy = false;
    strEnergyFrom = "";
    strEnergyTo = "";
    fDistSE = 100000; // TEMP very large value

    m_fDeltaPos = 0.03; // TEMP value. Set using SetMoveDischargeRate()

    /*
    * Init SCT Controller
    */

    sct = std::make_unique<SCT>(m_strSCTPath);
    // RLOG << "Loading SCT" << std::endl;

    /* Register controllable events */
    sct->add_callback(this, std::string("EV_moveToWork"),   &CWorker::Callback_MoveToWork,   NULL, NULL);
    sct->add_callback(this, std::string("EV_moveToCharge"), &CWorker::Callback_MoveToCharge, NULL, NULL);
    sct->add_callback(this, std::string("EV_work"),         &CWorker::Callback_Work,         NULL, NULL);
    sct->add_callback(this, std::string("EV_charge"),       &CWorker::Callback_Charge,       NULL, NULL);

    /* Register uncontrollable events */
    sct->add_callback(this, std::string("EV_atWork"),       NULL, &CWorker::Check_AtWork,       NULL);
    sct->add_callback(this, std::string("EV_notAtWork"),    NULL, &CWorker::Check_NotAtWork,    NULL);
    sct->add_callback(this, std::string("EV_atCharger"),    NULL, &CWorker::Check_AtCharger,    NULL);
    sct->add_callback(this, std::string("EV_notAtCharger"), NULL, &CWorker::Check_NotAtCharger, NULL);
    sct->add_callback(this, std::string("EV_lowEnergy"),    NULL, &CWorker::Check_LowEnergy,    NULL);
    sct->add_callback(this, std::string("EV_highEnergy"),   NULL, &CWorker::Check_HighEnergy,   NULL);

    Reset();
}

/****************************************/
/****************************************/

void CWorker::Reset() {
    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    cbyte_msg = CByteArray(MESSAGE_BYTE_SIZE, 255);
    m_pcRABAct->SetData(cbyte_msg);
}

/****************************************/
/****************************************/

void CWorker::SetLED(const CColor& c_color) {
    m_pcLEDs->SetAllColors(c_color);
}

/****************************************/
/****************************************/

RobotState CWorker::GetRobotState() const {
    return currentState;
}

/****************************************/
/****************************************/

std::string CWorker::GetMoveType() const {
    switch(currentMoveType) {
        case MoveType::MOVE_TO_WORK:
            return "MOVE_TO_WORK";
        case MoveType::MOVE_TO_CHARGE:
            return "MOVE_TO_CHARGE";
    }
    return "INVALID";
}

/****************************************/
/****************************************/

bool CWorker::IsWorking() {
    return bPerformingTask;
}

/****************************************/
/****************************************/

std::string CWorker::GetLastAction() const {
    return lastControllableAction;
}

/****************************************/
/****************************************/

Real CWorker::GetCurrentEnergy() const {
    return fEnergy;
}

/****************************************/
/****************************************/

bool CWorker::IsMoving() const {
    return bMoving;
}

/****************************************/
/****************************************/

bool CWorker::IsCharging() const {
    return bCharging;
}

/****************************************/
/****************************************/

bool CWorker::IsSharingEnergy() const {
    return bSharingEnergy;
}

/****************************************/
/****************************************/

std::string CWorker::GetEnergyTo() const {
    return strEnergyTo;
}

/****************************************/
/****************************************/

Real CWorker::GetDistToShareEnergy() const {
    return fTargetDistSE;
}

/****************************************/
/****************************************/

void CWorker::SetMoveDischargeRate(Real fDeltaPos, Real fMaxCapacity) {
    /* Energy to move per timestep (0.1s) */
    // m_fDeltaHopTravel = m_unConnectorTargetDistance / m_sWheelTurningParams.MaxSpeed * ((fDeltaTime + fDeltaPos) * 10.0) / fMaxCapacity;
    m_fDeltaPos = fDeltaPos;
}

/****************************************/
/****************************************/

void CWorker::ControlStep() {

    std::string id = this->GetId();
    // LOG << "---------- " << id << " ----------" << std::endl;

    initStepTimer++;

    // // TEMP: force movetype to default to Adjust
    // if(initStepTimer == 1) {
    //     currentMoveType = MoveType::ADJUST;
    // }

    const CCI_BatterySensor::SReading& sBattery = m_pcBattery->GetReading();

    /* Update own energy */
    fEnergy = sBattery.AvailableCharge;

    // RLOG << "teamToMove = " << teamToMove << ", prevTeamID = " << prevTeamID << std::endl;
    // RLOG << "moveType = " << (UInt8)currentMoveType << std::endl;
    // CBatteryEquippedEntity& cBattery = c_entity.GetBatterySensorEquippedEntity();
    // Real robot_energy = cBattery.GetAvailableCharge();
    // if(hopsDict.count(1) > 0)
        // RLOG << "Hop to CA = " << hopsDict[1].count << std::endl;  

    /*-----------------*/
    /* Energy level */
    /*-----------------*/
    if(fEnergy <= 0) {
        // RLOG << "Energy depleted" << std::endl;
        m_pcLEDs->SetAllColors(CColor::BLACK);
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        bMoving = false;
        bPerformingTask = false;
        Message msg = Message();
        msg.ID = id;
        msg.emsg.energyLevel = 'D';
        cbyte_msg = msg.GetCByteArray();
        m_pcRABAct->SetData(cbyte_msg);
        return;
    }

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/
    ResetVariables();

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    Update();

    /*--------------------*/
    /* Run SCT controller */
    /*--------------------*/
    // RLOG << "--- Supervisors ---" << std::endl;

    if(initStepTimer > 4)
        sct->run_step();    // Run the supervisor to get the next action

    // RLOG << "Action: " << lastControllableAction << std::endl;

    // if(GetId() == "F14" || GetId() == "F15") {
        // RLOG << ", " << sct->get_current_state_string() << std::endl;
    // }

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    Message msg = Message();

    msg.state = currentState;
    msg.ID = id;

    // if(bCharging) {
    //     m_pcLEDs->SetAllColors(CColor::YELLOW);
    // } else {
    //     m_pcLEDs->SetAllColors(CColor::GREEN);
    // }

    // CColor cColor = CColor(0, 191, 0, 255); // GREEN
    // // GREEN --- ORANGE --- RED
    // UInt8 redValue = std::min(1.0, 2.0 - 2 * fEnergy) * 255;
    // UInt8 greenValue = std::min(1.0, fEnergy + 0.5) * 191;
    // cColor = CColor(redValue, greenValue, 0, 255);
    // // LOG << redValue << " " << greenValue << " " << robot_energy/cBattery.GetFullCharge() << std::endl;
    // m_pcLEDs->SetAllColors(cColor);

    // Real numLEDs = fEnergy / (1.0/8.0);
    // // RLOG << "numLEDs: " << numLEDs << std::endl;
    // for(size_t i = 0; i < m_pcLEDs->GetNumLEDs(); i++) {
    //     if(numLEDs > i)
    //         m_pcLEDs->SetSingleColor(i, CColor::GREEN);
    //     else
    //         m_pcLEDs->SetSingleColor(i, CColor::RED);
    // }

    if(Check_LowEnergy(nullptr)) {
        m_pcLEDs->SetAllColors(CColor::RED);
    } else if(fEnergy >= 0.5) {
        m_pcLEDs->SetAllColors(CColor::GREEN);
    } else {
        m_pcLEDs->SetAllColors(CColor::YELLOW);
    }

    /* Movement */
    // switch(currentMoveType) {
    //     case MoveType::MOVE_TO_WORK: {
    //         Travel();
    //         break;
    //     }
    //     case MoveType::MOVE_TO_CHARGE: {
    //         Travel();
    //         break;
    //     }
    // }
    Travel();

    // /* Energy Message */
    // EnergyMsg emsg = EnergyMsg();
    // emsg.requestingEnergy = bRequestingEnergy;
    // if(Check_LowEnergy(nullptr))
    //     emsg.energyLevel = 'L';
    // else if(Check_HighEnergy(nullptr))
    //     emsg.energyLevel = 'H';
    // emsg.owner = this->GetId();
    // emsg.state = currentState;
    // if(Check_HighEnergy(nullptr))
    //     strEnergyFrom = ""; // Reset to stop receiving energy
    // emsg.from = strEnergyFrom;
    // emsg.to = strEnergyTo;
    // msg.emsg = emsg;

    cbyte_msg = msg.GetCByteArray();

    /*--------------*/
    /* Send message */
    /*--------------*/

    m_pcRABAct->SetData(cbyte_msg);

}

/****************************************/
/****************************************/

void CWorker::ResetVariables() {
    /* Clear messages received */

    workerMsgs.clear();
    chargerMsgs.clear();

    /* Energy sharing */
    bOtherLowEnergy = false;
    bAgreedToShareEnergy = false;
    fDistSE = 100000;
    m_fDistToMC = 100000;

    lastControllableAction = "";
}

/****************************************/
/****************************************/

void CWorker::GetMessages() {

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            Message msg = Message(tMsgs[i]);

            /* Store message */
            if(msg.state == RobotState::WORKER) {
                msg.ID = 'F' + msg.ID;
                workerMsgs.push_back(msg);
            } else if(msg.state == RobotState::CHARGER) {
                msg.ID = 'C' + msg.ID;
                chargerMsgs.push_back(msg);
            }
        }
    }
}

/****************************************/
/****************************************/

void CWorker::Update() {

    /* Position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());

    /* Distance to charging area */
    Real fChargingAreaX = -0.5;
    m_fDistToCharger = pos2d.GetX() - fChargingAreaX;

}

/****************************************/
/****************************************/

CVector2 CWorker::GetRobotRepulsionVector(std::vector<Message>& msgs) {
    CVector2 resVec = CVector2();

    for(size_t i = 0; i < msgs.size(); i++) {
        /* Calculate LJ */
        Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(msgs[i].direction.Length());
        resVec += CVector2(fLJ,
                    msgs[i].direction.Angle());
    }

    /* Calculate the average vector */
    if( !msgs.empty() )
        resVec /= msgs.size();

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CWorker::GetObstacleRepulsionVector() {
    /* Get proximity sensor readings */
    std::vector<Real> fProxReads = m_pcProximity->GetReadings();

    CVector2 resVec = CVector2();

    for(size_t i = 0; i < fProxReads.size(); i++) {
        CVector2 vec = CVector2();
        if(fProxReads[i] > 0.0f) {
            Real distance = -( log(fProxReads[i]) / log(exp(1)) );
            Real length = (0.1 - distance) / 0.1 * m_sWheelTurningParams.MaxSpeed;
            vec = CVector2(length, PROX_ANGLE[i]);
            
            resVec -= vec; // Subtract because we want the vector to repulse from the obstacle
        }
    }

    resVec /= 8;

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CWorker::Travel() {

    /* Add robots to repel from */
    std::vector<Message> repulseMsgs;
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(workerMsgs), std::end(workerMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(chargerMsgs), std::end(chargerMsgs));

    /* Calculate overall force applied to the robot */
    CVector2 travelForce   = GetTravelVector();
    CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);
    CVector2 obstacleForce = GetObstacleRepulsionVector();

    CVector2 sumForce = travelerAttraction*travelForce + travelerRepulsion*robotForce + travelerObstacle*obstacleForce;

    // RLOG << "travelForce: " << travelForce << std::endl;
    // RLOG << "robotForce: " << robotForce.Length() << std::endl;
    // RLOG << "obstacleForce: " << obstacleForce.Length() << std::endl;
    // RLOG << "sumForce.Length: " << sumForce.Length() << std::endl;

    /* Set Wheel Speed */
    if(sumForce.Length() > 0.5f) {
        SetWheelSpeedsFromVector(sumForce);
        bMoving = true;
    } else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        bMoving = false;
    }
}

/****************************************/
/****************************************/

// void CWorker::Travel() {

//     /* Add robots to repel from */
//     std::vector<Message> repulseMsgs;
//     repulseMsgs.insert(std::end(repulseMsgs), std::begin(workerMsgs), std::end(workerMsgs));
//     repulseMsgs.insert(std::end(repulseMsgs), std::begin(chargerMsgs), std::end(chargerMsgs));

//     /* Calculate overall force applied to the robot */
//     CVector2 travelForce   = GetTravelVector();
//     CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);
//     CVector2 obstacleForce = GetObstacleRepulsionVector();

//     CVector2 sumForce = travelerAttraction*travelForce + travelerRepulsion*robotForce + travelerObstacle*obstacleForce;

//     /* Set Wheel Speed */
//     if(sumForce.Length() > 0.5f) {
//         SetWheelSpeedsFromVector(sumForce);
//         bMoving = true;
//     } else {
//         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
//         bMoving = false;
//     }

//     // /* Set Wheel Speed */
//     // if(currentMoveType == MoveType::TRAVEL_TO_CA) {
//     //     SetWheelSpeedsFromVector(sumForce);
//     //     bMoving = true;
//     // } else if(sumForce.Length() > 0.5f) {
//     //     SetWheelSpeedsFromVector(sumForce);
//     //     bMoving = true;
//     // } else {
//     //     m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
//     //     bMoving = false;
//     // }

// }

/****************************************/
/****************************************/

CVector2 CWorker::GetTravelVector() {

    CVector2 resVec = CVector2();

    /* Position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());

    /* Orientation */
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    Real fWorkAreaX = 0.55; // TEMP hard-coded value
    Real fChargingAreaX = -0.55; // TEMP hard-coded value

    CVector2 desiredPosition;
    if(currentMoveType == MoveType::MOVE_TO_WORK) {
        desiredPosition = CVector2(fWorkAreaX, pos2d.GetY());
    } else if(currentMoveType == MoveType::MOVE_TO_CHARGE) {
        desiredPosition = CVector2(fChargingAreaX, pos2d.GetY());
    }

    /* Calculate a normalized vector that points to the next waypoint */
    resVec += desiredPosition - pos2d;
    resVec.Rotate((-cZAngle).SignedNormalize());
    resVec *= 100; // Convert from meter to centimeter

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CWorker::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    /* State transition logic */
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
        if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(m_sWheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
            break;
        }
    }
    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cHeadingAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CWorker::PrintName() {
    //RLOG << "";
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CWorker::Callback_MoveToWork(void* data) {
    lastControllableAction = "moveToWork";
    currentMoveType = MoveType::MOVE_TO_WORK;
    bCharging = false;
    bPerformingTask = false;
    RLOG << "ACTION: moveToWork" << std::endl;
}

void CWorker::Callback_MoveToCharge(void* data) {
    lastControllableAction = "moveToTeam";
    currentMoveType = MoveType::MOVE_TO_CHARGE;
    bCharging = false;
    bPerformingTask = false;
    RLOG << "ACTION: moveToCharge" << std::endl;
}

void CWorker::Callback_Work(void* data) {
    lastControllableAction = "work";
    bCharging = false;
    bPerformingTask = true;
    RLOG << "ACTION: work" << std::endl;
}

void CWorker::Callback_Charge(void* data) {
    lastControllableAction = "charge";
    bCharging = true;
    bPerformingTask = false;
    RLOG << "ACTION: charge" << std::endl;
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CWorker::Check_AtWork(void* data) {
    if(m_pcGround->GetReadings()[0] == CColor(255,191,191).ToGrayScale() / 255.0f) {
        // RLOG << "Event: atWork " << 1 << std::endl;
        return true;
    }
    // RLOG << "Event: atWork " << 0 << std::endl;
    return false;
}

unsigned char CWorker::Check_NotAtWork(void* data) {
    if(m_pcGround->GetReadings()[0] == CColor(255,191,191).ToGrayScale() / 255.0f) {
        // RLOG << "Event: notAtWork " << 0 << std::endl;
        return false;
    }
    // RLOG << "Event: notAtWork " << 1 << std::endl;
    return true;
}

unsigned char CWorker::Check_AtCharger(void* data) {
    if(m_pcGround->GetReadings()[0] == CColor(191,255,191).ToGrayScale() / 255.0f) {
        // RLOG << "Event: atCharger " << 1 << std::endl;
        return true;
    }
    // RLOG << "Event: atCharger " << 0 << std::endl;
    return false;
}

unsigned char CWorker::Check_NotAtCharger(void* data) {
    if(m_pcGround->GetReadings()[0] == CColor(191,255,191).ToGrayScale() / 255.0f) {
        // RLOG << "Event: notAtCharger " << 0 << std::endl;
        return false;
    }
    // RLOG << "Event: notAtCharger " << 1 << std::endl;
    return true;
}

unsigned char CWorker::Check_LowEnergy(void* data) {
    /* Return true when the current energy is below the lower threshold */    
    // RLOG << "m_fDistToCharger: " << m_fDistToCharger << std::endl;
    // RLOG << "m_sWheelTurningParams.MaxSpeed: " << m_sWheelTurningParams.MaxSpeed/100 << std::endl;
    // RLOG << "m_fDeltaPos: " << m_fDeltaPos << std::endl;
    // RLOG << "energyToReturn: " << (m_fDistToCharger / (m_sWheelTurningParams.MaxSpeed / 100)) * (m_fDeltaPos * 10) << std::endl;
    bool lowEnergy = fEnergy < ((m_fDistToCharger / (m_sWheelTurningParams.MaxSpeed / 100)) * (m_fDeltaPos * 10) + 10) / 100;
    // RLOG << "Event: lowEnergy " << lowEnergy << std::endl;
    // RLOG << "fEnergy: " << fEnergy << std::endl;
    // RLOG << "est " << ((m_fDistToCharger / (m_sWheelTurningParams.MaxSpeed / 100)) * (m_fDeltaPos * 10) + 10) / 100 << std::endl;
    return lowEnergy;
}

unsigned char CWorker::Check_HighEnergy(void* data) {
    /* Return true when the current energy is above the higher threshold */
    bool highEnergy = fEnergy >= fEnergyHighThres;
    // RLOG << "Event: highEnergy " << highEnergy << std::endl;
    return highEnergy;
}

/****************************************/
/****************************************/

void CWorkerMC::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcGround    = GetSensor  <CCI_GroundSensor                >("ground"               );
    m_pcBattery   = GetSensor  <CCI_BatterySensor               >("battery"              );
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

        /* Flocking-related */
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));

        /* Weights using for the motion*/
        GetNodeAttribute(GetNode(t_node, "traveler"), "attract",  travelerAttraction);
        GetNodeAttribute(GetNode(t_node, "traveler"), "repulse",  travelerRepulsion);
        GetNodeAttribute(GetNode(t_node, "traveler"), "obstacle", travelerObstacle);

        /* SCT Model */
        GetNodeAttribute(GetNode(t_node, "SCT"), "path", m_strSCTPath);

        /* Energy */
        GetNodeAttribute(GetNode(t_node, "energy"), "low_thres", fEnergyLowThres);
        GetNodeAttribute(GetNode(t_node, "energy"), "high_thres", fEnergyHighThres);
        GetNodeAttribute(GetNode(t_node, "energy"), "share_dist", fTargetDistSE);

    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Initialization */
    initStepTimer = 0;
    currentState = RobotState::WORKER;
    currentMoveType = MoveType::MOVE_TO_CHARGE;
    bPerformingTask = false;
    bMoving = false;
    bCharging = false;
    bSharingEnergy = false;
    bRequestingEnergy = false;
    strEnergyFrom = "";
    strEnergyTo = "";
    fDistSE = 100000; // TEMP very large value

    m_fDeltaPos = 0.03; // TEMP value. Set using SetMoveDischargeRate()

    /*
    * Init SCT Controller
    */

    sct = std::make_unique<SCT>(m_strSCTPath);
    // RLOG << "Loading SCT" << std::endl;

    /* Register controllable events */
    sct->add_callback(this, std::string("EV_moveToWork"),   &CWorkerMC::Callback_MoveToWork,   NULL, NULL);
    sct->add_callback(this, std::string("EV_moveToCharge"), &CWorkerMC::Callback_MoveToCharge, NULL, NULL);
    sct->add_callback(this, std::string("EV_work"),         &CWorkerMC::Callback_Work,         NULL, NULL);
    sct->add_callback(this, std::string("EV_charge"),       &CWorkerMC::Callback_Charge,       NULL, NULL);

    /* Register uncontrollable events */
    sct->add_callback(this, std::string("EV_atWork"),           NULL, &CWorkerMC::Check_AtWork,           NULL);
    sct->add_callback(this, std::string("EV_notAtWork"),        NULL, &CWorkerMC::Check_NotAtWork,        NULL);
    sct->add_callback(this, std::string("EV_atCharger"),        NULL, &CWorkerMC::Check_AtCharger,        NULL);
    sct->add_callback(this, std::string("EV_notAtCharger"),     NULL, &CWorkerMC::Check_NotAtCharger,     NULL);
    sct->add_callback(this, std::string("EV_lowEnergy"),        NULL, &CWorkerMC::Check_LowEnergy,        NULL);
    // sct->add_callback(this, std::string("EV_midEnergy"),        NULL, &CWorkerMC::Check_MidEnergy,        NULL);
    sct->add_callback(this, std::string("EV_highEnergy"),       NULL, &CWorkerMC::Check_HighEnergy,       NULL);
    // sct->add_callback(this, std::string("EV_foundCharger"),     NULL, &CWorkerMC::Check_FoundCharger,     NULL);
    // sct->add_callback(this, std::string("EV_notFoundCharger"),  NULL, &CWorkerMC::Check_NotFoundCharger,  NULL);

    Reset();
}

/****************************************/
/****************************************/

void CWorkerMC::ControlStep() {

    std::string id = this->GetId();
    // LOG << "---------- " << id << " ----------" << std::endl;

    initStepTimer++;

    // // TEMP: force movetype to default to Adjust
    // if(initStepTimer == 1) {
    //     currentMoveType = MoveType::ADJUST;
    // }

    const CCI_BatterySensor::SReading& sBattery = m_pcBattery->GetReading();

    /* Update own energy */
    fEnergy = sBattery.AvailableCharge;

    // RLOG << "teamToMove = " << teamToMove << ", prevTeamID = " << prevTeamID << std::endl;
    // RLOG << "moveType = " << (UInt8)currentMoveType << std::endl;
    // CBatteryEquippedEntity& cBattery = c_entity.GetBatterySensorEquippedEntity();
    // Real robot_energy = cBattery.GetAvailableCharge();
    // if(hopsDict.count(1) > 0)
        // RLOG << "Hop to CA = " << hopsDict[1].count << std::endl;  

    /*-----------------*/
    /* Energy level */
    /*-----------------*/
    if(fEnergy <= 0) {
        // RLOG << "Energy depleted" << std::endl;
        m_pcLEDs->SetAllColors(CColor::BLACK);
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        bMoving = false;
        bPerformingTask = false;
        Message msg = Message();
        msg.ID = id;
        msg.emsg.energyLevel = 'D';
        cbyte_msg = msg.GetCByteArray();
        m_pcRABAct->SetData(cbyte_msg);
        return;
    }

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/
    ResetVariables();

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    Update();

    /*--------------------*/
    /* Run SCT controller */
    /*--------------------*/
    // RLOG << "--- Supervisors ---" << std::endl;

    if(initStepTimer > 4)
        sct->run_step();    // Run the supervisor to get the next action

    // RLOG << "Action: " << lastControllableAction << std::endl;

    // if(GetId() == "F14" || GetId() == "F15") {
        // RLOG << ", " << sct->get_current_state_string() << std::endl;
    // }

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    Message msg = Message();

    msg.state = currentState;
    msg.ID = id;

    // if(bCharging) {
    //     m_pcLEDs->SetAllColors(CColor::YELLOW);
    // } else {
    //     m_pcLEDs->SetAllColors(CColor::GREEN);
    // }

    // CColor cColor = CColor(0, 191, 0, 255); // GREEN
    // // GREEN --- ORANGE --- RED
    // UInt8 redValue = std::min(1.0, 2.0 - 2 * fEnergy) * 255;
    // UInt8 greenValue = std::min(1.0, fEnergy + 0.5) * 191;
    // cColor = CColor(redValue, greenValue, 0, 255);
    // // LOG << redValue << " " << greenValue << " " << robot_energy/cBattery.GetFullCharge() << std::endl;
    // m_pcLEDs->SetAllColors(cColor);

    // Real numLEDs = fEnergy / (1.0/8.0);
    // // RLOG << "numLEDs: " << numLEDs << std::endl;
    // for(size_t i = 0; i < m_pcLEDs->GetNumLEDs(); i++) {
    //     if(numLEDs > i)
    //         m_pcLEDs->SetSingleColor(i, CColor::GREEN);
    //     else
    //         m_pcLEDs->SetSingleColor(i, CColor::RED);
    // }

    if(Check_LowEnergy(nullptr)) {
        m_pcLEDs->SetAllColors(CColor::RED);
    } else if(fEnergy >= 0.5) {
        m_pcLEDs->SetAllColors(CColor::GREEN);
    } else {
        m_pcLEDs->SetAllColors(CColor::YELLOW);
    }

    /* Movement */
    // switch(currentMoveType) {
    //     case MoveType::MOVE_TO_WORK: {
    //         Travel();
    //         break;
    //     }
    //     case MoveType::MOVE_TO_CHARGE: {
    //         Travel();
    //         break;
    //     }
    // }
    Travel();

    /* Energy Message */
    EnergyMsg emsg = EnergyMsg();
    emsg.requestingEnergy = bRequestingEnergy;
    if(Check_LowEnergy(nullptr)) {
        emsg.energyLevel = 'L';
    } else if(Check_HighEnergy(nullptr)) {
        emsg.energyLevel = 'H';
    }
    // emsg.owner = this->GetId();
    // emsg.state = currentState;
    if(Check_HighEnergy(nullptr))
        strEnergyFrom = ""; // Reset to stop receiving energy
    emsg.from = strEnergyFrom;
    emsg.to = strEnergyTo;
    msg.emsg = emsg;

    // RLOG << "requestingEnergy? " << bRequestingEnergy << std::endl;
    // if(bRequestingEnergy) {
    //     RLOG << "from: " << strEnergyFrom << std::endl;
    //     RLOG << "to: " << strEnergyTo << std::endl;
    // }

    cbyte_msg = msg.GetCByteArray();

    /*--------------*/
    /* Send message */
    /*--------------*/

    m_pcRABAct->SetData(cbyte_msg);

}

/****************************************/
/****************************************/

void CWorkerMC::Update() {

    /* Position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());

    /* Distance to charging area */
    Real fChargingAreaX = -0.5;
    m_fDistToCharger = pos2d.GetX() - fChargingAreaX;

    /* Broadcast to neighbors to request energy */
    if(!bRequestingEnergy) 
        bRequestingEnergy = Check_LowEnergy(nullptr);
    else
        bRequestingEnergy = !Check_HighEnergy(nullptr);

    /* If it requested for energy, check if someone is willing share */
    if(bRequestingEnergy)
        CheckEnergyProvider();
            
}

/****************************************/
/****************************************/

void CWorkerMC::Travel() {

    /* Add robots to repel from */
    std::vector<Message> repulseMsgs;
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(workerMsgs), std::end(workerMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(chargerMsgs), std::end(chargerMsgs));

    Message shareEnergyMsg;
    if(bRequestingEnergy) {
        for(const auto& msg : chargerMsgs) {
            if(msg.ID == strEnergyFrom)
                shareEnergyMsg = msg;
        }
    }

    // /* Search for target to receive energy */
    // if(bRequestingEnergy && !strEnergyFrom.empty() && currentMoveType == MoveType::MOVE_TO_CHARGE) {
    //     for(const auto& msg : chargerMsgs) {
    //         if(strEnergyFrom == msg.ID && Check_AtCharger(nullptr)) {
    //             /* Don't move when target is nearby */
    //             m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    //             bMoving = false;
    //             return;
    //         }
    //     }
    // }

    /* Calculate overall force applied to the robot */
    CVector2 travelForce;
    if(bRequestingEnergy && !strEnergyFrom.empty() && currentMoveType == MoveType::MOVE_TO_CHARGE) {
        travelForce = GetApproachToShareEnergyVector(shareEnergyMsg);
    } else {
        travelForce = GetTravelVector();
    }
    CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);
    CVector2 obstacleForce = GetObstacleRepulsionVector();

    CVector2 sumForce = travelerAttraction*travelForce + travelerRepulsion*robotForce + travelerObstacle*obstacleForce;

    // RLOG << "travelForce: " << travelForce.Length() << std::endl;
    // RLOG << "robotForce: " << robotForce.Length() << std::endl;
    // RLOG << "obstacleForce: " << obstacleForce.Length() << std::endl;
    // RLOG << "sumForce.Length: " << sumForce.Length() << std::endl;

    // if(GetId() == "F4") {
    //     RLOG << "bRequestingEnergy: " << bRequestingEnergy << std::endl;
    //     RLOG << "strEnergyFrom: " << strEnergyFrom << std::endl;
    // }

    /* Set Wheel Speed */
    if(sumForce.Length() > 0.5f) {
        SetWheelSpeedsFromVector(sumForce);
        bMoving = true;
    } else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        bMoving = false;
    }
}

/****************************************/
/****************************************/

CVector2 CWorkerMC::GetTravelVector() {

    CVector2 resVec = CVector2();

    /* Position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());

    /* Orientation */
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    Real fWorkAreaX = 0.55; // TEMP hard-coded value

    CVector2 desiredPosition;
    // if(currentMoveType == MoveType::MOVE_TO_WORK) {
        desiredPosition = CVector2(fWorkAreaX, pos2d.GetY());
    // } else if(currentMoveType == MoveType::MOVE_TO_CHARGE) {
    //     desiredPosition = CVector2(fWorkAreaX, pos2d.GetY());
    // }

    /* Calculate a normalized vector that points to the next waypoint */
    resVec += desiredPosition - pos2d;
    resVec.Rotate((-cZAngle).SignedNormalize());
    resVec *= 100; // Convert from meter to centimeter

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CWorkerMC::GetApproachToShareEnergyVector(Message& msg) {

    Real fOffset = 0.035 * 2; // e-puck radius * 2
    CVector2 resVec = CVector2(msg.direction.Length() - fOffset - 8, 
                               msg.direction.Angle());
    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }
    return resVec;
}

/****************************************/
/****************************************/

void CWorkerMC::CheckEnergyProvider() {
    /* Find a charger who is willing to share energy and check if it still exists. */

    /* Find current energy provider */
    if( !strEnergyFrom.empty() ) {
        for(const auto& msg : chargerMsgs) {
            // RLOG << "Checking msg.ID:" << msg.ID << " = from: " << strEnergyFrom << std::endl;
            if(msg.ID == strEnergyFrom && msg.emsg.to == this->GetId()) { // Is it still willing to share energy?
                m_fDistToMC = msg.direction.Length();
                RLOG << "Energy provider still " << msg.ID << ", dist = " << m_fDistToMC << std::endl;
                return;
            }
        }
    }

    strEnergyFrom = ""; /* No potential energy provider found */

    /* Find a new energy provider */
    Message closestProviderMsg;
    for(const auto& msg : chargerMsgs) {
        if(msg.emsg.to == this->GetId()) {
            if(closestProviderMsg.Empty() || msg.direction.Length() < closestProviderMsg.direction.Length()) {
                closestProviderMsg = msg;
                strEnergyFrom = msg.ID;
                m_fDistToMC = msg.direction.Length();
                RLOG << "Found new energy provider " << msg.ID << std::endl;
            }
        }
    }
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CWorkerMC::Callback_MoveToWork(void* data) {
    lastControllableAction = "moveToWork";
    currentMoveType = MoveType::MOVE_TO_WORK;
    bCharging = false;
    bPerformingTask = false;
    bRequestingEnergy = false;
    RLOG << "ACTION: moveToWork" << std::endl;
}

void CWorkerMC::Callback_MoveToCharge(void* data) {
    lastControllableAction = "moveToTeam";
    currentMoveType = MoveType::MOVE_TO_CHARGE;
    bCharging = false;
    bPerformingTask = false;
    RLOG << "ACTION: moveToCharge" << std::endl;
}

void CWorkerMC::Callback_Work(void* data) {
    lastControllableAction = "work";
    bCharging = false;
    bPerformingTask = true;
    RLOG << "ACTION: work" << std::endl;
}

void CWorkerMC::Callback_Charge(void* data) {
    lastControllableAction = "charge";
    bCharging = true;
    bPerformingTask = false;
    RLOG << "ACTION: charge" << std::endl;
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CWorkerMC::Check_AtWork(void* data) {
    if(m_pcGround->GetReadings()[0] == CColor(255,191,191).ToGrayScale() / 255.0f) {
        // RLOG << "Event: atWork " << 1 << std::endl;
        return true;
    }
    // RLOG << "Event: atWork " << 0 << std::endl;
    return false;
}

unsigned char CWorkerMC::Check_NotAtWork(void* data) {
    if(m_pcGround->GetReadings()[0] == CColor(255,191,191).ToGrayScale() / 255.0f) {
        // RLOG << "Event: notAtWork " << 0 << std::endl;
        return false;
    }
    // RLOG << "Event: notAtWork " << 1 << std::endl;
    return true;
}

unsigned char CWorkerMC::Check_AtCharger(void* data) {
    bool isNearCharger = m_fDistToMC <= fTargetDistSE;
    // RLOG << "Event: atCharger " << isNearCharger << std::endl;
    return isNearCharger;
}

unsigned char CWorkerMC::Check_NotAtCharger(void* data) {
    bool isNearCharger = m_fDistToMC <= fTargetDistSE;
    // RLOG << "Event: atCharger " << isNearCharger << std::endl;
    return !isNearCharger;
}

unsigned char CWorkerMC::Check_LowEnergy(void* data) {
    /* Return true when the current energy is below the lower threshold */    
    bool lowEnergy = fEnergy < ((m_fDistToCharger / (m_sWheelTurningParams.MaxSpeed / 100)) * (m_fDeltaPos * 10) + 10) / 100;
    // RLOG << "Event: lowEnergy " << lowEnergy << std::endl;
    // if(GetId() == "F1") {
    //     RLOG << "m_fDeltaPos: " << m_fDeltaPos << std::endl;
    //     RLOG << "fEnergy: " << fEnergy << std::endl;
    //     RLOG << "return at: " << ((m_fDistToCharger / (m_sWheelTurningParams.MaxSpeed / 100)) * (m_fDeltaPos * 10) + 10) << std::endl;
    //     RLOG << "est: " << ((m_fDistToCharger / (m_sWheelTurningParams.MaxSpeed / 100)) * (m_fDeltaPos * 10) + 10) / 100 << std::endl;
    // }
    return lowEnergy;
}

// unsigned char CWorkerMC::Check_MidEnergy(void* data) {
//     /* Return true when the current energy not below the lower threshold or above the higher threshold */    
//     if(GetId() == "F6")
//         RLOGERR << "Event: midEnergy " << !(Check_LowEnergy(nullptr) || Check_HighEnergy(nullptr)) << std::endl;
//     if(Check_LowEnergy(nullptr) || Check_HighEnergy(nullptr))
//         return false;
//     return true;
// }

unsigned char CWorkerMC::Check_HighEnergy(void* data) {
    /* Return true when the current energy is above the higher threshold */
    bool highEnergy = fEnergy >= fEnergyHighThres;
    // RLOG << "Event: highEnergy " << highEnergy << std::endl;
    return highEnergy;
}

// unsigned char CWorkerMC::Check_FoundCharger(void* data) {
//     /* Return true if it has a charger to receive energy from */
//     bool foundCharger = !strEnergyFrom.empty();
//     if(GetId() == "F6")
//         RLOGERR << "Event: foundCharger " << foundCharger << std::endl;
//     return foundCharger;
// }

// unsigned char CWorkerMC::Check_NotFoundCharger(void* data) {
//     /* Return true if it does not have a charger to receive energy from */
//     bool foundCharger = !strEnergyFrom.empty();
//     // RLOG << "Event: notFoundCharger " << !foundCharger << std::endl;
//     return !foundCharger;
// }

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CWorker, "worker_controller")
REGISTER_CONTROLLER(CWorkerMC, "worker_mc_controller")
