// libfpv - a quad's physical parameters, as its preset files give them.
// The simulation server hands them to the flight model (sim/flight.cpp,
// toStateInit); everything else reads them from here, so the client side
// never needs the flight model's own headers (docs/DECISIONS.md, D37).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdint>

#include "fpv_math.h"

namespace fpv {

struct PhysicsParams {
    // motors (per motor: kV, winding resistance, idle current)
    float motorKV[4] = {};
    float motorR[4] = {};
    float motorI0[4] = {};
    float motorRth = 0.0f;  // thermal resistance, °C per watt
    float motorCth = 0.0f;  // heat capacity, joules per °C
    float motorMaxT = 0.0f; // °C where the windings short out

    // propellers
    uint8_t propBladeCount = 0;
    float propMaxRpm = 0.0f;
    float propAFactor = 0.0f;
    float propTorqueFactor = 0.0f;
    float propInertia = 0.0f;
    Vec3 propThrustFactor;
    float propHarmonic1Amp = 0.0f;
    float propHarmonic2Amp = 0.0f;
    float propPitch = 0.0f;     // metres, 0 = the thrust curve of propThrustFactor
    float propDiameter = 0.0f;  // metres
    float rotorDragFactor = 0.0f;
    float ductThrustGain = 0.0f;
    float electronicsPower = 5.0f;  // watts besides the motors

    // frame
    Vec3 frameDragArea;
    float frameDragConstant = 0.0f;
    float quadMass = 0.0f;
    Vec3 quadInvInertia;  // the flight model's handedness: y (yaw) negative
    Vec3 quadMotorPos[4];  // metres from the centre of mass
    float quadMotorDir[4] = {};  // 1 CW, -1 CCW

    // battery
    float maxVoltageSag = 0.0f;
    float batInternalResistance = 0.0f;
    uint8_t quadBatCellCount = 1;
    float quadBatCapacityCharged = 0.0f;  // mAh at the start
    float quadBatCapacity = 0.0f;         // mAh rating

    // prop wash
    float minPropWashSpeed = 1.0f;
    float maxPropWashSpeed = 18.0f;
    float propWashAngleOfAttack = 0.5f;
    float propWashFactor = 1.0f;

    float ambientTemp = 0.0f;  // °C
};

}  // namespace fpv
