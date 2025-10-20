/**
 * @file MeMegaPiWrapper.h
 * @brief Minimal wrapper for MeMegaPi library to prevent conflicts
 * @date 2025-03-24
 */

#ifndef MEMEGAPI_WRAPPER_H
#define MEMEGAPI_WRAPPER_H

#include <Arduino.h>

// Forward declarations for MeMegaPi types we need
class MeEncoderOnBoard;
class MeMegaPiDCMotor;

// Slot definitions - copied from MeMegaPi.h
#ifndef SLOT1
#define SLOT1 1
#define SLOT2 2
#define SLOT3 3
#define SLOT4 4
#endif

// Port definitions
#ifndef PORT4A
#define PORT4A 0x01
#define PORT4B 0x02
#define PORT_5 0x05
#define S1_IN_S2_IN   0x00
#define S1_IN_S2_OUT  0x01
#define S1_OUT_S2_IN  0x02
#define S1_OUT_S2_OUT 0x03
#endif

// Include only required components from MeMegaPi.h
#ifdef MEMEGAPI_IMPLEMENTATION
// This is the minimal subset of files needed
#include <MeEncoderOnBoard.h>
#include <MeMegaPiDCMotor.h>
#include <MeLineFollower.h>
// Intentionally exclude MeIR.h and other conflicting files
#endif

#endif /* MEMEGAPI_WRAPPER_H */