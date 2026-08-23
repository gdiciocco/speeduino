/** @file Vehicle odometer and trip-meter integration. */

#ifndef VEHICLE_DISTANCE_H
#define VEHICLE_DISTANCE_H

#include <stdint.h>

constexpr uint8_t VEHICLE_DISTANCE_SOURCE_VSS = 0U;
constexpr uint8_t VEHICLE_DISTANCE_SOURCE_GPS = 1U;

/** Initialise the runtime integrators from the persisted page-15 counters. */
void initialiseVehicleDistance(void);

/** Integrate the selected VSS/GPS speed. Called by the 10 Hz main-loop task. */
void updateVehicleDistance(void);

/** Deterministic integration entry point, also used by native tests. */
void accumulateVehicleDistance(uint16_t speedKph, uint32_t elapsedMs);

/** Reset and immediately schedule persistence of the trip counter. */
void resetVehicleTrip(void);

/** Selected source speed after range validation, in km/h. */
uint16_t getVehicleDistanceSpeedKph(void);

/** Total and trip counters in 0.1 km units. */
uint32_t getVehicleOdometerDeciKm(void);
uint32_t getVehicleTripDeciKm(void);

#endif
