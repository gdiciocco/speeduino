/** @file Vehicle odometer and trip-meter integration. */

#include "vehicle_distance.h"

#include <Arduino.h>
#include <stdint.h>

#include "globals.h"
#include "storage.h"

#if defined(CAPONORD_BOARD) || defined(UNIT_TEST)

namespace {

constexpr uint32_t SPEED_MILLISECONDS_PER_DECI_KM = 360000UL;
constexpr uint16_t MAX_VALID_VEHICLE_SPEED_KPH = 1000U;
constexpr uint32_t MAX_UPDATE_INTERVAL_MS = 1000UL;
constexpr uint8_t DECI_KM_PER_PERSISTENCE_CHECKPOINT = 10U;

uint32_t totalRemainder = 0U;
uint32_t tripRemainder = 0U;
uint32_t lastUpdateMs = 0U;
uint8_t distanceSinceCheckpoint = 0U;
bool wasMoving = false;

uint32_t saturatingAdd(uint32_t value, uint32_t increment)
{
  return (increment > (UINT32_MAX - value)) ? UINT32_MAX : value + increment;
}

uint32_t integrateDeciKm(uint16_t speedKph, uint32_t elapsedMs, uint32_t &remainder)
{
  const uint32_t numerator = remainder + ((uint32_t)speedKph * elapsedMs);
  remainder = numerator % SPEED_MILLISECONDS_PER_DECI_KM;
  return numerator / SPEED_MILLISECONDS_PER_DECI_KM;
}

void sanitiseConfiguration(void)
{
  bool changed = false;
  if(configPage15.vehicleDistanceSource > VEHICLE_DISTANCE_SOURCE_GPS)
  {
    configPage15.vehicleDistanceSource = VEHICLE_DISTANCE_SOURCE_VSS;
    changed = true;
  }
  if(configPage15.gpsSpeedAuxChannel >= 16U)
  {
    configPage15.gpsSpeedAuxChannel = 0U;
    changed = true;
  }
  if(configPage15.vehicleOdometerDeciKm == UINT32_MAX)
  {
    configPage15.vehicleOdometerDeciKm = 0U;
    changed = true;
  }
  if(configPage15.vehicleTripDeciKm == UINT32_MAX)
  {
    configPage15.vehicleTripDeciKm = 0U;
    changed = true;
  }
  if(changed) { setEepromWritePending(true); }
}

} // namespace

void initialiseVehicleDistance(void)
{
  sanitiseConfiguration();
  totalRemainder = 0U;
  tripRemainder = 0U;
  distanceSinceCheckpoint = 0U;
  wasMoving = false;
  lastUpdateMs = millis();
}

uint16_t getVehicleDistanceSpeedKph(void)
{
  uint16_t speedKph = currentStatus.vss;
  if(configPage15.vehicleDistanceSource == VEHICLE_DISTANCE_SOURCE_GPS)
  {
    const uint8_t channel = configPage15.gpsSpeedAuxChannel;
    speedKph = (channel < 16U) ? currentStatus.canin[channel] : 0U;
  }
  return (speedKph <= MAX_VALID_VEHICLE_SPEED_KPH) ? speedKph : 0U;
}

void accumulateVehicleDistance(uint16_t speedKph, uint32_t elapsedMs)
{
  if(speedKph > MAX_VALID_VEHICLE_SPEED_KPH) { speedKph = 0U; }
  if(elapsedMs > MAX_UPDATE_INTERVAL_MS) { elapsedMs = MAX_UPDATE_INTERVAL_MS; }

  const uint32_t totalIncrement = integrateDeciKm(speedKph, elapsedMs, totalRemainder);
  const uint32_t tripIncrement = integrateDeciKm(speedKph, elapsedMs, tripRemainder);
  configPage15.vehicleOdometerDeciKm =
      saturatingAdd(configPage15.vehicleOdometerDeciKm, totalIncrement);
  configPage15.vehicleTripDeciKm =
      saturatingAdd(configPage15.vehicleTripDeciKm, tripIncrement);

  if(totalIncrement > 0U)
  {
    const uint32_t checkpointDistance = (uint32_t)distanceSinceCheckpoint + totalIncrement;
    if(checkpointDistance >= DECI_KM_PER_PERSISTENCE_CHECKPOINT)
    {
      setEepromWritePending(true);
      distanceSinceCheckpoint = (uint8_t)(checkpointDistance % DECI_KM_PER_PERSISTENCE_CHECKPOINT);
    }
    else
    {
      distanceSinceCheckpoint = (uint8_t)checkpointDistance;
    }
  }

  const bool moving = speedKph > 0U;
  if(wasMoving && !moving && (distanceSinceCheckpoint > 0U))
  {
    //Save the final partial kilometre when the selected source reports a stop.
    setEepromWritePending(true);
    distanceSinceCheckpoint = 0U;
  }
  wasMoving = moving;
}

void updateVehicleDistance(void)
{
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - lastUpdateMs;
  lastUpdateMs = nowMs;
  accumulateVehicleDistance(getVehicleDistanceSpeedKph(), elapsedMs);
}

void resetVehicleTrip(void)
{
  configPage15.vehicleTripDeciKm = 0U;
  tripRemainder = 0U;
  setEepromWritePending(true);
}

uint32_t getVehicleOdometerDeciKm(void)
{
  return configPage15.vehicleOdometerDeciKm;
}

uint32_t getVehicleTripDeciKm(void)
{
  return configPage15.vehicleTripDeciKm;
}

#else

void initialiseVehicleDistance(void) {}
void updateVehicleDistance(void) {}
void accumulateVehicleDistance(uint16_t, uint32_t) {}
void resetVehicleTrip(void) {}
uint16_t getVehicleDistanceSpeedKph(void) { return 0U; }
uint32_t getVehicleOdometerDeciKm(void) { return 0U; }
uint32_t getVehicleTripDeciKm(void) { return 0U; }

#endif
