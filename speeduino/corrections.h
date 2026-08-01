/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

void initialiseCorrections(void);
uint16_t correctionsFuel(void);
uint8_t calculateAfrTarget(table3d16RpmLoad &afrLookUpTable, const statuses &current, const config2 &page2, const config6 &page6);

/** @brief Ignition corrections. Advance is in tenths of a degree (@see ANGLE_TENTHS_PER_DEGREE) */
int16_t correctionsIgn(int16_t advanceTenths);
/** @brief Advance is in tenths of a degree (@see ANGLE_TENTHS_PER_DEGREE) */
int16_t correctionFixedTiming(int16_t advanceTenths);
/** @brief Advance is in tenths of a degree (@see ANGLE_TENTHS_PER_DEGREE) */
int16_t correctionCrankingFixedTiming(int16_t advanceTenths);

uint16_t correctionsDwell(uint16_t dwell);


#endif // CORRECTIONS_H
