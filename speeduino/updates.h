#ifndef UPDATES_H
#define UPDATES_H

/** @brief Reset the tune to defaults if the stored layout is not this
 * firmware's. There is no migration from older layouts - see updates.cpp. */
void doUpdates(void);

#endif
