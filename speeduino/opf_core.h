#ifndef OPF_CORE_H
#define OPF_CORE_H

void setupBoard();
void runLoop();

#ifdef CAPONORD_BOARD
void caponordResetPins();
void caponordSetPins();
#endif

#endif // OPF_CORE_H
