/** @file seq_trim_autotune.h
 * @brief Slow adaptive learning for the sequential per-injector fuel trims.
 */
#pragma once

#include <stdint.h>

enum SeqTrimAutotuneMode : uint8_t {
  SEQ_TRIM_AUTOTUNE_OFF = 0U,
  SEQ_TRIM_AUTOTUNE_OBSERVE = 1U,
  SEQ_TRIM_AUTOTUNE_LEARN = 2U,
};

enum SeqTrimAutotuneState : uint8_t {
  SEQ_TRIM_STATE_DISABLED = 0U,
  SEQ_TRIM_STATE_BLOCKED = 1U,
  SEQ_TRIM_STATE_STABILISING = 2U,
  SEQ_TRIM_STATE_OBSERVING = 3U,
  SEQ_TRIM_STATE_LEARNING = 4U,
  SEQ_TRIM_STATE_AUTHORITY = 5U,
};

enum SeqTrimAutotuneGate : uint16_t {
  SEQ_TRIM_GATE_DISABLED       = 1U << 0U,
  SEQ_TRIM_GATE_LAYOUT         = 1U << 1U,
  SEQ_TRIM_GATE_WIDEBAND       = 1U << 2U,
  SEQ_TRIM_GATE_NOT_RUNNING    = 1U << 3U,
  SEQ_TRIM_GATE_RUN_TIME       = 1U << 4U,
  SEQ_TRIM_GATE_COLD           = 1U << 5U,
  SEQ_TRIM_GATE_RPM            = 1U << 6U,
  SEQ_TRIM_GATE_LOAD           = 1U << 7U,
  SEQ_TRIM_GATE_TRANSIENT      = 1U << 8U,
  SEQ_TRIM_GATE_FUEL_TRANSIENT = 1U << 9U,
  SEQ_TRIM_GATE_MOTORSPORT     = 1U << 10U,
  SEQ_TRIM_GATE_PROTECTION     = 1U << 11U,
  SEQ_TRIM_GATE_BATTERY        = 1U << 12U,
  SEQ_TRIM_GATE_AFR_RANGE      = 1U << 13U,
  SEQ_TRIM_GATE_NO_REFERENCE   = 1U << 14U,
  SEQ_TRIM_GATE_CONFIG         = 1U << 15U,
};

struct SeqTrimAutotuneDiagnostics {
  uint16_t gateBits = SEQ_TRIM_GATE_DISABLED;
  uint16_t acceptedSamples = 0U;
  uint16_t cellUpdates = 0U;
  uint16_t secondsToNextSave = 0U;
  int16_t lastErrorTenthsPercent = 0;
  uint8_t state = SEQ_TRIM_STATE_DISABLED;
  uint8_t activeMask = 0U;
  uint8_t learnMask = 0U;
  uint8_t authorityMask = 0U;
  uint8_t lastTrim = UINT8_MAX;
  uint8_t lastCell = UINT8_MAX;
  int8_t lastDelta = 0;
};

/** Initialise volatile evidence and take no action on the trim tables. */
void seqTrimAutotuneInit(void);

/** Run one learner step. Call at 30 Hz after sensors and corrections update. */
void seqTrimAutotuneUpdate(void);

/** Current live diagnostics. */
const SeqTrimAutotuneDiagnostics& seqTrimAutotuneDiag(void);

#if defined(UNIT_TEST)
/** Test-only visibility into the retained, per-cell weighted evidence. */
int32_t seqTrimAutotuneCellAccumulator(uint8_t trim, uint8_t cell);
#endif

