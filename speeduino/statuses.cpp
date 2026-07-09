#include "statuses.h"
#include "atomic.h"
#include "decoder_builder.h"

statuses::statuses(void)
{
  (void)memset(this, 0, sizeof(*this));
  battery10 = 125; //Set battery voltage to sensible value for dwell correction for "flying start" (else ignition gets spurious pulses after boot)
  decoder = decoder_builder_t().build();
}

void statuses::setRpm(uint16_t rpm)
{
  ATOMIC()
  {
    this->RPM = rpm;
    this->RPMdiv100 = div100(rpm);
    this->longRPM = rpm;
  }
}
