

#define _DCCSimpleWebServer_ 1

#include "DCCCommander.h"


void setup () 
{
  Serial.begin(115200);

  setupDCCCommander ();
}

void loop ()
{
  loopDCCCommander ();
}
