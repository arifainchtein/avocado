                                                                                                                                    #!/usr/bin/env python
# -*- coding: utf-8 -*-

# https://raspberrytips.nl/tm1637-4-digit-led-display-raspberry-pi/


import sys
import time
import datetime
import RPi.GPIO as GPIO
import tm1637
import math

#CLK -> GPIO23 (Pin 16)
#DI0 -> GPIO24 (Pin 18)
#Display = tm1637.TM1637(23,24,tm1637.BRIGHT_TYPICAL)

#Display.Clear()
#Display2.Clear()
clockPin =int( sys.argv[1]);
dataPin =int( sys.argv[2]);
value1 =float( sys.argv[3]);
Display =  tm1637.TM1637(clockPin,dataPin,tm1637.BRIGHT_TYPICAL)
Display.SetBrightnes(1)


#displayToUse = int(sys.argv[1]);
if value1 != 9999:
	frac1, whole1 = math.modf(value1)
	w1 = int(whole1)
	f1 = int(frac1 * 100)
	currenttime = [ int(w1 / 10), w1 % 10, int(f1 / 10), f1 % 10 ]
	Display.Show(currenttime);

else:
	currenttime=[9,9,9,9]
	Display.ShowStale(currenttime);

