cmake_minimum_required(VERSION 3.5)

set(srcs
 		"led-display-manager.cpp"
		"LedPattern.cpp"
		"LedPatternSolid.cpp"
		"PatternColor.cpp"
		)

idf_component_register(SRCS "BoundedValue.cpp" "SaveableSetting.cpp" "LedSpecialPatternFPP.cpp" "LedPatternPride.cpp" "LedPatternParty.cpp" "LedPatternRainbowStripe.cpp" "LedPatternFire.cpp" "LedPatternStrobe.cpp" "LedPatternPolice.cpp" "LedPatternFadingMarquee.cpp" "LedPatternMarquee.cpp" "LedPatternPulse.cpp" "LedPatternKnightRider.cpp" "LedPatternJuggle.cpp" "LedPatternBPM.cpp" "LedPatternSinelon.cpp" "LedPatternConfetti.cpp" "LedPatternGlitter.cpp" "LedPatternRainbow.cpp" "${srcs}"
			INCLUDE_DIRS "." 
			REQUIRES FastLED-idf WS2812FX-idf nvs_flash console ESPAsyncE131 )
