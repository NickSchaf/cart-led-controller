cmake_minimum_required(VERSION 3.5)

set(srcs
 		"led-controller.cpp"
		)

idf_component_register(SRCS "PatternColor.cpp" "${srcs}"
			INCLUDE_DIRS "." 
			REQUIRES FastLED-idf WS2812FX-idf nvs_flash console ESPAsyncE131 )
