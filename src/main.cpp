#include <Arduino.h>
#include <FastLED.h>

#define NUM_LEDS 22
#define DATA_PIN 17
#define Brightness 255

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(Brightness);
  FastLED.clear();
}

void loop() {

    //Sets all LEDs to Red
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    delay(1000);

    //Sets all LEDs to Blue
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
    delay(1000);

    //Sets all LEDs to Black
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    //Makea a green loop
  for(int i = 0; i < 5; i++) {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Green;
        leds[i+1] = CRGB::Green;
        FastLED.show();
        delay(50);
        leds[i] = CRGB::Black;
    }
  }
}
