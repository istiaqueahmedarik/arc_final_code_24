#include <FastLED.h>
#define NUM_LEDS 63
CRGB leds[NUM_LEDS];
void setup()
{
    FastLED.addLeds<NEOPIXEL, 13>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
}
void white()
{
    for (int i = 0; i < 63; i++)
    {
        leds[i] = CRGB::White;
        FastLED.show();
    }
}
void black()
{
    for (int i = 0; i < 63; i++)
    {
        leds[i] = CRGB::Black;
        FastLED.show();
    }
}
void red()
{
    for (int i = 0; i < 63; i++)
    {
        leds[i] = CRGB(255, 0, 0);
        FastLED.show();
    }
}
void blue()
{
    for (int i = 0; i < 63; i++)
    {
        leds[i] = CRGB(0, 255, 0);
        FastLED.show();
    }
}
void yellow()
{
    for (int i = 0; i < 63; i++)
    {
        leds[i] = CRGB(255, 0, 255);
        FastLED.show();
    }
}
void loop()
{
    black();
    // leds[0] = CRGB::Black; FastLED.show(); delay(30);
}