// =============================================================================
//  MeteoPlaneRadar
//  Clock, live aircraft radar (adsb.fi), precipitation radar (CHMU or
//  RainViewer) and a weather forecast (Open-Meteo) on a round touchscreen.
// =============================================================================
//
//  Author:  Petr / chiptron.cz   (vyvoj / development: chiptron.cz)
//  Web:     https://chiptron.cz
//  Board:   Waveshare ESP32-S3-Touch-LCD-2.1
//           - ESP32-S3R8 (8 MB PSRAM, 16 MB flash)
//           - round 480x480 display, ST7701 controller (RGB interface)
//           - CST820 capacitive touch (I2C)
//           - TCA9554 I/O expander (LCD reset / CS / power control)
//
//  Screens (cycled by a LONG press; each can be switched off in the web UI):
//    1) Clock       - time, date, current temperature, seconds ring
//    2) Aircraft    - adsb.fi, tap an aircraft for details and its route
//    3) Weather     - animated precipitation, CHMU or RainViewer
//    4) Forecast    - next hours and days, plus air quality
//    5) Settings    - always reachable, and shows the web address
//
//  Configuration lives in a browser. The device serves its own settings page
//  permanently at http://meteoplaneradar.local/ once it is on the home network,
//  and a captive portal on its own access point before that. Firmware updates
//  are at /update on the same server.
//
//  Controls:
//    - swipe left/right             = change the range (radar screens)
//    - short tap                    = aircraft detail / day-night toggle
//    - long press LEFT half         = previous screen
//    - long press RIGHT half        = next screen
//    - hold BOOT at startup (~3 s)  = factory reset
//
//  Serial log: 115200 Bd over the connector marked "USB" (the ESP32-S3 native
//  USB). The other USB-C connector on the board will not show anything.
//
//  Libraries (Arduino IDE, ESP32 core 3.x):
//    - GFX Library for Arduino (moononournation) - drawing
//    - PNGdec (bitbank2)                          - CHMU frames, RainViewer tiles
//    - ArduinoJson (bblanchon, v7)                - all the JSON
//    - QRCode (ricmoo)                            - QR code (bundled)
//    - WebServer, DNSServer, ESPmDNS, Preferences, Wire, HTTPClient, Update,
//      esp_lcd                                    - part of the ESP32 core
//
//  Neither WiFiManager nor ElegantOTA is used any more - see the note at the
//  top of WiFiPortal.h and the /update section of WebConfig.cpp.
//
//  Data sources (attribution required, personal non-commercial use only):
//    - Aircraft:    adsb.fi, https://adsb.fi
//    - Route:       adsb.lol, https://adsb.lol
//                   (route data by https://github.com/vradarserver/standing-data)
//    - Precip. CZ:  CHMU, https://opendata.chmi.cz
//    - Precip. EU:  RainViewer, https://www.rainviewer.com
//    - Weather:     Open-Meteo, https://open-meteo.com
//    - Location:    ip-api.com
//    - Map:         Natural Earth (public domain), GeoNames (CC BY 4.0)
//    - Clock:       the "Date" header of the responses above. No NTP client.
//
//  Licence: MIT (see LICENSE.txt). Version: Version.h. History: CHANGELOG.md.
// =============================================================================

/*
FQBN: esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi

ESP32 core 3.3.8
Using library GFX Library for Arduino at version 1.6.7 in folder: E:\dev.moje\arduino\libraries\GFX_Library_for_Arduino 
Using library SPI at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\SPI 
Using library Wire at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\Wire 
Using library ArduinoJson at version 7.1.0 in folder: E:\dev.moje\arduino\libraries\ArduinoJson 
Using library WiFi at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\WiFi 
Using library Networking at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\Network 
Using library NetworkClientSecure at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\NetworkClientSecure 
Using library HTTPClient at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\HTTPClient 
Using library PNGdec at version 1.0.1 in folder: E:\dev.moje\arduino\libraries\PNGdec 
Using library Preferences at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\Preferences 
Using library WebServer at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\WebServer 
Using library FS at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\FS 
Using library DNSServer at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\DNSServer 
Using library ESP32 Async UDP at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\AsyncUDP 
Using library ESPmDNS at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\ESPmDNS 
Using library Update at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\Update 
Using library Hash at version 3.3.8 in folder: C:\Users\brouzda\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.8\libraries\Hash 
*/

#include <Wire.h>
#include <time.h>
#include "esp_heap_caps.h"
#include "esp_system.h"

#include "LGFX_ESP32S3_RGB_ESP32-8048S070.h"

#include "Settings.h"
#include "Version.h"
#include "Config.h"
#include "Lang.h"
#include "Layout.h"
#include "Status.h"
#include "UI.h"
#include "Net.h"
#include "WiFiPortal.h"
#include "WebConfig.h"
#include "GeoIP.h"
#include "ADSB.h"
#include "ScreenPlanes.h"
#include "CHMU.h"
#include "RainViewer.h"
#include "ScreenWeather.h"
#include "ScreenClock.h"
#include "ScreenForecast.h"
#include "ScreenSettings.h"
#include "Forecast.h"
#include "NightMode.h"
#include "Watchdog.h"
#include "Outside.h"
#include "Route.h"

// gfx = single off-screen canvas in PSRAM. Everything is drawn here, then
// flush() pushes the whole frame to the panel in one shot -> no flicker.
// Arduino_GFX* gfx = nullptr;


#include <LovyanGFX.hpp>

#include "LGFX_ESP32S3_RGB_ESP32-8048S070.h"

static LGFX lcd;  
LGFX * gfxReal = &lcd;

static LGFX_Sprite sprite(&lcd); // スプライトを使う場合はLGFX_Spriteのインスタンスを作成。
LGFX_Sprite * gfx = &sprite;



// yield() during long network transfers, feed the watchdog so multi-second
// downloads never trip it, and keep sampling the touch so a gesture made during
// a download is still the gesture that gets acted on.
static void netPoll() { 
    yield(); 
    Watchdog_Feed(); 
}

static void checkBootReset() {
  pinMode(BOOT_PIN, INPUT_PULLUP);
  if (digitalRead(BOOT_PIN) != LOW) return;
  gfx->fillScreen(C_BLACK);
  UI_TextCentered("Drzte pro reset...", LCD_HEIGHT / 2, C_WHITE, 2);
  gfx->flush();
  unsigned long start = millis();
  while (digitalRead(BOOT_PIN) == LOW) {
    if (millis() - start >= 3000) {
      UI_TextCentered("Mazu nastaveni", LCD_HEIGHT / 2 + 30, C_RED, 2);
      gfx->flush();
      Settings_ClearAll();
      delay(800);
      ESP.restart();
    }
    delay(20);
  }
}

// =============================================================================
//  Screen manager
//
//  Five screens, four of which can be switched off. "Off" means genuinely
//  invisible: the long press skips over it, the automatic cycling skips it, and
//  it has no dot at the top - so the dots always match what a long press will
//  actually do. Settings can never be switched off, or a device with everything
//  else disabled would have no way back to the web UI.
// =============================================================================
static int s_screen = SCREEN_PLANES_I;

static bool screenVisible(int i) { return Settings_ScreenEnabled((uint8_t)i); }

// The visible screens in order, used for the dots and for stepping.
static int visibleList(int* out) {
  int n = 0;
  for (int i = 0; i < SCREEN_N; i++) if (screenVisible(i)) out[n++] = i;
  return n;
}

static void drawScreenDots() {
  int vis[SCREEN_N];
  const int n = visibleList(vis);
  if (n <= 1) return;                    // one screen - a single dot says nothing

  const int gap = 20;
  const int cx = LCD_WIDTH / 2;
  const int y = LY_DOTS;
  const int startX = cx - (n - 1) * gap / 2;
  for (int i = 0; i < n; i++) {
    int x = startX + i * gap;
    if (vis[i] == s_screen) gfx->fillCircle(x, y, 4, C_WHITE);
    else                    gfx->drawCircle(x, y, 4, C_GRAY);
  }
}

static void drawActive() {
  switch (s_screen) {
    case SCREEN_CLOCK_I:    ScreenClock_Draw();    break;
    case SCREEN_PLANES_I:   ScreenPlanes_Draw();   break;
    case SCREEN_METEO_I:    ScreenWeather_Draw();  break;
    case SCREEN_FORECAST_I: ScreenForecast_Draw(); break;
    case SCREEN_SETTINGS_I: ScreenSettings_Draw(); break;
  }
  drawScreenDots();
  // gfx->flush();    // hand the framebuffer over; Canvas16 switches to the other
  sprite.pushSprite(0, 0);
}

static void enterActive() {
  switch (s_screen) {
    case SCREEN_CLOCK_I:    ScreenClock_Enter();    break;
    case SCREEN_PLANES_I:   ScreenPlanes_Enter();   break;
    case SCREEN_METEO_I:    ScreenWeather_Enter();  break;
    case SCREEN_FORECAST_I: ScreenForecast_Enter(); break;
    case SCREEN_SETTINGS_I: ScreenSettings_Enter(); break;
  }
  Serial.println("drawActive 3");
  drawActive();
}

// dir -1 = previous, +1 = next. Walks over disabled screens; the guard stops it
// looping forever if somehow nothing is enabled at all.
static void switchScreen(int dir) {
  int next = s_screen;
  for (int guard = 0; guard < SCREEN_N; guard++) {
    next = (next + dir + SCREEN_N) % SCREEN_N;
    if (screenVisible(next)) break;
  }
  if (next == s_screen) return;
  s_screen = next;
  Settings_SetScreen((uint8_t)s_screen);
  Serial.printf("Screen: %d\n", s_screen);
  enterActive();
}

// Jump straight to a screen. Used by the web remote control; a disabled screen
// is refused, which the handler has already checked.
static void gotoScreen(int idx) {
  if (idx < 0 || idx >= SCREEN_N) return;
  if (!screenVisible(idx)) return;
  if (idx == s_screen) return;
  s_screen = idx;
  Settings_SetScreen((uint8_t)s_screen);
  Serial.printf("Screen: %d (web)\n", s_screen);
  enterActive();
}

static bool activeTick() {
  switch (s_screen) {
    case SCREEN_CLOCK_I:    return ScreenClock_Tick();
    case SCREEN_PLANES_I:   return ScreenPlanes_Tick();
    case SCREEN_METEO_I:    return ScreenWeather_Tick();
    case SCREEN_FORECAST_I: return ScreenForecast_Tick();
    case SCREEN_SETTINGS_I: return ScreenSettings_Tick();
  }
  return false;
}

static void activeChangeRange(int dir) {
  switch (s_screen) {
    case SCREEN_PLANES_I: ScreenPlanes_ChangeRange(dir);  break;
    case SCREEN_METEO_I:  ScreenWeather_ChangeRange(dir); break;
    default: break;   // the other screens have no range
  }
}

static bool activeTap(int x, int y) {
  switch (s_screen) {
    case SCREEN_CLOCK_I:    return ScreenClock_HandleTap(x, y);
    case SCREEN_PLANES_I:   return ScreenPlanes_HandleTap(x, y);
    case SCREEN_SETTINGS_I: return ScreenSettings_HandleTap(x, y);
    default: return false;
  }
}

// Is a modal (the aircraft detail) open? Then swipe/long-press are captured to
// close it rather than change range / switch screen.
static bool activeModalOpen() {
  return (s_screen == SCREEN_PLANES_I) && ScreenPlanes_DetailOpen();
}

// --- Automatic cycling ------------------------------------------------------
// Paused by the gestures that mean the user is driving - a swipe or a long
// press - and held while an aircraft detail is open. A plain tap does not stop
// it; being switched away mid-sentence is annoying, but so is a device that
// stops cycling because someone brushed the glass.
static unsigned long s_lastRotate = 0;

static void autoRotateTick() {
  const uint16_t secs = Settings_AutoRotateSec();
  if (secs == 0) return;
  if (s_screen == SCREEN_SETTINGS_I) return;      // never cycle away from settings

  unsigned long now = millis();

  // An open aircraft detail holds the cycling: the user is reading it. The
  // timer is kept fresh rather than stopped, so when the panel closes - by a
  // tap, or because the aircraft dropped out of the data - cycling resumes with
  // a full interval instead of switching away immediately.
  if (activeModalOpen()) { s_lastRotate = now; return; }

  if (now - s_lastRotate < (unsigned long)secs * 1000UL) return;
  s_lastRotate = now;

  // Step to the next visible DATA screen, skipping settings.
  int next = s_screen;
  for (int guard = 0; guard < SCREEN_N; guard++) {
    next = (next + 1) % SCREEN_N;
    if (next != SCREEN_SETTINGS_I && screenVisible(next)) break;
  }
  if (next == s_screen || next == SCREEN_SETTINGS_I) return;
  s_screen = next;
  enterActive();
}

static const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "zapnuti napajeni";
    case ESP_RST_EXT:       return "externi reset (tlacitko)";
    case ESP_RST_SW:        return "softwarovy restart (napr. po OTA)";
    case ESP_RST_PANIC:     return "PANIC - vyjimka v programu";
    case ESP_RST_INT_WDT:   return "WATCHDOG (preruseni)";
    case ESP_RST_TASK_WDT:  return "WATCHDOG (zaseknuta smycka)";
    case ESP_RST_WDT:       return "WATCHDOG (jiny)";
    case ESP_RST_DEEPSLEEP: return "probuzeni z deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT - podpeti napajeni";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "neznamy";
  }
}

// The time zone has to be in the environment even though there is no NTP
// client: CHMU.cpp and the clock screen convert UTC with localtime_r(), and
// without TZ that quietly hands back UTC - labels an hour or two out.
static void applyTimezone() {
  setenv("TZ", TZ_INFO, 1);
  tzset();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n=== MeteoPlaneRadar v%s ===\n", FW_VERSION);
  Serial.printf("Duvod restartu: %s\n", resetReasonText());
  Serial.printf("Volna pamet: %u B\n", (unsigned)ESP.getFreeHeap());

  Settings_Begin();          // also sets the interface language
  applyTimezone();
  Layout_SelfTest();         // no-op unless LAYOUT_DEBUG is on

  lcd.init();
  // brightness lze nastavit pouze na fyzickem zarizeni
  lcd.setBrightness(128);
  lcd.setColorDepth(16); 
  lcd.setRotation(0);

  // inicializace canvasu
  sprite.setPsram(true);
  sprite.createSprite(lcd.width(), lcd.height() );
  sprite.setColorDepth(16); 

//  gfx->setRotation(0);
  gfx->setTextWrap(false);
  gfx->fillScreen(C_BLACK);
  gfx->flush();
  NightMode_Apply();          // now there is something to look at

  checkBootReset();

  ADSB_SetPollFn(netPoll);
  CHMU_SetPollFn(netPoll);
  Net_SetPollFn(netPoll);
  RainViewer_SetPollFn(netPoll);

  // Connects with the stored credentials, or brings up the access point and
  // leaves it up. Either way the web server is running when this returns.
  WiFi_Begin();

  if (WiFi_IsConnected()) {
    GeoIP_DetectIfNeeded();   // fill in the location by IP if none is stored
  }

  s_screen = Settings_Screen();
  if (s_screen >= SCREEN_N || !screenVisible(s_screen)) {
    // The stored screen was switched off since the last run - start on the
    // first one that is actually enabled.
    s_screen = SCREEN_SETTINGS_I;
    for (int i = 0; i < SCREEN_N; i++) if (screenVisible(i)) { s_screen = i; break; }
  }
  // In access-point mode the QR screen owns the display and must stay up until a
  // network is entered. Drawing a data screen here would paint straight over it
  // - which is what happened on a first run and after a factory reset. The
  // screen is still chosen above, so it is ready the moment WiFi comes up.
  if (!WiFi_IsAP()) enterActive();

  Watchdog_Begin();   // hardware watchdog for 24/7 operation
  Serial.println("Setup done");
}



void loop() {

  // A firmware upload owns the machine: writing flash suspends the PSRAM cache
  // from under the RGB DMA, so nothing may draw, and every spare cycle should
  // go to the transfer. The watchdog is fed by the OTA progress callback.
  if (WebConfig_UpdateBusy()) {
    WebConfig_Loop();
    delay(1);
    return;
  }

  // --- Access point: the portal owns the display ----------------------------
  // Until a network is entered there is no data to put on the screens anyway,
  // and the QR code is the only thing standing between the user and a working
  // device. So nothing below this block may draw, and no gesture may switch
  // away from it. The web server and the WiFi state machine keep running.
  static bool s_apOwnsScreen = false;
  static uint8_t s_apLang = 0xFF;
  if (WiFi_IsAP()) {
    if (!s_apOwnsScreen || s_apLang != (uint8_t)Lang_Get()) {
      // Redrawn on a language change too: the portal has its own selector, and
      // the instructions under the QR code should follow it.
      s_apLang = (uint8_t)Lang_Get();
      s_apOwnsScreen = true;
      WiFi_DrawApScreen();
    }
    WiFi_Loop();                         // may accept credentials and leave AP mode
    WebConfig_Loop();
    if (WebConfig_WantsRestart()) {
      Serial.println("Nastaveni zmeneno, restartuji");
      Serial.flush();
      delay(400);
      ESP.restart();
    }
    // No Outside_Tick/Forecast_Tick here - there is no route to the internet and
    // every attempt would just burn the loop on timeouts.
    NightMode_Tick();
    Settings_Tick();
    Watchdog_Feed();
    delay(5);
    return;
  }
  if (s_apOwnsScreen) {
    // A network was accepted: take the display back and start the cycling clock
    // from now, so the first switch comes a full interval after setup ends.
    s_apOwnsScreen = false;
    s_apLang = 0xFF;
    s_lastRotate = millis();
    enterActive();
  }

  // The same actions, asked for from the web instead of the glass. They are
  // carried out here rather than in the request handler for the same reason a
  // gesture is - drawing must not happen from anywhere else.
  {
    const int scr = WebConfig_TakeScreen();
    if (scr >= 0) {
      if (activeModalOpen()) ScreenPlanes_CloseDetail();
      gotoScreen(scr);
    }
    const int step = WebConfig_TakeScreenStep();
    if (step) {
      if (activeModalOpen()) ScreenPlanes_CloseDetail();
      switchScreen(step);
    }
    const int rng = WebConfig_TakeRangeStep();
    if (rng) {
      activeChangeRange(rng);
      Serial.println("drawActive 1");
      drawActive();
    }
  }

  WiFi_Loop();
  WebConfig_Loop();

  // The user asked to forget the network from the settings screen.
  if (ScreenSettings_WantsWifiReset()) {
    ScreenSettings_ClearWifiReset();
    WiFi_Reset();               // drops to the access point and redraws
    return;
  }

  // Some settings only take effect from a clean start (which screens exist,
  // which radar source, where we are). The web handler asks for it; doing it
  // here means the browser has already had its reply.
  if (WebConfig_WantsRestart()) {
    Serial.println("Nastaveni zmeneno, restartuji");
    Serial.flush();
    delay(400);
    ESP.restart();
  }


  // Redrawing is decoupled from reading the touch and capped at ~12 FPS.
  static unsigned long lastDraw = 0;
  bool wantDraw = activeTick();
  if (wantDraw && millis() - lastDraw >= 80) {
    Serial.println("drawActive 2");
    drawActive();
    lastDraw = millis();
  }

  autoRotateTick();

  Outside_Tick();     // outside temperature and the clock safety net
  Forecast_Tick();    // forecast, sun times and air quality
  NightMode_Tick();   // day/night brightness
  Route_Tick();       // pending "where is it flying from/to" lookup
  Settings_Tick();    // debounced persist of UI state to NVS
  Watchdog_Feed();
}
