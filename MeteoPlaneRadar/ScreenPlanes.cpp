// =============================================================================
//  MeteoPlaneRadar
//  Screen 1: aircraft radar (adsb.fi) + aircraft detail.
//
//  Project: MeteoPlaneRadar - live aircraft radar on a round touchscreen
//  Author:  Petr / chiptron.cz   (vyvoj / development: chiptron.cz)
//  Web:     https://chiptron.cz
//  Board:   Waveshare ESP32-S3-Touch-LCD-2.1 (round 480x480 display, ST7701)
// =============================================================================
#include "ScreenPlanes.h"
#include "ADSB.h"
#include "Settings.h"
#include "Config.h"
#include "Route.h"
#include "EuBorder.h"
#include "UI.h"
#include "Layout.h"
#include "Lang.h"
#include "Status.h"
#include "Geo.h"

#include <WiFi.h>
#include <math.h>
#include <string.h>   // strncpy / strcmp for the hex-based selection




// Round panel - radar centred on the middle of the screen.
// R_RADIUS = 230 px maps the selected range onto the radar circle.
#define R_CX (LCD_WIDTH / 2)
#define R_CY (LCD_HEIGHT / 2)
#define R_RADIUS 230

// Ranges (radius in km).
static const float RANGES_KM[] = PLANE_RANGES_KM;
static const int   RANGE_COUNT = sizeof(RANGES_KM) / sizeof(RANGES_KM[0]);
static int s_rangeIdx = 1;   // 25 km by default

int currentZoom = 25;
int maxPlanes = 13;
int minPlanes = 8;
int maxZoom = 110;
int minZoom = 5;
static float currentRange() { return (float)currentZoom; }

// Base poll interval by range. Larger areas return more data and are less
// time-critical, so they are polled less often - keeps us a light user of the
// free adsb.fi API.
static unsigned long basePeriodMs() {
  float r = currentRange();
  if (r <= ADSB_NEAR_KM) return ADSB_PERIOD_NEAR_MS;
  if (r <= ADSB_MID_KM)  return ADSB_PERIOD_MID_MS;
  return ADSB_PERIOD_FAR_MS;
}

static unsigned long s_nextFetch = 0;
static bool  s_dataOk = false;
static String s_status = "...";

// Aircraft detail - the selected aircraft is remembered by its ICAO hex address,
// NOT by its index in the list.
//
// The list is rebuilt from scratch on every fetch (every 5 s) and adsb.fi does
// not guarantee a stable order: one aircraft leaving the area shifts everything
// after it. Holding an index across a fetch therefore silently repoints the
// detail at a different aircraft. Keying on the hex means the panel either
// follows the aircraft you actually picked, or closes when it genuinely leaves.
//
// Empty string = nothing selected / detail closed.
static char s_selectedHex[8] = "";

// Grace period for the detail panel. adsb.fi sometimes omits an aircraft from a
// single poll and sends it again in the next one; closing the panel on the
// first miss looks like it closes by itself. We therefore keep the last known
// data and only give up after DETAIL_GRACE_POLLS consecutive misses.
static int      s_selMiss = 0;          // consecutive polls without the aircraft
static Aircraft s_selCache;             // last known data of the selected one
static bool     s_selCacheOk = false;

// Screen positions from the last draw, for tap-to-select. Parallel to the
// aircraft list, so s_planeHex[i] records which aircraft the point belongs to
// and the tap resolves to an identity rather than to a slot number.
static int  s_planeX[ADSB_MAX];
static int  s_planeY[ADSB_MAX];
static char s_planeHex[ADSB_MAX][8];
static int  s_planeN = 0;

bool ScreenPlanes_DetailOpen() { return s_selectedHex[0] != '\0'; }

// reason = short text for the serial log, so we can tell WHY the panel closed
// (user tap vs. the aircraft disappearing from the data).
static void selectNone(const char* reason) {
#if TOUCH_DEBUG
  if (s_selectedHex[0]) Serial.printf("SEL: zavreno (%s) hex=%s\n", reason, s_selectedHex);
#else
  (void)reason;
#endif
  s_selectedHex[0] = '\0';
  s_selMiss = 0;
  s_selCacheOk = false;
  Route_Clear();
}
static void selectHex(const char* hex) {
  strncpy(s_selectedHex, hex, sizeof(s_selectedHex) - 1);
  s_selectedHex[sizeof(s_selectedHex) - 1] = '\0';
  s_selMiss = 0;
  s_selCacheOk = false;
#if TOUCH_DEBUG
  Serial.printf("SEL: vybrano hex=%s\n", s_selectedHex);
#endif
}

// Convert an aircraft's lat/lon into screen coordinates (north up).
// --- Map orientation -------------------------------------------------------
// The user picks WHICH COMPASS BEARING IS AT THE TOP of the screen, i.e. the
// direction they are looking out of the window. Everything then follows:
//
//     screen angle of bearing b  =  b - topBearing      (0 = up, clockwise)
//
// So looking east (top = 90) puts north on the left, which is exactly where it
// is in reality. Setting an "amount to turn by" instead used to need the
// opposite value (360 - bearing) and felt like the map was mirrored.
//
// We do NOT rotate the display (the canvas only supports rotation 0 and turning
// the framebuffer would break touch mapping) - the PROJECTION is rotated.
// Borders and cities go through the same project(), so they follow along for
// free; only the aircraft icons need their heading corrected separately.
static float s_rotSin = 0.0f, s_rotCos = 1.0f;   // cached sin/cos of the angle
static uint16_t s_topDeg = 0;                    // bearing shown at the top

static void refreshRotation() {
  uint16_t deg = Settings_TopBearing();
  if (deg == s_topDeg) return;
  s_topDeg = deg;
  float r = (float)deg * 0.0174532925f;
  s_rotSin = sinf(r);
  s_rotCos = cosf(r);
}

static void project(float lat, float lon, double clat, double clon,
                    float rangeKm, int* sx, int* sy) {
  float latr = clat * 0.0174532925f;
  float dxKm = (lon - clon) * 111.0f * cosf(latr);
  float dyKm = (lat - clat) * 111.0f;
  // Rotate the local east/north vector so that the chosen bearing ends up at
  // the top: with top = 90 (looking east) a target due east appears up.
  float rx = dxKm * s_rotCos - dyKm * s_rotSin;
  float ry = dxKm * s_rotSin + dyKm * s_rotCos;
  float scale = (float)R_RADIUS / rangeKm;   // px per km
  *sx = R_CX + (int)(rx * scale);
  *sy = R_CY - (int)(ry * scale);
}

// Wrapper matching the ProjectFn signature (used to draw the map via EuBorder).
// Uses the current user location and the selected range.
static void cityProject(float lat, float lon, int* sx, int* sy) {
  project(lat, lon, Settings_Lat(), Settings_Lon(), currentRange(), sx, sy);
}

// --- Altitude colour bands ---
// The colour of an aircraft encodes its barometric altitude, so a glance tells
// you whether something is on approach overhead or just transiting at cruise.
// The bands are contiguous - every altitude falls into exactly one.
//
//   < 2 km   red     approach/departure, helicopters, light aircraft
//   2-6 km   orange  climb/descent, regional traffic
//   6-10 km  yellow  lower cruise levels
//   >= 10 km blue    long-haul cruise
//
// An aircraft reporting no altitude at all (altFt == 0 and not on the ground)
// is drawn grey rather than being forced into the "low" band, which would
// otherwise paint every unknown target an alarming red.
static uint16_t altColor(float altFt, bool known) {
  if (!known) return C_GRAY;
  float km = altFt * 0.0003048f;      // feet -> kilometres
  if (km <  2.0f) return C_RED;
  if (km <  6.0f) return C_ORANGE;
  if (km < 10.0f) return C_YELLOW;
  return C_BLUE;
}

// Aircraft icon - a winged arrow, rotated to match the ground track.
// Filled polygon; the nose points "forward" (locally up, fwd positive).
// When the track is unknown (hasTrack=false), a circle is drawn instead.
static void drawPlane(int x, int y, float trackDeg, bool hasTrack, uint16_t col) {
  if (!hasTrack) {
    // Track unknown - circle with a dot (orientation cannot be determined).
    gfx->drawCircle(x, y, 7, col);
    gfx->fillCircle(x, y, 2, col);
    return;
  }
  float a = trackDeg * 0.0174532925f;
  float ca = cosf(a), sa = sinf(a);
  // Local coordinates: right = to the right of the nose, fwd = forward (towards it).
  // Compass rotation: track 0 = up, 90 = right (east).
  //   screen_x = x + right*cos(a) + fwd*sin(a)
  //   screen_y = y - right*... ; derived so that fwd follows the track:
  //   nose (right=0, fwd=L) -> (x + L*sin(a), y - L*cos(a))
  auto rot = [&](float right, float fwd, int* ox, int* oy) {
    *ox = x + (int)(right * ca + fwd * sa);
    *oy = y + (int)(right * sa - fwd * ca);
  };
  // Arrow vertices (right, fwd). Nose ahead (fwd=+12), tail behind (fwd=-12).
  // Outline order: nose, right side, right wingtip, fuselage, right tailplane,
  //                rear centre, left tailplane, fuselage, left wingtip, left side
  const float P[10][2] = {
    { 0,  12}, { 3,  1}, { 13, -8}, { 3, -5}, { 3, -7},
    { 0, -12}, {-3, -7}, {-3, -5}, {-13, -8}, {-3,  1}
  };
  int px[10], py[10];
  for (int i = 0; i < 10; i++) rot(P[i][0], P[i][1], &px[i], &py[i]);

  // Fill as a triangle fan from the centre of the icon.
  for (int i = 0; i < 10; i++) {
    int j = (i + 1) % 10;
    gfx->fillTriangle(x, y, px[i], py[i], px[j], py[j], col);
  }
}

// --- Filters ----------------------------------------------------------------
// Set from the web UI. The point is not to hide traffic but to make a busy sky
// readable: over an approach path the interesting aircraft is the one at 2000
// ft, and at home under an airway it is the opposite.
//
// The filter applies to DRAWING only. The aircraft stays in the list, so the
// count, the emergency scan and a watched callsign all still see it - hiding an
// emergency because of an altitude filter would be a nasty surprise.
static bool passesFilter(const Aircraft& a) {
  if (Settings_OnlyWithCallsign() && !a.callsign[0]) return false;
  const uint16_t lo = Settings_AltMinFt(), hi = Settings_AltMaxFt();
  if (lo == 0 && hi >= 60000) return true;          // filter off
  if (a.altFt <= 0) return true;                    // unknown altitude - keep it
  if (a.altFt < (float)lo || a.altFt > (float)hi) return false;
  return true;
}

// Is this the aircraft the user asked to be told about? Matched against both
// the callsign and the ICAO address, because people quote whichever they have.
static bool isWatched(const Aircraft& a) {
  const char* w = Settings_WatchCallsign();
  if (!w || !*w) return false;
  if (a.callsign[0] && strcasecmp(a.callsign, w) == 0) return true;
  if (a.hex[0] && strcasecmp(a.hex, w) == 0) return true;
  return false;
}

void ScreenPlanes_Enter() {
  s_rangeIdx = Settings_PlaneRange();
  if (s_rangeIdx >= RANGE_COUNT) s_rangeIdx = 1;   // guard against a stale value
  s_nextFetch = 0;
}

long lastDraw = 0;
long adsb_last_fetch;
float lastRangeFetched = 0;

bool ScreenPlanes_Tick() {
  if (WiFi.status() != WL_CONNECTED) { s_status = T(S_WIFI_WAIT); return false; }

  // Odpoved na trasu prijde nezavisle na stahovani letadel, obvykle do vteriny
  // po klepnuti. Bez tohohle by se dokreslila az s pristim pollem, takze u
  // stredniho dosahu az o deset sekund pozdeji, a panel by mezitim zbytecne
  // ukazoval "zjistuji trasu". Ctenim priznak zhasne, takze to prekresli jednou.
  bool routeChanged = Route_TakeChanged();

  if (millis() >= s_nextFetch) {
    s_status = T(S_DOWNLOADING);
    // range zaokrouhlíme na 10 km nahoru
    int range = ((int)(currentRange()/10.0)+1) * 10;
    lastRangeFetched = (float)range;
    s_dataOk = ADSB_Fetch(Settings_Lat(), Settings_Lon(), range );
    if( s_dataOk ) {
      adsb_last_fetch = millis();
    }
    s_status = s_dataOk ? T(S_OK) : T(S_ERROR);
    // Normal cadence depends on range; after a failure back off to double the
    // interval rather than hammering the API at the normal rate.
    unsigned long period = basePeriodMs();
    s_nextFetch = millis() + (s_dataOk ? period : period * 2);

    // Selection bookkeeping happens HERE (once per poll), not while drawing -
    // drawing runs many times per second and would burn the grace period in an
    // instant. The selection is an ICAO hex, so a reordered list cannot move it
    // onto a different aircraft.
    if (s_dataOk && ScreenPlanes_DetailOpen()) {
      if (ADSB_FindByHex(s_selectedHex) >= 0) {
        s_selMiss = 0;                       // still there
      } else if (++s_selMiss > DETAIL_GRACE_POLLS) {
        selectNone("letadlo zmizelo z dat");
      }
#if TOUCH_DEBUG
      else Serial.printf("SEL: chybi v datech (%d/%d) hex=%s\n",
                         s_selMiss, DETAIL_GRACE_POLLS, s_selectedHex);
#endif
    }
    lastDraw = millis();
    return true;   // new data -> redraw
  }
  if( routeChanged ) {
    lastDraw = millis();
    return true;
  }
  if ( millis()-lastDraw > SCREEN_PLANES_REFRESH ) {
    lastDraw = millis();
    return true;
  }
  return false;
}

// Short tap: with the detail open any tap closes it (the units toggle that used
// to sit at the bottom of the panel now lives on the settings screen).
// Otherwise select the nearest aircraft.
bool ScreenPlanes_HandleTap(int x, int y) {
  if (ScreenPlanes_DetailOpen()) {
    selectNone("tap mimo panel");
    return true;
  }
  // Find the aircraft nearest the tap (within 30 px), then remember *which
  // aircraft* it was, not where it happened to sit in the list.
  int best = -1;
  long bestD = 30L * 30L;
  for (int i = 0; i < s_planeN; i++) {
    long dx = s_planeX[i] - x, dy = s_planeY[i] - y;
    long d = dx * dx + dy * dy;
    if (d < bestD) { bestD = d; best = i; }
  }
  if (best >= 0 && s_planeHex[best][0]) {
    selectHex(s_planeHex[best]);
    // The callsign is only known from the data, so the lookup is kicked off
    // where the aircraft is drawn (below) - here we just have the hex.
    return true;
  }
  return false;
}

// Swipe: change the range (both directions). Re-fetch immediately at the new
// radius. Ignored while the detail panel is open (close it first).
void ScreenPlanes_ChangeRange(int dir) {
  if (ScreenPlanes_DetailOpen()) return;
  s_rangeIdx = (s_rangeIdx + dir + RANGE_COUNT) % RANGE_COUNT;
  Settings_SetPlaneRange(s_rangeIdx);   // remember across restarts (debounced)
  Serial.printf("ADSB range: %.0f km\n", currentRange());
  s_nextFetch = 0;
}

void ScreenPlanes_RangeText(char* out, size_t cap) {
  if (!out || !cap) return;
  snprintf(out, cap, "%.0f %s", currentRange(), T(S_KM));
}


int colorByOrder(int x)
{
  int color;
  switch(x)
  {
    case 0:
      color = EG_MAGENTA;
      break;
    case 1:
      color = C_ORANGE;
      break;
    default:
      color = C_GREEN;
  }
  return color;
}


int aircraft_detail_y;


void display_aircraft_detail(Aircraft ac, int headerColor, float vzdalenost_km )
{
    const bool metric = Settings_MetricUnits();
    gfx->setTextSize(1);

    char line[200];

    // Panel centred so it fits inside the round display.
    const int px = 530;
    const int pw = 800-px;
    const int py = aircraft_detail_y;

    int ty = py;

    painter->setFont(&vetsiPismo);
    painter->startText(px , py, pw, 480 - aircraft_detail_y );
    gfx->setTextColor(headerColor);

    // Same fallback as the map label: no callsign broadcast -> show the ICAO
    // address, which is at least something the user can look up. It is only
    // ever a caption; the route lookup below gets the callsign or nothing.
    int offset = painter->printText(ac.callsign[0] ? ac.callsign : (ac.hex[0] ? ac.hex : "?"));
    gfx->setTextColor( C_WHITE );
    sprintf( line, " (%.1f km)", vzdalenost_km );
    painter->printText( line, offset );
    painter->textLf();

    // painter->setFont(&malePismo);
    gfx->setTextColor(C_WHITE);

    const char* desc = ac.desc[0] ? ac.desc : 
                                              (ac.type[0] ? ac.type : nullptr);
    if (desc) {
      // The label plus a long type plus a registration can outgrow the panel.
      painter->printText( desc );
      painter->textLf();
    }

    if( ac.reg[0] ) {
      sprintf(line, "%s: %s", TW(S_TYPE),
              ac.reg[0] ? ac.reg : "");
      painter->printText( line );
      painter->textLf();
    }

    // Altitude: ft or m. + baro rate
    sprintf( line, "%s: ", TW(S_ALTITUDE) );
    if (metric) sprintf(line + strlen(line), "%.0f m", ac.altFt * 0.3048f);
    else        sprintf(line + strlen(line), "%.0f ft", ac.altFt);
    // Climb/descent - short label + units so the line does not overflow.
    // An arrow instead of the words "climbing/descending".
    const char* ar = ac.baroRate > 100 ? "stoupá" : (ac.baroRate < -100 ? "klesá" : " ");
    /* if (metric) sprintf(line + strlen(line), ", %.1f m/s %s", ac.baroRate * 0.00508f, ar);
    else        sprintf(line + strlen(line), ", %.0f ft/m %s", ac.baroRate, ar); */
    // vertikální rychlost vždy ve ft/min
    sprintf(line + strlen(line), ", %s %.0f ft/m", ar, ac.baroRate );
    painter->printText( line );
    painter->textLf();

    // Speed: kt or km/h. + heading
    sprintf( line, "%s: ", TW(S_SPEED) );
    if (metric) sprintf(line + strlen(line), "%.0f km/h", ac.gsKt * 1.852f);
    else        sprintf(line + strlen(line), "%.0f kt", ac.gsKt);
    if (ac.hasTrack) sprintf(line + strlen(line), ", %.0f°", ac.track);
    painter->printText( line );
    painter->textLf();

    // Ask where this flight is going. The position goes with the request: the
    // server uses it to reject a route that does not fit where the aircraft
    // actually is, which is what used to put Athens - Istanbul on an aircraft
    // over Prague. Idempotent - a repeated call with the same aircraft does
    // nothing, and the answer is cached.
    Route_Select(ac.callsign, ac.lat, ac.lon);
    const RouteInfo* rt = Route_Get();

    // Route on TWO lines - "Z:" above "Do:". One line with an arrow between
    // the cities had to shrink to fit two names side by side and ended up
    // unreadably small; split in half, each line has the whole panel width to
    // itself and can be drawn much larger.
    //
    // The route starts at the same size and in the same colour as the flight
    // data above it - it is the same kind of information, and a different
    // colour only suggested a difference that is not there. It shrinks only
    // when it has to, because city names vary wildly ("Doha" against
    // "Frankfurt am Main"). Anything that still would not fit is cut and ends
    // with a full stop rather than running over the border.
    if (Route_GetState() == ROUTE_WAIT) {
      painter->printText( TW(S_ROUTE_WAIT) );
      painter->textLf();
    } else if (rt && (rt->from[0] || rt->to[0])) {
      char l1[50], l2[50];
      snprintf(l1, sizeof(l1), "%s: %s", TW(S_FROM), rt->from[0] ? rt->from : "?");
      snprintf(l2, sizeof(l2), "%s: %s", TW(S_TO), rt->to[0] ? rt->to : "?");
      painter->printText( l1 );
      painter->textLf();
      painter->printText( l2 );
      painter->textLf();
    }

    painter->textLf();
    aircraft_detail_y = painter->getY();
    painter->setFont(&malePismo);
}



// Close the detail panel (called on the long-press screen switch).
void ScreenPlanes_CloseDetail() { selectNone("dlouhy stisk / prepnuti obrazovky"); }

void ScreenPlanes_Draw() {
  painter->setFont(&malePismo);

  gfx->fillScreen(C_BLACK);
  Layout_Begin();
  refreshRotation();           // pick up a changed rotation setting
  float range = currentRange();

  // Reserve the chrome BEFORE the map is drawn. City labels and callsigns are
  // placed by data rather than by design, so they have to lose the argument -
  // and they can only lose it if the space is already taken when they ask.
  // Layout_ReserveBand(LY_DOTS - 6, 12);        // screen selector dots
  // Layout_ReserveBand(LY_STATUS - 3, 22);      // clock + outside temperature
  // The alert banner is taller than the count it replaces, and the legend has
  // its boundary numbers under the colour bar - both bands are sized for the
  // taller variant so a city label can never creep into either.
  // Layout_ReserveBand(LY_SUB - 6, 22);         // aircraft count / alert banner
  // Layout_ReserveBand(LY_LEGEND - 2, 24);      // altitude legend + numbers
  // Layout_ReserveBand(LY_RANGE - 2, 20);       // range readout
  // Layout_ReserveBand(LY_RANGE_DOTS - 6, 12);  // range dots

  // Nejprve kruhy, přes ně teprve popisky měst a hranice

  // Range rings and centre marker (2 circles, white cross).
  gfx->drawCircle(R_CX, R_CY, LCD_WIDTH / 2 - 2, C_DKGRAY);
  gfx->drawCircle(R_CX, R_CY, LCD_WIDTH / 4, C_DKGRAY);
  gfx->drawFastHLine(R_CX - 8, R_CY, 16, C_WHITE);
  gfx->drawFastVLine(R_CX, R_CY - 8, 16, C_WHITE);  

  // --- Compass marks ---
  // Bearing b appears on screen at angle (b - rotation), 0 = up, clockwise.
  // Small size-1 letters at r = 205 slot in between the screen dots (y=18),
  // the status line (y=30), the aircraft count (y=52) and the range row.
  {
    const int   cr = 230;
    const char* lbl[4] = { "S", "V", "J", "Z" };   // sever, vychod, jih, zapad
    const int   brg[4] = { 0, 90, 180, 270 };
    gfx->setTextSize(1);
    gfx->setTextColor(C_GRAY);
    for (int i = 0; i < 4; i++) {
      // Bearing b shows up at screen angle (b - topBearing).
      float a = (brg[i] - (int)s_topDeg) * 0.0174532925f;
      int cxp = R_CX + (int)(cr * sinf(a)) - 3;   // -3/-4 centres the glyph
      int cyp = R_CY - (int)(cr * cosf(a)) - 8;
      gfx->setCursor(cxp, cyp);
      gfx->print(lbl[i]);
    }
  }  

  // --- Map underlay ---
  // The map data spans the whole of Europe, so work out which slice of it can
  // actually be on screen and hand that window to EuBorder, which throws away
  // everything else before drawing. The radar circle has a radius of `range`
  // km, so the visible span is `range` in every direction; a 20% margin keeps
  // lines that only clip the edge of the view from being dropped.
  const double clat = Settings_Lat();
  const double clon = Settings_Lon();
  const float  marginKm = range * 1.2f;
  const float  dLat = marginKm / 111.0f;
  const float  dLon = marginKm / (111.0f * cosf(clat * 0.0174532925f));
  const float  lat0 = clat - dLat, lat1 = clat + dLat;
  const float  lon0 = clon - dLon, lon1 = clon + dLon;

  EuBorder_Draw(cityProject, C_GRAY, lat0, lat1, lon0, lon1);

  // Cities as an underlay (below the aircraft, so it is clear where they are).
  // Fewer of them survive as the range grows, otherwise a dense region such as
  // the Ruhr would bury the traffic under a wall of labels.
  {
    int rad = LCD_WIDTH / 2 - 4;
    bool showFull = (range <= 25.0f);    // full names only at close range
    uint8_t maxTier;
    // Tier 3 includes the Czech district towns, which is what makes the map
    // recognisable close to home; further out only the bigger places survive,
    // otherwise the labels would bury the traffic.
    if (range <= 50.0f) maxTier = 3;
    else                maxTier = 2;
    EuBorder_DrawCities(cityProject, R_CX, R_CY, rad, C_DKGRAY, C_GRAY,
                        showFull, maxTier, lat0, lat1, lon0, lon1);
  }

  // vymažeme prostor pro popisky
  gfx->fillRect(485, 0, 315, 480, C_BLACK);

  

  // Aircraft.
  int shown = 0;
  const Aircraft* list = ADSB_List();
  AircraftOrdering* ordering = ADSB_Ordering();
  int n = ADSB_Count();
  s_planeN = n;


  // nejprve si spočteme vzdálenost všech letadel - s extrapolací jejich pohybu
  for (int i = 0; i < n; i++) {
    ordering[i].idx = i;

    if (list[i].onGround) {
      ordering[i].distanceKm = 9999.0f;
      continue;
    }

    double lat, lon;
    if( !list[i].hasTrack ) {
      lat = list[i].lat;
      lon = list[i].lon;
    } else {
      Geo_ProjectPosition(
          list[i].lat,
          list[i].lon,
          list[i].gsKt,
          list[i].track,
          (millis()-adsb_last_fetch)/1000.0f,
          &lat,
          &lon
      );
    }
    ordering[i].distanceKm = Geo_DistanceKm( lat, lon, Settings_Lat(), Settings_Lon() );
  }

  // seřadíme podle vzdálenosti, nejbližší první
  for (int i = 0; i < n-1; i++) {
    for (int j = i+1; j < n; j++) {
      if( ordering[i].distanceKm > ordering[j].distanceKm ) {
        int idx = ordering[i].idx;
        float dist = ordering[i].distanceKm;
        ordering[i].distanceKm = ordering[j].distanceKm;
        ordering[i].idx = ordering[j].idx;
        ordering[j].distanceKm = dist;
        ordering[j].idx = idx;
      }
    }
  }
    
  painter->setFont(&vetsiPismo);

  const char* alertCode = nullptr;    // worst emergency seen this frame
  bool watchedSeen = false;

  double minDistance = 6000;

  aircraft_detail_y = 20;
  long now = millis();

  // nejprve vykreslíme značky letadel od nejvzdálenějšího k nejbližšímu - tak ty nejbližší přepíšou ty vzdálenější

  int i = 0;
  for (int x = n-1; x >= 0; x--) {
    if( ordering[x].distanceKm > currentRange() ) continue;

    i = ordering[x].idx;

    s_planeX[i] = -9999; s_planeY[i] = -9999;   // default: off-screen
    s_planeHex[i][0] = '\0';
    if (list[i].onGround) continue;

    // Scan for emergencies and the watched aircraft before the filter, so
    // neither can be hidden by an altitude setting.
    const char* em = Settings_SquawkAlert() ? ADSB_EmergencyCode(list[i]) : nullptr;
    const bool watched = isWatched(list[i]);
    if (em && !alertCode) alertCode = em;
    if (watched) watchedSeen = true;

    if (!em && !watched && !passesFilter(list[i])) continue;

    int sx, sy;
    double lat, lon;
    if( !list[i].hasTrack ) {
      lat = list[i].lat;
      lon = list[i].lon;
    } else {
      Geo_ProjectPosition(
          list[i].lat,
          list[i].lon,
          list[i].gsKt,
          list[i].track,
          (now-adsb_last_fetch)/1000.0f,
          &lat,
          &lon
      );
    }

    project(lat, lon, Settings_Lat(), Settings_Lon(), range, &sx, &sy);

    int dx = sx - R_CX, dy = sy - R_CY;
    if (dx * dx + dy * dy > R_RADIUS * R_RADIUS) continue;   // outside the circle
    // Store position *and* identity, so a tap resolves to an aircraft and not
    // to a list slot that may mean something else by the time it is used.
    s_planeX[i] = sx; s_planeY[i] = sy;
    strncpy(s_planeHex[i], list[i].hex, sizeof(s_planeHex[i]) - 1);
    s_planeHex[i][sizeof(s_planeHex[i]) - 1] = '\0';
    
    // An emergency gets a red ring, a watched aircraft a green one. Both are
    // drawn wider than the selection ring so they read at a glance.
    if (em)      { gfx->drawCircle(sx, sy, 20, C_RED);   gfx->drawCircle(sx, sy, 21, C_RED); }
    else if (watched) { gfx->drawCircle(sx, sy, 20, C_GREEN); gfx->drawCircle(sx, sy, 21, C_GREEN); }
    // Altitude is only meaningful when the aircraft actually reports it.
    bool altKnown = (list[i].altFt > 0.0f);
    // The map is turned, so the icon heading has to be corrected the same way -
    // otherwise the aircraft would point the wrong direction.
    float screenTrack = list[i].track - (float)s_topDeg;
    while (screenTrack < 0.0f) screenTrack += 360.0f;
    drawPlane(sx, sy, screenTrack, list[i].hasTrack,
              altColor(list[i].altFt, altKnown));
    
    // i pro značku letadla zarezervujeme prostor v Layoutu, aby jí další popisky nepřepsaly
    Layout_Reserve(sx-10, sy-10, 20, 20);

    // kolem prvních tří uděláme barevné kolečko 
    if(x<3) {
      gfx->drawCircle(sx, sy+1, 18, colorByOrder(x) );   
    }
    
    shown++;
  }
  
  // a teď vypíšeme popisky k letadlům, od nejbližšího k nejvzdálenějšímu, aby se popisky nepřekrývaly

  for (int x = 0; x < n; x++) {
    if( ordering[x].distanceKm > currentRange() ) continue;

    i = ordering[x].idx;

    s_planeX[i] = -9999; s_planeY[i] = -9999;   // default: off-screen
    s_planeHex[i][0] = '\0';
    if (list[i].onGround) continue;

    // Scan for emergencies and the watched aircraft before the filter, so
    // neither can be hidden by an altitude setting.
    const char* em = Settings_SquawkAlert() ? ADSB_EmergencyCode(list[i]) : nullptr;
    const bool watched = isWatched(list[i]);
    if (em && !alertCode) alertCode = em;
    if (watched) watchedSeen = true;

    if (!em && !watched && !passesFilter(list[i])) continue;

    int sx, sy;
    double lat, lon;
    if( !list[i].hasTrack ) {
      lat = list[i].lat;
      lon = list[i].lon;
    } else {
      Geo_ProjectPosition(
          list[i].lat,
          list[i].lon,
          list[i].gsKt,
          list[i].track,
          (now-adsb_last_fetch)/1000.0f,
          &lat,
          &lon
      );
    }

    project(lat, lon, Settings_Lat(), Settings_Lon(), range, &sx, &sy);

    int dx = sx - R_CX, dy = sy - R_CY;
    if (dx * dx + dy * dy > R_RADIUS * R_RADIUS) continue;   // outside the circle
    // Store position *and* identity, so a tap resolves to an aircraft and not
    // to a list slot that may mean something else by the time it is used.
    s_planeX[i] = sx; s_planeY[i] = sy;
    strncpy(s_planeHex[i], list[i].hex, sizeof(s_planeHex[i]) - 1);
    s_planeHex[i][sizeof(s_planeHex[i]) - 1] = '\0';
    
    // An emergency gets a red ring, a watched aircraft a green one. Both are
    // drawn wider than the selection ring so they read at a glance.
    if (em)      { gfx->drawCircle(sx, sy, 20, C_RED);   gfx->drawCircle(sx, sy, 21, C_RED); }
    else if (watched) { gfx->drawCircle(sx, sy, 20, C_GREEN); gfx->drawCircle(sx, sy, 21, C_GREEN); }
    // Altitude is only meaningful when the aircraft actually reports it.
    bool altKnown = (list[i].altFt > 0.0f);
    // The map is turned, so the icon heading has to be corrected the same way -
    // otherwise the aircraft would point the wrong direction.
    
    // pro první tři vypíšeme k nim detail vpravo
    if(x<3) {
      display_aircraft_detail(list[i], colorByOrder(x), ordering[x].distanceKm  );
    }

    // Label under the icon: the callsign, or the ICAO address when the aircraft
    // broadcasts none. The fallback lives HERE and nowhere else - Aircraft::
    // callsign is deliberately left empty in that case, because it is what gets
    // sent to the route API and a hex address there reads as a flight number
    // (see copyCallsign in ADSB.cpp). Drawing it is harmless; asking about it
    // is not.
    //const char* label = list[i].callsign[0] ? list[i].callsign : list[i].hex;
#define LABEL_SIZE 25
    char label[LABEL_SIZE];
    strncpy( label, list[i].callsign[0] ? list[i].callsign : list[i].hex, LABEL_SIZE-1 );
    label[LABEL_SIZE-1]=0;
    if( list[i].type[0]!=0 ) {
      int remain = LABEL_SIZE - strlen(label) - 3;
      if( remain>3 ) {
        strcat( label, " ");
        strncpy( label+strlen(label), list[i].type, remain-3 );
        label[LABEL_SIZE-1]=0;
      }
    }
    if (label[0]) {
      // Centred below the icon (radius ~14 px). It has to claim its space: two
      // aircraft close together used to print their labels straight through
      // each other, and near the rim one could land on the legend or the range
      // readout.
      int ty = sy + 16;
      int tx = sx;
      int16_t x1, y1;
      uint16_t w, h; 
      painter->getTextBounds(label, tx, ty, &x1, &y1, &w, &h);
      tx = sx - w/2;
      if( tx<0 ) tx = 0;
      if( tx+w > 480 ) tx = 480-w;
      if (Layout_Claim(tx, ty, w, h)) {
        if(x<3) {
          gfx->setTextColor( colorByOrder(x) );
        } else {
          gfx->setTextColor( C_WHITE );
        }
        painter->setFont( &malePismo );      
        painter->fillBackground( EG_BLACK, 1 );
        painter->printLabel( TextPainter::ALIGN_LEFT, tx, ty, label );
      }
    }
  }

  // Top of the screen, under the screen-selector dots at y=18:
  //   y 30..45  clock + outside temperature (size 2, the same as the range)
  //   y 52..59  aircraft count (size 1 - it is a secondary number)
  //   y 74      altitude legend
  UI_DrawStatusLine(LY_STATUS);


  painter->setFont( &malePismo );      
  painter->fillBackground( EG_BLACK );
  int tx = 1, ty = 26;
  
  // The line under the clock: normally the aircraft count, but an emergency
  // squawk takes it over. There is exactly one line for this, so the two can
  // never overlap - the alert simply outranks the count.
  char sub[40];
  if (alertCode) {
    const char* what = (strcmp(alertCode, SQUAWK_HIJACK) == 0) ? TW(S_HIJACK)
                     : (strcmp(alertCode, SQUAWK_RADIO)  == 0) ? TW(S_RADIO_FAIL)
                                                               : TW(S_EMERGENCY);
    snprintf(sub, sizeof(sub), "%s  %s", alertCode, what);
    gfx->setTextColor(C_YELLOW);
    painter->printLabel( TextPainter::ALIGN_LEFT, tx, ty, sub );
    ty += 18;
  }

  if (WiFi.status() != WL_CONNECTED || !s_dataOk) {
    gfx->setTextColor(C_YELLOW);
    painter->printLabel( TextPainter::ALIGN_LEFT, tx, ty, (char *)s_status.c_str() );
    ty += 18;
  } 

  snprintf(sub, sizeof(sub), "%s: %d", T(S_AIRCRAFT), shown );
  gfx->setTextColor( C_WHITE );
  painter->printLabel( TextPainter::ALIGN_LEFT, tx, ty, sub );
  ty += 18;

  sprintf(sub, "Zoom: %.0f km", range);
  gfx->setTextColor(C_WHITE);
  painter->printLabel( TextPainter::ALIGN_LEFT, tx, ty, sub );

  // --- Altitude legend ---
  // The icon colour means nothing without a key. Four swatches in a continuous
  // bar, with the band boundaries (2 / 6 / 10 km) marked underneath. It sits
  // below the aircraft count, where the round panel still has room - the bottom
  // of the screen is already taken by the range readout.
  {
    const uint16_t cols[4] = {C_RED, C_ORANGE, C_YELLOW, C_BLUE};
    const char*    bounds[3] = {"2", "6", "10"};   // km, at the band edges
    const int sw = 26;                 // swatch width
    const int barW = 4 * sw;
    const int lx = 0;
    const int ly = 455;

    // Continuous bar - the bands abut, matching the fact that the altitude
    // ranges themselves are contiguous with no gaps between them.
    for (int i = 0; i < 4; i++) {
      gfx->fillRect(lx + i * sw, ly, sw, 6, cols[i]);
    }

    // Boundary numbers, centred on each internal edge of the bar.
    painter->setFont( &malePismo );      
    gfx->setTextColor(C_GRAY);
    for (int i = 0; i < 3; i++) {
      int edge = lx + (i + 1) * sw;              // border between band i and i+1
      int tw = strlen(bounds[i]) * 6;
      gfx->setCursor(edge - tw / 2, ly + 8);
      gfx->print(bounds[i]);
    }
    // Unit, just past the right end of the bar.
    gfx->setCursor(lx + barW + 4, ly + 8);
    gfx->print("km");

    gfx->setTextColor(C_GRAY);
    painter->noBackground();
    painter->printLabel( TextPainter::ALIGN_LEFT, lx, ly-19, (char*)"Výška" );    
  }
  
  // přepočítáme zoom
  if( shown > maxPlanes && currentZoom > minZoom ) {
    // změna dolů je jednoduchá
    currentZoom -= 1;
    Serial.printf("ScreenPlanes: too many planes (%d), zoom in to %.0f km\n", shown, currentRange());
  }
  if( shown < minPlanes ) {
    // Nahoru můžeme snadno jen do lastRangeFetched, pak už se nám počet letadel nezvětší
    // Maximum je tedy lastRangeFetched + 1, abychom při příštím načítání načetli o 10 km více.
    // ale jen do stropu maxZoom
    if( currentZoom < (lastRangeFetched + 1)  &&   currentZoom < maxZoom) {
      currentZoom += 1;
      Serial.printf("ScreenPlanes: too few planes (%d), zoom out to %.0f km\n", shown, currentRange());
    }
  }
  
/*
  // Range indicator at the bottom (moved up a little to leave room for the
  // southern compass mark).
  char rbuf[16];
  snprintf(rbuf, sizeof(rbuf), "%.0f km", range);
  UI_TextCentered(rbuf, LY_RANGE, C_YELLOW, 2);

  int dotGap = 24;
  int totalW = (RANGE_COUNT - 1) * dotGap;
  int startX = R_CX - totalW / 2;
  int dotY = LY_RANGE_DOTS;
  for (int i = 0; i < RANGE_COUNT; i++) {
    int x = startX + i * dotGap;
    if (i == s_rangeIdx) gfx->fillCircle(x, dotY, 5, C_YELLOW);
    else                 gfx->drawCircle(x, dotY, 5, C_GRAY);
  }
  */
}
