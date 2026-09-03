// =============================================================================
//  MeteoPlaneRadar - trasa letu z adsb.lol (poloha + priznak verohodnosti).
//  Duvody, proc prave takhle, jsou v Route.h.
//
//  Author:  Petr / chiptron.cz   (vyvoj / development: chiptron.cz)
// =============================================================================
#include "Route.h"
#include "Config.h"
#include "NetSink.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

static void (*s_poll)() = nullptr;
void Route_SetPollFn(void (*fn)()) { s_poll = fn; }

// --- Text sanitising --------------------------------------------------------
// adsb.lol vraci nazvy mest v Unicode - Izmir prijde jako U+0130 ("I" s teckou,
// turecke velke I) plus "zmir", tedy dva bajty tam, kde font ceka jeden.
// Vestaveny GFX font je 7bitovy, takze se ty bajty vykreslily jako dva nahodne
// glyfy a na displeji stalo neco jako "-?zmir". Vsechno se proto sklada dolu
// na ASCII: diakritika pada, pismeno pod ni zustava. Dalsi realne pripady
// z odpovedi adsb.lol: Krakow (U+0142), Malaga (U+00E1).
static const char LAT1_MAP[65]  = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTsaaaaaaaceeeeiiiidnooooo/ouuuuyty";   // U+00C0..U+00FF
static const char LATA_MAP[129] = "AaAaAaCcCcCcCcDdDdEeEeEeEeEeGgGgGgGgHhHhIiIiIiIiIiIiJjKkkLlLlLlLlLlNnNnNnnNnOoOoOoOoRrRrRrSsSsSsSsTtTtTtUuUuUuUuUuUuWwYyYZzZzZzs";   // U+0100..U+017F

static void toAscii(char* dst, size_t cap, const char* src) {
  size_t o = 0;
  for (const unsigned char* p = (const unsigned char*)src; *p && o + 1 < cap; ) {
    unsigned char c = *p;
    uint32_t cp;
    if (c < 0x80)             { cp = c;                                p += 1; }
    else if ((c & 0xE0) == 0xC0 && p[1]) { cp = ((c & 0x1F) << 6) | (p[1] & 0x3F);  p += 2; }
    else if ((c & 0xF0) == 0xE0 && p[1] && p[2]) {
      cp = ((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);  p += 3;
    } else { p += 1; continue; }            // rozbity bajt - preskoc

    char out;
    if (cp < 0x80)                       out = (char)cp;
    else if (cp >= 0xC0 && cp <= 0xFF)   out = LAT1_MAP[cp - 0xC0];
    else if (cp >= 0x100 && cp <= 0x17F) out = LATA_MAP[cp - 0x100];
    else                                 continue;   // hadat nema cenu
    if (out < 0x20) continue;
    dst[o++] = out;
  }
  dst[o] = '\0';
}

// --- Cache ------------------------------------------------------------------
// Klicem je callsign (hex uz se sem neposila). Round-robin nahrazovani: pri
// tomhle poctu polozek stoji cokoli chytrejsiho vic, nez to usetri.
//
// Zaznam ma platnost. Bez ni by keska drzela odpoved do restartu, coz je
// spatne hned dvakrat:
//   1) Vysledek plati k POLOZE, se kterou se ptalo. Server podle ni pocital
//      "plausible" a u vicenohe trasy jsme podle ni vybrali usek. Letadlo
//      z Prahy do Dubaje pres Istanbul by po mezipristani ukazovalo porad
//      prvni usek, protoze odpoved se ulozila, kdyz bylo nad Ceskem.
//   2) Callsigny se recykluji. Zitrejsi OKL123 muze byt jiny let nez ten
//      dnesni - a prave tohle mel priznak "plausible" resit, jenze zapamatovana
//      odpoved uz zadnou kontrolu polohy neprojde.
// Cisla odpovidaji tomu, jak dlouho drzi svou kes samotne adsb.lol.
#define ROUTE_TTL_OK_MS    1200000UL   // 20 min - nalezena verohodna trasa
#define ROUTE_TTL_NONE_MS   120000UL   // 2 min  - "trasa neni"; letadlo tesne
                                       // po vzletu ji casto dostane az pozdeji

struct Entry {
  char          key[12] = "";
  RouteState    state   = ROUTE_IDLE;
  unsigned long stamp   = 0;    // millis() zapisu vysledku
  RouteInfo     info;
  float lat;
  float lon;
};
static Entry s_cache[ROUTE_CACHE_N];
static int   s_next = 0;

static Entry* findRequest() {
  for (int i = 0; i < ROUTE_CACHE_N; i++)
    if( s_cache[i].state==ROUTE_WAIT ) return &s_cache[i];
  return nullptr;
}


static Entry* find(const char* key) {
  if (!key || !*key) return nullptr;
  for (int i = 0; i < ROUTE_CACHE_N; i++)
    if (strncmp(s_cache[i].key, key, sizeof(s_cache[i].key)) == 0) return &s_cache[i];
  return nullptr;
}

// Je zaznam uz moc stary? Odecita se v unsigned, takze prsteneni millis()
// po 49 dnech nevadi - a tenhle displej bezi nepretrzite, takze na to dojde.
// Zamerne se NEvola z find(): kdyby zaznam vyprsel pod rukama zrovna
// otevrenemu detailu, panel by zhasnul a Route_Select() by se uz nezeptal,
// protoze klic se nezmenil. Platnost se tedy resi jen pri VYBERU letadla.
static bool expired(const Entry* e) {
  if (e->state == ROUTE_WAIT || e->state == ROUTE_IDLE) return false;   // jeste se ptame
  unsigned long ttl = (e->state == ROUTE_OK) ? ROUTE_TTL_OK_MS : ROUTE_TTL_NONE_MS;
  return (millis() - e->stamp) > ttl;
}

static Entry* insert(const char* key) {
  // Nejdriv volne misto - po vyprseni nebo po neuspesnem dotazu zaznamy mizi,
  // a bylo by hloupe prepsat platnou odpoved, kdyz vedle zeje prazdny slot.
  Entry* e = nullptr;
  for (int i = 0; i < ROUTE_CACHE_N && !e; i++)
    if (!s_cache[i].key[0]) e = &s_cache[i];
  if (!e) {                       // vsechno obsazeno - round robin
    e = &s_cache[s_next];
    s_next = (s_next + 1) % ROUTE_CACHE_N;
  }
  *e = Entry();          // ne memset - Entry ma inicializatory clenu
  strncpy(e->key, key, sizeof(e->key) - 1);
  e->state = ROUTE_WAIT;
  return e;
}

// Orizne mezery z obou stran - adsb.fi doplnuje callsign na osm znaku.
static void trim(char* s) {
  int n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
  int lead = 0;
  while (s[lead] == ' ' || s[lead] == '\t') lead++;
  if (lead) memmove(s, s + lead, strlen(s + lead) + 1);
}

// Callsign jde primo do cesty URL, takze do nej nesmi nic, co by ji rozbilo
// nebo prepsalo ('/', '?', '%', mezera). Realne callsigny jsou vzdy jen
// pismena a cislice; cokoli jineho znamena poskozena data a dotaz se
// neprovede vubec, misto aby se cokoli escapovalo.
static bool callsignSane(const char* s) {
  if (!s || !*s) return false;
  for (const char* p = s; *p; p++)
    if (!isalnum((unsigned char)*p)) return false;
  return true;
}

void Route_Select(const char* callsign, float lat, float lon) {
  char key[12] = "";
  if (callsign) strncpy(key, callsign, sizeof(key) - 1);
  trim(key);
  // Letadlo bez callsignu (TIS-B, MLAT, soukrome a vojenske stroje) se na
  // trasu neptame vubec. Drive se misto nej posilal ICAO hex a ten po
  // normalizaci vypada jako platny IATA let: "a31234" -> "A31234" je Aegean
  // Airlines 1234, takze letadlo nad Prahou dostalo trasu Atheny -> Istanbul.
  if (!callsignSane(key)) { return; }

  Entry* e = find(key);
  if (e && expired(e)) { *e = Entry(); e = nullptr; }   // stara odpoved se zahodi
  if (e) {
    // Zaznam uz existuje. Pokud ceka na nactenim, nastavime mu aktualni pozici.
    if( e->state == ROUTE_WAIT ) {
      e->lat = lat;
      e->lon = lon;
    }
    return;
  }
  e = insert(key);
  e->lat = lat;
  e->lon = lon;  
}

RouteState Route_GetState( const char* callsign ) {
  Entry* e = find(callsign);
  return e ? e->state : ROUTE_IDLE;
}

const RouteInfo* Route_Get( const char* callsign ) {
  Entry* e = find(callsign);
  return (e && e->state == ROUTE_OK) ? &e->info : nullptr;
}


// --- Fetching ---------------------------------------------------------------
// Vzdusna vzdalenost v km. Slouzi jen k porovnavani useku mezi sebou, takze
// na presnem polomeru Zeme nezalezi.
static float haversineKm(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0f, D = 0.017453293f;   // km, stupne -> radiany
  float dLat = (lat2 - lat1) * D, dLon = (lon2 - lon1) * D;
  float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
            cosf(lat1 * D) * cosf(lat2 * D) * sinf(dLon * 0.5f) * sinf(dLon * 0.5f);
  if (a < 0) a = 0; else if (a > 1) a = 1;
  return 2.0f * R * asinf(sqrtf(a));
}

// Popisek letiste: mesto ("Prague") se cte lip nez kod, ale u malych letist
// byva prazdne - pak IATA.
static void airportLabel(char* dst, size_t cap, JsonVariantConst ap) {
  const char* v = ap["location"].is<const char*>() ? ap["location"].as<const char*>() : nullptr;
  if (!v || !*v) v = ap["iata"].is<const char*>() ? ap["iata"].as<const char*>() : nullptr;
  if (!v) { dst[0] = '\0'; return; }
  toAscii(dst, cap, v);     // font neumi nic nez ASCII
}

// Jeden GET s filtrem. Vraci HTTP kod (nebo zaporny kod HTTPClient),
// pri uspechu 200 a rozparsovanym dokumentem.
static int getJson(const char* url, JsonDocument& filter, JsonDocument& doc) {
  WiFiClientSecure client; client.setInsecure();
  client.setHandshakeTimeout(NET_TLS_HANDSHAKE_S);
  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(8000);
  http.setReuse(false);
  if (!http.begin(client, url)) return -1000;
  // POZOR: setUserAgent(), NE addHeader(). ESP32 HTTPClient::addHeader() mlcky
  // zahodi Connection, Host, Accept-Encoding a prave User-Agent - vidi je jako
  // "handled by code" a nic nenahlasi. Nez se na to prislo, odchazela porad
  // vychozi "ESP32HTTPClient", adsb.lol na ni vracelo 403 "User-Agent too
  // generic" a v logu pritom svitila hlavicka, kterou nikdo neposlal.
  http.setUserAgent(HTTP_USER_AGENT);
  http.addHeader("Accept", "application/json");
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return code; }
  // Buffer the body first. http.getStream() is the RAW socket and does not
  // strip chunked encoding, so parsing straight off it would read the hex
  // block size as a value and report Ok on an empty document - the same
  // failure that silently blanked the aircraft radar.
  static uint8_t* s_buf = nullptr;
  static const size_t ROUTE_MAX = 8192;
  if (!s_buf) {
    s_buf = (uint8_t*)heap_caps_malloc(ROUTE_MAX, MALLOC_CAP_SPIRAM);
    if (!s_buf) s_buf = (uint8_t*)malloc(ROUTE_MAX);
    if (!s_buf) { http.end(); return -1002; }
  }
  long len = Net_ReadBody(http, s_buf, ROUTE_MAX, "ROUTE", s_poll);
  http.end();
  if (len <= 0) return -1001;

  DeserializationError err = deserializeJson(doc, s_buf, (size_t)len,
                                             DeserializationOption::Filter(filter));
  return err ? -1001 : HTTP_CODE_OK;
}

void Route_Tick() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!Net_HeapOk("ROUTE")) return;    // zkusi se znovu pristi kolo

  Entry* e = findRequest();
  if (!e) return;

  // Filtr pousti jen to, co se opravdu pouzije - dokument musi zustat maly.
  // U pole staci popsat prvni prvek, ArduinoJson ho aplikuje na vsechny.
  JsonDocument filter;
  filter["airport_codes"] = true;
  filter["plausible"]     = true;
  JsonObject ap = filter["_airports"].add<JsonObject>();
  ap["location"] = true;
  ap["iata"]     = true;
  ap["lat"]      = true;
  ap["lon"]      = true;

  JsonDocument doc;
  char url[160];
  snprintf(url, sizeof(url), "%s/%s/%.4f/%.4f",
           ROUTE_API_BASE, e->key, e->lat, e->lon );
  Serial.printf("TRASA %s: dotaz na %s\n", e->key, url);
  int code = getJson(url, filter, doc);
  if (code != HTTP_CODE_OK) {
    // Neuspech se NEZAPAMATUJE: zaznam se z kese zahodi misto ulozeni jako
    // "trasa neni". Server obcas vrati 500 u callsignu, ktery jeste nema
    // predpocitany (napr. DLH400 mimo cas letu), a zapamatovana chyba by
    // znamenala, ze uz se na nej do restartu nikdy nezeptame. Takhle se dotaz
    // zopakuje, jakmile uzivatel detail otevre znovu. Panel zatim neukaze nic.
    
    char key[12]; strncpy(key, e->key, sizeof(key)); key[sizeof(key) - 1] = '\0';
    // *e = Entry();

    // Záznam se uloží, protože nechceme na server posílat dotaz pořád dokola, pokud je problém s připojením nebo serverem.
    Serial.printf("TRASA %s: dotaz selhal (%d)\n", key, code);
    e->state = ROUTE_NONE; e->stamp = millis();
    return;
  }

  const char* codes = doc["airport_codes"] | "unknown";
  // Pri nenalezene trase pole "plausible" v odpovedi vubec neni, takze
  // vychozi hodnota musi byt false - jinak by neznama trasa prosla jako
  // verohodna.
  bool plausible = doc["plausible"] | false;

  if (strcmp(codes, "unknown") == 0 || !plausible) {
    e->state = ROUTE_NONE; e->stamp = millis();
    Serial.printf("TRASA %s: %s\n", e->key,
                  strcmp(codes, "unknown") == 0 ? "neni v databazi"
                                                : "nalezena, ale neverohodna k poloze");
    return;
  }

  JsonArrayConst aps = doc["_airports"].as<JsonArrayConst>();
  int n = aps.isNull() ? 0 : (int)aps.size();
  if (n < 2) {
    // Kody trasy jsou, ale letiste se v databazi nenasla - neni co popsat.
    e->state = ROUTE_NONE; e->stamp = millis();
    Serial.printf("TRASA %s: %s (letiste chybi)\n", e->key, codes);
    return;
  }

  // Mezipristani: u vicenohe trasy (napr. "LKPR-LTFM-OMDB") se drive ukazal
  // prvni odlet a posledni prilet, i kdyz letadlo letelo prostredni usek.
  // Vezme se ta sousedni dvojice letist, ke ktere je letadlo souhrnne nejbliz.
  int best = 0;
  if (n > 2) {
    float bestD = 1e30f;
    for (int i = 0; i + 1 < n; i++) {
      float d = haversineKm(e->lat, e->lon, aps[i]["lat"] | 0.0f, aps[i]["lon"] | 0.0f) +
                haversineKm(e->lat, e->lon, aps[i + 1]["lat"] | 0.0f, aps[i + 1]["lon"] | 0.0f);
      if (d < bestD) { bestD = d; best = i; }
    }
  }

  airportLabel(e->info.from, sizeof(e->info.from), aps[best]);
  airportLabel(e->info.to,   sizeof(e->info.to),   aps[best + 1]);

  bool any = (e->info.from[0] || e->info.to[0]);
  e->state = any ? ROUTE_OK : ROUTE_NONE;
  e->stamp = millis();   // od ted bezi platnost, viz expired()
  Serial.printf("TRASA %s: %s -> %s (%s, usek %d/%d)\n", e->key,
                e->info.from[0] ? e->info.from : "?",
                e->info.to[0]   ? e->info.to   : "?",
                codes, best + 1, n - 1);
}
