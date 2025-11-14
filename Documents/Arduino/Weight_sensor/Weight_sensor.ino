// ESP32 + HX711 grams (non-blocking) + I2C LCD (LiquidCrystal_I2C)
// - Single-line CLI: t, c <grams>, r, s, p, h
// - State machine (no delay, no blocking reads)
// Libraries (Library Manager):
//   - Rob Tillaart "HX711"
//   - "LiquidCrystal I2C" (johnrickman or marcoschwartz variant supports ESP32)

#include <HX711.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ===== LCD =====
const uint8_t LCD_ADDR = 0x27;  // try 0x3F if nothing shows
const uint8_t LCD_COLS = 16;    // set 20 for 20x4
const uint8_t LCD_ROWS = 2;     // set 4 for 20x4
LiquidCrystal_I2C g_lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ===== Pins (ESP32) =====
const uint8_t PIN_DOUT = 19;    // HX711 DOUT
const uint8_t PIN_SCK  = 18;    // HX711 SCK
// If you wired different I2C pins, call Wire.begin(SDA, SCL) in setup.

// ===== Timing =====
const uint32_t READ_INTERVAL_MS   = 100; // Read sensor every 100ms
const uint32_t PRINT_INTERVAL_MS  = 200;  // Update display every 200ms
const uint32_t PWRDN_MS           = 1000; //  Keep HX711 off for 1 second during reset

// ===== Filtering =====
const float G_ALPHA      = 0.30f; // how much to smooth
const float G_DEADBAND_G = 0.20f;

// ===== Variance checker =====
const uint8_t VAR_N      = 8;          // ring-buffer length
const uint32_t VAR_SPACING_MS = 64;    // ~ every 64 ms we push a new sample
const float VAR_THRESH_G2 = 0.05f;     // variance < 0.05 g²  -> stable
const uint32_t VAR_STABLE_MS = 1500;   // must be stable for 1.5 s

// ===== Globals =====
HX711 g_scale;
Preferences g_prefs;

float g_filtered_g = 0.0f;
bool  g_show_raw   = false;

float g_scale_fac  = 1.0f;   // HX711 divisor (so get_units()==grams after cal)
long  g_offset     = 0;

uint32_t g_t_next_read  = 0;
uint32_t g_t_next_print = 0;

// CLI buffer
char g_linebuf[32]; // store what user type
uint8_t g_linepos = 0;

// State machine
enum g_state_t { G_IDLE, G_TARE, G_CAL, G_PWRDN, G_PWRUP };
g_state_t g_state = G_IDLE;

struct { uint16_t target=20, got=0; int64_t sum=0; } g_tare;
struct { float known=0; uint16_t target=10, got=0; double sum_units=0; } g_cal;
uint32_t g_t_power_mark = 0;

// LCD line caches to prevent flicker
String g_lcd_prev[4];

// ===== Variance helper =====
struct {
  float buf[VAR_N];
  uint8_t idx = 0;
  uint32_t t_last_push = 0;
  uint32_t t_stable_start = 0;
  bool stable = false;
} g_var;

void var_push(float g) {
  uint32_t now = millis();
  if (now - g_var.t_last_push < VAR_SPACING_MS) return;
  g_var.t_last_push = now;

  g_var.buf[g_var.idx] = g;
  g_var.idx = (g_var.idx + 1) % VAR_N;

  // compute variance
  float sum = 0, sum2 = 0;
  for (uint8_t i = 0; i < VAR_N; ++i) {
    sum  += g_var.buf[i];
    sum2 += g_var.buf[i] * g_var.buf[i];
  }
  float mean = sum / VAR_N;
  float var  = (sum2 / VAR_N) - (mean * mean);

  if (var < VAR_THRESH_G2) {
    if (g_var.t_stable_start == 0) g_var.t_stable_start = now;
    if (now - g_var.t_stable_start >= VAR_STABLE_MS) g_var.stable = true;
  } else {
    g_var.t_stable_start = 0;
    g_var.stable = false;
  }
}

// ===== Small helpers =====
static inline String stateToStr() {
  switch (g_state) {
    case G_IDLE:  return "IDLE";
    case G_TARE:  return "TARE";
    case G_CAL:   return "CAL";
    case G_PWRDN: return "PWRDN";
    case G_PWRUP: return "PWRUP";
  }
  return "?";
}

void lcdWriteLine(uint8_t row, const String& text) {
  if (row >= LCD_ROWS) return;
  String out = text;
  // pad/trim to LCD_COLS (avoid String(' ', N) ambiguity on ESP32)
  if (out.length() < LCD_COLS) {
    size_t need = LCD_COLS - out.length();
    for (size_t i = 0; i < need; i++) out += ' ';
  } else if (out.length() > LCD_COLS) {
    out.remove(LCD_COLS);
  }
  if (out != g_lcd_prev[row]) {
    g_lcd_prev[row] = out;
    g_lcd.setCursor(0, row);
    g_lcd.print(out);
  }
}

// ===== Persistence =====
void apply_cal() {
  g_scale.set_scale(g_scale_fac);
  g_scale.set_offset(g_offset);
}
void save_cal() {
  g_prefs.begin("hx711", false);
  g_prefs.putFloat("scale", g_scale_fac);
  g_prefs.putLong("offset", g_offset);
  g_prefs.end();
  Serial.println(F("Calibration saved to memory!"));
}
bool load_cal() {
  g_prefs.begin("hx711", true);
  bool ok = g_prefs.isKey("scale") && g_prefs.isKey("offset");
  if (ok) {
    g_scale_fac = g_prefs.getFloat("scale", 1.0f);
    g_offset    = g_prefs.getLong("offset", 0);
  }
  g_prefs.end();
  if (ok) apply_cal();
  return ok;
}

// ===== Menu / Info =====
void print_menu() {
  Serial.println();
  Serial.println(F("=== HX711 Menu (non-blocking) ==="));
  Serial.println(F("t           : tare"));
  Serial.println(F("c <grams>   : calibrate (e.g., 'c 500')"));
  Serial.println(F("r           : toggle raw/grams"));
  Serial.println(F("s           : show settings"));
  Serial.println(F("p           : power-cycle HX711"));
  Serial.println(F("h           : help"));
  Serial.println();
}
void show_settings() {
  Serial.print(F("scale = "));  Serial.println(g_scale_fac, 6);
  Serial.print(F("offset = ")); Serial.println(g_offset);
  Serial.print(F("mode  = "));  Serial.println(g_show_raw ? F("RAW") : F("GRAMS"));
  Serial.print(F("state = "));  Serial.println(stateToStr());
}

// ===== Actions (non-blocking) =====
void start_tare(uint16_t samples=20) {
  if (g_state != G_IDLE) return;
  g_tare.target = samples; g_tare.got = 0; g_tare.sum = 0;
  g_state = G_TARE;
  g_var.stable = false;
  g_var.t_stable_start = 0;
  Serial.println(F("[..] Tare started. Remove all weight."));
  lcdWriteLine(0, "REMOVE WEIGHT");
  lcdWriteLine(1, "Zeroing ...");
}
void progress_tare() {
  if (!g_scale.is_ready()) return;
  long raw = g_scale.read();
  g_tare.sum += raw; g_tare.got++;
  if (g_tare.got >= g_tare.target) {
    int64_t avg = g_tare.sum / (int32_t)g_tare.target;
    g_offset = (long)avg; apply_cal();
    Serial.print(F("[OK] Tare complete. New offset = "));
    Serial.println(g_offset);
    lcdWriteLine(0, "Tare OK");
    lcdWriteLine(1, "Ready");
    g_state = G_IDLE;
  }
}

void start_cal(float known_g, uint16_t samples=10) {
  if (g_state != G_IDLE) return;
  if (known_g <= 0) { Serial.println(F("[ERR] Mass must be > 0 g")); return; }
  g_cal.known = known_g; g_cal.target = samples; g_cal.got = 0; g_cal.sum_units = 0;
  g_state = G_CAL;
  Serial.print(F("[..] Cal with ")); Serial.print(known_g, 2); Serial.println(F(" g. Keep steady."));
}
void progress_cal() {
  if (g_scale.is_ready()) {
    long raw = g_scale.read();
    double units_now = (raw - (double)g_offset) / (double)g_scale_fac;
    g_cal.sum_units += units_now; g_cal.got++;
    if (g_cal.got >= g_cal.target) {
      double avg_units = g_cal.sum_units / (double)g_cal.target;
      // Make get_units() == grams: new_scale = old_scale * (avg_units / known_g)
      g_scale_fac = g_scale_fac * (avg_units / (double)g_cal.known);
      apply_cal();
      Serial.print(F("[OK] Calibrated. scale = ")); Serial.println(g_scale_fac, 6);
      save_cal();
      g_state = G_IDLE;
    }
  }
}

void start_power_cycle() {
  if (g_state != G_IDLE) return;
  g_scale.power_down();
  g_t_power_mark = millis();
  g_state = G_PWRDN;
  Serial.println(F("[..] HX711 power-down..."));
}
void progress_power_cycle() {
  uint32_t now = millis();
  if (g_state == G_PWRDN) {
    if (now - g_t_power_mark >= PWRDN_MS) {
      g_scale.power_up();
      g_t_power_mark = now;
      g_state = G_PWRUP;
      Serial.println(F("[..] HX711 power-up..."));
    }
  } else if (g_state == G_PWRUP) {
    if (now - g_t_power_mark >= 200) { // settle time, non-blocking
      Serial.println(F("[OK] HX711 ready."));
      g_state = G_IDLE;
    }
  }
}

// ===== Serial line parser (non-blocking) =====
void handle_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_linebuf[g_linepos] = '\0'; g_linepos = 0;
      if (!g_linebuf[0]) return;
      char cmd = g_linebuf[0];
      float arg = 0.0f;
      if (strlen(g_linebuf) > 2) arg = atof(g_linebuf + 2);  // expects "c 500"
      switch (cmd) {
        case 't': case 'T': start_tare(); break;
        case 'c': case 'C': start_cal(arg); break;
        case 'r': case 'R': g_show_raw = !g_show_raw; Serial.printf("[DBG] raw=%s\n", g_show_raw ? "ON":"OFF"); break;
        case 's': case 'S': show_settings(); break;
        case 'p': case 'P': start_power_cycle(); break;
        case 'h': case 'H': print_menu(); break;
        default: Serial.println(F("[ERR] Unknown cmd. Use h for help.")); break;
      }
    } else {
      if (g_linepos < sizeof(g_linebuf)-1) g_linebuf[g_linepos++] = c;
      else g_linepos = 0; // overflow -> reset
    }
  }
}

// ===== Periodic sensor & LCD =====
bool read_ready_and_filter() {
  if (!g_scale.is_ready()) return false;
  long raw = g_scale.read();
  double units_now = (raw - (double)g_offset) / (double)g_scale_fac;  // grams after cal
  g_filtered_g = G_ALPHA * (float)units_now + (1.0f - G_ALPHA) * g_filtered_g;
  var_push(g_filtered_g);          // update stability detector
  return true;
}
void maybe_print_and_lcd() {
  uint32_t now = millis();
  if (now < g_t_next_print) return;
  g_t_next_print = now + PRINT_INTERVAL_MS;

  // Serial
  if (g_show_raw) {
    long raw_est = (long)(g_filtered_g * (double)g_scale_fac + (double)g_offset);
    Serial.printf("Raw: %ld\n", raw_est);
  } else {
    float show = (fabsf(g_filtered_g) < G_DEADBAND_G ) ? 0.0f : g_filtered_g;
    Serial.print(F("Weight: ")); Serial.print(show, 2); Serial.println(F(" g"));
  }

  // LCD
  String line1, line2;
  if (g_show_raw) {
    long raw_est = (long)(g_filtered_g * (double)g_scale_fac + (double)g_offset);
    line1 = "RAW:" + String(raw_est);
  } else {
    float show = (fabsf(g_filtered_g) < G_DEADBAND_G) ? 0.0f : g_filtered_g;
    line1 = "Wt:" + String(show, 2) + " g";
  }
  line2 = (g_show_raw ? "Mode:RAW " : "Mode:G  ") + stateToStr();

  if (LCD_ROWS >= 1) lcdWriteLine(0, line1);
  if (LCD_ROWS >= 2) lcdWriteLine(1, line2);
  // if (LCD_ROWS >= 3) lcdWriteLine(2, "scale:" + String(g_scale_fac, 3));
  // if (LCD_ROWS >= 4) lcdWriteLine(3, "off:"   + String(g_offset));
}

// ===== Setup / Loop =====
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("HX711 grams – ESP32 (non-blocking + LiquidCrystal_I2C)"));
  Serial.print(F("LIB VERSION: ")); Serial.println(HX711_LIB_VERSION);

  // If you used non-default I2C pins: Wire.begin(SDA, SCL);
  Wire.begin();                // ESP32 defaults SDA=21, SCL=22
  g_lcd.init();
  g_lcd.backlight();
  for (uint8_t r=0; r<LCD_ROWS; r++) g_lcd_prev[r] = "";
  lcdWriteLine(0, "HX711 Scale");
  lcdWriteLine(1, "Init...");

  g_scale.begin(PIN_DOUT, PIN_SCK);
  g_scale.set_gain(128);

  if (load_cal()) {
    Serial.println(F("[OK] Loaded calibration from NVS."));
  } else {
    Serial.println(F("[!!] No stored calibration. Do 't' then 'c <grams>'."));
    g_scale_fac = 1.0f; g_offset = 0; apply_cal();
  }

  g_t_next_read  = millis();
  g_t_next_print = millis();
  print_menu();
  lcdWriteLine(0, "Ready: t / c g");
  lcdWriteLine(1, "h=help  r=mode");
}
const char* getStateName(g_state_t state) {
  switch(state) {
    case G_IDLE:  return "G_IDLE";
    case G_TARE:  return "G_TARE";
    case G_CAL:   return "G_CAL";
    case G_PWRDN: return "G_PWRDN";
    case G_PWRUP: return "G_PWRUP";
    default:      return "UNKNOWN";
  }
}

void loop() {
  // 1) CLI
  handle_serial();
  
  // 2) Advance long ops
  switch (g_state) {
    case G_TARE:
      if (g_var.stable) progress_tare();
      break;
    case G_CAL:   progress_cal();  break;
    case G_PWRDN:
    case G_PWRUP: progress_power_cycle(); break;
    default: break;
  }

  // 3) Periodic sensor read + filter
  uint32_t now = millis();
  if (now >= g_t_next_read) {
    g_t_next_read = now + READ_INTERVAL_MS;
    (void)read_ready_and_filter();
  }

  // 4) Periodic Serial + LCD
  maybe_print_and_lcd();
}