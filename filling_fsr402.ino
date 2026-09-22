/*
  方案 B · 双通道充盈监测演示（FSR402 版 · v2 两点标定）
  Arduino Uno + 2× FSR402 + 6 颗 LED
  非医疗器械，仅用于原理演示

  ── v2 改了什么（针对"最大只到 680、红灯不亮"）──────
    原来的 5 点标定表最高点是 760，超出传感器实际能到的 680，
    等效容量永远算不到 400mL，红灯物理上不可能亮。
    现在改为【两点标定】：
      空的时候发 z  → 记为低点（0 mL）
      推到最满发 u  → 记为高点（500 mL）
    程序把 0–500mL 线性铺在两点之间，三级灯必然都能亮。
    标定自动存 EEPROM，断电不丢，换气球后重新按 z/u 即可，
    不用再改代码、不用重新上传。

  ── 接线（16 根线，不变）───────────────────────
    5V  → FSR-A 一脚、FSR-B 一脚
    A0  ← FSR-A 另一脚，同时经 10kΩ 到 GND，并 100nF 到 GND   （A = 膀胱）
    A1  ← FSR-B 另一脚，同时经 10kΩ 到 GND，并 100nF 到 GND   （B = 直肠）
    D5/D6/D7   → 220Ω → A 组 绿/黄/红 LED → GND
    D8/D9/D10  → 220Ω → B 组 绿/黄/红 LED → GND
    D0/D1 什么都不要接（USB 串口用）
    两只 10kΩ 的地接 Uno 的一个 GND，六颗 LED 的地接另一个 GND

  ── 标定流程（每次换气球后重做，共两步）─────────
    1. 泄压到空（或推到接触点），发 z   → 低点已记，自动保存
    2. 缓慢推水到你打算演示的最大程度，发 u → 高点已记，自动保存
    发 t 可随时查看两点和三级灯对应的 ADC。

  ── 串口命令（115200）────────────────────────
    z 记低点   u 记高点   t 查看标定   d 恢复默认
    m 数值     p 绘图器   c 原始ADC    h 帮助
*/

#include <EEPROM.h>

// 串口绘图器的"标签:数值"格式只有 Arduino IDE 2.x 支持；用 1.8.x 请改成 0
#define PLOT_LABELS 1

const uint8_t PIN_A = A0, PIN_B = A1;
const uint8_t LA_G = 5, LA_Y = 6, LA_R = 7;    // A 膀胱
const uint8_t LB_G = 8, LB_Y = 9, LB_R = 10;   // B 直肠

// ── 两点标定（默认值按你实测最大 680 预设，上电若 EEPROM 有存档则覆盖）──
const int DEF_LO = 340, DEF_HI = 620;   // 默认：高点 620 给 680 留余量
int loA = DEF_LO, hiA = DEF_HI;
int loB = DEF_LO, hiB = DEF_HI;

const int SPAN_MIN = 100;              // 高低点至少差 100 ADC，否则拒绝记录

const int FULL_ML = 500;               // 高点对应的等效容量
const int TH[3] = { 150, 300, 400 };   // 初感 / 明显 / 急迫（等效 mL）
                                       // 即量程的 30% / 60% / 80%
const int HYST  = 40;                  // 回差，防 FSR 蠕变 + 乳胶松弛导致灯自己退级

const int ADC_OPEN = 5;                // 低于此值判为 FSR 开路
const int ADC_SAT  = 1000;             // 高于此值判为接近 ADC 饱和

int  lvA = 0, lvB = 0;
char mode = 'm';
unsigned long tOut = 0;

const char* NM[4] = { "EMPTY ", "MILD  ", "STRONG", "URGENT" };

// ── EEPROM 存档 ──
struct Cal { uint8_t magic; int loA, hiA, loB, hiB; };
const uint8_t MAGIC = 0xB7;

bool calValid(int lo, int hi) {
  return lo >= 0 && hi <= 1023 && (hi - lo) >= SPAN_MIN;
}

void saveCal() {
  Cal c = { MAGIC, loA, hiA, loB, hiB };
  EEPROM.put(0, c);                    // put 只写有变化的字节，不用担心寿命
}

bool loadCal() {
  Cal c;
  EEPROM.get(0, c);
  if (c.magic != MAGIC) return false;
  if (!calValid(c.loA, c.hiA) || !calValid(c.loB, c.hiB)) return false;
  loA = c.loA; hiA = c.hiA;
  loB = c.loB; hiB = c.hiB;
  return true;
}

// ── 采样：16 次，跨度 20ms（正好一个 50Hz 周期），排序后取中间 8 个平均 ──
int readRaw(uint8_t pin) {
  int b[16];
  for (uint8_t i = 0; i < 16; i++) {
    b[i] = analogRead(pin);
    delayMicroseconds(1146);
  }
  for (uint8_t i = 1; i < 16; i++) {
    int k = b[i]; int8_t j = i - 1;
    while (j >= 0 && b[j] > k) { b[j + 1] = b[j]; j--; }
    b[j + 1] = k;
  }
  int s = 0;
  for (uint8_t i = 4; i < 12; i++) s += b[i];
  return s / 8;
}

// 两点线性映射：ADC → 等效容量 mL（低点=0，高点=500，两端夹紧）
int toML(int raw, int lo, int hi) {
  if (raw <= lo) return 0;
  if (raw >= hi) return FULL_ML;
  return (int)((long)(raw - lo) * FULL_ML / (hi - lo));
}

// 某等级的触发 ADC（打印标定信息用）
int thAdc(int lo, int hi, int thML) {
  return lo + (int)((long)(hi - lo) * thML / FULL_ML);
}

int gradeOf(int ml, int cur) {
  int n = (ml >= TH[2]) ? 3 : (ml >= TH[1]) ? 2 : (ml >= TH[0]) ? 1 : 0;
  if (n < cur) {                                   // 下降：逐级检查回差
    for (int L = cur; L > n; L--)
      if (ml > TH[L - 1] - HYST) { n = L; break; }
  }
  return n;
}

void showLED(uint8_t g, uint8_t y, uint8_t r, int lv) {
  bool slow = (millis() / 700) % 2;
  bool fast = (millis() / 300) % 2;
  digitalWrite(g, (lv == 0) || (lv == 1 && slow));
  digitalWrite(y, lv >= 2);                        // 等级3 时黄灯常亮，避免全黑
  digitalWrite(r, (lv == 3) && fast);
}

void say(const __FlashStringHelper* s) {
  if (mode != 'p') Serial.println(s);
}

void printCal() {
  if (mode == 'p') return;
  Serial.println(F("# 标定  通道   低点(0mL)  高点(500mL)  绿闪  黄亮  红闪 (ADC)"));
  Serial.print(F("#        A      "));
  Serial.print(loA); Serial.print(F("        ")); Serial.print(hiA);
  Serial.print(F("        ")); Serial.print(thAdc(loA, hiA, TH[0]));
  Serial.print(F("   ")); Serial.print(thAdc(loA, hiA, TH[1]));
  Serial.print(F("   ")); Serial.println(thAdc(loA, hiA, TH[2]));
  Serial.print(F("#        B      "));
  Serial.print(loB); Serial.print(F("        ")); Serial.print(hiB);
  Serial.print(F("        ")); Serial.print(thAdc(loB, hiB, TH[0]));
  Serial.print(F("   ")); Serial.print(thAdc(loB, hiB, TH[1]));
  Serial.print(F("   ")); Serial.println(thAdc(loB, hiB, TH[2]));
}

// z：把当前状态记为低点（要求此刻是空/接触点状态）
void setLow() {
  int rA = readRaw(PIN_A), rB = readRaw(PIN_B);
  bool okA = calValid(rA, hiA), okB = calValid(rB, hiB);
  if (okA) loA = rA;
  if (okB) loB = rB;
  if (okA || okB) saveCal();
  if (mode == 'p') return;
  Serial.print(F("# 记低点  A adc=")); Serial.print(rA);
  Serial.print(okA ? F(" 已记") : F(" 拒绝(离高点不足100,先泄压或先用u记高点)"));
  Serial.print(F("  |  B adc=")); Serial.print(rB);
  Serial.println(okB ? F(" 已记") : F(" 拒绝(离高点不足100)"));
  if (okA || okB) { Serial.println(F("# 已自动保存,断电不丢")); printCal(); }
}

// u：把当前状态记为高点（要求此刻推到演示最大程度）
void setHigh() {
  int rA = readRaw(PIN_A), rB = readRaw(PIN_B);
  bool okA = calValid(loA, rA), okB = calValid(loB, rB);
  if (okA) hiA = rA;
  if (okB) hiB = rB;
  if (okA || okB) saveCal();
  if (mode == 'p') return;
  Serial.print(F("# 记高点  A adc=")); Serial.print(rA);
  Serial.print(okA ? F(" 已记") : F(" 拒绝(比低点高不足100,先推水加压)"));
  Serial.print(F("  |  B adc=")); Serial.print(rB);
  Serial.println(okB ? F(" 已记") : F(" 拒绝(比低点高不足100)"));
  if (okA || okB) { Serial.println(F("# 已自动保存,断电不丢")); printCal(); }
}

void setDefaults() {
  loA = DEF_LO; hiA = DEF_HI;
  loB = DEF_LO; hiB = DEF_HI;
  saveCal();
  say(F("# 已恢复默认标定并保存"));
  printCal();
}

void setup() {
  pinMode(LA_G, OUTPUT); pinMode(LA_Y, OUTPUT); pinMode(LA_R, OUTPUT);
  pinMode(LB_G, OUTPUT); pinMode(LB_Y, OUTPUT); pinMode(LB_R, OUTPUT);
  Serial.begin(115200);
  delay(600);
  // 横幅里不能出现英文冒号，否则串口绘图器会把它当成一条数据系列
  Serial.println(F("# 充盈监测 v2  A=bladder B=rectum   cmd  z记低点 u记高点 t标定 d默认 m p c h"));

  if (loadCal()) Serial.println(F("# 已读取上次保存的标定"));
  else           Serial.println(F("# 使用默认标定(低340 高620)。建议先 z 后 u 重新标"));
  printCal();

  int rA = readRaw(PIN_A), rB = readRaw(PIN_B);
  Serial.print(F("# 开机 ADC A=")); Serial.print(rA);
  Serial.print(F(" B="));            Serial.println(rB);
  if (rA < ADC_OPEN || rB < ADC_OPEN)
    Serial.println(F("# 警告 有通道 ADC 近 0，FSR 可能开路或 5V 未接"));
  Serial.println(F("# 标定两步  空的时候发 z  →  推到最满发 u  →  完成"));
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch >= 'A' && ch <= 'Z') ch += 32;
    if (ch == 'm' || ch == 'p' || ch == 'c') {
      mode = ch;
      if (mode != 'p') { Serial.print(F("# mode=")); Serial.println(ch); }
    }
    else if (ch == 'z') setLow();
    else if (ch == 'u') setHigh();
    else if (ch == 't') printCal();
    else if (ch == 'd') setDefaults();
    else if (ch == 'h' || ch == '?')
      say(F("# z=记低点(空时) u=记高点(满时) t=标定 d=默认 m=数值 p=绘图器 c=原始ADC"));
    else if (ch != '\r' && ch != '\n' && ch != ' ')
      say(F("# 未知命令，发 h 看帮助"));
  }

  int rawA = readRaw(PIN_A), rawB = readRaw(PIN_B);

  int mlA = toML(rawA, loA, hiA);
  int mlB = toML(rawB, loB, hiB);

  lvA = gradeOf(mlA, lvA);
  lvB = gradeOf(mlB, lvB);
  showLED(LA_G, LA_Y, LA_R, lvA);
  showLED(LB_G, LB_Y, LB_R, lvB);

  if (millis() - tOut >= 150) {
    tOut += 150;
    if (millis() - tOut >= 150) tOut = millis();

    if (mode == 'p') {
#if PLOT_LABELS
      Serial.print(F("Bladder:")); Serial.print(mlA);
      Serial.print(F(",Rectum:")); Serial.print(mlB);
      Serial.print(F(",T1:"));     Serial.print(TH[0]);
      Serial.print(F(",T2:"));     Serial.print(TH[1]);
      Serial.print(F(",T3:"));     Serial.println(TH[2]);
#else
      Serial.print(mlA);    Serial.print(',');
      Serial.print(mlB);    Serial.print(',');
      Serial.print(TH[0]);  Serial.print(',');
      Serial.print(TH[1]);  Serial.print(',');
      Serial.println(TH[2]);
#endif
    }
    else if (mode == 'c') {
      Serial.print(F("ADC  A=")); Serial.print(rawA);
      Serial.print(F("  B="));    Serial.print(rawB);
      if (rawA > ADC_SAT || rawB > ADC_SAT) Serial.print(F("   << 接近饱和,停止推注"));
      if (rawA < ADC_OPEN || rawB < ADC_OPEN) Serial.print(F("   << 近 0,检查接线"));
      Serial.println();
    }
    else {
      Serial.print(F("A bladder "));
      if (mlA < 100) Serial.print(' ');
      if (mlA < 10)  Serial.print(' ');
      Serial.print(mlA); Serial.print(F(" mL "));
      Serial.print(NM[lvA]);
      Serial.print(F("  |  B rectum "));
      if (mlB < 100) Serial.print(' ');
      if (mlB < 10)  Serial.print(' ');
      Serial.print(mlB); Serial.print(F(" mL "));
      Serial.print(NM[lvB]);
      Serial.print(F("  |  adc ")); Serial.print(rawA);
      Serial.print('/');            Serial.print(rawB);
      if (rawA > ADC_SAT || rawB > ADC_SAT) Serial.print(F("  << 接近饱和"));
      if (rawA < ADC_OPEN || rawB < ADC_OPEN) Serial.print(F("  << 可能开路"));
      Serial.println();
    }
  }
}
