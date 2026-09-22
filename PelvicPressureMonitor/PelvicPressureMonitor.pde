/*
  ============================================================
  Pelvic Pressure Monitor  (v3, simple mode: only 3 keys)
  Companion for Arduino sketch: filling_fsr402.ino (v2)
  Demo model only, NOT a medical device.
  ============================================================

  HOW TO RUN
   1. Install Processing 4 (processing.org). Unzip and run, no installer.
   2. Create a folder named exactly:  PelvicPressureMonitor
      Put this file inside, named:    PelvicPressureMonitor.pde
      (Processing requires folder name == main .pde name)
   3. Flash filling_fsr402.ino to the Uno, then CLOSE the Arduino
      Serial Monitor / Serial Plotter. Only one program can hold
      a serial port at a time.
   4. Open this file in Processing and press Run.

  ONLY 3 KEYS
   z      set EMPTY point (press once while the system is empty)
   u      set FULL point  (press once while pushed to the max)
   SPACE  drop a numbered marker (press once per 20 mL step)

  EVERYTHING ELSE IS AUTOMATIC
   - Port: fixed to the macOS USB serial device
     /dev/cu.usbserial-1110 and retries every 3 s until the
     Arduino answers. No key needed.
   - CSV log: starts by itself as soon as data arrives, saved in
     the sketch folder as log_YYYYMMDD_HHMMSS.csv with a marker
     column. Closed automatically when you quit.
   - Chart image: chart_YYYYMMDD_HHMMSS.png is refreshed in the
     sketch folder every time you press SPACE, so after the last
     step you already have the final picture.
   - Time axis: always shows the whole run, no zoom keys.

  A FULL RECORDING SESSION IS JUST:
   empty the system, press z, then for each 20 mL: push, press
   SPACE. At the max push press u. Close the window. Done: the
   CSV and the PNG are in the sketch folder.
*/

import processing.serial.*;

// ================= USER SETTINGS =================
// macOS: use the exact Arduino serial device path.
// Your current Arduino USB serial port is:
// /dev/cu.usbserial-1110
final String SERIAL_PORT = "/dev/cu.usbserial-1110";
final int   BAUD         = 115200;  // must match Serial.begin() in the Arduino sketch
final float Y_MAX        = 520;     // vertical axis limit (equivalent mL)

// ================= STATE =================
Serial port = null;
String[] portList = new String[0];
String[] cands = new String[0];   // ordered candidates for auto-scan
int  candIdx = 0;
int  connState = 0;               // 0 = waiting to retry, 1 = scanning, 2 = connected
int  phase = 0;                   // scan phase: 1 = waiting boot, 2 = listening
long phaseT0 = 0;
long retryT0 = 0;
String portName = "-";
String status = "";

ArrayList<float[]> data  = new ArrayList<float[]>();   // {t_sec, A, B}
ArrayList<float[]> marks = new ArrayList<float[]>();   // {t_sec, id}
int markerCount = 0;
int pendingMark = 0;              // marker id waiting to be written to the CSV
boolean saveChartNext = false;    // save the PNG at the end of the next frame

float th1 = 150, th2 = 300, th3 = 400;   // thresholds, overwritten by Arduino
float curA = 0, curB = 0;
boolean gotData = false;

PrintWriter logw = null;
String logName = "";
String chartName = "";

int t0 = 0;
PFont fUI, fSmall, fBig;

// layout
final int ML = 78, MR = 236, MT = 122, MB = 70;

// ================= SETUP =================
void settings() {
  size(1120, 680);
}

void setup() {
  surface.setTitle("Pelvic Pressure Monitor");
  frameRate(60);
  t0 = millis();

  fBig   = createFont("SansSerif", 38, true);
  fUI    = createFont("SansSerif", 15, true);
  fSmall = createFont("SansSerif", 12, true);

  String stamp = nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_"
               + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  logName   = "log_" + stamp + ".csv";
  chartName = "chart_" + stamp + ".png";

  startScan();
}

// Build candidate order: USB-serial style names first, then the
// remaining ports in REVERSE list order (on Windows, COM1/COM3 are
// usually motherboard or Bluetooth ports; a CH340 Uno typically gets
// a higher COM number, so try high numbers first).
String[] orderCandidates(String[] all) {
  ArrayList<String> pref = new ArrayList<String>();
  ArrayList<String> rest = new ArrayList<String>();
  for (int i = 0; i < all.length; i++) {
    String l = all[i].toLowerCase();
    if (l.indexOf("usbserial") >= 0 || l.indexOf("usbmodem") >= 0 ||
        l.indexOf("wchusb")    >= 0 || l.indexOf("ttyusb")   >= 0 ||
        l.indexOf("ttyacm")    >= 0) pref.add(all[i]);
    else rest.add(all[i]);
  }
  for (int i = rest.size() - 1; i >= 0; i--) pref.add(rest.get(i));
  return pref.toArray(new String[0]);
}

void startScan() {
  closePort();
  gotData = false;

  // On macOS we use the exact USB serial device instead of scanning
  // Serial.list(), because the device path is already known.
  portList = Serial.list();
  printArray(portList);

  cands = new String[] { SERIAL_PORT };
  candIdx = 0;
  connState = 1;
  phase = 0;
}

void waitAndRetry(String msg) {
  connState = 0;
  retryT0 = millis();
  status = msg;
}

void closePort() {
  if (port != null) {
    try { port.stop(); } catch (Exception e) { }
    port = null;
  }
}

// Non-blocking connect state machine, called every frame.
void tickConnect() {
  if (connState == 2) return;

  if (connState == 0) {                       // idle: retry every 3 s, forever
    if (millis() - retryT0 > 3000) startScan();
    return;
  }

  if (port == null) {                         // open next candidate
    if (candIdx >= cands.length) {
      waitAndRetry("No answer on any port. Close the Arduino Serial Monitor if it is open. Retrying in 3 s ...");
      return;
    }
    portName = cands[candIdx];
    status = "Trying " + portName + "  (" + (candIdx + 1) + "/" + cands.length + ") ...";
    try {
      port = new Serial(this, portName, BAUD);
      phase = 1;                              // opened: wait for Uno auto-reset boot
      phaseT0 = millis();
    } catch (Exception e) {
      println("open failed on " + portName + ": " + e);
      candIdx++;                              // busy or invalid, next candidate
      port = null;
    }
    return;
  }

  if (phase == 1) {                           // Uno reboots on port open (DTR)
    if (millis() - phaseT0 > 2200) {
      try { port.write("p"); } catch (Exception e) { }
      phase = 2;
      phaseT0 = millis();
      status = "Opened " + portName + ", listening ...";
    }
  } else if (phase == 2) {
    if (gotData) {                            // parseLine() saw a valid data row
      connState = 2;
      status = "Connected on " + portName;
      openLog();
      return;
    }
    if (millis() - phaseT0 > 2500) {          // silent: not our device
      println("no data on " + portName + ", moving on");
      closePort();
      candIdx++;
    }
  }
}

void openLog() {
  if (logw != null) return;
  logw = createWriter(logName);
  logw.println("t_sec,bladder_mL,rectum_mL,marker");
}

// ================= SERIAL RX =================
// Polled from draw() on the main thread. serialEvent() runs on the
// serial thread and races draw() over the data list; polling doesn't.
void pumpSerial() {
  if (port == null) return;
  int guard = 0;
  while (port.available() > 0 && guard < 200) {
    String s = port.readStringUntil('\n');
    if (s == null) break;                   // incomplete line, next frame
    parseLine(s);
    guard++;
  }
}

void parseLine(String raw) {
  String s = trim(raw);
  if (s.length() == 0) return;
  if (s.charAt(0) == '#') { println("[arduino] " + s); return; }  // notes go to console only

  boolean labeled = (s.indexOf(':') >= 0);
  String[] parts = split(s, ',');
  if (parts.length < 2) return;

  float[] v = new float[5];
  int n = 0;
  for (int i = 0; i < parts.length && n < 5; i++) {
    String t = trim(parts[i]);
    if (labeled) {
      int k = t.indexOf(':');
      if (k < 0) continue;
      t = trim(t.substring(k + 1));
    }
    if (t.length() == 0) continue;
    try {
      v[n] = Float.parseFloat(t);
      n++;
    } catch (Exception e) {
      // ignore non-numeric tokens; never let one bad line kill the app
    }
  }
  if (n < 2) return;

  curA = v[0];
  curB = v[1];
  if (n >= 5) { th1 = v[2]; th2 = v[3]; th3 = v[4]; }

  float t = (millis() - t0) / 1000.0f;
  data.add(new float[] { t, curA, curB });
  gotData = true;

  while (data.size() > 30000) data.remove(0);

  if (logw != null) {
    String mk = (pendingMark > 0) ? str(pendingMark) : "";
    logw.println(nf(t, 0, 2) + "," + nf(curA, 0, 1) + "," + nf(curB, 0, 1) + "," + mk);
    pendingMark = 0;
  }
}

// ================= DRAW =================
void draw() {
  tickConnect();
  pumpSerial();
  background(252, 253, 254);

  // always show the whole run, from 0 to now
  float tNow = (millis() - t0) / 1000.0f;
  float tMax = max(60, tNow);
  float tMin = 0;

  drawHeader();
  drawPanel();
  drawPlot(tMin, tMax);
  drawFooter();

  if (saveChartNext) {          // frame is fully drawn at this point
    saveChartNext = false;
    saveFrame(chartName);
  }
}

void drawHeader() {
  noStroke();
  fill(26, 29, 33);
  rect(0, 0, width, 52);

  textFont(fUI);
  textAlign(LEFT, CENTER);
  fill(255);
  text("Pelvic Filling Monitor - Live Chart", 20, 26);

  textFont(fSmall);
  fill(160, 170, 180);
  String st = (connState == 2) ? ("Port " + portName + "  @" + BAUD)
            : (connState == 1) ? "scanning..." : "retrying...";
  textAlign(RIGHT, CENTER);
  text(st, width - 20, 26);

  // status bar
  fill(246, 248, 250);
  rect(0, 52, width, 34);
  textAlign(LEFT, CENTER);
  if (connState == 2)      fill(90, 98, 106);
  else if (connState == 1) fill(181, 71, 8);
  else                     fill(164, 38, 44);
  String note = status;
  if (note.length() == 0) note = "Running";
  text(note, 20, 69);

  textAlign(RIGHT, CENTER);
  fill(logw != null ? color(17, 99, 41) : color(150, 158, 166));
  String ls = (logw != null) ? ("REC " + logName) : "log starts when connected";
  text(ls, width - 20, 69);

  // Show the fixed macOS serial port while connecting.
  if (connState != 2) {
    textAlign(LEFT, TOP);
    fill(120, 128, 136);
    textFont(fSmall);
    text("Serial port: " + SERIAL_PORT, 20, 94);
  }
}

int levelOf(float v) {
  if (v >= th3) return 3;
  if (v >= th2) return 2;
  if (v >= th1) return 1;
  return 0;
}

String levelName(int L) {
  if (L == 0) return "EMPTY";
  if (L == 1) return "MILD";
  if (L == 2) return "STRONG";
  return "URGENT";
}

color levelColor(int L) {
  if (L == 0) return color(47, 158, 68);
  if (L == 1) return color(130, 201, 30);
  if (L == 2) return color(245, 159, 0);
  return color(224, 49, 49);
}

void drawPanel() {
  int x = width - MR + 16;
  int w = MR - 36;

  drawReadout(x, MT, w, "A - Bladder", curA, color(201, 162, 39));
  drawReadout(x, MT + 168, w, "B - Rectum", curB, color(138, 107, 69));
}

void drawReadout(int x, int y, int w, String title, float v, color accent) {
  noStroke();
  fill(255);
  rect(x, y, w, 148, 8);
  stroke(223, 227, 232);
  strokeWeight(1);
  noFill();
  rect(x, y, w, 148, 8);
  noStroke();
  fill(accent);
  rect(x, y, 4, 148, 8, 0, 0, 8);

  textFont(fUI);
  textAlign(LEFT, TOP);
  fill(74, 81, 88);
  text(title, x + 14, y + 12);

  int L = levelOf(v);
  textFont(fBig);
  fill(26, 29, 33);
  text(nf(v, 0, 0), x + 14, y + 38);
  textFont(fSmall);
  fill(120, 128, 136);
  textAlign(LEFT, BOTTOM);
  text("mL", x + 14 + textWidth(nf(v, 0, 0)) + 66, y + 84);

  // level chip + three dots mirroring the hardware LEDs
  noStroke();
  fill(levelColor(L));
  rect(x + 14, y + 96, 74, 24, 5);
  fill(255);
  textFont(fSmall);
  textAlign(CENTER, CENTER);
  text(levelName(L), x + 51, y + 108);

  for (int i = 0; i < 3; i++) {
    boolean on = (i == 0 && L <= 1) || (i == 1 && L >= 2) || (i == 2 && L == 3);
    color c = (i == 0) ? color(47, 158, 68) : (i == 1) ? color(245, 159, 0) : color(224, 49, 49);
    fill(on ? c : color(228, 232, 236));
    ellipse(x + 108 + i * 24, y + 108, 15, 15);
  }
}

float px(float t, float tMin, float tMax) {
  return map(t, tMin, tMax, ML, width - MR);
}
float py(float v) {
  return map(constrain(v, 0, Y_MAX), 0, Y_MAX, height - MB, MT);
}

void drawPlot(float tMin, float tMax) {
  float x0 = ML, x1 = width - MR, y0 = MT, y1 = height - MB;

  noStroke();
  fill(255);
  rect(x0, y0, x1 - x0, y1 - y0);

  // grid + y ticks
  textFont(fSmall);
  for (int v = 0; v <= 500; v += 100) {
    float y = py(v);
    stroke(238, 241, 244);
    strokeWeight(1);
    line(x0, y, x1, y);
    noStroke();
    fill(140, 148, 156);
    textAlign(RIGHT, CENTER);
    text(v, x0 - 10, y);
  }
  noStroke();
  fill(120, 128, 136);
  textAlign(RIGHT, CENTER);
  text("mL", x0 - 10, y0 - 14);

  // threshold lines
  drawTh(th1, x0, x1, color(130, 201, 30), "MILD ");
  drawTh(th2, x0, x1, color(245, 159, 0), "STRONG ");
  drawTh(th3, x0, x1, color(224, 49, 49), "URGENT ");

  // time axis: pick a tick step that fits the whole run
  float span = tMax - tMin;
  float step = 10;
  if (span > 120)  step = 30;
  if (span > 360)  step = 60;
  if (span > 900)  step = 120;
  if (span > 1800) step = 300;
  float tStart = ceil(tMin / step) * step;
  for (float t = tStart; t <= tMax; t += step) {
    float x = px(t, tMin, tMax);
    stroke(240, 243, 246);
    line(x, y0, x, y1);
    noStroke();
    fill(150, 158, 166);
    textAlign(CENTER, TOP);
    text(nf(t, 0, 0) + "s", x, y1 + 8);
  }

  // markers
  for (int i = 0; i < marks.size(); i++) {
    float[] m = marks.get(i);
    if (m[0] < tMin || m[0] > tMax) continue;
    float x = px(m[0], tMin, tMax);
    stroke(138, 92, 240, 170);
    strokeWeight(1.4f);
    for (float y = y0; y < y1; y += 8) line(x, y, x, min(y + 4, y1));
    noStroke();
    fill(138, 92, 240);
    ellipse(x, y0 + 10, 18, 18);
    fill(255);
    textAlign(CENTER, CENTER);
    textFont(fSmall);
    text(nf(m[1], 0, 0), x, y0 + 10);
  }

  // series
  drawSeries(1, color(201, 162, 39), tMin, tMax);
  drawSeries(2, color(138, 107, 69), tMin, tMax);

  // frame
  noFill();
  stroke(214, 220, 226);
  strokeWeight(1);
  rect(x0, y0, x1 - x0, y1 - y0);

  // legend
  noStroke();
  textFont(fSmall);
  textAlign(LEFT, CENTER);
  fill(201, 162, 39);
  rect(x0 + 10, y0 + 12, 22, 3);
  fill(90, 98, 106);
  text("A Bladder", x0 + 38, y0 + 14);
  fill(138, 107, 69);
  rect(x0 + 118, y0 + 12, 22, 3);
  fill(90, 98, 106);
  text("B Rectum", x0 + 146, y0 + 14);

  if (!gotData) {
    fill(150, 158, 166);
    textFont(fUI);
    textAlign(CENTER, CENTER);
    text("Waiting for data...  (board flashed with filling_fsr402.ino? Serial Monitor closed?)",
         (x0 + x1) / 2, (y0 + y1) / 2);
  }
}

void drawTh(float v, float x0, float x1, color c, String label) {
  float y = py(v);
  stroke(c, 150);
  strokeWeight(1.2f);
  for (float x = x0; x < x1; x += 9) line(x, y, min(x + 5, x1), y);
  noStroke();
  fill(c);
  textFont(fSmall);
  textAlign(LEFT, BOTTOM);
  text(label + nf(v, 0, 0), x1 - 88, y - 3);
}

void drawSeries(int idx, color c, float tMin, float tMax) {
  noFill();
  stroke(c);
  strokeWeight(2.2f);
  beginShape();
  int drawn = 0;
  for (int i = 0; i < data.size(); i++) {
    float[] d = data.get(i);
    if (d[0] < tMin || d[0] > tMax) continue;
    vertex(px(d[0], tMin, tMax), py(d[idx]));
    drawn++;
  }
  endShape();

  if (drawn > 0) {
    float[] d = data.get(data.size() - 1);
    if (d[0] >= tMin && d[0] <= tMax) {
      noStroke();
      fill(c);
      ellipse(px(d[0], tMin, tMax), py(d[idx]), 8, 8);
    }
  }
}

void drawFooter() {
  noStroke();
  fill(246, 248, 250);
  rect(0, height - 34, width, 34);
  fill(120, 128, 136);
  textFont(fSmall);
  textAlign(LEFT, CENTER);
  text("SPACE add marker (once per 20 mL)      z set EMPTY point      u set FULL point      everything else is automatic",
       20, height - 17);
  textAlign(RIGHT, CENTER);
  text("markers " + markerCount + "   samples " + data.size(), width - 20, height - 17);
}

// ================= INPUT =================
float now() { return (millis() - t0) / 1000.0f; }

void send(String s) {
  if (connState == 2 && port != null) {
    try { port.write(s); } catch (Exception e) { }
  }
}

void keyPressed() {
  if (key == 'z' || key == 'Z') {
    send("z");
    status = "Sent z: EMPTY point set (system should be empty right now)";
  } else if (key == 'u' || key == 'U') {
    send("u");
    status = "Sent u: FULL point set (system should be at max right now)";
  } else if (key == ' ') {
    markerCount++;
    marks.add(new float[] { now(), markerCount });
    pendingMark = markerCount;      // written into the next CSV row
    saveChartNext = true;           // refresh the chart PNG after this frame
    status = "Marker " + markerCount + " dropped, chart image updated";
  }
}

// Must be public: PApplet.dispose() is public and Java forbids
// overriding with weaker access.
public void dispose() {
  if (logw != null) {
    logw.flush();
    logw.close();
    logw = null;
  }
  super.dispose();
}
