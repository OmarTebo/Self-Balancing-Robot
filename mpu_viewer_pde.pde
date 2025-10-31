/**
 * mpu_viewer_2d.pde
 *
 * A lightweight, OpenGL-free Processing viewer (JAVA2D) for MPU6050 JSON over serial.
 * Shows an artificial horizon (sky/ground) rotated by roll and shifted by pitch,
 * plus numeric readouts and small history graph.
 *
 * Set PORT_INDEX to the index printed by Serial.list() if needed.
 *
 * Controls:
 *  - p : pause/resume updating
 *  - t : toggle trail/history drawing
 *  - c : clear trail/history
 *  - s : save screenshot
 */

import processing.serial.*;
import java.util.ArrayList;

Serial myPort;
int PORT_INDEX = 0;        // change if automatic pick is wrong
int BAUD = 115200;
String portLabel = "none";

boolean paused = false;
boolean drawTrail = true;
float cubeSize = 150; // unused but kept for familiarity

// latest angles
float roll = 0;
float pitch = 0;
float yaw = 0;
boolean haveAngles = false;

// history arrays for plotting
int HIST = 300;
float[] histRoll = new float[HIST];
float[] histPitch = new float[HIST];
int histPos = 0;

// trail for small marker
ArrayList<PVector> trail = new ArrayList<PVector>();
int maxTrail = 300;

void setup() {
  size(900, 700);    // JAVA2D default
  println("Available serial ports:");
  String[] ports = Serial.list();
  for (int i=0; i<ports.length; i++) {
    println(i + ": " + ports[i]);
  }
  if (ports.length == 0) {
    println("No serial ports detected. Plug in board and restart sketch.");
    myPort = null;
    portLabel = "none";
  } else {
    portLabel = ports[PORT_INDEX];
    println("Opening port index " + PORT_INDEX + " -> " + portLabel);
    try {
      myPort = new Serial(this, portLabel, BAUD);
      myPort.bufferUntil('\n');
      println("Serial opened at " + BAUD + " baud.");
    } catch (Exception e) {
      println("Could not open serial port: " + e.getMessage());
      myPort = null;
      portLabel = "none";
    }
  }

  for (int i=0;i<HIST;i++) {
    histRoll[i] = 0;
    histPitch[i] = 0;
  }
  textFont(createFont("Arial", 12));
  frameRate(60);
}

void draw() {
  background(30);

  // HUD
  fill(230);
  textAlign(LEFT, TOP);
  text("MPU6050 Viewer (JAVA2D)", 12, 8);
  text("Port: " + portLabel, 12, 28);
  text("Paused: " + paused + "  Trail: " + drawTrail, 12, 44);
  text("Press 'h' for help", 12, 60);

  // Draw artificial horizon box
  int hudTop = 90;
  int w = width - 24;
  int h = height - hudTop - 160;
  int cx = width/2;
  int cy = hudTop + h/2;

  pushMatrix();
  translate(cx, cy);

  // pitch offset (map ±90° to ±h/2)
  float pitchOffset = map(pitch, -90, 90, -h/2, h/2);
  // roll rotation:
  pushMatrix();
  rotate(radians(-roll)); // negative to match typical aircraft horizon behavior

  // sky
  noStroke();
  fill(70, 140, 230);
  rectMode(CENTER);
  rect(0, -h/2 + pitchOffset, w*1.2, h);

  // ground
  fill(140, 100, 50);
  rect(0, h/2 + pitchOffset, w*1.2, h);

  // horizon line
  stroke(255, 220);
  strokeWeight(2);
  line(-w, pitchOffset, w, pitchOffset);

  // center markers
  stroke(255, 200);
  strokeWeight(1.5);
  line(-40, pitchOffset, -10, pitchOffset);
  line(40, pitchOffset, 10, pitchOffset);

  // bank scale at top
  popMatrix();

  // rim
  noFill();
  stroke(200);
  strokeWeight(2);
  rectMode(CENTER);
  rect(0, 0, w, h);

  popMatrix();

  // draw center fixed aircraft symbol
  pushMatrix();
  translate(cx, cy);
  stroke(255, 200);
  strokeWeight(2);
  line(-30, 0, -10, 0);
  line(30, 0, 10, 0);
  line(0, -10, 0, 10);
  popMatrix();

  // small trail (project roll/pitch into 2D)
  if (drawTrail && haveAngles) {
    PVector pt = orientationToPoint(roll, pitch);
    trail.add(pt);
    if (trail.size() > maxTrail) trail.remove(0);
    pushMatrix();
    translate(cx, cy);
    noFill();
    stroke(0,255,180,150);
    beginShape();
    for (PVector p : trail) vertex(p.x, p.y);
    endShape();
    popMatrix();
  }

  // Numeric readouts
  fill(255);
  textAlign(LEFT, TOP);
  text("Roll: " + nf(roll, 1, 3) + "°", 12, hudTop + h + 12);
  text("Pitch: " + nf(pitch, 1, 3) + "°", 12, hudTop + h + 32);
  text("Yaw: " + nf(yaw, 1, 3) + "°", 12, hudTop + h + 52);

  // history plot
  drawHistoryPlot(0, hudTop + h + 90, width-24, 120);
}

PVector orientationToPoint(float r, float p) {
  float R = 200;
  float a = radians(r);
  float b = radians(p);
  float x = R * sin(a) * cos(b);
  float y = -R * sin(b);
  return new PVector(x, y);
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = line.trim();
  if (line.length() == 0) return;

  boolean parsed = false;
  float r = parseKeyFloat(line, "\"roll\"");
  float pch = parseKeyFloat(line, "\"pitch\"");
  float yv = parseKeyFloat(line, "\"yaw\"");
  if (!Float.isNaN(r) && !Float.isNaN(pch)) {
    roll = r;
    pitch = pch;
    yaw = Float.isNaN(yv) ? yaw : yv;
    haveAngles = true;
    parsed = true;
  } else {
    // try without quotes
    r = parseKeyFloat(line, "roll");
    pch = parseKeyFloat(line, "pitch");
    yv = parseKeyFloat(line, "yaw");
    if (!Float.isNaN(r) && !Float.isNaN(pch)) {
      roll = r; pitch = pch; if (!Float.isNaN(yv)) yaw = yv;
      haveAngles = true;
      parsed = true;
    }
  }
  if (parsed && !paused) {
    histRoll[histPos] = roll;
    histPitch[histPos] = pitch;
    histPos = (histPos + 1) % HIST;
  }
}

float parseKeyFloat(String s, String key) {
  int idx = s.indexOf(key);
  if (idx == -1) return Float.NaN;
  int colon = s.indexOf(':', idx);
  if (colon == -1) return Float.NaN;
  int i = colon+1;
  while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
  int j = i;
  boolean started = false;
  while (j < s.length()) {
    char c = s.charAt(j);
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c=='E') {
      started = true; j++;
    } else break;
  }
  if (!started) return Float.NaN;
  String num = s.substring(i, j);
  try {
    return Float.parseFloat(num);
  } catch (Exception e) { return Float.NaN; }
}

void drawHistoryPlot(int x, int y, int w, int h) {
  noStroke();
  fill(20);
  rect(x+12, y, w, h);
  stroke(50);
  for (int g=0; g<=4; g++) {
    float yy = map(g, 0, 4, y, y+h);
    line(x+12, yy, x+w, yy);
  }
  // roll
  noFill();
  stroke(200,80,80);
  beginShape();
  for (int k=0; k<HIST; k++) {
    int idx = (histPos + k) % HIST;
    float v = histRoll[idx];
    float xx = map(k, 0, HIST-1, x+12, x+w);
    float yy = map(v, -90, 90, y+h, y);
    vertex(xx, yy);
  }
  endShape();
  // pitch
  stroke(80,200,120);
  beginShape();
  for (int k=0; k<HIST; k++) {
    int idx = (histPos + k) % HIST;
    float v = histPitch[idx];
    float xx = map(k, 0, HIST-1, x+12, x+w);
    float yy = map(v, -90, 90, y+h, y);
    vertex(xx, yy);
  }
  endShape();
  // legends
  noStroke();
  fill(200,80,80); rect(x+18, y+6, 10, 10); fill(255); text("roll", x+34, y+4);
  fill(80,200,120); rect(x+100, y+6, 10, 10); fill(255); text("pitch", x+116, y+4);
}

void keyPressed() {
  if (key == 'p' || key == 'P') paused = !paused;
  else if (key == 't' || key == 'T') drawTrail = !drawTrail;
  else if (key == 'c' || key == 'C') { trail.clear(); histPos = 0; for (int i=0;i<HIST;i++){ histRoll[i]=0; histPitch[i]=0;} }
  else if (key == 's' || key == 'S') saveFrame("mpu_snapshot-####.png");
  else if (key == 'h' || key == 'H') {
    println("Controls: p=pause, t=toggle trail, c=clear, s=save screenshot, h=help");
  }
}
