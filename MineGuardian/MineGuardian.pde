import processing.serial.*;
import java.util.ArrayDeque;
import com.jogamp.newt.opengl.GLWindow;

// Mine Guardian | Ground deformation monitor | Processing 4, Java mode
final int BAUD_RATE = 115200;
final float ALTITUDE_THRESHOLD_CM = 8.0; // Positive lift above baseline.
volatile Serial port;
String portName = "";
String portInput = "";
boolean editingPort;
String portMessage = "";

final float GRID_SIZE = 650;
final float GRID_STEP = 50;
final float[][] nodes = {{-260, 0, -180}, {260, 0, -180}, {0, 0, 250}};
final String[] nodeNames = {"Node A", "Node B", "Node C"};

float rotX = radians(-29), rotY = radians(-26), zoom = 1;
float targetRotX = rotX, targetRotY = rotY;
int lastMouseX, lastMouseY;
boolean isDragging, paused;
boolean showGrid = true, showLabels = true;

// The serial thread publishes complete readings. Each frame uses one snapshot.
class SensorReading {
  final float roll, pitch, yaw, vibration;
  final int altitude;
  final float heightChangeCm;  // Signed estimate from this node's reference height.
  final long sequence;
  SensorReading(float roll, float pitch, float yaw,
                float vibration, int altitude, long sequence,
                float heightChangeCm) {
    this.roll = roll;
    this.pitch = pitch;
    this.yaw = yaw;
    this.vibration = vibration;
    this.altitude = altitude;
    this.sequence = sequence;
    this.heightChangeCm = heightChangeCm;
  }
  SensorReading zeroAngles() {
    return new SensorReading(0, 0, 0, vibration, altitude, sequence, heightChangeCm);
  }
  float tilt() { return max(abs(roll), abs(pitch)); }
}

SensorReading latestA, latestB, viewA, viewB;
// Positive heightChangeCm is lift above baseline. Positive Processing Y is down.
final float UNITS_PER_CM=3.6, MAX_SECTION_DROP=72, BOWL_RADIUS=225;
float lastLiftA, lastLiftB, targetLiftA, targetLiftB, liftA, liftB;
float depthScale=UNITS_PER_CM;
long lastMotionTime=-1;
float viewZoom = 1;
final long sessionStart = System.nanoTime();
final int WINDOW_MS = 30000, GAP_MS = 2000, EVENT_ROWS = 3;
color NODE_GREEN = #17754B, NODE_PINK = #C23A79;
long viewTime;
final Trace traceA = new Trace(), traceB = new Trace();
final ArrayDeque<AlertLine> events = new ArrayDeque<AlertLine>();
AlertLine[] visibleEvents = new AlertLine[0];

long sessionMillis() { return (System.nanoTime()-sessionStart)/1000000L; }

class VibrationSample {
  final long time;
  final float value;
  VibrationSample(long t, float v) { time=t; value=v; }
}

// The receiver owns the queue. The renderer uses a time-aligned snapshot.
class Trace {
  final ArrayDeque<VibrationSample> incoming = new ArrayDeque<VibrationSample>();
  VibrationSample[] frame = new VibrationSample[256];
  int count;
  void expire(long now) {
    while (!incoming.isEmpty() && incoming.peekFirst().time < now-WINDOW_MS) incoming.removeFirst();
  }
  void add(long now, float value) {
    expire(now);
    incoming.addLast(new VibrationSample(now,value));
  }
  void capture(long now) {
    expire(now);
    count=incoming.size();
    if (count>frame.length) frame=new VibrationSample[count*2];
    incoming.toArray(frame);
  }
}

class AlertLine {
  final long time;
  final String message;
  final color ink;
  AlertLine(long t, String s, color c) { time=t; message=s; ink=c; }
}

String eventTime(long t) {
  long seconds=t/1000;
  return (seconds/60)+":"+nf(int(seconds%60),2);
}

void addEvent(long now, String node, String detail, color ink) {
  AlertLine entry=new AlertLine(now,"Node "+node+" · "+detail,ink);
  events.addFirst(entry);
  if (events.size()>EVENT_ROWS) events.removeLast();
  println("[MIGO "+eventTime(now)+"] "+entry.message);
}

int tiltBand(float value) { return value>=20 ? 2 : value>=8 ? 1 : 0; }
int vibrationBand(float value) { return value>2 ? 2 : value>1 ? 1 : 0; }

String bandLabel(int band) {
  return band==2 ? "critical" : band==1 ? "warning" : "normal";
}
color bandInk(int band) { return band==2 ? RED : band==1 ? AMBER : TEAL; }

float angleChange(float a, float b) {
  return abs(degrees(atan2(sin(radians(a-b)),cos(radians(a-b)))));
}

void detectChanges(String node, SensorReading prev, SensorReading r, long now) {
  int tilt=tiltBand(r.tilt()), vibration=vibrationBand(r.vibration);
  int oldTilt=prev==null ? 0 : tiltBand(prev.tilt());
  int oldVibration=prev==null ? 0 : vibrationBand(prev.vibration);
  if (tilt!=oldTilt) {
    addEvent(now,node,"Tilt "+bandLabel(tilt)+" ("+nf(r.tilt(),1,1)+"°)",bandInk(tilt));
  } else if (prev!=null) {
    float delta=max(angleChange(r.roll,prev.roll),angleChange(r.pitch,prev.pitch));
    if (delta>=20) addEvent(now,node,"Tilt changed by "+nf(delta,1,1)+"°",AMBER);
  }
  if (vibration!=oldVibration) {
    addEvent(now,node,"Vibration "+bandLabel(vibration)+" ("+nf(r.vibration,1,2)+")",bandInk(vibration));
  } else if (prev!=null && abs(r.vibration-prev.vibration)>2) {
    addEvent(now,node,"Vibration changed to "+nf(r.vibration,1,2),AMBER);
  }
  boolean stateChanged=r.altitude!=(prev==null ? 0 : prev.altitude);
  boolean gainedEstimate=prev!=null && !Float.isFinite(prev.heightChangeCm) && Float.isFinite(r.heightChangeCm);
  if (stateChanged || (r.altitude>0 && gainedEstimate)) {
    String detail="Altitude "+getAltitudeLabel(r.altitude).toLowerCase(java.util.Locale.ROOT);
    if (Float.isFinite(r.heightChangeCm)) detail+=" (est. "+heightText(r.heightChangeCm)+")";
    addEvent(now,node,detail,bandInk(r.altitude));
  }
}

// The live surface and schematic geological section use separate color keys.
color PAPER = #F7F5EE, WHITE = #FDFAF4, INK = #242821;
color SECONDARY = #50594F, MUTED = #626A60, RULE = #CDD0C3;
color TEAL = #236660, SLATE = #36596E, OXIDE = #BD4529;
color AMBER = #855510, RED = #AA3520;
PFont regular, bold, display, mono;
PGraphics model;
PShape rock, surveyGrid, surfaceGrid;
RockPart[] rockParts;
final int VIEW_Y = 159;
float uiScale=1, pageWidth=1250, pageHeight=800, railX=828;
float footerY=740;
final float FLOOR_Y = 111;
float[] labelX = new float[3], labelY = new float[3];

void settings() {
  size(1250, 800, P3D);
  smooth(4);
}

float darkness=0, tiltA, tiltB;
boolean sliding=false, themeDirty=true;

void updateTheme() {
  float bg=255*(1-darkness);
  boolean dark=bg<118;
  PAPER=color(bg); WHITE=PAPER; INK=dark ? color(255) : color(0);
  SECONDARY=lerpColor(INK,PAPER,0.12); MUTED=lerpColor(INK,PAPER,0.19);
  RULE=lerpColor(PAPER,INK,0.30);
  TEAL=dark ? #91E8BE : #215F51; SLATE=dark ? #C3E2FA : #36596E;
  OXIDE=dark ? #FFAA85 : #A33C22; AMBER=dark ? #FFDB96 : #714400;
  RED=dark ? #FFA79C : #922B1A;
  NODE_GREEN=dark ? #86EFAB : #11663F; NODE_PINK=dark ? #FFA2C9 : #A62D67;
  if(surveyGrid!=null) surveyGrid.setStroke(lerpColor(PAPER,INK,0.36));
}
float sliderTop() { return VIEW_Y+64; }
void dragBrightness() {
  darkness=constrain((mouseY/uiScale-sliderTop())/154,0,1);
  updateTheme(); themeDirty=true;
}
void brightnessBar() {
  float y=sliderTop();
  noStroke(); fill(PAPER); rect(1,y-24,45,201,6);
  words("Light",23,y-8,regular,11,INK,CENTER);
  rule(23,y,23,y+154,MUTED,2);
  stroke(INK); strokeWeight(2); fill(PAPER); circle(23,y+154*darkness,14); noStroke();
  words("Dark",23,y+173,regular,11,INK,CENTER);
}
void setup() {
  updateTheme();
  surface.setTitle("Mine Guardian | Ground deformation monitor");
  surface.setResizable(true);
  regular = createFont("Segoe UI", 17, true);
  bold = createFont("Segoe UI Semibold", 22, true);
  display = createFont("Arial Narrow Bold", 55, true);
  mono = createFont("Consolas", 16, true);
  rock = makeGeology();
  rockParts = new RockPart[rock.getChildCount()];
  for (int i=0; i<rockParts.length; i++) rockParts[i]=new RockPart(rock.getChild(i));
  prepareMesh();
  surveyGrid = makeSurveyGrid();
  updateTheme();
  printAvailablePorts();
}

void printAvailablePorts() {
  println("Available serial ports:");
  printArray(Serial.list());
}

String normalizePort(String input) {
  String name = trim(input).toUpperCase(java.util.Locale.ROOT);
  if (name.matches("[0-9]+")) name = "COM" + name;
  return name.matches("COM[1-9][0-9]*") ? name : "";
}

void openReceiver(String input) {
  String requested = normalizePort(input);
  if (requested.isEmpty()) {
    portMessage = "Enter a port such as 12 or COM12";
    return;
  }

  boolean available = false;
  for (String name : Serial.list()) {
    if (name.equalsIgnoreCase(requested)) {
      requested = name;
      available = true;
      break;
    }
  }
  if (!available) {
    portMessage = requested + " not found. See console for ports.";
    printAvailablePorts();
    return;
  }

  Serial previous = port;
  port = null;
  if (previous != null) previous.stop();
  resetReceiverData();

  Serial opened = null;
  portName = requested;
  try {
    opened = new Serial(this, portName, BAUD_RATE);
    opened.clear();
    opened.bufferUntil('\n');
    port = opened;
    portMessage = "";
    println("Connected to " + portName + " at " + BAUD_RATE + " baud");
  } catch (RuntimeException e) {
    if (opened != null) opened.stop();
    portMessage = "Cannot open " + portName + ". Close Serial Monitor.";
    println("Could not open " + portName + " at " + BAUD_RATE + ": " + e.getMessage());
  }
}

synchronized void resetReceiverData() {
  latestA = latestB = viewA = viewB = null;
  lastLiftA = lastLiftB = targetLiftA = targetLiftB = liftA = liftB = 0;
  traceA.incoming.clear();
  traceB.incoming.clear();
  traceA.count = traceB.count = 0;
  events.clear();
  visibleEvents = new AlertLine[0];
  paused = false;
}

public synchronized void serialEvent(Serial p) {
  if (p != port) return;
  String line;
  while ((line = p.readStringUntil('\n')) != null) {
    parseNodeLine(trim(line));
  }
}

// Original brain: gx,gy,gz,roll,pitch,altitudeState,sequence (no yaw).
// Extended: gx,gy,gz,roll,pitch,yaw,altitudeState,sequence[,heightChangeCm].
// The optional ninth field is displacement from the node's baseline, in cm
// (positive = up). Eight-field packets supply only the categorical state.
synchronized void parseNodeLine(String line) {
  if (!line.startsWith("DATA:")) return;
  String[] parts = split(line, ':');
  if (parts.length < 3) return;
  String node = trim(parts[1]);
  if (!node.equals("A") && !node.equals("B")) return;
  String[] f = split(trim(parts[2]), ',');
  if (f.length < 7) return;
  boolean hasYaw=f.length>=8;
  int stateIndex=hasYaw ? 6 : 5;

  try {
    float gx = float(trim(f[0])), gy = float(trim(f[1])), gz = float(trim(f[2]));
    float roll = float(trim(f[3])), pitch = float(trim(f[4]));
    float yaw = hasYaw ? float(trim(f[5])) : Float.NaN;
    float state = float(trim(f[stateIndex]));
    int altitude = (state==0 || state==1 || state==2) ? int(state) : -1;
    long seq = Long.parseLong(trim(f[stateIndex+1]));
    float cm = f.length>=9 ? float(trim(f[8])) : Float.NaN;
    if (!Float.isFinite(cm)) cm=Float.NaN;
    if (Float.isFinite(cm) && altitude!=2) {
      altitude = cm > ALTITUDE_THRESHOLD_CM ? 1 : 0;
    }
    float vibration=sqrt(gx*gx+gy*gy+gz*gz);
    if (!Float.isFinite(vibration) || !Float.isFinite(roll) ||
        !Float.isFinite(pitch) || (hasYaw && !Float.isFinite(yaw))) return;
    SensorReading reading = new SensorReading(roll,pitch,yaw,vibration,altitude,seq,cm);
    long now=sessionMillis();
    boolean isA=node.equals("A");
    detectChanges(node,isA ? latestA : latestB,reading,now);
    // An eight-field packet contains no height observation. Hold the last
    // valid target so a missing value cannot manufacture a recovery.
    if (Float.isFinite(cm)) {
      if (isA) lastLiftA=cm;
      else lastLiftB=cm;
    }
    if (isA) { latestA=reading; traceA.add(now,vibration); }
    else { latestB=reading; traceB.add(now,vibration); }
  } catch (Exception e) {
    println("Parse error: " + e.getMessage());
  }
}

synchronized void snapshot() {
  viewTime=sessionMillis();
  viewA = latestA;
  viewB = latestB;
  targetLiftA=viewA!=null && Float.isFinite(viewA.heightChangeCm) ? lastLiftA : 0;
  targetLiftB=viewB!=null && Float.isFinite(viewB.heightChangeCm) ? lastLiftB : 0;
  traceA.capture(viewTime);
  traceB.capture(viewTime);
  visibleEvents=events.toArray(new AlertLine[events.size()]);
}

String getAltitudeLabel(int state) {
  if (state == 1) return "Increased";
  if (state == 2) return "Deformed";
  return state==0 ? "Normal" : "Unknown";
}

String heightText(float cm) {
  if (!Float.isFinite(cm)) return "Unavailable";
  String magnitude=String.format(java.util.Locale.ROOT,"%.1f",abs(cm));
  if (magnitude.endsWith(".0")) magnitude=magnitude.substring(0,magnitude.length()-2);
  if (magnitude.equals("0")) return "0 cm";
  return (cm<0 ? "−" : "+")+magnitude+" cm";
}

String getState(SensorReading r) {
  if (r == null) return "Awaiting data";
  int band=max(tiltBand(r.tilt()),vibrationBand(r.vibration));
  return band==2 ? "Critical" : band==1 ? "Warning" : "Normal";
}

// Rescale the typography uniformly, and give any extra width to the scene
// and sidebar. Extra height enlarges the scene and vibration plot.
boolean fitLayout() {
  uiScale=max(0.01,min(width/1250.0,height/800.0));
  pageWidth=width/uiScale;
  pageHeight=height/uiScale;
  footerY=pageHeight-60;
  railX=pageWidth-constrain(pageWidth*0.34,422,560);
  int mw=max(1,round(railX)), mh=max(1,round(footerY-282));
  if (model!=null && model.width==mw && model.height==mh) return false;
  if (model!=null) model.dispose();
  model=createGraphics(mw,mh,P3D);
  model.smooth(4);
  return true;
}

void draw() {
  boolean resized=fitLayout();
  long now=sessionMillis();
  float dt=lastMotionTime<0 ? 1.0/60.0 : constrain((now-lastMotionTime)/1000.0,0,0.1);
  lastMotionTime=now;
  if (!paused || frameCount == 1) {
    snapshot();
    if (advanceGround(dt)) {
      for (RockPart part : rockParts) part.update();
    }
    rotX += (targetRotX - rotX) * 0.1;
    rotY += (targetRotY - rotY) * 0.1;
    viewZoom = zoom;
  }
  if (!paused || resized || themeDirty || frameCount==1) renderModel();
  themeDirty=false;
  background(PAPER);
  hint(DISABLE_DEPTH_TEST);
  pushMatrix();
  scale(uiScale);
  masthead();
  viewport();
  brightnessBar();
  instruments();
  statusBar();
  popMatrix();
  hint(ENABLE_DEPTH_TEST);
}

void words(String s, float x, float y, PFont font, float size, color ink, int align) {
  fill(ink);
  textFont(font, size);
  textAlign(align, BASELINE);
  text(s, x, y);
}

void rule(float x1, float y1, float x2, float y2, color ink, float weight) {
  stroke(ink); strokeWeight(weight);
  line(x1,y1,x2,y2);
  noStroke();
}

void masthead() {
  noStroke();
  fill(INK);
  // A cut and folded survey section forms the Mine Guardian mark.
  beginShape();
  vertex(29,36); vertex(43,13); vertex(53,13); vertex(39,36); endShape(CLOSE);
  fill(OXIDE);
  beginShape();
  vertex(45,36); vertex(53,22); vertex(62,36); endShape(CLOSE);
  words("Mine Guardian", 76, 37, bold, 23, INK, LEFT);
  float right = pageWidth-29;
  String status = portMessage;
  if (status.isEmpty() && port != null) {
    status = viewA != null && viewB != null ? "A + B received"
      : viewA != null ? "A received. Waiting for B."
      : viewB != null ? "B received. Waiting for A." : "Port open. Waiting for A + B.";
  }
  if (status.isEmpty()) status = "Enter Node C’s Windows COM port";
  words(status, right-193, 34, mono, 13, SECONDARY, RIGHT);
  stroke(editingPort ? OXIDE : RULE);
  fill(PAPER);
  rect(right-175, 12, 175, 34, 5);
  noStroke();
  String label = editingPort ? portInput + "_" : portName.isEmpty() ? "Set Node C port" : portName + "  /  Change";
  words(label, right-87, 34, bold, 14, INK, CENTER);
  rule(0,58,pageWidth,58,INK,1.5);
}

void viewport() {
  rule(30,89,55,89,OXIDE,3);
  words("ORTHOGRAPHIC VIEW", 65, 93, mono, 13, SECONDARY, LEFT);
  words("Ground deformation", 28, 146, display, 53, INK, LEFT);
  words(round(viewZoom*100)+"%", railX-30, 143, mono, 16, SECONDARY, RIGHT);
  image(model, 0, VIEW_Y);
  pushMatrix(); translate(0,footerY-740);
  rule(30,632,railX-30,632,RULE,1);
  words("Modeled subsidence", 30, 664, bold, 17, INK, LEFT);
  float sx = 30, sy = 681;
  noStroke();
  beginShape(QUADS);
  for (int i = 0; i < 32; i++) {
    float x0 = sx+6*i;
    fill(thermalColor(25*i/32.0)); vertex(x0,sy); vertex(x0,sy+8);
    fill(thermalColor(25*(i+1)/32.0)); vertex(x0+6,sy+8); vertex(x0+6,sy);
  }
  endShape();
  words("Low", sx, sy+26, mono, 12, SECONDARY, LEFT);
  words("High", sx+192, sy+26, mono, 12, SECONDARY, RIGHT);
  rule(257,650,257,713,RULE,1);
  for (int i=0; i<3; i++) {
    fill(i==0 ? #B99873 : i==1 ? #8B6858 : #596663);
    quad(283,660+i*4,302,655+i*4,311,658+i*4,292,663+i*4);
  }
  words("Illustrative geology", 321, 666, bold, 15, INK, LEFT);
  words("Height shift or tilt drives the surface down", 283, 695, regular, 14, SECONDARY, LEFT);
  words("Drag to orbit", railX-30, 665, regular, 14, SECONDARY, RIGHT);
  words("Scroll to zoom", railX-30, 695, regular, 14, SECONDARY, RIGHT);
  popMatrix();
}

color stateInk(SensorReading r) {
  String state = getState(r);
  if (state.equals("Critical")) return RED;
  if (state.equals("Warning")) return AMBER;
  return r == null ? MUTED : TEAL;
}

void instruments() {
  float left=railX+22, right=pageWidth-24;
  noStroke(); fill(WHITE);
  rect(railX,59,pageWidth-railX,footerY-59);
  rule(railX,59,railX,footerY,RULE,1);
  words("Instruments",left,94,display,32,INK,LEFT);
  words("TILT MAGNITUDE",right,90,mono,12,MUTED,RIGHT);
  drawTiltGauge(lerp(left,right,0.25),176,"Node A",viewA);
  drawTiltGauge(lerp(left,right,0.75),176,"Node B",viewB);
  rule(left,230,right,230,INK,1);
  float colA=lerp(left,right,0.645);
  words("READING",left,247,mono,11,MUTED,LEFT);
  words("NODE A",colA,247,mono,11,NODE_GREEN,RIGHT);
  words("NODE B",right,247,mono,11,NODE_PINK,RIGHT);
  String[] telemetryRowLabels={"Roll","Pitch","Yaw","Vibration","Reported altitude","Height change","Sequence"};
  for (int i=0;i<telemetryRowLabels.length;i++) {
    float y=264+i*19;
    words(telemetryRowLabels[i],left,y,regular,14,SECONDARY,LEFT);
    readout(viewA,i,colA,y); readout(viewB,i,right,y);
  }
  drawMigo(left,right);
  drawVibrationChart(left,right,514,footerY-24);
}

String readingText(SensorReading r, int field) {
  if (r==null) return "—";
  switch(field) {
    case 0: return nf(r.roll,1,2)+"°";
    case 1: return nf(r.pitch,1,2)+"°";
    case 2: return Float.isFinite(r.yaw) ? nf(r.yaw,1,2)+"°" : "—";
    case 3: return nf(r.vibration,1,2);
    case 4: return getAltitudeLabel(r.altitude);
    case 5: return Float.isFinite(r.heightChangeCm) ? heightText(r.heightChangeCm) : "No cm data";
    default: return Long.toString(r.sequence);
  }
}

void readout(SensorReading r, int field, float right, float y) {
  String s=readingText(r,field);
  PFont f=field==4 ? bold : mono;
  textFont(f,16);
  float sz=min(16,16*117/max(1,textWidth(s)));
  color ink=INK;
  if (field==4 && r!=null) ink=r.altitude<0 ? MUTED : bandInk(r.altitude);
  if (field==5 && r!=null && !Float.isFinite(r.heightChangeCm)) ink=MUTED;
  words(s,right,y,f,sz,ink,RIGHT);
}

void drawMigo(float left,float right) {
  rule(left,388,right,388,INK,1);
  noStroke(); fill(OXIDE); rect(left,402,4,18);
  words("MIGO",left+12,419,display,24,INK,LEFT);
  words("Mine Guardian",left+74,418,bold,14,SECONDARY,LEFT);
  if (visibleEvents.length==0) {
    String msg=viewA==null && viewB==null ? "Waiting for sensor data" : "No significant changes recorded";
    words(msg,left,452,regular,15,MUTED,LEFT);
  }
  for (int i=0;i<visibleEvents.length;i++) {
    AlertLine e=visibleEvents[i];
    float y=443+i*22;
    words(eventTime(e.time),left,y,mono,13,MUTED,LEFT);
    words(e.message,left+65,y,bold,14,darkness>0.53 ? lerpColor(e.ink,color(255),0.65) : e.ink,LEFT);
  }
}

void traceKey(float x,float y,String s,color ink) {
  rule(x,y-4,x+16,y-4,ink,2.5);
  words(s,x+22,y,regular,13,INK,LEFT);
}

void drawVibrationChart(float left,float right,float top,float bottom) {
  rule(left,top-13,right,top-13,INK,1);
  words("Vibration",left,top+15,display,26,INK,LEFT);
  traceKey(right-154,top+12,"Node A",NODE_GREEN);
  traceKey(right-71,top+12,"Node B",NODE_PINK);
  words("Gyro magnitude",left,top+37,regular,13,SECONDARY,LEFT);
  float x0=left+35,x1=right-3,y0=top+52,y1=bottom-20;
  float limit=2;
  for (int i=0;i<traceA.count;i++) while(traceA.frame[i].value>limit) limit*=2;
  for (int i=0;i<traceB.count;i++) while(traceB.frame[i].value>limit) limit*=2;
  for (int i=0;i<=2;i++) {
    float y=lerp(y1,y0,i/2.0), tick=limit*i/2;
    rule(x0,y,x1,y,i==0 ? MUTED : RULE,1);
    String label=tick<10000 ? nf(tick,1,0) : String.format(java.util.Locale.ROOT,"%.0e",tick);
    words(label,x0-8,y+4,mono,11,MUTED,RIGHT);
  }
  words("−30 s",x0,bottom,mono,12,MUTED,LEFT);
  words("−15 s",(x0+x1)/2,bottom,mono,12,MUTED,CENTER);
  words(paused ? "Paused" : "Now",x1,bottom,mono,12,MUTED,RIGHT);
  if (traceA.count==0 && traceB.count==0) {
    words("No samples in the last 30 s",(x0+x1)/2,lerp(y0,y1,0.4),regular,14,MUTED,CENTER);
    return;
  }
  plotTrace(traceA,NODE_GREEN,3,x0,x1,y0,y1,limit);
  plotTrace(traceB,NODE_PINK,1.8,x0,x1,y0,y1,limit);
}

void plotTrace(Trace t,color ink,float weight,float x0,float x1,float y0,float y1,float limit) {
  if(t.count==0) return;
  float x=0,y=0;
  stroke(ink); strokeWeight(weight); strokeCap(ROUND); noFill();
  int first=0;
  for(int i=0;i<t.count;i++) {
    VibrationSample p=t.frame[i];
    x=lerp(x0,x1,1-(viewTime-p.time)/float(WINDOW_MS));
    y=lerp(y1,y0,p.value/limit);
    if(i==first) beginShape();
    vertex(x,y);
    if(i==t.count-1 || t.frame[i+1].time-p.time>GAP_MS) {
      endShape();
      if(i==first) point(x,y);
      first=i+1;
    }
  }
  noStroke(); fill(ink); circle(x,y,weight+2);
}

void statusBar() {
  rule(0,footerY,pageWidth,footerY,INK,1.4);
  words("Node C  /  Fixed reference",30,footerY+37,regular,14,SECONDARY,LEFT);
  if (paused) words("Display paused",300,footerY+37,bold,14,AMBER,LEFT);
  words("F  Full screen     R  Reset angles     T  Grid     L  Grid labels     SPACE  "+(paused ? "Resume" : "Pause"),
        pageWidth-28,footerY+37,mono,13,SECONDARY,RIGHT);
}

// Render the model into its own clipped viewport. HUD labels stay upright.
void renderModel() {
  model.beginDraw();
  model.background(PAPER);
  model.hint(ENABLE_DEPTH_TEST);
  model.camera();
  model.ortho(-model.width/2.0, model.width/2.0, -model.height/2.0, model.height/2.0, -2000, 3000);
  model.pushMatrix();
  model.translate(model.width*0.50, model.height*0.390);
  model.scale(0.84*viewZoom*min(model.width/878.0,model.height/458.0));
  model.rotateX(rotX);
  model.rotateY(rotY);
  if (showGrid) model.shape(surveyGrid);
  groundShadow();
  for (RockPart part : rockParts) {
    if (part.shape!=surfaceGrid || showGrid) part.draw();
  }
  drawDeformedSurface();
  drawNodes();
  if (showGrid && showLabels) projectRulers();
  float axisX = model.screenX(50, 0, 0) - model.screenX(0, 0, 0);
  float axisY = model.screenY(50, 0, 0) - model.screenY(0, 0, 0);
  float axisZx = model.screenX(0, 0, 50) - model.screenX(0, 0, 0);
  float axisZy = model.screenY(0, 0, 50) - model.screenY(0, 0, 0);
  model.popMatrix();
  model.perspective();
  model.camera();
  model.hint(DISABLE_DEPTH_TEST);
  if (showGrid && showLabels) drawRulers();
  for (int i = 0; i < nodes.length; i++) drawNodeLabel(i);
  drawAxes(axisX, axisY, axisZx, axisZy);
  model.hint(ENABLE_DEPTH_TEST);
  model.endDraw();
}

// Rest geometry for the illustrated section. Its layers move with the live
// displacement field and taper to a fixed base.
final float X0=-338, X1=338, Z0=-241, Z1=292;
final color[] strata = {#D8C6A2,#B79A74,#D8BF92,#A66F58,#795846,#87918B,#BAC0AA,#596560};
final float[] depths = {0,23,32,45,60,66,83,90,104};

float skin(float x,float z) {
  float ridge = sin(x*0.014+0.8*sin(z*0.015)+z*0.006);
  float flank = cos(z*0.019-x*0.005+0.6*cos(x*0.011));
  return 11.0+4.8*ridge+3.3*flank+1.1*sin(x*0.026+z*0.028)
    +5.1*(noise(8+x*0.014,8+z*0.014)-0.5);
}

float seam(int layer,float x,float z) {
  if(layer==0) return skin(x,z);
  if(layer==depths.length-1) return depths[layer];
  float fold=sin(x*0.014+z*0.006+0.64*sin(z*0.012))+0.6*cos(z*0.021-x*0.006);
  float lens=4.2*sin(x*0.023+z*0.008+layer*0.43);
  return depths[layer]+(2.4+0.43*layer)*fold+lens+1.6*(noise(6+x*0.017,6+z*0.017)-0.5);
}

PShape makeGeology() {
  noiseSeed(19);
  PShape group=createShape(GROUP);
  PShape top=createShape(), contours=createShape(), faces=createShape(), seams=createShape();
  top.beginShape(QUADS); top.noStroke();
  contours.beginShape(LINES); contours.noFill(); contours.stroke(#9F9D87); contours.strokeWeight(0.75);
  int nx=58,nz=46;
  for(int i=0;i<nx;i++) for(int j=0;j<nz;j++) {
    float x=lerp(X0,X1,i/float(nx)),xx=lerp(X0,X1,(i+1)/float(nx));
    float z=lerp(Z0,Z1,j/float(nz)),zz=lerp(Z0,Z1,(j+1)/float(nz));
    float a=skin(x,z),b=skin(xx,z),c=skin(xx,zz),d=skin(x,zz);
    topVertex(top,x,z,a); topVertex(top,xx,z,b); topVertex(top,xx,zz,c); topVertex(top,x,zz,d);
    // Intersect elevation levels with the two triangles making up the cell.
    for(float level=ceil(min(min(a,b),min(c,d))/1.8)*1.8;level<=max(max(a,b),max(c,d));level+=1.8) {
      contourEdge(contours,level,x,z,a,xx,z,b); contourEdge(contours,level,xx,z,b,xx,zz,c);
      contourEdge(contours,level,xx,zz,c,x,z,a);
      contourEdge(contours,level,x,z,a,xx,zz,c); contourEdge(contours,level,xx,zz,c,x,zz,d);
      contourEdge(contours,level,x,zz,d,x,z,a);
    }
  }
  top.endShape(); contours.endShape();
  faces.beginShape(QUADS); faces.noStroke();
  seams.beginShape(LINES); seams.noFill(); seams.strokeWeight(0.75);
  rockFace(faces,seams,X0,Z0,X1,Z0,0.09);
  rockFace(faces,seams,X1,Z0,X1,Z1,0.16);
  rockFace(faces,seams,X1,Z1,X0,Z1,0.01);
  rockFace(faces,seams,X0,Z1,X0,Z0,0.19);
  faces.fill(#535851);
  faces.vertex(X0,104,Z0); faces.vertex(X1,104,Z0); faces.vertex(X1,104,Z1); faces.vertex(X0,104,Z1);
  faces.endShape(); seams.endShape();
  group.addChild(faces); group.addChild(top); group.addChild(contours); group.addChild(seams);
  surfaceGrid=makeSurfaceGrid();
  group.addChild(surfaceGrid);
  return group;
}

// Survey lines on the soil follow the surface. The separate floor grid and
// edge rulers remain fixed so the operator can judge movement against them.
PShape makeSurfaceGrid() {
  PShape grid=createShape();
  grid.beginShape(LINES);
  grid.noFill(); grid.stroke(#998D76,115); grid.strokeWeight(0.7);
  for (float x=ceil(X0/GRID_STEP)*GRID_STEP; x<=X1; x+=GRID_STEP) {
    for(float z=Z0;z<Z1;z+=8) {
      float end=min(z+8,Z1);
      grid.vertex(x,skin(x,z)-0.5,z); grid.vertex(x,skin(x,end)-0.5,end);
    }
  }
  for (float z=ceil(Z0/GRID_STEP)*GRID_STEP; z<=Z1; z+=GRID_STEP) {
    for(float x=X0;x<X1;x+=8) {
      float end=min(x+8,X1);
      grid.vertex(x,skin(x,z)-0.5,z); grid.vertex(end,skin(end,z)-0.5,z);
    }
  }
  grid.endShape();
  return grid;
}

void topVertex(PShape shape,float x,float z,float y) {
  float t=constrain(0.28+0.61*(skin(x+4,z)-y)+0.28*(skin(x,z+4)-y),0,1);
  shape.fill(lerpColor(#E8DCC3,#BAAB89,t));
  shape.vertex(x,y,z);
}

void contourEdge(PShape s,float level,float x,float z,float a,float xx,float zz,float b) {
  if((a<level)==(b<level)) return;
  float t=(level-a)/(b-a);
  s.vertex(lerp(x,xx,t),level-0.32,lerp(z,zz,t));
}

void rockFace(PShape faces,PShape seams,float x0,float z0,float x1,float z1,float shade) {
  int n=72;
  // Offset ink toward the outside of each face to keep thin strata legible.
  float dx=(z1-z0)/dist(x0,z0,x1,z1)*0.35, dz=-(x1-x0)/dist(x0,z0,x1,z1)*0.35;
  for(int i=0;i<n;i++) {
    float x=lerp(x0,x1,i/float(n)),xx=lerp(x0,x1,(i+1)/float(n));
    float z=lerp(z0,z1,i/float(n)),zz=lerp(z0,z1,(i+1)/float(n));
    for(int k=0;k<strata.length;k++) {
      float a=seam(k,x,z),b=seam(k,xx,zz),c=seam(k+1,xx,zz),d=seam(k+1,x,z);
      color clay=lerpColor(strata[k],#3B403B,shade);
      faces.fill(clay);
      faces.vertex(x,a,z); faces.vertex(xx,b,zz); faces.vertex(xx,c,zz); faces.vertex(x,d,z);
      seams.stroke(lerpColor(clay,#383D39,0.35));
      seams.vertex(x+dx,a,z+dz); seams.vertex(xx+dx,b,zz+dz);
      // Fine bedding follows its parent layer across each cut face.
      for(int j=1;j<=3;j++) {
        if(sin(i*0.21+j*2.6+k*1.8)<-0.46) continue;
        float f=j/4.0;
        seams.stroke(lerpColor(clay,j%2==0 ? #EEE2C9 : #494B40,0.24));
        seams.vertex(x+dx,lerp(a,d,f),z+dz); seams.vertex(xx+dx,lerp(b,c,f),zz+dz);
      }
    }
  }
}

void groundShadow() {
  model.noStroke();
  for(int k=0;k<8;k++) {
    float out=(8-k)*2;
    model.fill(#49443A,5);
    model.beginShape(QUADS);
    model.vertex(X0-out,FLOOR_Y-0.1*k,Z0-out);
    model.vertex(X1+out,FLOOR_Y-0.1*k,Z0-out);
    model.vertex(X1+out,FLOOR_Y-0.1*k,Z1+out);
    model.vertex(X0-out,FLOOR_Y-0.1*k,Z1+out);
    model.endShape();
  }
}

PShape makeSurveyGrid() {
  PShape grid=createShape();
  grid.beginShape(LINES);
  grid.noFill(); grid.strokeWeight(0.85);
  for(int i=-int(GRID_SIZE);i<=GRID_SIZE;i+=int(GRID_STEP)) {
    if(i%250==0) {
      grid.stroke(#DBDACE);
      grid.vertex(i,FLOOR_Y,-GRID_SIZE); grid.vertex(i,FLOOR_Y,GRID_SIZE);
      grid.vertex(-GRID_SIZE,FLOOR_Y,i); grid.vertex(GRID_SIZE,FLOOR_Y,i);
    }
    grid.stroke(#C3C4B7);
    for(int j=-int(GRID_SIZE);j<=GRID_SIZE;j+=int(GRID_STEP)) {
      grid.vertex(i-2.5,FLOOR_Y,j); grid.vertex(i+2.5,FLOOR_Y,j);
      grid.vertex(i,FLOOR_Y,j-2.5); grid.vertex(i,FLOOR_Y,j+2.5);
    }
  }
  grid.endShape();
  return grid;
}

// Rulers show axis distances from Node C in model coordinates. Tick spacing is
// thinned at small zoom levels. Edge intersections also work after orbiting.
final int TICKS = 27;
float[] xTick = new float[TICKS], zTick = new float[TICKS];

float intersectEdge(float a, float b, float c, float d, float edge) {
  if (abs(d-b) < 0.01) return Float.NaN;
  float t = (edge-b)/(d-b);
  return t >= 0 && t <= 1 ? lerp(a,c,t) : Float.NaN;
}

void projectRulers() {
  for (int n = 0; n < TICKS; n++) {
    float i = -GRID_SIZE + n*GRID_STEP;
    xTick[n] = intersectEdge(model.screenX(i,FLOOR_Y,-GRID_SIZE), model.screenY(i,FLOOR_Y,-GRID_SIZE),
                             model.screenX(i,FLOOR_Y,GRID_SIZE), model.screenY(i,FLOOR_Y,GRID_SIZE), model.height-14);
    zTick[n] = intersectEdge(model.screenY(-GRID_SIZE,FLOOR_Y,i), model.screenX(-GRID_SIZE,FLOOR_Y,i),
                             model.screenY(GRID_SIZE,FLOOR_Y,i), model.screenX(GRID_SIZE,FLOOR_Y,i), 15);
  }
}

void drawRulers() {
  model.textFont(mono, 11);
  model.textAlign(CENTER, BASELINE);
  float lastX = -10000, lastZ = -10000;
  for (int n = 0; n < TICKS; n++) {
    float i = -GRID_SIZE + n*GRID_STEP;
    float x = xTick[n], y = zTick[n];
    if (x > 48 && x < model.width-74 && abs(x-lastX) > 52) {
      model.stroke(#B1B4A5); model.strokeWeight(1);
      model.line(x, model.height-8, x, model.height-4);
      model.fill(SECONDARY);
      model.text(str(int(abs(i-nodes[2][0]))), x, model.height-14);
      lastX = x;
    }
    if (y > 20 && y < model.height-30 && abs(y-lastZ) > 36) {
      model.stroke(#B1B4A5); model.strokeWeight(1);
      model.line(3, y, 8, y);
      model.fill(SECONDARY);
      model.text(str(int(abs(i-nodes[2][2]))), 23, y+4);
      lastZ = y;
    }
  }
}

void drawAxes(float xx, float xy, float zx, float zy) {
  float x = model.width-40, y = model.height-36;
  float s = 0.46 / viewZoom;
  model.strokeWeight(1.5);
  model.stroke(SLATE); model.line(x,y,x+xx*s,y+xy*s);
  model.stroke(TEAL); model.line(x,y,x+zx*s,y+zy*s);
  model.noStroke(); model.fill(INK); model.circle(x,y,4);
  model.textFont(mono, 11); model.textAlign(CENTER,CENTER);
  model.fill(SLATE); model.text("X", x+xx*s*1.45, y+xy*s*1.45);
  model.fill(TEAL); model.text("Z", x+zx*s*1.45, y+zy*s*1.45);
}

color thermalColor(float elevation) {
  float mag = constrain(abs(elevation)/25.0, 0, 1);
  if (mag < 0.33) return lerpColor(#43768E, #70A9AA, mag/0.33);
  if (mag < 0.66) return lerpColor(#70A9AA, #DFB25E, (mag-0.33)/0.33);
  return lerpColor(#DFB25E, #B54C32, (mag-0.66)/0.34);
}

// Compact footprint: full depth at the affected node, zero outside the bowl.
// The reference node C lies outside both A and B footprints.
float influence(float x,float z,int node) {
  float dx=x-nodes[node][0], dz=z-nodes[node][2];
  float along=0.94*dx+0.342*dz, across=-0.342*dx+0.94*dz;
  float t=min(1,sqrt(along*along+1.55*across*across)/BOWL_RADIUS);
  return (1-t)*(1-t)*(1+2*t);
}

float easedLift(float current,float requested,float alpha) {
  float next=lerp(current,requested,alpha);
  return abs(next-requested)<0.002 ? requested : next;
}

// Reference sketch: inverse-distance weights, zero tilt at C, 5 units per degree.
float referenceWeight(float x,float z,int idx) {
  float a=1/(80+dist(x,z,nodes[0][0],nodes[0][2]));
  float b=1/(80+dist(x,z,nodes[1][0],nodes[1][2]));
  float c=1/(80+dist(x,z,nodes[2][0],nodes[2][2]));
  return (idx==0 ? a : b)/(a+b+c);
}
boolean advanceGround(float dt) {
  float alpha=1-exp(-dt/0.07);
  float a=easedLift(liftA,targetLiftA,alpha), b=easedLift(liftB,targetLiftB,alpha);
  float ta=easedLift(tiltA,viewA!=null && !Float.isFinite(viewA.heightChangeCm) ? viewA.tilt() : 0,alpha);
  float tb=easedLift(tiltB,viewB!=null && !Float.isFinite(viewB.heightChangeCm) ? viewB.tilt() : 0,alpha);
  if(a==liftA && b==liftB && ta==tiltA && tb==tiltB) return false;
  tiltA=ta; tiltB=tb;
  liftA=a; liftB=b;
  // Fit exceptional shifts into the cutaway without inverting the strata.
  // The numeric readout and color scale always use the unscaled cm values.
  depthScale=min(UNITS_PER_CM,MAX_SECTION_DROP/max(1,max(abs(a),abs(b))+max(ta,tb)*5/UNITS_PER_CM));
  return true;
}

// Each point is recomputed from its original coordinates. Markers, contours,
// seam lines and the land cannot accumulate drift over a lift/release cycle.
class RockPart {
  final PShape shape;
  final float[] x,y,z,weightA,weightB;
  final float[] drawnY,refA,refB;
  final int[] lineInk;
  final int[] restColor;
  final float[] lightA,lightB;
  RockPart(PShape part) {
    shape=part;
    int n=shape.getVertexCount();
    x=new float[n]; y=new float[n]; z=new float[n]; drawnY=new float[n];
    lineInk=part.getKind()==LINES ? new int[n] : null;
    weightA=new float[n]; weightB=new float[n]; refA=new float[n]; refB=new float[n];
    boolean isTop=part==rock.getChild(1);
    restColor=isTop ? new int[n] : null;
    lightA=isTop ? new float[n] : null;
    lightB=isTop ? new float[n] : null;
    float base=depths[depths.length-1];
    for(int i=0;i<n;i++) {
      x[i]=shape.getVertexX(i); y[i]=shape.getVertexY(i); z[i]=shape.getVertexZ(i);
      drawnY[i]=y[i];
      if(lineInk!=null) lineInk[i]=shape.getStroke(i);
      float fade=constrain((base-y[i])/(base-skin(x[i],z[i])),0,1);
      refA[i]=referenceWeight(x[i],z[i],0)*fade;
      refB[i]=referenceWeight(x[i],z[i],1)*fade;
      weightA[i]=influence(x[i],z[i],0)*fade;
      weightB[i]=influence(x[i],z[i],1)*fade;
      if(isTop) {
        restColor[i]=shape.getFill(i);
        lightA[i]=(influence(x[i]+2,z[i]+2,0)-influence(x[i]-2,z[i]-2,0))/5.657;
        lightB[i]=(influence(x[i]+2,z[i]+2,1)-influence(x[i]-2,z[i]-2,1))/5.657;
      }
    }
  }
  void update() {
    float downA=-liftA*depthScale,downB=-liftB*depthScale;
    for(int i=0;i<x.length;i++) {
      if(refA[i]!=0 || refB[i]!=0) {
        drawnY[i]=y[i]+downA*weightA[i]+downB*weightB[i]+(tiltA*refA[i]+tiltB*refB[i])*5*depthScale/UNITS_PER_CM;
        if(lineInk==null) shape.setVertex(i,x[i],drawnY[i],z[i]);
        if(restColor!=null) {
          float slope=lightA[i]*downA+lightB[i]*downB;
          color light=slope>0 ? #594C39 : #FFF5D9;
          shape.setFill(i,lerpColor(restColor[i],light,constrain(abs(slope)*1.1,0,0.62)));
        }
      }
    }
  }
  void draw() {
    if(lineInk==null) {
      model.shape(shape);
      return;
    }
    // Submit line endpoints together: P3D expands each stroke into triangles.
    // This keeps its generated stroke geometry attached as the soil moves.
    model.noFill(); model.strokeWeight(0.75);
    model.beginShape(LINES);
    for(int i=0;i<x.length;i+=2) {
      model.stroke(shape==surfaceGrid ? lerpColor(PAPER,INK,0.67) : lineInk[i]);
      model.vertex(x[i],drawnY[i],z[i]);
      model.vertex(x[i+1],drawnY[i+1],z[i+1]);
    }
    model.endShape();
  }
}

// Draped sensor mesh. Y is the original terrain height plus the same downward
// displacement used on the cutaway. A small offset avoids coplanar flicker.
final int DIVISIONS = 44;
float[][] meshX = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshZ = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshY = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshRest = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshCm = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshWeightA = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshWeightB = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshTiltA = new float[DIVISIONS+1][DIVISIONS+1];
float[][] meshTiltB = new float[DIVISIONS+1][DIVISIONS+1];

void surfaceVertex(int i, int j) {
  float h = meshY[i][j];
  // Translucent survey overlay leaves the shaded soil visible through the cells.
  model.fill((viewA!=null || viewB!=null) ? thermalColor(meshCm[i][j]) : #A7B8B1,95);
  model.vertex(meshX[i][j], h, meshZ[i][j]);
}

void meshSegment(int i, int j, int k, int l) {
  model.line(meshX[i][j], meshY[i][j]-0.7, meshZ[i][j],
             meshX[k][l], meshY[k][l]-0.7, meshZ[k][l]);
}

void prepareMesh() {
  for (int i = 0; i <= DIVISIONS; i++) {
    for (int j = 0; j <= DIVISIONS-i; j++) {
      float b = i/float(DIVISIONS), c = j/float(DIVISIONS), a = 1-b-c;
      float x = a*nodes[0][0]+b*nodes[1][0]+c*nodes[2][0];
      float z = a*nodes[0][2]+b*nodes[1][2]+c*nodes[2][2];
      meshX[i][j]=x; meshZ[i][j]=z; meshRest[i][j]=skin(x,z)-1.2;
      meshWeightA[i][j]=influence(x,z,0);
      meshWeightB[i][j]=influence(x,z,1);
      meshTiltA[i][j]=referenceWeight(x,z,0)*5/UNITS_PER_CM;
      meshTiltB[i][j]=referenceWeight(x,z,1)*5/UNITS_PER_CM;
    }
  }
}

void drawDeformedSurface() {
  for (int i=0;i<=DIVISIONS;i++) {
    for (int j=0;j<=DIVISIONS-i;j++) {
      float cm=liftA*meshWeightA[i][j]+liftB*meshWeightB[i][j]
        -tiltA*meshTiltA[i][j]-tiltB*meshTiltB[i][j];
      meshCm[i][j]=cm;
      meshY[i][j]=meshRest[i][j]-depthScale*cm;
    }
  }
  model.noStroke();
  model.beginShape(TRIANGLES);
  for (int i = 0; i < DIVISIONS; i++) {
    for (int j = 0; j < DIVISIONS-i; j++) {
      surfaceVertex(i,j); surfaceVertex(i+1,j); surfaceVertex(i,j+1);
      if (i+j+1 < DIVISIONS) {
        surfaceVertex(i+1,j); surfaceVertex(i+1,j+1); surfaceVertex(i,j+1);
      }
    }
  }
  model.endShape();
  model.stroke(INK, 130);
  model.strokeWeight(0.8/viewZoom);
  for (int i = 0; i < DIVISIONS; i++) {
    for (int j = 0; j < DIVISIONS-i; j++) {
      if (i % 4 == 0) meshSegment(i,j,i,j+1);
      if (j % 4 == 0) meshSegment(i,j,i+1,j);
      if ((i+j+1) % 4 == 0) meshSegment(i+1,j,i,j+1);
    }
  }
  model.stroke(INK); model.strokeWeight(1.2/viewZoom);
  for(int i=0;i<DIVISIONS;i++) {
    int j=DIVISIONS-i;
    meshSegment(i,0,i+1,0); meshSegment(0,i,0,i+1);
    meshSegment(i,j,i+1,j-1);
  }
}

void drawNodes() {
  model.noStroke();
  for(int i=0;i<nodes.length;i++) {
    SensorReading r=i==0 ? viewA : i==1 ? viewB : null;
    float x=nodes[i][0], z=nodes[i][2];
    float up=(i==0 ? liftA : i==1 ? liftB : 0)*depthScale;
    model.pushMatrix(); model.translate(x,skin(x,z)-4,z);
    // Mark the change between the reference plane and the current position.
    if(abs(up)>1) {
      model.stroke(#57655E); model.strokeWeight(1.1/viewZoom);
      float end=max(4,4-up);
      for(float y=min(4,4-up);y<end;y+=7) model.line(0,y,0,0,min(y+3,end),0);
      model.noStroke();
    }
    model.translate(0,-up,0);
    cylinder(4,-4,4.5,#555C55);
    model.rotateZ(radians(r==null ? 0 : r.roll));
    model.rotateX(radians(r==null ? 0 : r.pitch));
    // Ribbed field enclosure, gasket, cap and four captive bolt heads.
    cylinder(19,-3,0,#424B49);
    cylinder(17,-15,-3,#505E5A);
    for(int k=0;k<3;k++) cylinder(18,-6-k*3,-5-k*3,#34423E);
    cylinder(17,-18,-15,#B7ADA0);
    cylinder(14.5,-19,-18,i==2 ? #44565A : #C07746);
    for(int k=0;k<4;k++) {
      float a=HALF_PI*k+QUARTER_PI;
      model.pushMatrix(); model.translate(13*cos(a),0,13*sin(a));
      cylinder(1.65,-19.8,-18,#535952); model.popMatrix();
    }
    if(i==2) {
      model.pushMatrix(); model.translate(7,0,-4);
      cylinder(2.7,-33,-19,#3E4843); cylinder(1.3,-42,-33,#6C7771); model.popMatrix();
    } else {
      model.pushMatrix(); model.translate(0,-19.7,0);
      model.fill(#EEE7D4); model.box(12,1.3,2); model.popMatrix();
    }
    labelX[i]=model.screenX(0,i==2 ? -42 : -19,0);
    labelY[i]=model.screenY(0,i==2 ? -42 : -19,0);
    model.popMatrix();
  }
}

void cylinder(float radius,float top,float bottom,color material) {
  int sides=22;
  model.noStroke(); model.beginShape(QUAD_STRIP);
  for(int i=0;i<=sides;i++) {
    float a=TWO_PI*i/sides, x=radius*cos(a), z=radius*sin(a);
    model.fill(lerpColor(material,#212823,0.10+0.23*(1+cos(a+0.65))));
    model.vertex(x,top,z); model.vertex(x,bottom,z);
  }
  model.endShape();
  model.fill(material); model.beginShape(TRIANGLE_FAN);
  model.vertex(0,top,0);
  for(int i=0;i<=sides;i++) {
    float a=TWO_PI*i/sides; model.vertex(radius*cos(a),top,radius*sin(a));
  }
  model.endShape();
}

void drawNodeLabel(int i) {
  float x=labelX[i],y=labelY[i];
  if(x<4 || x>model.width-4 || y<4 || y>model.height-4) return;
  float sign=i==0 ? -1 : 1;
  if(x<96) sign=1;
  if(x>model.width-96) sign=-1;
  float y1=y<42 ? y+24 : y-25, elbow=x+sign*16, end=elbow+sign*12;
  model.stroke(INK); model.strokeWeight(1.1);
  model.line(x,y-3,elbow,y1); model.line(elbow,y1,end,y1);
  model.noStroke();
  String s=nodeNames[i]; model.textFont(bold,14);
  float w=ceil(model.textWidth(s))+14, left=sign>0 ? end : end-w;
  model.fill(PAPER,247); model.rect(left,y1-11,w,21);
  model.fill(INK); model.textAlign(CENTER,BASELINE); model.text(s,left+w/2,y1+5);
}

// Original orbit, zoom limits, grid/label toggles, angle reset and pause.
void mousePressed() {
  float x = mouseX/uiScale, y = mouseY/uiScale;
  if (mouseButton == LEFT && x >= pageWidth-204 && x <= pageWidth-29 && y >= 12 && y <= 46) {
    if (!editingPort) {
      editingPort = true;
      portInput = "";
      portMessage = "Type a port number and press Enter";
    }
    return;
  }
  if(mouseButton==LEFT && mouseX/uiScale<=46 && mouseY/uiScale>=sliderTop()-14 && mouseY/uiScale<=sliderTop()+168) {
    sliding=true; isDragging=false; dragBrightness(); return;
  }
  if (mouseButton == LEFT) {
    isDragging = true;
    lastMouseX = mouseX; lastMouseY = mouseY;
  }
}

void mouseDragged() {
  if(sliding) { dragBrightness(); return; }
  if (!isDragging) return;
  targetRotY += radians((mouseX-lastMouseX)*0.8);
  targetRotX += radians((mouseY-lastMouseY)*0.8);
  targetRotX = constrain(targetRotX, -PI/2+0.01, PI/2-0.01);
  lastMouseX = mouseX; lastMouseY = mouseY;
}

void mouseReleased() { sliding=false; isDragging = false; }

void mouseWheel(processing.event.MouseEvent event) {
  zoom = constrain(zoom*(1.0-event.getCount()*0.08), 0.55, 1.8);
}

void keyPressed(processing.event.KeyEvent event) {
  if (editingPort) {
    char typed = event.getKey();
    if (typed == ESC) {
      key = 0; // Prevent Processing from closing the sketch.
      editingPort = false;
      portMessage = "";
    } else if (typed == ENTER || typed == RETURN) {
      editingPort = false;
      openReceiver(portInput);
    } else if (typed == BACKSPACE && !portInput.isEmpty()) {
      portInput = portInput.substring(0, portInput.length()-1);
    } else if (portInput.length() < 12 && Character.toString(typed).matches("[a-zA-Z0-9]")) {
      portInput += typed;
    }
    return;
  }
  if (event.getKey()=='f' || event.getKey()=='F') {
    if (!event.isAutoRepeat()) {
      final GLWindow window=(GLWindow)surface.getNative();
      // Resizing can request another repaint. Dispatch outside the draw call.
      java.awt.EventQueue.invokeLater(new Runnable() {
        public void run() { window.setFullscreen(!window.isFullscreen()); }
      });
    }
    return;
  }
  keyPressed();
}

synchronized void keyPressed() {
  if (key == 'r' || key == 'R') {
    if (latestA != null) latestA = latestA.zeroAngles();
    if (latestB != null) latestB = latestB.zeroAngles();
  }
  if (key == 't' || key == 'T') showGrid = !showGrid;
  if (key == 'l' || key == 'L') showLabels = !showLabels;
  if (key == ' ') paused = !paused;
}

// Each dial spans 0 to 30+ degrees. The value below it keeps full precision.
void drawTiltGauge(float cx, float cy, String name, SensorReading r) {
  float radius=45;
  noFill(); strokeCap(SQUARE);
  for (int i=0; i<30; i++) {
    float a=PI+PI*i/30.0, b=PI+PI*(i+1)/30.0;
    stroke(i<8 ? #A8B7AC : i<20 ? #C5A160 : #B6684F);
    strokeWeight(4);
    arc(cx,cy,radius*2,radius*2,a,b+0.003);
  }
  for (int i=0;i<=30;i++) {
    float a=PI+PI*i/30.0;
    float len=i%5==0 ? 9 : 4;
    rule(cx+(radius-6)*cos(a),cy+(radius-6)*sin(a),
         cx+(radius-6-len)*cos(a),cy+(radius-6-len)*sin(a),i%5==0 ? INK : MUTED,1);
  }
  words("0",cx-radius+3,cy+10,mono,10,MUTED,CENTER);
  words("30+",cx+radius-3,cy+10,mono,10,MUTED,CENTER);
  if (r!=null) {
    float a = PI+PI*constrain(r.tilt()/30,0,1);
    float vx=cos(a),vy=sin(a);
    noStroke(); fill(INK);
    triangle(cx-vy*3,cy+vx*3,cx+vy*3,cy-vx*3,cx+vx*35,cy+vy*35);
    fill(INK); circle(cx,cy,10); fill(WHITE); circle(cx,cy,3.5);
  } else {
    noStroke(); fill(#A4AAA0); circle(cx,cy,5);
  }
  words(name,cx,cy-55,bold,15,INK,CENTER);
  words(r==null ? "—" : nf(r.tilt(),1,2)+"°",cx,cy+27,display,25,INK,CENTER);
  words(getState(r),cx,cy+45,bold,13,stateInk(r),CENTER);
}
