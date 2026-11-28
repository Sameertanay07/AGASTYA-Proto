#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <math.h>
#include <string.h>

constexpr uint8_t INDICATOR_PIN = 4;
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t OFFLINE_MS = 2000;
constexpr uint32_t HOLD_MS = 2500;
constexpr float GYRO_DELTA_TH = 0.15f;
constexpr float ROLL_DELTA_TH = 3.0f;
constexpr float PITCH_DELTA_TH = 2.0f;
constexpr float HEIGHT_TH_CM = 15.0f;

const uint8_t brainMAC[6] = {0x30, 0xAE, 0xA4, 0xC9, 0xF9, 0x4C};

const char AP_NAME[] = "SUBSIDENCE_MONITOR";
const char AP_PASSWORD[] = "12345678";
WebServer server(80);

struct SensorPacket {
  char nodeID[8];
  uint32_t sequence;
  uint32_t timestamp;
  float gyroX;
  float gyroY;
  float gyroZ;
  float roll;
  float pitch;
  float yaw;
  uint8_t altitudeState;
  float relativeHeightCM;
  float baselinePressure;
};

static_assert(sizeof(SensorPacket) == 52, "Nodes A and B send 52-byte packets");

struct NodeState {
  SensorPacket latest = {};
  bool online = false;
  bool hasBaseline = false;
  bool alertActive = false;
  float baseRoll = 0, basePitch = 0;
  float lastGX = 0, lastGY = 0, lastGZ = 0;
  uint32_t lastRxMs = 0;
  uint32_t lastAlertMs = 0;
};

struct Received {
  SensorPacket packet;
  uint32_t at;
};

NodeState nodeA, nodeB;
QueueHandle_t inbox = nullptr;

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *, const uint8_t *buf, int len);
#else
void onDataRecv(const uint8_t *, const uint8_t *buf, int len);
#endif
void sendToProcessing(char node, const SensorPacket &packet);
void processNode(NodeState &node, const SensorPacket &packet, uint32_t receivedAt);
void updateNodeStatus(NodeState &node, uint32_t now);
String getNodeStatus(const NodeState &node);

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *, const uint8_t *buf, int len) {
#else
void onDataRecv(const uint8_t *, const uint8_t *buf, int len) {
#endif
  if (len != sizeof(SensorPacket)) return;

  Received message;
  memcpy(&message.packet, buf, sizeof(message.packet));
  message.at = millis();
  xQueueSend(inbox, &message, 0);
}

void sendToProcessing(char node, const SensorPacket &packet) {
  // DATA:<node>:gx,gy,gz,roll,pitch,yaw,altitudeState,sequence,heightChangeCm
  Serial.print("DATA:");
  Serial.print(node);
  Serial.print(':');
  Serial.print(packet.gyroX, 3);
  Serial.print(',');
  Serial.print(packet.gyroY, 3);
  Serial.print(',');
  Serial.print(packet.gyroZ, 3);
  Serial.print(',');
  Serial.print(packet.roll, 2);
  Serial.print(',');
  Serial.print(packet.pitch, 2);
  Serial.print(',');
  Serial.print(packet.yaw, 2);
  Serial.print(',');
  Serial.print(packet.altitudeState);
  Serial.print(',');
  Serial.print(packet.sequence);
  Serial.print(',');
  Serial.println(packet.relativeHeightCM, 2);
}

void processNode(NodeState &node, const SensorPacket &packet, uint32_t receivedAt) {
  if (!node.hasBaseline) {
    node.baseRoll = packet.roll;
    node.basePitch = packet.pitch;
    node.hasBaseline = true;
  }

  const float gyroDelta = fabsf(packet.gyroX - node.lastGX) +
                          fabsf(packet.gyroY - node.lastGY) +
                          fabsf(packet.gyroZ - node.lastGZ);
  const bool vibration = node.online && receivedAt - node.lastRxMs < 500 &&
                         gyroDelta >= GYRO_DELTA_TH;
  const bool tilt = fabsf(packet.roll - node.baseRoll) >= ROLL_DELTA_TH ||
                    fabsf(packet.pitch - node.basePitch) >= PITCH_DELTA_TH;
  const bool deformation = packet.altitudeState != 0 ||
                           fabsf(packet.relativeHeightCM) >= HEIGHT_TH_CM;

  if (vibration || tilt || deformation) {
    node.lastAlertMs = receivedAt;
    node.alertActive = true;
  }

  node.latest = packet;
  node.lastGX = packet.gyroX;
  node.lastGY = packet.gyroY;
  node.lastGZ = packet.gyroZ;
  node.lastRxMs = receivedAt;
  node.online = true;
}

void updateNodeStatus(NodeState &node, uint32_t now) {
  if (now - node.lastRxMs > OFFLINE_MS) node.online = false;
  if (now - node.lastAlertMs >= HOLD_MS) node.alertActive = false;
}

String getAltitudeStatus(uint8_t state) {
  if (state == 1) return "INCREASED";
  if (state == 2) return "DEFORMED";
  return "NORMAL";
}

String getNodeStatus(const NodeState &node) {
  if (!node.hasBaseline) return "WAITING";
  if (!node.online) return "OFFLINE";

  // Check altitude state
  if (node.latest.altitudeState == 2) return "CRITICAL";
  if (node.latest.altitudeState == 1) return "WARNING";

  // Check tilt
  float rollChange = fabs(node.latest.roll);
  float pitchChange = fabs(node.latest.pitch);

  if (rollChange >= 8.0 || pitchChange >= 5.0) return "CRITICAL";
  if (rollChange >= 3.0 || pitchChange >= 2.0) return "WARNING";

  return "STABLE";
}

// ============================================================
// WEB PAGE - SPATIAL UI
// ============================================================

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Subsidence Monitor</title>
<style>
:root {
  --bg: #0a0a0f;
  --glass: rgba(255, 255, 255, 0.03);
  --border: rgba(255, 255, 255, 0.08);
  --text: #ffffff;
  --text-dim: rgba(255, 255, 255, 0.5);
  --green: #00ff88;
  --orange: #ff8800;
  --red: #ff0044;
  --blue: #0088ff;
}

* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
  background: var(--bg);
  color: var(--text);
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 20px;
  position: relative;
  overflow-x: hidden;
}

/* Animated background grid */
body::before {
  content: '';
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background:
    linear-gradient(rgba(255,255,255,0.02) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255,255,255,0.02) 1px, transparent 1px);
  background-size: 50px 50px;
  animation: gridMove 20s linear infinite;
  pointer-events: none;
}

@keyframes gridMove {
  0% { transform: translate(0, 0); }
  100% { transform: translate(50px, 50px); }
}

/* Floating orbs */
.orb {
  position: fixed;
  border-radius: 50%;
  filter: blur(80px);
  opacity: 0.3;
  pointer-events: none;
  animation: float 10s ease-in-out infinite;
}

.orb-1 {
  width: 300px;
  height: 300px;
  background: #0088ff;
  top: -100px;
  left: -100px;
}

.orb-2 {
  width: 400px;
  height: 400px;
  background: #ff0044;
  bottom: -150px;
  right: -150px;
  animation-delay: -5s;
}

@keyframes float {
  0%, 100% { transform: translate(0, 0); }
  50% { transform: translate(30px, 30px); }
}

.header {
  text-align: center;
  margin: 30px 0;
  position: relative;
  z-index: 1;
}

.header h1 {
  font-size: 2.5em;
  font-weight: 300;
  letter-spacing: 5px;
  background: linear-gradient(135deg, #fff 0%, #888 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.header .subtitle {
  color: var(--text-dim);
  font-size: 0.8em;
  letter-spacing: 3px;
  margin-top: 5px;
}

.container {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  gap: 30px;
  width: 100%;
  max-width: 1000px;
  position: relative;
  z-index: 1;
}

.card {
  background: var(--glass);
  backdrop-filter: blur(40px);
  -webkit-backdrop-filter: blur(40px);
  border: 1px solid var(--border);
  border-radius: 20px;
  padding: 30px;
  width: 400px;
  transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
  position: relative;
  overflow: hidden;
}

.card::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 1px;
  background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
}

.card:hover {
  transform: translateY(-5px);
  border-color: rgba(255, 255, 255, 0.2);
  box-shadow: 0 20px 40px rgba(0, 0, 0, 0.4);
}

.node-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 30px;
}

.node-title {
  font-size: 1.2em;
  font-weight: 600;
  letter-spacing: 2px;
  color: var(--text);
}

.node-indicator {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background: var(--text-dim);
  transition: all 0.5s ease;
}

.node-indicator.stable {
  background: var(--green);
  box-shadow: 0 0 20px var(--green);
}

.node-indicator.warning {
  background: var(--orange);
  box-shadow: 0 0 20px var(--orange);
  animation: pulse 2s infinite;
}

.node-indicator.critical {
  background: var(--red);
  box-shadow: 0 0 20px var(--red);
  animation: pulse 1s infinite;
}

.node-indicator.waiting {
  background: var(--text-dim);
}

.node-indicator.offline {
  background: #444;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

/* ROW 1: GYROSCOPE */
.gyro-section {
  margin-bottom: 25px;
}

.section-label {
  font-size: 0.7em;
  text-transform: uppercase;
  letter-spacing: 3px;
  color: var(--text-dim);
  margin-bottom: 15px;
}

.gyro-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 10px;
}

.gyro-item {
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border);
  border-radius: 12px;
  padding: 15px;
  text-align: center;
  transition: all 0.3s ease;
}

.gyro-item:hover {
  background: rgba(255, 255, 255, 0.06);
  border-color: rgba(255, 255, 255, 0.2);
}

.gyro-value {
  font-size: 1.5em;
  font-weight: 700;
  font-family: 'Monaco', 'Courier New', monospace;
  transition: all 0.3s ease;
}

.gyro-label {
  font-size: 0.7em;
  color: var(--text-dim);
  margin-top: 5px;
  letter-spacing: 1px;
}

/* ROW 2: ALTITUDE STATUS */
.altitude-section {
  text-align: center;
}

.altitude-status {
  font-size: 2em;
  font-weight: 700;
  letter-spacing: 2px;
  padding: 20px;
  border-radius: 15px;
  transition: all 0.5s cubic-bezier(0.4, 0, 0.2, 1);
  position: relative;
  overflow: hidden;
}

.altitude-status::before {
  content: '';
  position: absolute;
  top: 0;
  left: -100%;
  width: 100%;
  height: 100%;
  background: linear-gradient(90deg, transparent, rgba(255,255,255,0.1), transparent);
  animation: shimmer 2s infinite;
}

@keyframes shimmer {
  0% { left: -100%; }
  100% { left: 100%; }
}

.altitude-status.normal {
  background: rgba(0, 255, 136, 0.1);
  border: 1px solid rgba(0, 255, 136, 0.3);
  color: var(--green);
}

.altitude-status.increased {
  background: rgba(255, 136, 0, 0.1);
  border: 1px solid rgba(255, 136, 0, 0.3);
  color: var(--orange);
  animation: pulse 2s infinite;
}

.altitude-status.deformed {
  background: rgba(255, 0, 68, 0.1);
  border: 1px solid rgba(255, 0, 68, 0.3);
  color: var(--red);
  animation: pulse 1s infinite;
}

.altitude-status.waiting {
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border);
  color: var(--text-dim);
}

.altitude-status.offline {
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid #444;
  color: #888;
}

/* SYSTEM STATUS */
.system-status {
  margin-top: 40px;
  text-align: center;
  font-size: 1.5em;
  font-weight: 300;
  letter-spacing: 3px;
  padding: 20px 40px;
  border-radius: 15px;
  background: var(--glass);
  border: 1px solid var(--border);
  backdrop-filter: blur(40px);
  -webkit-backdrop-filter: blur(40px);
  position: relative;
  z-index: 1;
  transition: all 0.5s ease;
}

.system-status.stable {
  color: var(--green);
  border-color: rgba(0, 255, 136, 0.3);
  box-shadow: 0 0 30px rgba(0, 255, 136, 0.1);
}

.system-status.warning {
  color: var(--orange);
  border-color: rgba(255, 136, 0, 0.3);
  box-shadow: 0 0 30px rgba(255, 136, 0, 0.1);
  animation: pulse 2s infinite;
}

.system-status.critical {
  color: var(--red);
  border-color: rgba(255, 0, 68, 0.3);
  box-shadow: 0 0 30px rgba(255, 0, 68, 0.1);
  animation: pulse 1s infinite;
}

.system-status.waiting {
  color: var(--text-dim);
}

.system-status.offline {
  color: #888;
}

/* Mobile Responsive */
@media (max-width: 768px) {
  .header h1 {
    font-size: 1.5em;
    letter-spacing: 3px;
  }

  .container {
    gap: 20px;
    padding: 10px;
  }

  .card {
    width: 100%;
    max-width: 400px;
    padding: 20px;
  }

  .gyro-value {
    font-size: 1.2em;
  }

  .altitude-status {
    font-size: 1.5em;
  }

  .system-status {
    font-size: 1.2em;
    margin-top: 30px;
  }
}

@media (max-width: 480px) {
  body {
    padding: 10px;
  }

  .header h1 {
    font-size: 1.2em;
  }

  .card {
    padding: 15px;
    border-radius: 15px;
  }

  .gyro-item {
    padding: 10px;
  }

  .gyro-value {
    font-size: 1em;
  }
}
</style>
</head>
<body>

<div class="orb orb-1"></div>
<div class="orb orb-2"></div>

<div class="header">
  <h1>SUBSIDENCE MONITOR</h1>
  <div class="subtitle">REAL-TIME STRUCTURAL ANALYSIS</div>
</div>

<div class="container">

<!-- NODE A -->
<div class="card">
  <div class="node-header">
    <div class="node-title">NODE A</div>
    <div id="aIndicator" class="node-indicator waiting"></div>
  </div>

  <!-- ROW 1: GYROSCOPE -->
  <div class="gyro-section">
    <div class="section-label">Gyroscope</div>
    <div class="gyro-grid">
      <div class="gyro-item">
        <div class="gyro-value" id="aGX">0.000</div>
        <div class="gyro-label">X-AXIS</div>
      </div>
      <div class="gyro-item">
        <div class="gyro-value" id="aGY">0.000</div>
        <div class="gyro-label">Y-AXIS</div>
      </div>
      <div class="gyro-item">
        <div class="gyro-value" id="aGZ">0.000</div>
        <div class="gyro-label">Z-AXIS</div>
      </div>
    </div>
  </div>

  <!-- ROW 2: ALTITUDE STATUS -->
  <div class="altitude-section">
    <div class="section-label">Altitude Status</div>
    <div id="aAltitudeStatus" class="altitude-status waiting">WAITING</div>
  </div>
</div>

<!-- NODE B -->
<div class="card">
  <div class="node-header">
    <div class="node-title">NODE B</div>
    <div id="bIndicator" class="node-indicator waiting"></div>
  </div>

  <!-- ROW 1: GYROSCOPE -->
  <div class="gyro-section">
    <div class="section-label">Gyroscope</div>
    <div class="gyro-grid">
      <div class="gyro-item">
        <div class="gyro-value" id="bGX">0.000</div>
        <div class="gyro-label">X-AXIS</div>
      </div>
      <div class="gyro-item">
        <div class="gyro-value" id="bGY">0.000</div>
        <div class="gyro-label">Y-AXIS</div>
      </div>
      <div class="gyro-item">
        <div class="gyro-value" id="bGZ">0.000</div>
        <div class="gyro-label">Z-AXIS</div>
      </div>
    </div>
  </div>

  <!-- ROW 2: ALTITUDE STATUS -->
  <div class="altitude-section">
    <div class="section-label">Altitude Status</div>
    <div id="bAltitudeStatus" class="altitude-status waiting">WAITING</div>
  </div>
</div>

</div>

<!-- SYSTEM STATUS -->
<div class="system-status waiting" id="systemStatus">
  SYSTEM: WAITING
</div>

<script>
function updateData() {
  fetch('/data')
    .then(response => response.json())
    .then(data => {
      // NODE A
      document.getElementById("aGX").innerText = data.A.gx.toFixed(3);
      document.getElementById("aGY").innerText = data.A.gy.toFixed(3);
      document.getElementById("aGZ").innerText = data.A.gz.toFixed(3);

      // Altitude Status (NORMAL/INCREASED/DEFORMED)
      updateAltitudeStatus("aAltitudeStatus", data.A.altitudeStatus);

      // Node indicator (STABLE/WARNING/CRITICAL)
      updateIndicator("aIndicator", data.A.status);

      // NODE B
      document.getElementById("bGX").innerText = data.B.gx.toFixed(3);
      document.getElementById("bGY").innerText = data.B.gy.toFixed(3);
      document.getElementById("bGZ").innerText = data.B.gz.toFixed(3);

      // Altitude Status (NORMAL/INCREASED/DEFORMED)
      updateAltitudeStatus("bAltitudeStatus", data.B.altitudeStatus);

      // Node indicator (STABLE/WARNING/CRITICAL)
      updateIndicator("bIndicator", data.B.status);

      // SYSTEM STATUS
      let system = data.system;
      let sysElement = document.getElementById("systemStatus");
      sysElement.innerText = "SYSTEM: " + system;
      sysElement.className = "system-status " + system.toLowerCase();
    });
}

function updateAltitudeStatus(id, status) {
  let element = document.getElementById(id);
  element.innerText = status;
  element.className = "altitude-status " + status.toLowerCase();
}

function updateIndicator(id, status) {
  let element = document.getElementById(id);
  element.className = "node-indicator " + status.toLowerCase();
}

setInterval(updateData, 500);
updateData();
</script>

</body>
</html>
)rawliteral";

// ============================================================
// WEB DATA API
// ============================================================

void handleData() {
  String statusA = getNodeStatus(nodeA);
  String statusB = getNodeStatus(nodeB);

  String systemStatus = "STABLE";

  if (statusA == "CRITICAL" || statusB == "CRITICAL") {
    systemStatus = "CRITICAL";
  }
  else if (statusA == "WARNING" || statusB == "WARNING") {
    systemStatus = "WARNING";
  }
  else if (statusA == "WAITING" || statusB == "WAITING") {
    systemStatus = "WAITING";
  }
  else if (statusA == "OFFLINE" || statusB == "OFFLINE") {
    systemStatus = "OFFLINE";
  }

  String json = "{";

  json += "\"A\":{";
  json += "\"gx\":" + String(nodeA.latest.gyroX, 3);
  json += ",\"gy\":" + String(nodeA.latest.gyroY, 3);
  json += ",\"gz\":" + String(nodeA.latest.gyroZ, 3);
  json += ",\"altitudeStatus\":\"" + getAltitudeStatus(nodeA.latest.altitudeState) + "\"";
  json += ",\"status\":\"" + statusA + "\"";
  json += "},";

  json += "\"B\":{";
  json += "\"gx\":" + String(nodeB.latest.gyroX, 3);
  json += ",\"gy\":" + String(nodeB.latest.gyroY, 3);
  json += ",\"gz\":" + String(nodeB.latest.gyroZ, 3);
  json += ",\"altitudeStatus\":\"" + getAltitudeStatus(nodeB.latest.altitudeState) + "\"";
  json += ",\"status\":\"" + statusB + "\"";
  json += "},";

  json += "\"system\":\"" + systemStatus + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void startupFailed(const char *step) {
  Serial.print("Node C startup failed: ");
  Serial.println(step);
  while (true) {
    digitalWrite(INDICATOR_PIN, HIGH);
    delay(100);
    digitalWrite(INDICATOR_PIN, LOW);
    delay(100);
  }
}

void setup() {
  pinMode(INDICATOR_PIN, OUTPUT);
  digitalWrite(INDICATOR_PIN, LOW);
  Serial.begin(115200);
  delay(500);

  inbox = xQueueCreate(64, sizeof(Received));
  if (!inbox) startupFailed("receive queue");
  if (!WiFi.mode(WIFI_AP_STA)) startupFailed("Wi-Fi AP+station mode");
  if (esp_wifi_set_mac(WIFI_IF_STA, brainMAC) != ESP_OK) startupFailed("station MAC");
  // The soft AP sets the shared radio channel for the dashboard and ESP-NOW.
  if (!WiFi.softAP(AP_NAME, AP_PASSWORD, ESPNOW_CHANNEL)) startupFailed("dashboard access point");
  if (esp_now_init() != ESP_OK) startupFailed("ESP-NOW initialization");
  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) startupFailed("receive callback");

  server.on("/", []() {
    server.send_P(200, "text/html", webpage);
  });
  server.on("/data", handleData);
  server.begin();

  Serial.print("Node C MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  Received message;
  while (xQueueReceive(inbox, &message, 0) == pdTRUE) {
    const SensorPacket &packet = message.packet;
    if (strncmp(packet.nodeID, "NODE_A", sizeof(packet.nodeID)) == 0) {
      processNode(nodeA, packet, message.at);
      sendToProcessing('A', packet);
    } else if (strncmp(packet.nodeID, "NODE_B", sizeof(packet.nodeID)) == 0) {
      processNode(nodeB, packet, message.at);
      sendToProcessing('B', packet);
    }
  }

  const uint32_t now = millis();
  updateNodeStatus(nodeA, now);
  updateNodeStatus(nodeB, now);
  if (nodeA.alertActive || nodeB.alertActive) {
    digitalWrite(INDICATOR_PIN, HIGH);
  } else if (!nodeA.online || !nodeB.online) {
    digitalWrite(INDICATOR_PIN, (now % 1500) < 120);
  } else {
    digitalWrite(INDICATOR_PIN, LOW);
  }
  server.handleClient();
  delay(1);
}
