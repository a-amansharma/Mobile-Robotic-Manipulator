import processing.serial.*;

Serial esp32;

// Change this only if your ESP32 port is different
String preferredPort = "/dev/cu.usbserial-0001";

// Data received from ESP32
float receivedRoll = 0;
float receivedPitch = 0;
float receivedYaw = 0;

// Smooth displayed values
float displayRoll = 0;
float displayPitch = 0;
float displayYaw = 0;

boolean serialConnected = false;
boolean dataReceived = false;

String connectionMessage = "Searching for ESP32...";

long lastDataTime = 0;

void setup() {
  size(1000, 700, P3D);

  smooth(8);
  frameRate(60);

  surface.setTitle(
    "Mobile Robotic Manipulator - Live Orientation"
  );

  connectToESP32();
}

void connectToESP32() {
  println("Available serial ports:");
  printArray(Serial.list());

  String selectedPort = "";

  // First check the preferred port
  for (String portName : Serial.list()) {
    if (portName.equals(preferredPort)) {
      selectedPort = portName;
      break;
    }
  }

  // Automatically search if preferred port is not found
  if (selectedPort.equals("")) {
    for (String portName : Serial.list()) {
      String lowerName = portName.toLowerCase();

      if (
        lowerName.contains("usbserial") ||
        lowerName.contains("usbmodem") ||
        lowerName.contains("wchusbserial") ||
        lowerName.contains("slab_usb")
      ) {
        selectedPort = portName;
        break;
      }
    }
  }

  if (selectedPort.equals("")) {
    serialConnected = false;
    connectionMessage = "ESP32 serial port not found";
    return;
  }

  try {
    esp32 = new Serial(
      this,
      selectedPort,
      115200
    );

    esp32.clear();
    esp32.bufferUntil('\n');

    serialConnected = true;

    connectionMessage =
      "Connected to " + selectedPort;

    println(connectionMessage);
  }
  catch (Exception error) {
    serialConnected = false;
    connectionMessage = "Unable to open serial port";

    println(error.getMessage());
  }
}

void draw() {
  background(17, 20, 27);

  updateSmoothOrientation();

  drawHeader();
  drawOrientationModel();
  drawInformationPanel();
}

void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');

  if (line == null) {
    return;
  }

  line = trim(line);

  if (line.length() == 0) {
    return;
  }

  // Ignore ESP32 status messages
  if (line.startsWith("#")) {
    println(line);
    return;
  }

  String[] values = split(line, ',');

  if (values.length != 3) {
    return;
  }

  try {
    float newRoll =
      float(trim(values[0]));

    float newPitch =
      float(trim(values[1]));

    float newYaw =
      float(trim(values[2]));

    if (
      Float.isNaN(newRoll) ||
      Float.isNaN(newPitch) ||
      Float.isNaN(newYaw)
    ) {
      return;
    }

    // MPU sensor is mounted 90 degrees relative to vehicle
    receivedRoll = newPitch;
    receivedPitch = newRoll;
    receivedYaw = newYaw;

    dataReceived = true;
    lastDataTime = millis();
  }
  catch (Exception error) {
    println("Invalid serial data: " + line);
  }
}

void updateSmoothOrientation() {
  float smoothing = 0.15;

  displayRoll =
    lerp(
      displayRoll,
      receivedRoll,
      smoothing
    );

  displayPitch =
    lerp(
      displayPitch,
      receivedPitch,
      smoothing
    );

  displayYaw =
    interpolateAngle(
      displayYaw,
      receivedYaw,
      smoothing
    );
}

float interpolateAngle(
  float currentAngle,
  float targetAngle,
  float amount
) {
  float difference =
    targetAngle - currentAngle;

  while (difference > 180) {
    difference -= 360;
  }

  while (difference < -180) {
    difference += 360;
  }

  return currentAngle +
    difference * amount;
}

void drawHeader() {
  hint(DISABLE_DEPTH_TEST);

  noStroke();

  fill(13, 16, 22);
  rect(0, 0, width, 85);

  fill(255);
  textAlign(LEFT, CENTER);
  textSize(25);

  text(
    "Mobile Robotic Manipulator",
    35,
    31
  );

  fill(155, 165, 180);
  textSize(14);

  text(
    "Live MPU orientation visualizer",
    35,
    60
  );

  hint(ENABLE_DEPTH_TEST);
}

void drawOrientationModel() {
  pushMatrix();

  translate(
    width * 0.60,
    height * 0.54,
    0
  );

  // Fixed camera viewing angle
  rotateX(radians(-22));
  rotateY(radians(10));

  // Corrected live vehicle orientation
  rotateY(radians(displayYaw));
  rotateX(radians(-displayPitch));
  rotateZ(radians(displayRoll));

  drawSimpleVehicleBox();

  popMatrix();
}

void drawSimpleVehicleBox() {
  // Main cuboid vehicle body
  pushMatrix();

  stroke(235);
  strokeWeight(2);

  fill(46, 135, 215);

  /*
    Width  = 220
    Height = 90
    Length = 340

    The box is now longer in the arrow direction.
  */
  box(220, 90, 340);

  popMatrix();

  // Top surface
  pushMatrix();

  translate(0, -47, 0);

  noStroke();
  fill(70, 160, 235);

  // Swapped width and length
  box(180, 5, 300);

  popMatrix();

  // Arrow on top
  drawTopArrow();
}

void drawTopArrow() {
  pushMatrix();

  // Place arrow slightly above the box
  translate(0, -55, 0);

  // Place arrow flat on top surface
  rotateX(HALF_PI);

  stroke(255);
  strokeWeight(2);

  fill(255, 195, 45);

  /*
    Arrow points toward negative Z,
    representing the front of the vehicle.
  */

  beginShape();

  // Arrow tip
  vertex(0, -120, 0);

  // Right side of arrow head
  vertex(40, -65, 0);

  // Right side of arrow body
  vertex(16, -65, 0);
  vertex(16, 95, 0);

  // Left side of arrow body
  vertex(-16, 95, 0);
  vertex(-16, -65, 0);

  // Left side of arrow head
  vertex(-40, -65, 0);

  endShape(CLOSE);

  popMatrix();
}

void drawInformationPanel() {
  hint(DISABLE_DEPTH_TEST);

  noLights();

  float panelX = 30;
  float panelY = 120;
  float panelWidth = 285;
  float panelHeight = 500;

  noStroke();

  fill(24, 29, 39);
  rect(
    panelX,
    panelY,
    panelWidth,
    panelHeight,
    18
  );

  fill(255);
  textAlign(LEFT, TOP);
  textSize(20);

  text(
    "Orientation",
    panelX + 24,
    panelY + 24
  );

  fill(145, 155, 170);
  textSize(13);

  text(
    "Real-time sensor values",
    panelX + 24,
    panelY + 54
  );

  drawValueCard(
    "ROLL",
    displayRoll,
    panelX + 20,
    panelY + 95
  );

  drawValueCard(
    "PITCH",
    displayPitch,
    panelX + 20,
    panelY + 190
  );

  drawValueCard(
    "YAW",
    displayYaw,
    panelX + 20,
    panelY + 285
  );

  drawConnectionStatus(
    panelX + 24,
    panelY + 400
  );

  fill(125, 135, 150);
  textSize(12);

  text(
    connectionMessage,
    panelX + 24,
    panelY + 448,
    panelWidth - 48,
    40
  );

  hint(ENABLE_DEPTH_TEST);
  lights();
}

void drawValueCard(
  String label,
  float value,
  float x,
  float y
) {
  noStroke();

  fill(35, 42, 55);

  rect(
    x,
    y,
    245,
    76,
    12
  );

  fill(145, 155, 170);
  textSize(12);
  textAlign(LEFT, TOP);

  text(
    label,
    x + 17,
    y + 13
  );

  fill(255);
  textSize(29);

  text(
    nf(value, 1, 2) + "°",
    x + 17,
    y + 33
  );
}

void drawConnectionStatus(
  float x,
  float y
) {
  boolean receivingLiveData =
    dataReceived &&
    millis() - lastDataTime < 1000;

  String statusText;

  if (!serialConnected) {
    fill(245, 85, 80);
    statusText = "DISCONNECTED";
  } else if (!dataReceived) {
    fill(245, 185, 65);
    statusText = "WAITING FOR DATA";
  } else if (!receivingLiveData) {
    fill(245, 185, 65);
    statusText = "DATA STOPPED";
  } else {
    fill(70, 220, 125);
    statusText = "LIVE";
  }

  noStroke();
  ellipse(x + 6, y + 8, 11, 11);

  fill(220);
  textAlign(LEFT, TOP);
  textSize(14);

  text(
    statusText,
    x + 22,
    y
  );
}