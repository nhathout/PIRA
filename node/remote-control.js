const express = require("express");
const app = express();
const dgram = require("dgram");
const path = require("path");
const bodyParser = require("body-parser");

// Constants for ports and IP addresses
const EXPRESS_PORT = 3000;         // Port for the Express server
const ESP32_IP = "192.168.0.127";  // IP address of the ESP32
const ESP32_UDP_PORT = 8081;       // UDP port on which the ESP32 is listening
// Bounds for the destination
const BOUNDS = {
  bottomRight: { x: 1204, z: -872 },
  bottomLeft: { x: -1159, z: -754 },
  topRight: { x: 1168, z: 1119 },
  topLeft: { x: -1173, z: 1171 },
};

// Create a UDP client socket
const udpClient = dgram.createSocket("udp4");

// Middleware to parse JSON request bodies
app.use(bodyParser.json());

// Function to check if a point is within bounds
function isWithinBounds(x, z) {
  return (
    x >= BOUNDS.bottomLeft.x &&
    x <= BOUNDS.bottomRight.x &&
    z >= BOUNDS.bottomRight.z &&
    z <= BOUNDS.topRight.z
  );
}

// Serve static files from the 'public' directory
app.use(express.static(path.join(__dirname, "..", "public")));

// Create the HTML for the remote control
const controlPanelHTML = `
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Remote Control</title>
  <style>
    body {
      background-image: url('https://homesoul.com.au/cdn/shop/files/4813_BH_Suburbs_110x160cms.jpg?v=1718507785'); 
      background-size: cover;
      font-family: Arial, sans-serif;
      text-align: center;
      color: #fff;
      margin: 0;
      padding: 0;
      overflow: hidden;
    }
    .header-container {
      background-color: rgba(0, 0, 0, 0.8);
      padding: 20px;
      margin-top: 20px;
      display: inline-block;
      border-radius: 10px;
    }
    .header-container h1,
    .header-container p {
      color: hotpink;
      margin: 0;
    }
    .header-container #modeDisplay {
      font-weight: bold;
    }
    .control-buttons {
      margin-top: 20px;
    }
    button {
      padding: 15px 30px;
      font-size: 18px;
      margin: 10px;
      cursor: pointer;
      border-radius: 10px;
      border: none;
      background-color: rgb(255, 105, 180);
      color: yellow;
      transition: background-color 0.3s;
    }
    button:hover {
      background-color: rgb(160, 32, 240);
    }
    #feedback {
      margin-top: 20px;
      font-size: 18px;
      color: green;
      background-color: #FFFFFF;
      padding: 10px 20px;
      display: inline-block;
      border-radius: 10px;
    }
    #car {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%) rotate(0deg);
      width: 200px;
      height: auto;
    }
  </style>
</head>
<body>
  <div class="header-container">
    <h1>WASD Control</h1>
    <p>Current Mode: <span id="modeDisplay">Manual</span></p>
  </div>
  <div class="control-buttons">
    <button onclick="sendCommand('w')">Forward (W)</button>
    <button onclick="sendCommand('a')">Left (A)</button>
    <button onclick="sendCommand('s')">Backward (S)</button>
    <button onclick="sendCommand('d')">Right (D)</button>
    <button onclick="sendCommand('x')">Stop (X)</button>
    <button onclick="toggleMode()">Toggle Mode (P)</button>
    <div style="margin-top: 20px;">
      <label for="xCoord">X Coordinate:</label>
      <input type="number" id="xCoord" placeholder="Enter X">
      <label for="zCoord">Z Coordinate:</label>
      <input type="number" id="zCoord" placeholder="Enter Z">
      <button onclick="sendDestination()">Set Destination</button>
    </div>
  </div>
  <div id="feedback"></div>
  <img id="car" src="pngegg.png" alt="Car Image"> 
  <script>
    let currentMode = 'Manual';
    let carRotation = 0; // Initial rotation angle

    function updateModeDisplay() {
      document.getElementById('modeDisplay').textContent = currentMode;
    }

    function showFeedback(message, isError = false) {
      const feedbackElement = document.getElementById('feedback');
      feedbackElement.textContent = message;
      feedbackElement.style.color = isError ? 'red' : 'green';
      feedbackElement.style.display = 'inline-block';
      setTimeout(() => {
        feedbackElement.textContent = '';
        feedbackElement.style.display = 'none';
      }, 2000);
    }

    function rotateCar(angle) {
      carRotation = (carRotation + angle) % 360;
      const carElement = document.getElementById('car');
      carElement.style.transform = \`translate(-50%, -50%) rotate(\${carRotation}deg)\`;
    }

    function sendCommand(key) {
      if (key === 'p') {
        toggleMode();
        return;
      }

      if (currentMode === 'Autonomous' && key !== 'p') {
        showFeedback(\`Command '\${key}' ignored in Autonomous mode.\`, true);
        return;
      }

      fetch('/send-message?key=' + key)
        .then(response => response.ok ? showFeedback(\`Command '\${key.toUpperCase()}' sent\`) : response.text().then(text => { throw new Error(text); }))
        .catch(error => {
          console.error(error);
          showFeedback('Error sending command', true);
        });
    }

    function toggleMode() {
      currentMode = (currentMode === 'Manual') ? 'Autonomous' : 'Manual';
      updateModeDisplay();
      fetch('/send-message?key=p')
        .then(response => response.ok ? showFeedback(\`Mode toggled to \${currentMode}\`) : response.text().then(text => { throw new Error(text); }))
        .catch(error => {
          console.error(error);
          showFeedback('Error toggling mode', true);
        });
    }

    function sendDestination() {
      const x = document.getElementById('xCoord').value;
      const z = document.getElementById('zCoord').value;

      if (!x || !z) {
        showFeedback("Please enter valid X and Z coordinates", true);
        return;
      }

      const destination = { x: parseFloat(x), z: parseFloat(z) };
      fetch('/send-destination', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(destination)
      })
        .then(response => response.ok ? showFeedback(\`Destination set to X: \${x}, Z: \${z}\`) : response.text().then(text => { throw new Error(text); }))
        .catch(error => {
          console.error(error);
          showFeedback('Error setting destination', true);
        });
    }

    updateModeDisplay();
  </script>
</body>
</html>
`;

// Serve the control panel
app.get("/", (req, res) => {
  res.send(controlPanelHTML);
});

// Handle WASD commands
app.get("/send-message", (req, res) => {
  const key = req.query.key;
  const validKeys = ['w', 'a', 's', 'd', 'x', 'p'];
  if (validKeys.includes(key)) {
    udpClient.send(key, ESP32_UDP_PORT, ESP32_IP, (err) => {
      if (err) {
        console.error("Error sending UDP message:", err);
        res.status(500).send("Error sending message");
      } else {
        console.log(`Sent message '${key}' to ${ESP32_IP}:${ESP32_UDP_PORT}`);
        res.send(`Sent message '${key}'`);
      }
    });
  } else {
    res.status(400).send("Invalid key");
  }
});


// Handle destination setting
app.post("/send-destination", (req, res) => {
  const { x, z } = req.body;

  if (typeof x === "number" && typeof z === "number") {
    if (!isWithinBounds(x, z)) {
      res.status(400).send("Destination out of bounds");
      return;
    }

    const destinationMessage = `[${x},${z}]`; // Send as a list
    udpClient.send(destinationMessage, ESP32_UDP_PORT, ESP32_IP, (err) => {
      if (err) {
        console.error("Error sending UDP destination message:", err);
        res.status(500).send("Error sending destination");
      } else {
        console.log(`Sent destination '${destinationMessage}' to ${ESP32_IP}:${ESP32_UDP_PORT}`);
        res.send(`Destination set to X: ${x}, Z: ${z}`);
      }
    });
  } else {
    res.status(400).send("Invalid coordinates");
  }
});

// Start the Express server
app.listen(EXPRESS_PORT, () => {
  console.log(`Express server listening on port ${EXPRESS_PORT}`);
});