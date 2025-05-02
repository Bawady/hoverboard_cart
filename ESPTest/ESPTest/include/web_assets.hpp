#ifndef WEB_ASSETS_HPP
#define WEB_ASSETS_HPP

#include <Arduino.h>

const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <script src="https://cdn.jsdelivr.net/npm/raphael@2.3.0/raphael.min.js" defer></script>
  <script src="https://cdn.jsdelivr.net/npm/justgage@1.3.5/justgage.min.js" defer></script>
  <title>ESP32 Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      background-color: #f4f4f4;
      padding: 20px;
    }
    .container {
      max-width: 600px;
      margin: auto;
      background: white;
      padding: 20px;
      border-radius: 10px;
      box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1);
    }
    h2 {
      color: #333;
    }
    input[type="range"] {
      width: 100%;
      margin: 10px 0;
    }
    .gauges {
      display: flex;
      justify-content: center;
      gap: 20px;
      flex-wrap: wrap;
      margin-top: 20px;
    }
    .footer {
      margin-top: 20px;
      font-size: 12px;
      color: #777;
    }
    .network-list button {
      display: block;
      margin: 5px auto;
      padding: 8px 12px;
      width: 80%;
      background-color: #e0e0e0;
      border: 1px solid #ccc;
      border-radius: 5px;
      cursor: pointer;
    }
    #manualSSID, #networkForm, #wifiConfig {
      display: none;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>Bierkisten-Wagerl Control Center</h2>
    <!-- Control Options -->
    <div style="margin-bottom: 20px;">
      <fieldset>
        <legend>Drive Mode</legend>
        <label><input type="radio" name="driveMode" value="0" onchange="updateControlOptions()"> Front</label>
        <label><input type="radio" name="driveMode" value="1" onchange="updateControlOptions()"> Rear</label>
        <label><input type="radio" name="driveMode" value="2" onchange="updateControlOptions()" checked> All Wheel</label>
      </fieldset>
    </div>

    <div class="gauges" id="gaugesAxis1">
      <div id="axis1Left" style="width:250px; height:160px;"></div>
      <div id="axis1Right" style="width:250px; height:160px;"></div>
    </div>
    <div class="gauges" id="gaugesAxis2">
      <div id="axis2Left" style="width:250px; height:160px;"></div>
      <div id="axis2Right" style="width:250px; height:160px;"></div>
    </div>
  </div>

  <div class="container">
    <button onclick="toggleWifiConfig()">Configure WiFi</button>
    <div id="wifiConfig">
      <p id="loadingText">Loading networks...</p>
      <div id="networkList" class="network-list"></div>

      <form id="networkForm" action="/save_ap" method="post">
        <input type="hidden" name="ssid" id="selectedSSID">
        <label for="password">Password:</label>
        <input type="password" name="password"><br>
        <input type="submit" value="Save">
      </form>

      <button onclick="showManualInput()">Enter SSID manually</button>
      <form id="manualSSID" action="/save_ap" method="post">
        SSID: <input type="text" name="ssid"><br>
        Password: <input type="password" name="password"><br>
        <input type="submit" value="Save">
      </form>
    </div>
  </div>

  <div class="footer">Bierkisten-Wagerl Control Center</div>

  <script>
    let gauges;

    function updateValue(val) {
      fetch('/set?value=' + val);
    }

    function rpm_to_kmh(rpm) {
      let kmh  = rpm * 0.547 * 60/ 1000;
      return kmh;
    }

    function fetchState() {
      fetch('/get_state')
        .then(res => res.json())
        .then(data => {
          // assuming the server returns:
          // { speed_actual: <number>, speed_set: <number> }

          for (let i = 0; i < data.axes.length; i++) {
            const axisData = data.axes[i];
            const axisGauges = gauges[`axis${i}`];
    
            if (axisGauges) {
              let leftRpm = Math.abs(axisData.left_rpm);
              let rightRpm = Math.abs(axisData.right_rpm);
              axisGauges.left.refresh(rpm_to_kmh(leftRpm));
              axisGauges.right.refresh(rpm_to_kmh(rightRpm));
            } else {
              console.warn(`Gauges for axis${i} not found`);
            }
          }
        })
        .catch(err => console.error('Failed to fetch state:', err));
    }

    function updateControlOptions() {
      const controlMode = document.querySelector('input[name="controlMode"]:checked');
      const driveMode = document.querySelector('input[name="driveMode"]:checked');

      if (driveMode) {
        const driveValue = driveMode.value;

        fetch(`/set_mode?drive=${driveValue}`)
          .then(res => {
            if (!res.ok) throw new Error("Request failed");
          })
          .catch(err => {
            console.error("Failed to update control options:", err);
          });
      }
    }

    function toggleWifiConfig() {
      const config = document.getElementById("wifiConfig");
      config.style.display = "block";
      document.getElementById("loadingText").style.display = "block";
      document.getElementById("networkList").innerHTML = "";

      fetch("/get_networks")
        .then(res => res.json())
        .then(data => {
          document.getElementById("loadingText").style.display = "none";
          if (data.length === 0) {
            showManualInput();
            return;
          }
          const list = document.getElementById("networkList");
          document.getElementById("networkForm").style.display = "block";
          data.forEach(net => {
            const btn = document.createElement("button");
            btn.type = "button";
            btn.textContent = `${net.ssid} (${net.rssi} dBm)`;
            btn.onclick = () => {
              document.getElementById("selectedSSID").value = net.ssid;
              alert("Selected network: " + net.ssid);
            };
            list.appendChild(btn);
          });
        })
        .catch(err => {
          document.getElementById("loadingText").textContent = "Failed to load networks.";
          console.error("Network fetch error:", err);
        });
    }

    function showManualInput() {
      document.getElementById("manualSSID").style.display = "block";
    }

    window.onload = function () {
      gauges = {
        axis0: {
          left: new JustGage({ id: "axis1Left", value: 0, min: -255, max: 255, title: "km/h" }),
          right: new JustGage({ id: "axis1Right", value: 0, min: -255, max: 255, title: "km/h" })
        },
        axis1: {
          left: new JustGage({ id: "axis2Left", value: 0, min: -255, max: 255, title: "km/h" }),
          right: new JustGage({ id: "axis2Right", value: 0, min: -255, max: 255, title: "km/h" })
        }
      };

      updateControlOptions();

      setInterval(fetchState, 500);
    };
  </script>
</body>
</html>
)rawliteral";

//      <fieldset>
//        <legend>Control Mode</legend>
//        <label><input type="radio" name="controlMode" value="0" onchange="updateControlOptions()" checked> Web</label>
//        <label><input type="radio" name="controlMode" value="1" onchange="updateControlOptions()"> Throttle</label>
//      </fieldset>
//
//    <input type="range" min="-255" max="255" value="0" id="slider" oninput="updateValue(this.value)">

#endif // WEB_ASSETS_HPP