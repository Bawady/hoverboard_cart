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
            max-width: 400px;
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
        .value-display {
            font-size: 24px;
            font-weight: bold;
            color: #007BFF;
        }
        .footer {
            margin-top: 20px;
            font-size: 12px;
            color: #777;
        }
    </style>
    <script>
			let actualGauge, setGauge;
      async function fetchCartState() {
        try {
          const response = await fetch('/cartstate');
          const data = await response.json();
	        actualGauge.refresh(data.speed_actual);
	        setGauge.refresh(data.speed_set);
        } catch (err) {
          console.error('Failed to fetch sensor data:', err);
        }
      }

			window.onload = function () {
			  actualGauge = new JustGage({
			    id: "actualSpeed",
			    value: 0,
			    min: -255,
			    max: 255,
			    title: "Actual Speed"
			  });
			  setGauge = new JustGage({
			    id: "setSpeed",
			    value: 0,
			    min: -255,
			    max: 255,
			    title: "Set Speed"
			  });
			  setInterval(fetchCartState, 500);  // Fetch every 0.5s
			};
    </script>
</head>
<body>
    <div class="container">
        <h2>Bierkisten-Wagerl Control Center</h2>
        <input type="range" min="-255" max="255" value="0" id="slider" oninput="updateValue(this.value)">
    </div>
  	<div id="actualSpeed" style="width:250px; height:160px;"></div>
  	<div id="setSpeed" style="width:250px; height:160px;"></div>
		<script>
				function updateValue(val) {
						document.getElementById('speed').innerText = val;
						fetch('/set?value=' + val);
				}
		</script>
    <div class="footer">Bierkisten-Wagerl Control Center</div>
</body>
</html>
)rawliteral";

#endif // WEB_ASSETS_HPP