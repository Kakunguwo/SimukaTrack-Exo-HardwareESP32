# SIMUKA-EXO: Robotic Wrist Exoskeleton Control System

## Project Overview

**SIMUKA-EXO** is a rehabilitation exoskeleton control system designed to assist patients in performing controlled movements for physical therapy, focusing on flexion/extension and adduction/abduction of joints. The system integrates an ESP32 microcontroller for real-time servo control and a React-based frontend for monitoring and managing therapy sessions. The project ensures patient safety through precise force control, smooth emergency stop behavior, and robust error handling in the user interface.

The system addresses two critical issues:
1. **Excessive Force**: Ensures forces during flexion/extension stay within a safe range (0.04 to 1.8 N), as derived from Euler-Lagrange equations.
2. **Emergency Stop Safety**: Implements a smooth ramp-down to neutral position over 2 seconds to prevent abrupt movements that could harm the patient.
3. **Frontend Reliability**: Prevents crashes due to undefined data by providing fallback session summaries.

This repository contains the ESP32 firmware (`ExoskeletonESP32.ino`) and the React frontend (`Session.tsx`), along with supporting configurations for MQTT communication and WebSocket integration.

## Features

- **Real-Time Control**: Uses a PID controller to drive MG996R servos for precise joint movement.
- **Safety Mechanisms**:
  - Limits forces to 0.04–1.8 N for flexion/extension and up to 1 N for adduction/abduction.
  - Smooth servo transition to neutral position (60°) during emergency stops.
- **Frontend Monitoring**: Displays live angles, forces, torques, and session summaries via a React interface with Recharts visualizations.
- **MQTT Communication**: Transmits dynamics data (angles, velocities, forces, torques) to the frontend for real-time monitoring.
- **WebSocket Integration**: Ensures robust data streaming and session control.
- **Error Handling**: Prevents frontend crashes by handling undefined data gracefully.

## System Architecture

The system comprises:
- **ESP32 Firmware** (`ExoskeletonESP32.ino`): Controls two MG996R servos for flexion/extension and adduction/abduction, computes dynamics using Euler-Lagrange equations, and communicates via MQTT.
- **React Frontend** (`Session.tsx`): Provides a user interface for therapists to start/stop sessions, monitor live data, and view session summaries.
- **MQTT Broker**: Facilitates communication between the ESP32 and frontend (e.g., `mqtt.eclipseprojects.io`).
- **WebSocket Server**: Streams real-time data to the frontend using Socket.IO.

## Key Computations

### 1. Force Calculation (Flexion/Extension)
The force for flexion/extension (\( F_1 \)) is computed using the Euler-Lagrange-derived equation:
\[
F_1 = \frac{\tau_{\text{total}}}{l_1} = \frac{\tau_1 + \tau_g}{l_1}
\]
where:
- \( \tau_1 \): Control torque from the PID controller (N·m).
- \( \tau_g = m_1 \cdot g \cdot l_1 \cdot \sin(\theta_1) \): Gravity torque.
- \( m_1 = 0.05057 \, \text{kg} \): Mass of the arched element.
- \( l_1 = 0.075 \, \text{m} \): Length of the arched element.
- \( g = 9.81 \, \text{m/s}^2 \): Gravitational acceleration.
- \( \theta_1 \): Joint angle (radians).

The safe force range is 0.04 to 1.8 N, corresponding to \( \tau_1 \in [0.003, 0.1] \, \text{N·m} \) and \( \theta_1 \in [-60^\circ, 60^\circ] \). For example, at \( \theta_1 = 60^\circ \):
\[
\tau_g = 0.05057 \cdot 9.81 \cdot 0.075 \cdot \sin(60^\circ) \approx 0.003284617 \, \text{N·m}
\]
\[
F_1 = \frac{\tau_1 + 0.003284617}{0.075}
\]
To keep \( F_1 \leq 1.8 \, \text{N} \), \( \tau_1 \leq 0.1 \, \text{N·m} \), enforced by `TORQUE_LIMIT_FLEX = 0.1` and `FORCE_LIMIT_FLEX = 1.8`.

### 2. Emergency Stop Ramp-Down
During an emergency stop, servos ramp to the neutral position (60°) over 2 seconds using linear interpolation:
- Steps: 100 (20 ms each).
- Step size: \( \text{step} = \frac{\text{SERVO_NEUTRAL} - \text{current_angle}}{\text{steps}} \).
- Example: From 120° to 60°, step size = \( \frac{60 - 120}{100} = -0.6^\circ/\text{step} \).

This ensures smooth transitions, preventing patient injury.

### 3. PID Control
The PID controller computes torque as:
\[
\tau = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{de}{dt}
\]
where:
- \( e = \text{desired_angle} - \text{actual_angle} \).
- \( K_p = 0.6 \), \( K_i = 0.05 \), \( K_d = 0.1 \).
- Torque is constrained to `[-TORQUE_LIMIT_FLEX, TORQUE_LIMIT_FLEX]` (0.1 N·m) for flexion/extension and `[-TORQUE_LIMIT_ABD, TORQUE_LIMIT_ABD]` (0.065 N·m) for adduction/abduction.

## Prerequisites

- **Hardware**:
  - ESP32 development board.
  - Two MG996R servos (for flexion/extension and adduction/abduction).
  - Push buttons for start (GPIO 19) and emergency stop (GPIO 21).
  - LEDs for status indication (GPIO 2, 4, 18).
- **Software**:
  - Arduino IDE with ESP32 board support.
  - Node.js (v18 or higher) for the frontend.
  - MQTT broker (e.g., `mqtt.eclipseprojects.io`).
  - WebSocket server with Socket.IO.
- **Libraries (ESP32)**:
  - `WiFi.h`
  - `PubSubClient.h`
  - `ESPmDNS.h`
  - `WiFiUdp.h`
  - `ArduinoOTA.h`
  - `ESP32Servo.h`
  - `ArduinoJson.h`
- **Libraries (Frontend)**:
  - React (`react`, `react-dom`)
  - Socket.IO (`socket.io-client`)
  - Recharts (`recharts`)
  - Axios (`axios`)
  - React Toastify (`react-toastify`)
  - Lucide-React (`lucide-react`)
  - Tailwind CSS for styling.

## Installation

### 1. ESP32 Firmware
1. Clone the repository:
   ```bash
   git clone https://github.com/Kakunguwo/SimukaTrack-Exo-HardwareESP32.git
   cd SimukaTrack-Exo-HardwareESP32
   ```
2. Open `simukatrack_exo_code.ino` in the Arduino IDE.
3. Install required libraries via the Arduino Library Manager:
   - `ESP32Servo`
   - `PubSubClient`
   - `ArduinoJson`
   - `ESPmDNS`
   - `WiFi`
   - `WiFiUdp`
   - `ArduinoOTA`
4. Update WiFi and MQTT settings in `simukatrack_exo_code.ino`:
   ```cpp
   const char* ssid = "your-wifi-ssid";
   const char* password = "your-wifi-password";
   const char* mqtt_server = "your-mqtt-broker";
   ```
5. Connect the ESP32 to your computer and upload the firmware.

### 2. Frontend
1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```
2. Install dependencies:
   ```bash
   npm install
   ```
3. Update the WebSocket and API configurations in `src/lib/socket.io.ts` and `src/context/axios.ts`:
   ```typescript
   // src/lib/socket.io.ts
   const socket = io('your-websocket-server-url');
   ```
   ```typescript
   // src/context/axios.ts
   const apiClient = axios.create({
     baseURL: 'your-api-base-url',
   });
   ```
4. Start the development server:
   ```bash
   npm start
   ```

### 3. MQTT Broker
- Use a public MQTT broker like `mqtt.eclipseprojects.io` or set up your own (e.g., Mosquitto).
- Ensure the ESP32 and frontend are configured to connect to the same broker.

## Usage

1. **Start the System**:
   - Power on the ESP32; it will connect to WiFi and the MQTT broker.
   - The ESP32 calibrates servos to the neutral position (60°) on startup.
   - Launch the frontend application in a browser.

2. **Start a Session**:
   - Enter a session ID in the frontend and click "Enter" to fetch session data.
   - Click "Start Session" to begin therapy, sending session parameters to the ESP32 via WebSocket and MQTT.
   - Example session JSON:
     ```json
     {
         "sessionId": "test_session",
         "movementType": "flexion/extension",
         "mode": "active",
         "targetAnglePositive": 60,
         "targetAngleNegative": -60,
         "duration": 60,
         "resistanceLevel": 1,
         "cycleDuration": 2000
     }
     ```

3. **Monitor Session**:
   - The frontend displays real-time angles, forces, torques, and dynamics data.
   - Forces are visualized in a line chart, ensuring they stay within 0.04–1.8 N for flexion/extension.
   - The emergency stop status is shown with a red banner if activated.

4. **Emergency Stop**:
   - Press the emergency stop button (GPIO 21) on the ESP32 or click "Emergency Stop" in the frontend.
   - Servos smoothly ramp to 60° over 2 seconds, and the session ends.

5. **Session Summary**:
   - On session completion, the frontend displays:
     - Repetitions (based on angle changes > 10°).
     - Average force.
     - Range of motion.
     - Maximum torque (from `torque1` and `torque2`).
   - If no data is available (e.g., early termination), a fallback summary is shown.

6. **AI Insights**:
   - Click "Insights" to open a modal for entering patient and therapist feedback.
   - Generate simulated AI insights based on session data and feedback.

## Key Code Components

### 1. Force Limitation (ESP32)
Ensures flexion/extension forces stay within 0.04–1.8 N by clamping `dynamics.torque1`:
```cpp
if (dynamics.force1 > FORCE_LIMIT_FLEX) {
    dynamics.torque1 = (FORCE_LIMIT_FLEX * l1 - dynamics.gravity_torque);
    dynamics.force1 = FORCE_LIMIT_FLEX;
    Serial.println("Warning: Flexion force clamped to 1.8 N");
} else if (dynamics.force1 < -FORCE_LIMIT_FLEX) {
    dynamics.torque1 = (-FORCE_LIMIT_FLEX * l1 - dynamics.gravity_torque);
    dynamics.force1 = -FORCE_LIMIT_FLEX;
    Serial.println("Warning: Flexion force clamped to -1.8 N");
}
```

### 2. Smooth Emergency Stop (ESP32)
Ramps servos to neutral (60°) over 2 seconds:
```cpp
const int steps = RAMP_DOWN_DURATION / 20;
float flex_step = (SERVO_NEUTRAL - current_flex_angle) / (float)steps;
float abd_step = (SERVO_NEUTRAL - current_abd_angle) / (float)steps;
for (int i = 0; i < steps; i++) {
    int flex_angle = round(current_flex_angle + flex_step * i);
    int abd_angle = round(current_abd_angle + abd_step * i);
    flex_angle = constrain(flex_angle, SERVO_MIN, SERVO_MAX);
    abd_angle = constrain(abd_angle, SERVO_MIN, SERVO_MAX);
    servo_flex.write(flex_angle);
    servo_abd.write(abd_angle);
    delay(20);
}
```

### 3. Frontend Session Summary (React)
Handles undefined `liveData` to prevent `toFixed` errors:
```typescript
if (liveData && liveData.angleHistory.length > 0) {
    const activeAngleHistory = sessionData.movementType === 'flexion/extension'
        ? liveData.angleHistory.map(item => item.theta1)
        : liveData.angleHistory.map(item => item.theta2);
    const summary: SessionSummaryData = {
        reps: Math.floor(activeAngleHistory.filter(angle => Math.abs(angle) > 10).length / 2),
        avgForce: liveData.forceHistory.length > 0
            ? Math.round(liveData.forceHistory.reduce((a, b) => a + b, 0) / liveData.forceHistory.length * 100) / 100
            : 0,
        rangeOfMotion: `${Math.min(...activeAngleHistory)}° to ${Math.max(...activeAngleHistory)}°`,
        progressCompared: `Max torque: ${(liveData.torque1 !== undefined && liveData.torque2 !== undefined ? Math.max(liveData.torque1, liveData.torque2) : 0).toFixed(2)} Nm`
    };
    setSessionSummary(summary);
} else {
    setSessionSummary({
        reps: 0,
        avgForce: 0,
        rangeOfMotion: '0° to 0°',
        progressCompared: 'Max torque: 0.00 Nm'
    });
}
```

## Testing and Validation

1. **Force Safety**:
   - Start a session with `movementType: "flexion/extension"`, `targetAnglePositive: 60`, `targetAngleNegative: -60`.
   - Monitor `exoskeleton/live/dynamics` via an MQTT client. Verify `force1` stays within 0.04–1.8 N.
   - Check Serial Monitor for clamping warnings if `force1` exceeds limits.

2. **Emergency Stop**:
   - Trigger an emergency stop (GPIO 21 or frontend button).
   - Observe servos moving smoothly to 60° over 2 seconds.
   - Verify the frontend shows "EMERGENCY STOP ACTIVE" and a valid session summary.

3. **Frontend Reliability**:
   - End a session immediately after starting (before WebSocket data arrives).
   - Confirm the `Session Summary` card displays fallback values (`reps: 0`, `avgForce: 0`, etc.).
   - Check browser console for no `toFixed` errors.

4. **Data Visualization**:
   - During a session, verify the frontend displays real-time angles (`theta1`, `theta2`) and forces (`force1`, `force2`) in charts.
   - Ensure `force1` is plotted correctly in the `Force & Torque` card.

## Troubleshooting

- **ESP32 Not Connecting**:
  - Verify WiFi credentials and MQTT broker settings.
  - Check Serial Monitor for connection errors.
- **Frontend Crashes**:
  - Inspect browser console for errors.
  - Ensure WebSocket server is running and configured correctly.
- **Excessive Force**:
  - Monitor `force1` in `exoskeleton/live/dynamics`. If > 1.8 N, verify `TORQUE_LIMIT_FLEX` and PID gains.
- **Jerky Emergency Stop**:
  - Adjust `RAMP_DOWN_DURATION` (e.g., to 3000 ms) for slower transitions.

## Contributing

Contributions are welcome! Please:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/your-feature`).
3. Commit changes (`git commit -m 'Add your feature'`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact

For questions or support, contact [kakunguwo.ron@gmail.com] or open an issue on GitHub.

Portfolio: [ronniekakunguwo.vercel.app]