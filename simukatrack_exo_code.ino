#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <math.h>

// WiFi credentials
const char* ssid = "TECNO SPARK 10 Pro";
const char* password = "kronicles";

// MQTT configuration
const char* mqtt_server = "mqtt.eclipseprojects.io";
// const char* mqtt_server = "172.22.176.1";
// const char* mqtt_server = " 192.168.138.242"; // wireless connection
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_client_id = "ESP32Exoskeleton";

// MQTT topics
const char* stop_topic = "exoskeleton/stop";
const char* session_topic_prefix = "exoskeleton/sessions/";
const char* live_data_topic = "exoskeleton/live/data";
const char* dynamics_data_topic = "exoskeleton/live/dynamics";
const char* esp32Status = "exoskeleton/wifi_mqtt/status";

// Hardware pins
const int esp_led = 2;
const int mqtt_wifi_led = 4;
const int mode_led = 18;
const int start_switch = 19;
const int emergency_stop_switch = 21;
const int servo_flex_pin = 26;
const int servo_abd_pin = 27;

// Servo configuration
Servo servo_flex;
Servo servo_abd;
const int SERVO_NEUTRAL = 60;
const int SERVO_MIN = 0;
const int SERVO_MAX = 120;

// Dynamic model parameters
const float m1 = 0.05057; // Mass of arched element (kg)
const float l1 = 0.075;   // Length of arched element (m)
const float m2 = 0.01144; // Mass of T-element (kg)
const float l2 = 0.065;   // Length of T-element (m)
const float g = 9.81;     // Gravitational acceleration (m/s^2)
const float b = 0.01;     // Damping coefficient (N.m.s/rad)
const float I1 = m1 * l1 * l1; // Moment of inertia for arched element (kg.m^2)
const float I2 = m2 * l2 * l2; // Moment of inertia for T-element (kg.m^2)
const float dt = 0.02;    // Time step for numerical integration (s)

// PID parameters
const float Kp = 0.6;     // Reduced for smoother control
const float Ki = 0.05;    // Increased for better steady-state
const float Kd = 0.1;
const float TORQUE_LIMIT_FLEX = 0.1; // N.m for flexion/extension
const float TORQUE_LIMIT_ABD = 0.065; // N.m for adduction/abduction
const float FORCE_LIMIT_FLEX = 1.8; // N for flexion/extension

// Range of motion limits
const float theta1_min = -60.0; // Degrees
const float theta1_max = 60.0;
const float theta2_min = -40.0;
const float theta2_max = 40.0;

// Timing constants
const unsigned long HEARTBEAT_INTERVAL = 1000;
const unsigned long RECONNECT_DELAY = 5000;
const unsigned long LIVE_DATA_INTERVAL = 100;
const unsigned long DYNAMICS_UPDATE_INTERVAL = 20;
const unsigned long WIFI_TIMEOUT = 10000;
const unsigned long STATUS_UPDATE_INTERVAL = 3000;
const int DEBOUNCE_DELAY = 300;
const int MAX_STATUS_RETRIES = 3;
const unsigned long RAMP_DOWN_DURATION = 2000; // 2 seconds for emergency stop ramp-down

// Explicit PI definition
#define PI 3.14159265358979323846

// Helper macros
#define RADIANS(deg) ((deg) * PI / 180.0)
#define DEGREES(rad) ((rad) * 180.0 / PI)

// PID state structure
struct PIDState {
    float error_prev1;
    float error_prev2;
    float integral1;
    float integral2;
} pid = {0, 0, 0, 0};

// Session state structure
struct SessionState {
    String session_id;
    unsigned long start_time;
    unsigned long duration;
    String movement_type;
    String mode;
    int target_angle;
    int target_angle_positive;
    int target_angle_negative;
    int resistance_level;
    unsigned long cycle_duration;
    bool active;
    bool emergency_stop;
    int current_angle;
} state = {"", 0, 0, "", "", 0, 0, 0, 0, 2000, false, false, 0};

// Dynamic state structure
struct DynamicState {
    float theta1;
    float theta2;
    float theta1_dot;
    float theta2_dot;
    float theta1_ddot;
    float theta2_ddot;
    float torque1;
    float torque2;
    float gravity_torque;
    float force1;
    float force2;
} dynamics = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Connection status tracking
bool lastWifiStatus = false;
bool lastMqttStatus = false;
unsigned long lastStatusUpdate = 0;
bool isCalibrated = false;

WiFiClient espClient;
PubSubClient client(espClient);

void calibrate_servos() {
    Serial.println("Starting servo calibration...");
    digitalWrite(mode_led, HIGH);
    update_connection_status("calibration");

    servo_flex.write(SERVO_NEUTRAL);
    servo_abd.write(SERVO_NEUTRAL);
    delay(500);

    Serial.println("Servo calibration completed");
    isCalibrated = true;
    digitalWrite(mode_led, LOW);
    update_connection_status("online");
}

void setup() {
    Serial.begin(115200);
    Serial.println("Exoskeleton initializing...");

    pinMode(esp_led, OUTPUT);
    pinMode(mqtt_wifi_led, OUTPUT);
    pinMode(mode_led, OUTPUT);
    digitalWrite(esp_led, HIGH);
    digitalWrite(mqtt_wifi_led, LOW);
    digitalWrite(mode_led, LOW);

    pinMode(start_switch, INPUT_PULLUP);
    pinMode(emergency_stop_switch, INPUT_PULLUP);

    servo_flex.attach(servo_flex_pin, 500, 2500);
    servo_abd.attach(servo_abd_pin, 500, 2500);
    if (!servo_flex.attached() || !servo_abd.attached()) {
        Serial.println("Error: Servo attachment failed!");
        while (true);
    }

    calibrate_servos();
    setup_wifi();
    setup_ota();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    update_connection_status("online");
}

void setup_wifi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long start_attempt = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - start_attempt < WIFI_TIMEOUT) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed! Retrying in loop...");
        digitalWrite(mqtt_wifi_led, LOW);
        update_connection_status("offline");
        return;
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(mqtt_wifi_led, HIGH);
    update_connection_status("online");
}

void maintain_wifi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        digitalWrite(mqtt_wifi_led, LOW);
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        unsigned long start_attempt = millis();

        while (WiFi.status() != WL_CONNECTED && millis() - start_attempt < WIFI_TIMEOUT) {
            Serial.print(".");
            delay(500);
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi reconnected");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            digitalWrite(mqtt_wifi_led, HIGH);
            update_connection_status("online");
        } else {
            Serial.println("\nWiFi reconnection failed!");
            digitalWrite(mqtt_wifi_led, LOW);
            update_connection_status("offline");
        }
    }
}

void setup_ota() {
    ArduinoOTA.setHostname("Exoskeleton_ESP32");

    ArduinoOTA
        .onStart([]() {
            Serial.println("OTA update started");
            stop_session();
        })
        .onEnd([]() {
            Serial.println("OTA update completed");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("OTA Error[%u]: ", error);
            switch (error) {
                case OTA_AUTH_ERROR: Serial.println("Auth Failed"); break;
                case OTA_BEGIN_ERROR: Serial.println("Begin Failed"); break;
                case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
                case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
                case OTA_END_ERROR: Serial.println("End Failed"); break;
            }
        });

    ArduinoOTA.begin();
    Serial.println("OTA initialized");
}

void update_connection_status(const char* status) {
    Serial.println("Checking connection status...");

    bool currentWifiStatus = (WiFi.status() == WL_CONNECTED);
    bool currentMqttStatus = client.connected();

    if (currentWifiStatus != lastWifiStatus || currentMqttStatus != lastMqttStatus || millis() - lastStatusUpdate > STATUS_UPDATE_INTERVAL || strcmp(status, "calibration") == 0) {
        Serial.print("Connection status - WiFi: ");
        Serial.print(currentWifiStatus ? "Connected" : "Disconnected");
        Serial.print(", MQTT: ");
        Serial.println(currentMqttStatus ? "Connected" : "Disconnected");
        Serial.print("Publishing status: ");
        Serial.println(status);

        if (currentWifiStatus || strcmp(status, "calibration") == 0) {
            int retries = 0;
            bool published = false;
            while (retries < MAX_STATUS_RETRIES && !published && (client.connected() || strcmp(status, "calibration") == 0)) {
                if (client.publish(esp32Status, status, true)) {
                    Serial.print("ESP32 status published successfully: ");
                    Serial.println(status);
                    digitalWrite(mqtt_wifi_led, currentMqttStatus ? HIGH : LOW);
                    published = true;
                } else {
                    Serial.print("Failed to publish ESP32 status (attempt ");
                    Serial.print(retries + 1);
                    Serial.print("): ");
                    Serial.println(status);
                    retries++;
                    delay(200);
                }
            }
            if (!published) {
                Serial.println("Failed to publish ESP32 status after max retries or MQTT disconnected");
                digitalWrite(mqtt_wifi_led, LOW);
            }
        } else {
            Serial.println("Cannot publish status: WiFi disconnected");
            digitalWrite(mqtt_wifi_led, LOW);
        }

        lastWifiStatus = currentWifiStatus;
        lastMqttStatus = currentMqttStatus;
        lastStatusUpdate = millis();
    }
}

void reconnect() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected, attempting to maintain WiFi...");
        digitalWrite(mqtt_wifi_led, LOW);
        maintain_wifi();
        return;
    }

    if (!client.connected()) {
        Serial.print("Attempting MQTT connection to ");
        Serial.print(mqtt_server);
        Serial.println("...");
        digitalWrite(mqtt_wifi_led, LOW);

        if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
            Serial.println("MQTT connected successfully");
            digitalWrite(mqtt_wifi_led, HIGH);
            
            client.subscribe(stop_topic, 1);
            String session_topic = String(session_topic_prefix) + "+";
            client.subscribe(session_topic.c_str(), 1);
            
            Serial.println("Subscribed to topics:");
            Serial.println("- " + String(stop_topic));
            Serial.println("- " + session_topic);
            
            update_connection_status("online");
            delay(100);
        } else {
            Serial.print("MQTT connection failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            update_connection_status("offline");
            delay(RECONNECT_DELAY);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received on topic [");
    Serial.print(topic);
    Serial.print("] Length: ");
    Serial.println(length);

    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    
    Serial.println("Payload content:");
    Serial.println(message);

    if (String(topic) == stop_topic) {
        Serial.println("Stop command received");
        stop_session();
        return;
    }

    if (String(topic).startsWith(session_topic_prefix)) {
        Serial.println("Session data received on topic: " + String(topic));
        Serial.println("Payload: " + String(message));
        process_session_data(message);
    }
}

void process_session_data(String payload) {
    Serial.println("Processing session data...");
    Serial.println("Raw JSON:");
    Serial.println(payload);

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    state.session_id = doc["sessionId"].as<String>();
    state.duration = doc["duration"] | 0;
    state.movement_type = doc["movementType"].as<String>();
    state.mode = doc["mode"].as<String>();
    state.target_angle = doc["targetAngle"] | 0;
    state.target_angle_positive = doc["targetAnglePositive"] | 0;
    state.target_angle_negative = doc["targetAngleNegative"] | 0;
    state.resistance_level = doc["resistanceLevel"] | 0;
    state.cycle_duration = doc["cycleDuration"] | 2000;

    if (state.session_id.isEmpty() || state.duration == 0 || state.movement_type.isEmpty() || state.mode.isEmpty()) {
        Serial.println("Error: Invalid session data - missing required fields");
        return;
    }

    if (state.cycle_duration < 500 || state.cycle_duration > 5000) {
        Serial.println("Warning: cycleDuration out of range (500-5000ms)");
        state.cycle_duration = constrain(state.cycle_duration, 500, 5000);
    }

    start_session();
}

void start_session() {
    if (!isCalibrated) {
        Serial.println("Error: Cannot start session - servos not calibrated");
        return;
    }

    if (digitalRead(emergency_stop_switch) == LOW) {
        Serial.println("Error: Cannot start session - emergency stop switch is active");
        state.emergency_stop = true;
        digitalWrite(mode_led, LOW);
        return;
    }

    state.emergency_stop = false;
    state.start_time = millis();
    state.active = true;

    dynamics.theta1 = 0;
    dynamics.theta2 = 0;
    dynamics.theta1_dot = 0;
    dynamics.theta2_dot = 0;
    dynamics.theta1_ddot = 0;
    dynamics.theta2_ddot = 0;
    dynamics.torque1 = 0;
    dynamics.torque2 = 0;
    dynamics.gravity_torque = 0;
    dynamics.force1 = 0;
    dynamics.force2 = 0;

    pid.error_prev1 = 0;
    pid.error_prev2 = 0;
    pid.integral1 = 0;
    pid.integral2 = 0;

    digitalWrite(mode_led, state.mode == "active" ? HIGH : LOW);

    Serial.println("===== Session Started =====");
    Serial.println("Session ID: " + state.session_id);
    Serial.println("Duration: " + String(state.duration) + " ms");
    Serial.println("Movement Type: " + state.movement_type);
    Serial.println("Mode: " + state.mode);
    Serial.println("Target Angle: " + String(state.target_angle));
    Serial.println("Target Angle Positive: " + String(state.target_angle_positive));
    Serial.println("Target Angle Negative: " + String(state.target_angle_negative));
    Serial.println("Resistance Level: " + String(state.resistance_level));
    Serial.println("Cycle Duration: " + String(state.cycle_duration) + " ms");
    Serial.println("Active: " + String(state.active));
    Serial.println("Emergency Stop: " + String(state.emergency_stop));
    Serial.println("==========================");
}

void stop_session() {
    Serial.println("Stopping session...");

    // Get current servo positions
    int current_flex_angle = servo_flex.read();
    int current_abd_angle = servo_abd.read();
    
    // Calculate steps for smooth ramp-down (2 seconds, 20ms per step = 100 steps)
    const int steps = RAMP_DOWN_DURATION / 20;
    float flex_step = (SERVO_NEUTRAL - current_flex_angle) / (float)steps;
    float abd_step = (SERVO_NEUTRAL - current_abd_angle) / (float)steps;

    Serial.println("Ramping down servos to neutral position...");
    for (int i = 0; i < steps; i++) {
        int flex_angle = round(current_flex_angle + flex_step * i);
        int abd_angle = round(current_abd_angle + abd_step * i);
        
        flex_angle = constrain(flex_angle, SERVO_MIN, SERVO_MAX);
        abd_angle = constrain(abd_angle, SERVO_MIN, SERVO_MAX);
        
        servo_flex.write(flex_angle);
        servo_abd.write(abd_angle);
        delay(20); // 20ms per step for smooth motion
    }
    
    // Ensure final position is exactly neutral
    servo_flex.write(SERVO_NEUTRAL);
    servo_abd.write(SERVO_NEUTRAL);
    
    // Reset dynamic and PID states
    dynamics.theta1 = 0;
    dynamics.theta2 = 0;
    dynamics.theta1_dot = 0;
    dynamics.theta2_dot = 0;
    dynamics.theta1_ddot = 0;
    dynamics.theta2_ddot = 0;
    dynamics.torque1 = 0;
    dynamics.torque2 = 0;
    dynamics.gravity_torque = 0;
    dynamics.force1 = 0;
    dynamics.force2 = 0;
    pid.error_prev1 = 0;
    pid.error_prev2 = 0;
    pid.integral1 = 0;
    pid.integral2 = 0;

    state.active = false;
    state.emergency_stop = true;
    digitalWrite(mode_led, LOW);
    Serial.println("Session stopped");
}

void check_emergency_stop() {
    static unsigned long last_emergency_press = 0;
    static bool last_emergency_state = HIGH;

    bool current_emergency_state = digitalRead(emergency_stop_switch);

    if (current_emergency_state != last_emergency_state && millis() - last_emergency_press > DEBOUNCE_DELAY) {
        last_emergency_press = millis();
        last_emergency_state = current_emergency_state;

        if (current_emergency_state == LOW && !state.emergency_stop) {
            Serial.println("Emergency stop switch pressed at " + String(millis()) + "ms!");
            state.emergency_stop = true;
            stop_session();
            send_emergency_stop();
        } else if (current_emergency_state == HIGH && state.emergency_stop) {
            Serial.println("Emergency stop switch released at " + String(millis()) + "ms.");
            state.emergency_stop = false;
        }
    }

    if (millis() - last_emergency_press > 500) {
        Serial.print("Emergency stop switch state at ");
        Serial.print(millis());
        Serial.print("ms: ");
        Serial.println(current_emergency_state == HIGH ? "Released" : "Pressed");
        last_emergency_press = millis();
    }
}

void check_start_switch() {
    static unsigned long last_press = 0;
    if (digitalRead(start_switch) == LOW && !state.active && millis() - last_press > DEBOUNCE_DELAY) {
        Serial.println("Start switch pressed - waiting for MQTT session command");
        last_press = millis();
    }
}

float compute_pid(float desired_angle, float actual_angle, int joint) {
    float error = RADIANS(desired_angle - actual_angle);
    float* integral = (joint == 1) ? &pid.integral1 : &pid.integral2;
    float* error_prev = (joint == 1) ? &pid.error_prev1 : &pid.error_prev2;

    *integral += error * dt;
    float derivative = (error - *error_prev) / dt;
    float torque = Kp * error + Ki * *integral + Kd * derivative;

    // Apply joint-specific torque limits
    torque = constrain(torque, 
                       (joint == 1) ? -TORQUE_LIMIT_FLEX : -TORQUE_LIMIT_ABD, 
                       (joint == 1) ? TORQUE_LIMIT_FLEX : TORQUE_LIMIT_ABD);
    *error_prev = error;

    return torque;
}

void update_dynamics() {
    float current_theta1 = RADIANS(servo_flex.read() - SERVO_NEUTRAL);
    float current_theta2 = RADIANS(servo_abd.read() - SERVO_NEUTRAL);

    dynamics.gravity_torque = m1 * g * l1 * sin(current_theta1);

    dynamics.theta1_ddot = (dynamics.torque1 - b * dynamics.theta1_dot - dynamics.gravity_torque) / I1;
    dynamics.theta2_ddot = (dynamics.torque2 - b * dynamics.theta2_dot) / I2;

    dynamics.theta1_dot += dynamics.theta1_ddot * dt;
    dynamics.theta2_dot += dynamics.theta2_ddot * dt;

    dynamics.theta1 = constrain(current_theta1, RADIANS(theta1_min), RADIANS(theta1_max));
    dynamics.theta2 = constrain(current_theta2, RADIANS(theta2_min), RADIANS(theta2_max));

    float tau_total1 = dynamics.torque1 + dynamics.gravity_torque;
    float tau_total2 = dynamics.torque2;
    dynamics.force1 = tau_total1 / l1;
    dynamics.force2 = tau_total2 / l2;

    // Clamp force1 to safe limit (1.8 N)
    if (dynamics.force1 > FORCE_LIMIT_FLEX) {
        dynamics.torque1 = (FORCE_LIMIT_FLEX * l1 - dynamics.gravity_torque);
        dynamics.force1 = FORCE_LIMIT_FLEX;
        Serial.println("Warning: Flexion force clamped to 1.8 N");
    } else if (dynamics.force1 < -FORCE_LIMIT_FLEX) {
        dynamics.torque1 = (-FORCE_LIMIT_FLEX * l1 - dynamics.gravity_torque);
        dynamics.force1 = -FORCE_LIMIT_FLEX;
        Serial.println("Warning: Flexion force clamped to -1.8 N");
    }
}

void control_servos() {
    if (!state.active || state.emergency_stop) {
        Serial.println("Control servos skipped: Session inactive or emergency stop");
        return;
    }

    unsigned long current_time = millis();
    if (current_time - state.start_time > state.duration) {
        Serial.println("Session duration completed");
        stop_session();
        return;
    }

    if (state.mode != "active") {
        Serial.println("Control servos skipped: Mode not active");
        return;
    }

    unsigned long elapsed = current_time - state.start_time;
    float cycle_position = fmod(elapsed / (float)state.cycle_duration, 2.0) * PI;

    auto move_servo = [&](Servo& servo, const String& movement, int joint) {
        int target_positive = state.target_angle_positive;
        int target_negative = state.target_angle_negative;
        float sin_value = sin(cycle_position);
        int desired_angle;
        if (sin_value >= 0) {
            desired_angle = SERVO_NEUTRAL + (target_positive * sin_value);
        } else {
            desired_angle = SERVO_NEUTRAL + (abs(target_negative) * sin_value);
        }
        desired_angle = constrain(desired_angle, 
                                 SERVO_NEUTRAL - abs(state.target_angle_negative), 
                                 SERVO_NEUTRAL + abs(state.target_angle_positive));
        desired_angle = constrain(desired_angle, SERVO_MIN, SERVO_MAX);

        int current_angle = servo.read();
        float torque = compute_pid(desired_angle, current_angle, joint);
        
        if (joint == 1) {
            dynamics.torque1 = torque;
            state.current_angle = DEGREES(dynamics.theta1);
        } else {
            dynamics.torque2 = torque;
            state.current_angle = DEGREES(dynamics.theta2);
        }

        servo.write(desired_angle);
        Serial.print("Moving servo (");
        Serial.print(movement);
        Serial.print(") to angle: ");
        Serial.println(desired_angle);
    };

    if (state.movement_type == "flexion/extension") {
        move_servo(servo_flex, "flexion/extension", 1);
        dynamics.torque2 = 0;
        dynamics.theta2 = 0;
    } else if (state.movement_type == "adduction/abduction") {
        move_servo(servo_abd, "adduction/abduction", 2);
        dynamics.torque1 = 0;
        dynamics.theta1 = 0;
    } else if (state.movement_type == "combined") {
        move_servo(servo_flex, "flexion/extension", 1);
        move_servo(servo_abd, "adduction/abduction", 2);
    }

    update_dynamics();
}

void send_live_data() {
    if (!client.connected()) {
        Serial.println("Cannot send live data: MQTT not connected");
        return;
    }

    DynamicJsonDocument doc(256);
    
    doc["sessionId"] = state.session_id;
    doc["angle"] = state.current_angle;
    doc["timestamp"] = millis();
    doc["emergency"] = state.emergency_stop;

    char buffer[256];
    serializeJson(doc, buffer);

    if (client.publish(live_data_topic, buffer, false)) {
        Serial.println("Live data sent successfully");
    } else {
        Serial.println("Failed to send live data");
    }
}

void send_dynamics_data() {
    if (!client.connected()) {
        Serial.println("Cannot send dynamics data: MQTT not connected");
        return;
    }

    DynamicJsonDocument doc(512);
    
    doc["sessionId"] = state.session_id;
    doc["theta1"] = DEGREES(dynamics.theta1);
    doc["theta2"] = DEGREES(dynamics.theta2);
    doc["theta1_dot"] = dynamics.theta1_dot;
    doc["theta2_dot"] = dynamics.theta2_dot;
    doc["theta1_ddot"] = dynamics.theta1_ddot;
    doc["theta2_ddot"] = dynamics.theta2_ddot;
    doc["torque1"] = dynamics.torque1;
    doc["torque2"] = dynamics.torque2;
    doc["gravity_torque"] = dynamics.gravity_torque;
    doc["force1"] = dynamics.force1;
    doc["force2"] = dynamics.force2;
    doc["timestamp"] = millis();

    char buffer[512];
    serializeJson(doc, buffer);

    if (client.publish(dynamics_data_topic, buffer, false)) {
        Serial.println("Dynamics data sent successfully");
    } else {
        Serial.println("Failed to send dynamics data");
    }
}

void send_emergency_stop() {
    if (!client.connected()) {
        Serial.println("Cannot send emergency stop data: MQTT not connected");
        return;
    }

    DynamicJsonDocument doc(256);
    
    doc["sessionId"] = state.session_id;
    doc["emergency"] = true;
    doc["switchState"] = digitalRead(emergency_stop_switch) == LOW ? "Pressed" : "Released";
    doc["message"] = "Emergency stop triggered";
    doc["timestamp"] = millis();

    char buffer[256];
    serializeJson(doc, buffer);

    if (client.publish(live_data_topic, buffer, false)) {
        Serial.println("Emergency stop data sent to live_data_topic");
    } else {
        Serial.println("Failed to send emergency stop data to live_data_topic");
    }

    if (client.publish(dynamics_data_topic, buffer, false)) {
        Serial.println("Emergency stop data sent to dynamics_data_topic");
    } else {
        Serial.println("Failed to send emergency stop data to dynamics_data_topic");
    }
}

void loop() {
    maintain_wifi();
    ArduinoOTA.handle();

    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
        Serial.println("MQTT disconnected, attempting to reconnect...");
        reconnect();
    }
    client.loop();

    check_emergency_stop();
    check_start_switch();
    control_servos();

    static unsigned long last_live_data = 0;
    if (millis() - last_live_data > LIVE_DATA_INTERVAL) {
        last_live_data = millis();
        send_live_data();
    }

    static unsigned long last_dynamics_data = 0;
    if (state.active && millis() - last_dynamics_data > DYNAMICS_UPDATE_INTERVAL) {
        last_dynamics_data = millis();
        send_dynamics_data();
    }

    if (millis() - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
        Serial.println("Periodic status update triggered");
        update_connection_status("online");
    }

    static unsigned long last_blink = 0;
    if (millis() - last_blink > HEARTBEAT_INTERVAL) {
        last_blink = millis();
        digitalWrite(esp_led, !digitalRead(esp_led));
    }
}