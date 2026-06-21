/*
基础功能版
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <DHT.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// WiFi 配置（部署时修改为实际值）
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// 服务器地址（部署时修改为实际服务器IP）
const char* serverUrl = "http://YOUR_SERVER_IP:3000/api/upload";
const char* configUrl = "http://YOUR_SERVER_IP:3000/api/config";

// NTP 时间服务器
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;      // 东八区
const int daylightOffset_sec = 0;


#define DHTPIN            1   // DHT
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

#define LIGHT_SENSOR_PIN  3
#define SOIL_SENSOR_PIN   4

// RS485 EC传感器（引脚38一般不冲突，保留）
#define RS485_DE_RE_PIN   38
#define EC_UART_NUM       2
#define EC_BAUD           9600
#define EC_SLAVE_ADDR     0x01

// 执行器PWM引脚
#define PUMP1_PWM_PIN     40   // 浇水泵
#define PUMP2_PWM_PIN     2    // 肥料A泵 
#define PUMP3_PWM_PIN     12   // 肥料B泵
#define LIGHT_PWM_PIN     39   // 补光灯
#define BUZZER_PIN        21   // 蜂鸣器

// PWM 配置
#define PWM_FREQ          5000
#define PWM_RESOLUTION    8

// I2C OLED 配置
#define OLED_SDA_PIN      7
#define OLED_SCL_PIN      18
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);


// 数据结构
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} DateTime;

typedef struct {
    float temperature;
    float humidity;
    uint16_t light;
    uint16_t soil;
    uint16_t ec;
    int health;
    DateTime time;
} SensorData;

enum GrowthStage {
    STAGE_SEEDLING = 0,
    STAGE_GROWTH = 1,
    STAGE_FLOWER = 2,
    STAGE_FRUIT = 3,
    STAGE_UNKNOWN = 4
};
const char* stageNames[] = {"幼苗期", "生长期", "开花期", "结果期", "未知"};

typedef struct {
    int plantAgeDays;
    int soilTargetMin;
    int soilTargetMax;
    int ecGrowthMin;
    int ecGrowthMax;
    int wateringCooldown;
    int fertilizerCooldown;
} SystemConfig;

// 全局变量
SensorData currentData;
SystemConfig sysConfig = {
    .plantAgeDays = 30,
    .soilTargetMin = 1800,
    .soilTargetMax = 2500,
    .ecGrowthMin = 150,
    .ecGrowthMax = 300,
    .wateringCooldown = 5,
    .fertilizerCooldown = 60
};

bool timeSynced = false;
struct tm timeinfo;
bool wifiConnected = false;
bool watering = false;
bool fertilizingA = false;
bool fertilizingB = false;

// 定时器
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 15 * 60 * 1000;
unsigned long lastLightUpdate = 0;
const unsigned long lightUpdateInterval = 60 * 1000;
unsigned long lastConfigFetch = 0;
const unsigned long configFetchInterval = 60 * 60 * 1000;
unsigned long lastSensorUpload = 0;
const unsigned long sensorUploadInterval = 60 * 60 * 1000;

// 执行器计时
unsigned long wateringStartTime = 0;
unsigned long fertilizingAStartTime = 0;
unsigned long fertilizingBStartTime = 0;
unsigned long lastWateringTime = 0;
unsigned long lastFertilizingATime = 0;
unsigned long lastFertilizingBTime = 0;

// 历史数据
float ecHistory[72] = {0};
float soilHistory[72] = {0};
uint8_t ecIndex = 0;
uint8_t soilIndex = 0;
float healthTrend = 0;

// 函数声明
void setupWiFi();
void syncNTP();
void initHardware();
void readSensors();
uint16_t readEC();
uint16_t crc16(uint8_t *data, size_t len);
bool fetchConfig();
void updateHistory();
void calculateTrends();
GrowthStage identifyGrowthStage();
void controlWatering();
void controlFertilizing();
void updateLightPWM();
void uploadSensorData();
void displayOLED();
void beep(int durationMs, int times);
void logData(const char* event);
String getTimeString();

// CRC16 Modbus
uint16_t crc16(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 读取EC传感器
uint16_t readEC() {
    uint8_t request[8] = {EC_SLAVE_ADDR, 0x03, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = crc16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delay(10);
    Serial2.write(request, 8);
    Serial2.flush();
    delay(10);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    uint8_t response[7] = {0};
    int idx = 0;
    unsigned long timeout = millis() + 200;
    while (millis() < timeout && idx < 7) {
        if (Serial2.available()) {
            response[idx++] = Serial2.read();
        }
    }

    if (idx == 7 && response[0] == EC_SLAVE_ADDR && response[1] == 0x03) {
        crc = crc16(response, 5);
        if ((crc & 0xFF) == response[5] && ((crc >> 8) & 0xFF) == response[6]) {
            return (response[3] << 8) | response[4];
        }
    }
    return 0;
}

// 获取远程配置
bool fetchConfig() {
    if (WiFi.status() != WL_CONNECTED) return false;
    HTTPClient http;
    http.begin(configUrl);
    http.setTimeout(5000);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println("收到配置: " + payload);
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.print("JSON解析失败: ");
            Serial.println(error.c_str());
            http.end();
            return false;
        }
        sysConfig.plantAgeDays       = doc["plant_age_days"] | sysConfig.plantAgeDays;
        sysConfig.soilTargetMin      = doc["soil_target_min"] | sysConfig.soilTargetMin;
        sysConfig.soilTargetMax      = doc["soil_target_max"] | sysConfig.soilTargetMax;
        sysConfig.ecGrowthMin        = doc["ec_growth_min"] | sysConfig.ecGrowthMin;
        sysConfig.ecGrowthMax        = doc["ec_growth_max"] | sysConfig.ecGrowthMax;
        sysConfig.wateringCooldown   = doc["watering_cooldown"] | sysConfig.wateringCooldown;
        sysConfig.fertilizerCooldown = doc["fertilizer_cooldown"] | sysConfig.fertilizerCooldown;
        Serial.println("配置更新成功");
        http.end();
        return true;
    } else {
        Serial.printf("获取配置失败，HTTP错误: %d\n", httpCode);
        http.end();
        return false;
    }
}

// 读取所有传感器
void readSensors() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
        currentData.humidity = h;
        currentData.temperature = t;
    }
    const int samples = 5;
    uint32_t lightSum = 0, soilSum = 0;
    for (int i = 0; i < samples; i++) {
        lightSum += analogRead(LIGHT_SENSOR_PIN);
        soilSum += analogRead(SOIL_SENSOR_PIN);
        delay(10);
    }
    currentData.light = lightSum / samples;
    currentData.soil = soilSum / samples;
    currentData.ec = readEC();
    if (currentData.ec == 0) currentData.ec = 150;
    currentData.health = 0; // 无摄像头
    if (timeSynced) {
        getLocalTime(&timeinfo);
        currentData.time.year = timeinfo.tm_year + 1900;
        currentData.time.month = timeinfo.tm_mon + 1;
        currentData.time.day = timeinfo.tm_mday;
        currentData.time.hour = timeinfo.tm_hour;
        currentData.time.minute = timeinfo.tm_min;
        currentData.time.second = timeinfo.tm_sec;
    }
    updateHistory();
    logData("传感器读取完成");
}

void updateHistory() {
    ecHistory[ecIndex] = currentData.ec;
    ecIndex = (ecIndex + 1) % 72;
    soilHistory[soilIndex] = currentData.soil;
    soilIndex = (soilIndex + 1) % 72;
}

void calculateTrends() {
    // EC趋势计算（可省略）
    healthTrend = 0;
}

GrowthStage identifyGrowthStage() {
    float scores[4] = {0,0,0,0};
    if (sysConfig.plantAgeDays < 15) {
        scores[STAGE_SEEDLING] += 30; scores[STAGE_GROWTH] += 10;
    } else if (sysConfig.plantAgeDays < 30) {
        scores[STAGE_SEEDLING] += 10; scores[STAGE_GROWTH] += 30; scores[STAGE_FLOWER] += 5;
    } else if (sysConfig.plantAgeDays < 45) {
        scores[STAGE_GROWTH] += 20; scores[STAGE_FLOWER] += 30; scores[STAGE_FRUIT] += 10;
    } else {
        scores[STAGE_GROWTH] += 10; scores[STAGE_FLOWER] += 20; scores[STAGE_FRUIT] += 30;
    }
    if (currentData.ec > 0) {
        if (currentData.ec < sysConfig.ecGrowthMin * 0.6) scores[STAGE_SEEDLING] += 15;
        else if (currentData.ec >= sysConfig.ecGrowthMin && currentData.ec <= sysConfig.ecGrowthMax) scores[STAGE_GROWTH] += 20;
        else if (currentData.ec > sysConfig.ecGrowthMax && currentData.ec <= sysConfig.ecGrowthMax * 1.3) scores[STAGE_FLOWER] += 25;
        else if (currentData.ec > sysConfig.ecGrowthMax * 0.8 && currentData.ec <= sysConfig.ecGrowthMax * 1.2) scores[STAGE_FRUIT] += 20;
    }
    if (currentData.light > 2000) {
        scores[STAGE_FLOWER] += 10; scores[STAGE_FRUIT] += 5;
    } else {
        scores[STAGE_SEEDLING] += 5; scores[STAGE_GROWTH] += 5;
    }
    int maxScore = 0; GrowthStage stage = STAGE_GROWTH;
    for (int i=0; i<4; i++) { if (scores[i] > maxScore) { maxScore = scores[i]; stage = (GrowthStage)i; } }
    return stage;
}

void controlWatering() {
    if (watering) {
        if (millis() - wateringStartTime >= 5000) {
            ledcWrite(PUMP1_PWM_PIN, 0);
            watering = false;
            lastWateringTime = millis();
            logData("浇水完成");
        }
        return;
    }
    if (millis() - lastWateringTime < sysConfig.wateringCooldown * 60 * 1000) return;
    if (currentData.soil < sysConfig.soilTargetMin) {
        ledcWrite(PUMP1_PWM_PIN, 255);
        watering = true;
        wateringStartTime = millis();
        logData("开始浇水");
        beep(100, 2);
    }
}

void controlFertilizing() {
    if (currentData.ec == 0) return;
    if (currentData.ec > 600) {
        if (fertilizingA || fertilizingB) {
            ledcWrite(PUMP2_PWM_PIN, 0);
            ledcWrite(PUMP3_PWM_PIN, 0);
            fertilizingA = fertilizingB = false;
            logData("EC过高，停止施肥");
            beep(200, 3);
        }
        if (!watering && millis() - lastWateringTime > 60000) {
            ledcWrite(PUMP1_PWM_PIN, 255);
            watering = true;
            wateringStartTime = millis();
            logData("EC过高，启动稀释浇水");
        }
        return;
    }
    int ecMin, ecMax;
    GrowthStage stage = identifyGrowthStage();
    switch (stage) {
        case STAGE_SEEDLING: ecMin = sysConfig.ecGrowthMin * 0.6; ecMax = sysConfig.ecGrowthMax * 0.6; break;
        case STAGE_GROWTH:   ecMin = sysConfig.ecGrowthMin;     ecMax = sysConfig.ecGrowthMax;     break;
        case STAGE_FLOWER:   ecMin = sysConfig.ecGrowthMin * 1.2; ecMax = sysConfig.ecGrowthMax * 1.2; break;
        case STAGE_FRUIT:    ecMin = sysConfig.ecGrowthMin * 1.0; ecMax = sysConfig.ecGrowthMax * 1.0; break;
        default:             ecMin = sysConfig.ecGrowthMin;     ecMax = sysConfig.ecGrowthMax;
    }
    int ecDev = 0;
    if (currentData.ec < ecMin) ecDev = ecMin - currentData.ec;
    else if (currentData.ec > ecMax) ecDev = ecMax - currentData.ec;
    const uint32_t FERT_DURATION = 3000;
    if (fertilizingA) {
        if (millis() - fertilizingAStartTime >= FERT_DURATION) {
            ledcWrite(PUMP2_PWM_PIN, 0);
            fertilizingA = false;
            lastFertilizingATime = millis();
            logData("肥料A施肥完成");
        }
        return;
    }
    if (fertilizingB) {
        if (millis() - fertilizingBStartTime >= FERT_DURATION) {
            ledcWrite(PUMP3_PWM_PIN, 0);
            fertilizingB = false;
            lastFertilizingBTime = millis();
            logData("肥料B施肥完成");
        }
        return;
    }
    bool canA = (millis() - lastFertilizingATime) > sysConfig.fertilizerCooldown * 60 * 1000;
    bool canB = (millis() - lastFertilizingBTime) > sysConfig.fertilizerCooldown * 60 * 1000;
    if (ecDev > 0) {
        if (ecDev > 100) {
            if (canA && canB) {
                ledcWrite(PUMP2_PWM_PIN, 255);
                ledcWrite(PUMP3_PWM_PIN, 255);
                fertilizingA = fertilizingB = true;
                fertilizingAStartTime = fertilizingBStartTime = millis();
                logData("严重缺肥，同时施肥");
                beep(100, 3);
            }
        } else {
            bool useA = (stage == STAGE_SEEDLING || stage == STAGE_GROWTH);
            if (useA && canA) {
                ledcWrite(PUMP2_PWM_PIN, 255);
                fertilizingA = true;
                fertilizingAStartTime = millis();
                logData("施用肥料A");
                beep(100, 2);
            } else if (!useA && canB) {
                ledcWrite(PUMP3_PWM_PIN, 255);
                fertilizingB = true;
                fertilizingBStartTime = millis();
                logData("施用肥料B");
                beep(100, 2);
            }
        }
    }
}

void updateLightPWM() {
    if (!timeSynced) { ledcWrite(LIGHT_PWM_PIN, 128); return; }
    getLocalTime(&timeinfo);
    int totalMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    int brightness = 0;
    if (totalMinutes >= 300 && totalMinutes < 480) brightness = map(totalMinutes, 300, 480, 0, 255);
    else if (totalMinutes >= 480 && totalMinutes < 1020) brightness = 255;
    else if (totalMinutes >= 1020 && totalMinutes < 1200) brightness = map(totalMinutes, 1020, 1200, 255, 0);
    ledcWrite(LIGHT_PWM_PIN, brightness);
}

void uploadSensorData() {
    if (WiFi.status() != WL_CONNECTED) return;
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    GrowthStage stage = identifyGrowthStage();
    String jsonPayload = "{";
    jsonPayload += "\"temp\":" + String(currentData.temperature) + ",";
    jsonPayload += "\"humidity\":" + String(currentData.humidity) + ",";
    jsonPayload += "\"light\":" + String(currentData.light) + ",";
    jsonPayload += "\"soil\":" + String(currentData.soil) + ",";
    jsonPayload += "\"ec\":" + String(currentData.ec) + ",";
    jsonPayload += "\"health\":0,";
    jsonPayload += "\"stage\":" + String(stage) + ",";
    jsonPayload += "\"stage_name\":\"" + String(stageNames[stage]) + "\",";
    jsonPayload += "\"trend\":" + String(healthTrend) + ",";
    jsonPayload += "\"time\":\"" + getTimeString() + "\"";
    jsonPayload += "}";
    int httpCode = http.POST(jsonPayload);
    if (httpCode > 0) {
        Serial.printf("传感器数据上传成功，HTTP状态码: %d\n", httpCode);
        String response = http.getString();
        Serial.println("服务器响应: " + response);
    } else {
        Serial.printf("传感器数据上传失败，错误: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
}

void displayOLED() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (timeSynced) { u8g2.setCursor(0,10); u8g2.print(&timeinfo,"%H:%M"); }
    u8g2.setCursor(45,10); u8g2.print(stageNames[identifyGrowthStage()]);
    u8g2.setCursor(0,25); u8g2.print("H:0%");
    u8g2.setCursor(0,40); u8g2.print("T:"); u8g2.print(currentData.temperature,1);
    u8g2.setCursor(70,40); u8g2.print("H:"); u8g2.print(currentData.humidity,1);
    u8g2.setCursor(0,55); u8g2.print("EC:"); u8g2.print(currentData.ec);
    if (watering) { u8g2.setCursor(70,55); u8g2.print("W"); }
    if (fertilizingA) u8g2.print("A");
    if (fertilizingB) u8g2.print("B");
    u8g2.sendBuffer();
}

void beep(int durationMs, int times) {
    for (int i=0; i<times; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i<times-1) delay(100);
    }
}

void logData(const char* event) {
    Serial.printf("[%lu] %s | T=%.1f H=%.1f L=%d S=%d EC=%d\n",
                  millis()/1000, event,
                  currentData.temperature, currentData.humidity,
                  currentData.light, currentData.soil,
                  currentData.ec);
}

String getTimeString() {
    if (!timeSynced) return "1970-01-01 00:00:00";
    getLocalTime(&timeinfo);
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buf);
}

void setupWiFi() {
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
        delay(500);
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("WiFi连接成功");
        beep(100,2);
    } else {
        wifiConnected = false;
        Serial.println("WiFi连接失败");
    }
}

void syncNTP() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    int retry = 0;
    while (!getLocalTime(&timeinfo) && retry < 30) {
        delay(500);
        retry++;
    }
    if (retry < 30) {
        timeSynced = true;
        Serial.println("时间同步成功");
    } else {
        timeSynced = false;
    }
}

void initHardware() {
    Serial.begin(115200);
    Serial.println("\n=== 智能植物养护系统 V5.0 ===");
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    dht.begin();
    analogReadResolution(12);
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);
    Serial2.begin(EC_BAUD, SERIAL_8N1, 16, 17);
    if (!ledcAttach(LIGHT_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) Serial.println("补光灯PWM初始化失败！");
    if (!ledcAttach(PUMP1_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) Serial.println("水泵1PWM初始化失败！");
    if (!ledcAttach(PUMP2_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) Serial.println("水泵2PWM初始化失败！");
    if (!ledcAttach(PUMP3_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) Serial.println("水泵3PWM初始化失败！");
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    u8g2.begin();
    beep(200,1);
    Serial.println("硬件初始化完成");
}

void setup() {
    initHardware();
    setupWiFi();
    if (wifiConnected) {
        syncNTP();
        fetchConfig();
    }
    readSensors();
    lastSensorRead = millis();
    lastLightUpdate = millis();
    lastConfigFetch = millis();
    lastSensorUpload = millis();
    beep(300,3);
}

void loop() {
    unsigned long now = millis();
    if (now - lastSensorRead >= sensorInterval) {
        readSensors();
        controlWatering();
        controlFertilizing();
        lastSensorRead = now;
    }
    if (now - lastConfigFetch >= configFetchInterval) {
        if (wifiConnected) fetchConfig();
        lastConfigFetch = now;
    }
    if (now - lastSensorUpload >= sensorUploadInterval) {
        if (wifiConnected) uploadSensorData();
        lastSensorUpload = now;
    }
    if (now - lastLightUpdate >= lightUpdateInterval) {
        updateLightPWM();
        lastLightUpdate = now;
    }
    static unsigned long lastDisplay = 0;
    if (now - lastDisplay >= 2000) {
        displayOLED();
        lastDisplay = now;
    }
    delay(10);
}