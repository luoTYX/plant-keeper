/*
 智能植物养护系统 V5.0
 功能：传感器读取、智能水肥决策、拍照健康评估、远程配置、上传*/

 //=================================================禁用 Octal SPI PSRAM释放35.36.37引脚
#define CONFIG_SPIRAM_MODE_QUAD 1
#define CONFIG_SPIRAM_OCT_CS_IO -1
#define CONFIG_SPIRAM_OCT_IOPS 0
//==================================================
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <DHT.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// WiFi 配置（部署时修改为实际值）
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// 服务器地址（部署时修改为实际服务器IP）
const char* serverUrl = "http://YOUR_SERVER_IP:3000/api/upload";
const char* configUrl = "http://YOUR_SERVER_IP:3000/api/config";

//  NTP 时间服务器 
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;      // 东八区
const int daylightOffset_sec = 0;

//  传感器引脚 
#define DHTPIN 36                         // AM2301 数据引脚
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

#define LIGHT_SENSOR_PIN  3
#define SOIL_SENSOR_PIN   4

//  RS485 EC传感器 
#define RS485_DE_RE_PIN   38
#define EC_UART_NUM       2
#define EC_BAUD           9600
#define EC_SLAVE_ADDR     0x01

// 执行器PWM引脚 
#define PUMP1_PWM_PIN     40   // 浇水泵
#define PUMP2_PWM_PIN     37   // 肥料A泵
#define PUMP3_PWM_PIN     12   // 肥料B泵
#define LIGHT_PWM_PIN     39   // 补光灯
#define BUZZER_PIN        21   //蜂鸣器

//  PWM 配置 
#define PWM_FREQ          5000
#define PWM_RESOLUTION    8
// ESP32 3.x 版本不要通道信号

//  I2C OLED 配置
#define OLED_SDA_PIN      7
#define OLED_SCL_PIN      18
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);

//  摄像头引脚 配置
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     41
#define SIOC_GPIO_NUM     42
#define Y9_GPIO_NUM       15
#define Y8_GPIO_NUM       2
#define Y7_GPIO_NUM       8
#define Y6_GPIO_NUM       47
#define Y5_GPIO_NUM       48
#define Y4_GPIO_NUM       35
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       9
#define VSYNC_GPIO_NUM    11
#define HREF_GPIO_NUM     20
#define PCLK_GPIO_NUM     6

//  数据结构 
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

// 生长阶段枚举
enum GrowthStage {
    STAGE_SEEDLING = 0,
    STAGE_GROWTH = 1,
    STAGE_FLOWER = 2,
    STAGE_FRUIT = 3,
    STAGE_UNKNOWN = 4
};

const char* stageNames[] = {"幼苗期", "生长期", "开花期", "结果期", "未知"};

// 远程配置结构
typedef struct {
    int plantAgeDays;          // 植物日龄
    int soilTargetMin;         // 土壤湿度目标下限
    int soilTargetMax;         // 土壤湿度目标上限
    int ecGrowthMin;           // 生长期EC下限（其他阶段由此偏移）
    int ecGrowthMax;           // 生长期EC上限
    int wateringCooldown;      // 浇水冷却时间（分钟）
    int fertilizerCooldown;    // 施肥冷却时间（分钟）
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

// 时间同步标志
bool timeSynced = false;
struct tm timeinfo;

// 系统状态
bool wifiConnected = false;
bool cameraInitialized = false;
bool watering = false;
bool fertilizingA = false;
bool fertilizingB = false;

// 定时器
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 15 * 60 * 1000;      // 15分钟
unsigned long lastStageUpdate = 0;
const unsigned long stageUpdateInterval = 24 * 60 * 60 * 1000; // 24小时
unsigned long lastPhotoTime = 0;
const unsigned long photoInterval = 24 * 60 * 60 * 1000;  // 24小时
unsigned long lastLightUpdate = 0;
const unsigned long lightUpdateInterval = 60 * 1000;      // 1分钟
unsigned long lastConfigFetch = 0;
const unsigned long configFetchInterval = 60 * 60 * 1000; // 1小时

// 执行器计时
unsigned long wateringStartTime = 0;
unsigned long fertilizingAStartTime = 0;
unsigned long fertilizingBStartTime = 0;
unsigned long lastWateringTime = 0;
unsigned long lastFertilizingATime = 0;
unsigned long lastFertilizingBTime = 0;

// 历史数据用于趋势分析
float ecHistory[72] = {0};      // 72小时EC历史
float soilHistory[72] = {0};     // 72小时土壤湿度历史
int healthHistory[30] = {0};     // 30天健康历史
uint8_t ecIndex = 0;
uint8_t soilIndex = 0;
uint8_t healthIndex = 0;
float healthTrend = 0;

//  函数声明 
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
int calculatePlantHealth(camera_fb_t *fb);
bool takePhoto();
void uploadToServer(camera_fb_t *fb);
void displayOLED();
void beep(int durationMs, int times);
void logData(const char* event);
String getTimeString();

//  CRC16 Modbus 
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

//  读取EC传感器
uint16_t readEC() {
    uint8_t request[8] = {EC_SLAVE_ADDR, 0x03, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00};//rs485
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

//  从服务器获取配置
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
    // DHT21
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
        currentData.humidity = h;
        currentData.temperature = t;
    }

    // 模拟传感器（中值滤波方式）
    const int samples = 5;
    uint32_t lightSum = 0, soilSum = 0;
    for (int i = 0; i < samples; i++) {
        lightSum += analogRead(LIGHT_SENSOR_PIN);
        soilSum += analogRead(SOIL_SENSOR_PIN);
        delay(10);
    }
    currentData.light = lightSum / samples;
    currentData.soil = soilSum / samples;

    // EC传感器
    currentData.ec = readEC();
    if (currentData.ec == 0) {
        currentData.ec = 150; // 默认值（可更改）
    }

    // 更新时间
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

//  更新历史数据 
void updateHistory() {
    ecHistory[ecIndex] = currentData.ec;
    ecIndex = (ecIndex + 1) % 72;

    soilHistory[soilIndex] = currentData.soil;
    soilIndex = (soilIndex + 1) % 72;
}

//  计算趋势 
void calculateTrends() {
    // EC趋势（最近24小时）
    int ecValid = 0;
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < 24; i++) {
        int idx = (ecIndex - 1 - i + 72) % 72;
        if (ecHistory[idx] > 0) {
            float x = i;
            float y = ecHistory[idx];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
            ecValid++;
        }
    }
    if (ecValid > 1) {
        // 可保存ecTrend供后续使用，未使用
    }

    // 健康趋势（最近7天）
    int healthValid = 0;
    sumX = 0; sumY = 0; sumXY = 0; sumX2 = 0;
    for (int i = 0; i < 7; i++) {
        int idx = (healthIndex - 1 - i + 30) % 30;
        if (healthHistory[idx] > 0) {
            float x = i * 24; // 小时为单位
            float y = healthHistory[idx];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
            healthValid++;
        }
    }
    if (healthValid > 1) {
        healthTrend = (healthValid * sumXY - sumX * sumY) / (healthValid * sumX2 - sumX * sumX);
    } else {
        healthTrend = 0;
    }
}

//  自动识别生长阶段 
GrowthStage identifyGrowthStage() {
    float scores[4] = {0, 0, 0, 0};

    // 日龄得分
    if (sysConfig.plantAgeDays < 15) {
        scores[STAGE_SEEDLING] += 30;
        scores[STAGE_GROWTH] += 10;
    } else if (sysConfig.plantAgeDays < 30) {
        scores[STAGE_SEEDLING] += 10;
        scores[STAGE_GROWTH] += 30;
        scores[STAGE_FLOWER] += 5;
    } else if (sysConfig.plantAgeDays < 45) {
        scores[STAGE_GROWTH] += 20;
        scores[STAGE_FLOWER] += 30;
        scores[STAGE_FRUIT] += 10;
    } else {
        scores[STAGE_GROWTH] += 10;
        scores[STAGE_FLOWER] += 20;
        scores[STAGE_FRUIT] += 30;
    }

    // 健康度得分
    if (currentData.health > 0) {
        if (currentData.health < 40) {
            scores[STAGE_SEEDLING] += 20;
        } else if (currentData.health > 70) {
            scores[STAGE_FLOWER] += 20;
            scores[STAGE_FRUIT] += 15;
        } else {
            scores[STAGE_GROWTH] += 20;
        }
    }

    // EC值得分（根据当前阶段的目标范围，此处简化为基于ecGrowthMin/Max的比例）
    if (currentData.ec > 0) {
        if (currentData.ec < sysConfig.ecGrowthMin * 0.6) {
            scores[STAGE_SEEDLING] += 15;
        } else if (currentData.ec >= sysConfig.ecGrowthMin && currentData.ec <= sysConfig.ecGrowthMax) {
            scores[STAGE_GROWTH] += 20;
        } else if (currentData.ec > sysConfig.ecGrowthMax && currentData.ec <= sysConfig.ecGrowthMax * 1.3) {
            scores[STAGE_FLOWER] += 25;
        } else if (currentData.ec > sysConfig.ecGrowthMax * 0.8 && currentData.ec <= sysConfig.ecGrowthMax * 1.2) {
            scores[STAGE_FRUIT] += 20;
        }
    }

    // EC趋势得分
    calculateTrends();
    // 此处可添加趋势判断

    // 光照需求
    if (currentData.light > 2000) {
        scores[STAGE_FLOWER] += 10;
        scores[STAGE_FRUIT] += 5;
    } else {
        scores[STAGE_SEEDLING] += 5;
        scores[STAGE_GROWTH] += 5;
    }

    int maxScore = 0;
    GrowthStage stage = STAGE_GROWTH;
    for (int i = 0; i < 4; i++) {
        if (scores[i] > maxScore) {
            maxScore = scores[i];
            stage = (GrowthStage)i;
        }
    }
    return stage;
}

//  浇水控制 
void controlWatering() {
    // 检查是否正在浇水
    if (watering) {
        if (millis() - wateringStartTime >= 5000) { // 固定浇水5秒，可优化为根据需水量动态
            ledcWrite(PUMP1_PWM_PIN, 0);
            watering = false;
            lastWateringTime = millis();
            logData("浇水完成");
        }
        return;
    }

    // 冷却检查
    if (millis() - lastWateringTime < sysConfig.wateringCooldown * 60 * 1000) {
        return;
    }

    // 根据土壤湿度决策
    if (currentData.soil < sysConfig.soilTargetMin) {
        ledcWrite(PUMP1_PWM_PIN, 255);
        watering = true;
        wateringStartTime = millis();
        logData("开始浇水");
        beep(100, 2);
    }
}

//  施肥控制 
void controlFertilizing() {
    if (currentData.ec == 0) return;

    // EC过高紧急处理
    if (currentData.ec > 600) {
        if (fertilizingA || fertilizingB) {
            ledcWrite(PUMP2_PWM_PIN, 0);
            ledcWrite(PUMP3_PWM_PIN, 0);
            fertilizingA = fertilizingB = false;
            logData("EC过高，停止施肥");
            beep(200, 3);
        }
        // 大量浇水稀释
        if (!watering && millis() - lastWateringTime > 60000) {
            ledcWrite(PUMP1_PWM_PIN, 255);
            watering = true;
            wateringStartTime = millis();
            logData("EC过高，启动稀释浇水");
        }
        return;
    }

    // 获取当前生长阶段对应的EC目标范围
    int ecMin, ecMax;
    GrowthStage stage = identifyGrowthStage(); // 可缓存此值以节省计算
    switch (stage) {
        case STAGE_SEEDLING:
            ecMin = sysConfig.ecGrowthMin * 0.6;
            ecMax = sysConfig.ecGrowthMax * 0.6;
            break;
        case STAGE_GROWTH:
            ecMin = sysConfig.ecGrowthMin;
            ecMax = sysConfig.ecGrowthMax;
            break;
        case STAGE_FLOWER:
            ecMin = sysConfig.ecGrowthMin * 1.2;
            ecMax = sysConfig.ecGrowthMax * 1.2;
            break;
        case STAGE_FRUIT:
            ecMin = sysConfig.ecGrowthMin * 1.0;
            ecMax = sysConfig.ecGrowthMax * 1.0;
            break;
        default:
            ecMin = sysConfig.ecGrowthMin;
            ecMax = sysConfig.ecGrowthMax;
    }

    int ecDev = 0;
    if (currentData.ec < ecMin) {
        ecDev = ecMin - currentData.ec;
    } else if (currentData.ec > ecMax) {
        ecDev = ecMax - currentData.ec; // 负值表示过高
    }

    // 肥料泵控制（定量3秒）
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

    // 冷却检查
    bool canA = (millis() - lastFertilizingATime) > sysConfig.fertilizerCooldown * 60 * 1000;
    bool canB = (millis() - lastFertilizingBTime) > sysConfig.fertilizerCooldown * 60 * 1000;

    if (ecDev > 0) {
        // 需要施肥
        if (ecDev > 100) { // 严重缺肥，两种一起施
            if (canA && canB) {
                ledcWrite(PUMP2_PWM_PIN, 255);
                ledcWrite(PUMP3_PWM_PIN, 255);
                fertilizingA = fertilizingB = true;
                fertilizingAStartTime = fertilizingBStartTime = millis();
                logData("严重缺肥，同时施肥");
                beep(100, 3);
            }
        } else { // 轻度缺肥，根据阶段选择配方
            // 简单规则：幼苗期/生长期多用氮（泵A），开花/结果期多用磷钾（泵B）
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

//  更新补光灯 
void updateLightPWM() {
    if (!timeSynced) {
        ledcWrite(LIGHT_PWM_PIN, 128);
        return;
    }
    getLocalTime(&timeinfo);
    int hour = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;
    int totalMinutes = hour * 60 + minute;
    int brightness = 0;
    if (totalMinutes >= 300 && totalMinutes < 480) {
        brightness = map(totalMinutes, 300, 480, 0, 255);
    } else if (totalMinutes >= 480 && totalMinutes < 1020) {
        brightness = 255;
    } else if (totalMinutes >= 1020 && totalMinutes < 1200) {
        brightness = map(totalMinutes, 1020, 1200, 255, 0);
    }
    ledcWrite(LIGHT_PWM_PIN, brightness);
}

//  拍照和健康评估
int calculatePlantHealth(camera_fb_t *fb) {
    if (!fb) return 50;

    uint16_t *rgbBuf = (uint16_t *)ps_malloc(320 * 240 * 2);
    if (!rgbBuf) return 50;

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, (uint8_t*)rgbBuf);
    if (!converted) {
        free(rgbBuf);
        return 50;
    }

    int healthy = 0, stressed = 0, diseased = 0, dead = 0;
    int total = 320 * 240;

    for (int i = 0; i < total; i++) {
        uint16_t pixel = rgbBuf[i];
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;
        r = (r << 3) | (r >> 2);
        g = (g << 2) | (g >> 4);
        b = (b << 3) | (b >> 2);

        uint8_t minRGB = min(r, min(g, b));
        uint8_t maxRGB = max(r, max(g, b));
        uint8_t delta = maxRGB - minRGB;
        float hue = 0;
        float saturation = (maxRGB == 0) ? 0 : (float)delta / maxRGB * 100;
        float value = (float)maxRGB / 255 * 100;

        if (delta != 0) {
            if (maxRGB == r) {
                hue = 60 * ((float)(g - b) / delta);
            } else if (maxRGB == g) {
                hue = 60 * (2 + (float)(b - r) / delta);
            } else {
                hue = 60 * (4 + (float)(r - g) / delta);
            }
            if (hue < 0) hue += 360;
        }

        if (hue >= 80 && hue <= 140 && saturation > 30 && value > 30) {
            if (saturation > 60 && value > 50 && g > 150) healthy += 3;
            else healthy += 2;
        } else if (hue >= 50 && hue < 80 && saturation > 20 && value > 20) {
            stressed += 2;
        } else if ((hue < 50 || hue > 300) && value > 20) {
            diseased += 2;
        } else if (value < 30 || saturation < 20) {
            dead += 3;
        }
    }

    free(rgbBuf);

    int totalScore = healthy + stressed + diseased + dead;
    if (totalScore == 0) return 50;

    float score = (healthy * 1.0 + stressed * 0.5 + diseased * 0.2) / totalScore * 100;
    score = constrain(score, 0, 100);
    return (int)score;
}

bool takePhoto() {
    if (!cameraInitialized) return false;

    Serial.println("准备拍照...");
    ledcWrite(LIGHT_PWM_PIN, 255);
    delay(500);

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 10);

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        updateLightPWM();
        return false;
    }

    int health = calculatePlantHealth(fb);
    currentData.health = health;

    // 更新健康历史
    healthHistory[healthIndex] = health;
    healthIndex = (healthIndex + 1) % 30;

    if (wifiConnected) {
        uploadToServer(fb);
    }

    esp_camera_fb_return(fb);
    updateLightPWM();
    logData("拍照完成");
    beep(200, 1);
    return true;
}

// 上传数据到服务器 
void uploadToServer(camera_fb_t *fb) {
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
    http.begin(serverUrl);
    http.setTimeout(30000);

    String boundary = "----WebKitFormBoundary" + String(millis());
    String contentType = "multipart/form-data; boundary=" + boundary;
    http.addHeader("Content-Type", contentType);

    String bodyStart = "--" + boundary + "\r\n";
    bodyStart += "Content-Disposition: form-data; name=\"image\"; filename=\"plant.jpg\"\r\n";
    bodyStart += "Content-Type: image/jpeg\r\n\r\n";

    String bodyMid = "\r\n--" + boundary + "\r\n";
    bodyMid += "Content-Disposition: form-data; name=\"data\"\r\n\r\n";

    GrowthStage stage = identifyGrowthStage();
    String sensorJson = "{";
    sensorJson += "\"temp\":" + String(currentData.temperature) + ",";
    sensorJson += "\"humidity\":" + String(currentData.humidity) + ",";
    sensorJson += "\"light\":" + String(currentData.light) + ",";
    sensorJson += "\"soil\":" + String(currentData.soil) + ",";
    sensorJson += "\"ec\":" + String(currentData.ec) + ",";
    sensorJson += "\"health\":" + String(currentData.health) + ",";
    sensorJson += "\"stage\":" + String(stage) + ",";
    sensorJson += "\"stage_name\":\"" + String(stageNames[stage]) + "\",";
    sensorJson += "\"trend\":" + String(healthTrend) + ",";
    sensorJson += "\"time\":\"" + getTimeString() + "\"";
    sensorJson += "}";

    String bodyEnd = "\r\n--" + boundary + "--\r\n";

    size_t totalLen = bodyStart.length() + fb->len + bodyMid.length() + sensorJson.length() + bodyEnd.length();
    http.addHeader("Content-Length", String(totalLen));

    WiFiClient *client = http.getStreamPtr();
    client->print(bodyStart);

    size_t fbLen = fb->len;
    uint8_t *fbBuf = fb->buf;
    size_t chunkSize = 1024;
    for (size_t i = 0; i < fbLen; i += chunkSize) {
        size_t len = min(chunkSize, fbLen - i);
        client->write(fbBuf + i, len);
        delay(5);
    }

    client->print(bodyMid);
    client->print(sensorJson);
    client->print(bodyEnd);

    int httpCode = http.sendRequest("POST");
    if (httpCode > 0) {
        Serial.printf("上传成功: %d\n", httpCode);
    } else {
        Serial.printf("上传失败: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
}

//  OLED显示 
void displayOLED() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    if (timeSynced) {
        u8g2.setCursor(0, 10);
        u8g2.print(&timeinfo, "%H:%M");
    }
    u8g2.setCursor(45, 10);
    u8g2.print(stageNames[identifyGrowthStage()]);

    u8g2.setCursor(0, 25);
    u8g2.print("H:");
    u8g2.print(currentData.health);
    u8g2.print("%");
    if (healthTrend > 0.5) {
        u8g2.setCursor(50, 25);
        u8g2.print("↑");
    } else if (healthTrend < -0.5) {
        u8g2.setCursor(50, 25);
        u8g2.print("↓");
    }

    u8g2.setCursor(0, 40);
    u8g2.print("T:");
    u8g2.print(currentData.temperature, 1);
    u8g2.setCursor(70, 40);
    u8g2.print("H:");
    u8g2.print(currentData.humidity, 1);

    u8g2.setCursor(0, 55);
    u8g2.print("EC:");
    u8g2.print(currentData.ec);
    if (watering) u8g2.setCursor(70, 55); u8g2.print("W");
    if (fertilizingA) u8g2.print("A");
    if (fertilizingB) u8g2.print("B");

    u8g2.sendBuffer();
}

//  蜂鸣器 
void beep(int durationMs, int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < times - 1) delay(100);
    }
}

// 日志
void logData(const char* event) {
    Serial.printf("[%lu] %s | T=%.1f H=%.1f L=%d S=%d EC=%d Health=%d\n",
                  millis() / 1000, event,
                  currentData.temperature, currentData.humidity,
                  currentData.light, currentData.soil,
                  currentData.ec, currentData.health);
}

//  时间字符串 
String getTimeString() {
    if (!timeSynced) return "1970-01-01 00:00:00";
    getLocalTime(&timeinfo);
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buf);
}

//  WiFi连接 
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
        beep(100, 2);
    } else {
        wifiConnected = false;
        Serial.println("WiFi连接失败");
    }
}

//  NTP同步 
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

// 硬件初始化
void initHardware() {
    Serial.begin(115200);
    Serial.println("\n=== 智能动植物养护系统 V5.0  ===");
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    dht.begin();
    analogReadResolution(12);

    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);
    Serial2.begin(EC_BAUD, SERIAL_8N1, 16, 17);

    // 新版 ESP32 3.x PWM 初始化 - 直接使用引脚
    if (!ledcAttach(LIGHT_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("补光灯PWM初始化失败！");
    }
    if (!ledcAttach(PUMP1_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("水泵1PWM初始化失败！");
    }
    if (!ledcAttach(PUMP2_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("水泵2PWM初始化失败！");
    }
    if (!ledcAttach(PUMP3_PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("水泵3PWM初始化失败！");
    }

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    u8g2.begin();

    // 摄像头初始化
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
        cameraInitialized = true;
        Serial.println("摄像头初始化成功");
    } else {
        Serial.printf("摄像头初始化失败: 0x%x\n", err);
    }

    beep(200, 1);
    Serial.println("硬件初始化完成");
}

// == 主函数 ==
void setup() {
    initHardware();
    setupWiFi();
    if (wifiConnected) {
        syncNTP();
        fetchConfig(); // 获取远程配置
    }

    readSensors();

    lastSensorRead = millis();
    lastStageUpdate = millis();
    lastPhotoTime = millis();
    lastLightUpdate = millis();
    lastConfigFetch = millis();

    beep(300, 3);

    // ========== 调试：上电执行器自检 ==========
    delay(2000);
    Serial.println("===== 执行器测试 =====");
    ledcWrite(PUMP1_PWM_PIN, 255); delay(3000); ledcWrite(PUMP1_PWM_PIN, 0);
    ledcWrite(PUMP2_PWM_PIN, 255); delay(3000); ledcWrite(PUMP2_PWM_PIN, 0);
    ledcWrite(PUMP3_PWM_PIN, 255); delay(3000); ledcWrite(PUMP3_PWM_PIN, 0);
    ledcWrite(LIGHT_PWM_PIN, 255); delay(3000); ledcWrite(LIGHT_PWM_PIN, 0);
    digitalWrite(BUZZER_PIN, HIGH); delay(1000); digitalWrite(BUZZER_PIN, LOW);
    Serial.println("===== 测试完成 =====");
    // 实际部署时可删除以上测试代码
}

void loop() {
    unsigned long now = millis();

    // 每15分钟读取传感器并执行控制
    if (now - lastSensorRead >= sensorInterval) {
        readSensors();
        controlWatering();
        controlFertilizing();
        lastSensorRead = now;
    }

    // 每小时获取一次配置
    if (now - lastConfigFetch >= configFetchInterval) {
        if (wifiConnected) {
            fetchConfig();
        }
        lastConfigFetch = now;
    }

    // 每天更新生长阶段可省略定时
    if (now - lastStageUpdate >= stageUpdateInterval) {
        // 仅用于记录，不需要额外操作
        lastStageUpdate = now;
    }

    // 每天拍照（正式部署时使用此条件）
    // if (now - lastPhotoTime >= photoInterval) {
    //     if (cameraInitialized) takePhoto();
    //     lastPhotoTime = now;
    // }

    // ========== 调试：10秒拍照测试（正式部署时改回上面的24小时间隔） ==========
    if (now - lastPhotoTime >= 10000) {
        if (cameraInitialized) {
            takePhoto();
        }
        lastPhotoTime = now;
    }

    // 每分钟更新补光灯
    if (now - lastLightUpdate >= lightUpdateInterval) {
        updateLightPWM();
        lastLightUpdate = now;
    }

    // 更新显示（2秒一次）
    static unsigned long lastDisplay = 0;
    if (now - lastDisplay >= 2000) {
        displayOLED();
        lastDisplay = now;
    }

    delay(10);
}
// 作者：李政辉 蒋子宇
// 开发周期：2026.3.11 - 2026.3.21