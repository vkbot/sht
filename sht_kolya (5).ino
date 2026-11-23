#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <stddef.h>
#include "Config.h"

// Config.h may define SENSOR_RETRY_MS. Provide a default only when absent.
#ifndef SENSOR_RETRY_MS
#define SENSOR_RETRY_MS 30000UL
#endif

// ============================================================
// EEPROM SETTINGS AND INTERNAL DATA STRUCTURES
// ============================================================


struct StoredSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t intervalMin;
  uint16_t reserved1; // Reserved EEPROM field; preserves layout of earlier versions.
  uint16_t reserved2; // Reserved EEPROM field; preserves layout of earlier versions.
  uint32_t lastProcessedMessageId;
  uint32_t crc;
};

StoredSettings settings;

// Owner identity and a separate VK history cursor are stored after the
// existing settings block, preserving the current settings layout.
static const uint32_t OWNER_STATE_MAGIC = 0x4F574E52UL; // "OWNR"
struct OwnerState {
  uint32_t magic;
  uint32_t ownerId;
  uint32_t lastProcessedMessageId;
  uint32_t cursorInitialized;
  uint32_t crc;
};
OwnerState ownerState;

uint32_t crc32Bytes(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }
  return ~crc;
}

bool settingsValid(const StoredSettings& s) {
  if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) return false;
  if (s.intervalMin < MIN_INTERVAL_MIN || s.intervalMin > MAX_INTERVAL_MIN) return false;
  uint32_t expected = crc32Bytes((const uint8_t*)&s, offsetof(StoredSettings, crc));
  return expected == s.crc;
}

void saveSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;
  settings.crc = crc32Bytes((const uint8_t*)&settings, offsetof(StoredSettings, crc));
  EEPROM.put(0, settings);
  bool ok = EEPROM.commit();
  Serial.println(ok ? F("EEPROM: settings saved") : F("EEPROM: SAVE FAILED"));
}

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE_BYTES);
  EEPROM.get(0, settings);

  if (!settingsValid(settings)) {
    memset(&settings, 0, sizeof(settings));
    settings.magic = SETTINGS_MAGIC;
    settings.version = SETTINGS_VERSION;
    settings.intervalMin = DEFAULT_INTERVAL_MIN;
    settings.reserved1 = 0;
    settings.reserved2 = 0;
    settings.lastProcessedMessageId = 0;
    saveSettings();
    Serial.println(F("EEPROM: defaults loaded"));
  } else {
    Serial.println(F("EEPROM: saved settings loaded"));
  }
}

bool saveOwnerState() {
  ownerState.magic = OWNER_STATE_MAGIC;
  ownerState.crc = crc32Bytes((const uint8_t*)&ownerState, offsetof(OwnerState, crc));
  EEPROM.put(sizeof(StoredSettings), ownerState);
  bool ok = EEPROM.commit();
  Serial.println(ok ? F("EEPROM: owner state saved") : F("EEPROM: OWNER STATE SAVE FAILED"));
  return ok;
}

void loadOwnerState() {
  EEPROM.get(sizeof(StoredSettings), ownerState);
  uint32_t expected = crc32Bytes((const uint8_t*)&ownerState, offsetof(OwnerState, crc));
  bool valid = ownerState.magic == OWNER_STATE_MAGIC &&
               ownerState.ownerId <= 2147483647UL &&
               expected == ownerState.crc;
  if (!valid) {
    memset(&ownerState, 0, sizeof(ownerState));
    ownerState.magic = OWNER_STATE_MAGIC;
    // No public owner is assigned by default: the fixed administrator remains
    // limited to hidden commands until /owner <user_id> is issued.
    ownerState.ownerId = 0;
    ownerState.lastProcessedMessageId = 0;
    ownerState.cursorInitialized = 0;
    saveOwnerState();
    Serial.println(F("EEPROM: no public owner assigned"));
  } else {
    Serial.print(F("EEPROM: owner ID "));
    Serial.println(ownerState.ownerId);
  }
}

// ============================================================
// RUNTIME STATE
// ============================================================

uint32_t lastWiFiAttempt = 0;
uint32_t lastVKPoll = 0;
uint32_t lastVKAttempt = 0;
uint32_t nextSendAt = 0;
bool scheduleStarted = false;
bool wifiWasConnected = false;
bool haveReading = false;
bool vkAttempted = false;
bool lastVKFailureWasReply = false;
bool waitingForIntervalInput = false;
bool firmwareUpdateRunning = false;

float lastTemperature = 0.0f;
float lastHumidity = 0.0f;
// Admin and owner replies are separated so a hidden admin command can never
// accidentally receive the owner's public reply (or vice versa).
String pendingVKAdminReply;
String pendingVKOwnerReply;
String pendingVKReport;
uint32_t activeCommandPeerId = 0;

// Network-heavy hidden commands are deferred until messages.getHistory has
// returned and its response String / ArduinoJson document have been released.
uint32_t pendingPersonalReportPeerId = 0;
// Owner greeting is deferred until VK history parsing releases its buffers.
uint32_t pendingOwnerWelcomePeerId = 0;
uint32_t pendingTestTargetPeerId = 0;
uint32_t pendingTestCommandPeerId = 0;
uint32_t pendingFirmwareUpdatePeerId = 0;
String pendingFirmwareUpdateUrl;
// Verify successful send responses against the next messages.getHistory result.
uint32_t pendingVKVerifyPeerId = 0;
uint32_t pendingVKVerifyMessageId = 0;

bool timeReached(uint32_t target) {
  return (int32_t)(millis() - target) >= 0;
}

// ============================================================
// SHT40 / SHT41 LOW-LEVEL DRIVER (no external sensor library)
// ============================================================

uint8_t shtCRC(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
  }
  return crc;
}

bool readSHT(float& temperature, float& humidity) {
  Wire.beginTransmission(SHT_ADDRESS);
  Wire.write(0xFD); // High precision measurement, no heater
  if (Wire.endTransmission() != 0) return false;

  delay(SHT_MEASUREMENT_DELAY_MS);
  uint8_t received = Wire.requestFrom((uint8_t)SHT_ADDRESS, (uint8_t)6);
  if (received != 6) {
    while (Wire.available()) Wire.read();
    return false;
  }

  uint8_t data[6];
  for (uint8_t i = 0; i < 6; i++) data[i] = Wire.read();

  if (shtCRC(data, 2) != data[2]) {
    Serial.println(F("SHT: temperature CRC error"));
    return false;
  }
  if (shtCRC(data + 3, 2) != data[5]) {
    Serial.println(F("SHT: humidity CRC error"));
    return false;
  }

  uint16_t rawTemp = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawHum  = ((uint16_t)data[3] << 8) | data[4];

  temperature = -45.0f + 175.0f * ((float)rawTemp / 65535.0f);
  humidity = -6.0f + 125.0f * ((float)rawHum / 65535.0f);
  humidity = constrain(humidity, 0.0f, 100.0f);
  return true;
}

// ============================================================
// URL ENCODING FOR VK FORM PARAMETERS (UTF-8 safe)
// ============================================================

String urlEncode(const String& value) {
  const char hex[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 3);

  for (size_t i = 0; i < value.length(); i++) {
    uint8_t c = (uint8_t)value[i];
    bool unreserved =
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') ||
      c == '-' || c == '_' || c == '.' || c == '~';

    if (unreserved) {
      encoded += (char)c;
    } else {
      encoded += '%';
      encoded += hex[c >> 4];
      encoded += hex[c & 0x0F];
    }
  }
  return encoded;
}

bool vkConfigured() {
  return VK_TOKEN[0] != '\0' &&
         strcmp(VK_TOKEN, "PASTE_COMMUNITY_TOKEN_HERE") != 0 &&
         VK_GROUP_ID > 0 && VK_ALLOWED_USER_ID > 0;
}

// Makes a POST to one VK API method. Extra parameters must start with '&'.
bool vkPost(const char* method, const String& extraParams, String& responseText) {
  if (WiFi.status() != WL_CONNECTED || !vkConfigured()) return false;

  WiFiClientSecure tls;
  tls.setInsecure(); // Temporary bootstrap mode; certificate is encrypted but not verified.

  HTTPClient http;
  http.setTimeout(VK_HTTP_TIMEOUT_MS);
  http.useHTTP10(true);

  String url = String(VK_API_BASE) + method;
  if (!http.begin(tls, url)) {
    Serial.println(F("VK: HTTP begin failed"));
    return false;
  }

  http.addHeader(F("Content-Type"), F("application/x-www-form-urlencoded"));

  String body;
  body.reserve(300 + extraParams.length());
  body = "access_token=";
  body += urlEncode(VK_TOKEN);
  body += "&v=";
  body += VK_API_VERSION;
  body += "&group_id=";
  body += String(VK_GROUP_ID);
  body += extraParams;

  int httpCode = http.POST(body);
  if (httpCode > 0) responseText = http.getString();
  else responseText = "";
  http.end();

  Serial.print(F("VK "));
  Serial.print(method);
  Serial.print(F(" HTTP: "));
  Serial.println(httpCode);

  return httpCode == HTTP_CODE_OK;
}

// ============================================================
// VK MESSAGE QUEUE AND SENDING
// ============================================================

void appendVKQueue(String& target, const String& message, const __FlashStringHelper* label) {
  if (target.length() == 0) {
    target = message;
  } else if (target.length() + message.length() < VK_QUEUE_MAX_LENGTH) {
    target += "\n\n";
    target += message;
  } else {
    Serial.print(F("VK queue full; extra "));
    Serial.print(label);
    Serial.println(F(" omitted"));
  }
}

void queueVKMessage(const String& message) {
  if (!vkConfigured()) {
    Serial.println(F("VK: token/IDs not configured; notification not queued"));
    return;
  }

  if (activeCommandPeerId == VK_ALLOWED_USER_ID) {
    appendVKQueue(pendingVKAdminReply, message, F("admin reply"));
  } else if (activeCommandPeerId != 0 && activeCommandPeerId == ownerState.ownerId) {
    appendVKQueue(pendingVKOwnerReply, message, F("owner reply"));
  } else {
    Serial.println(F("VK: no valid reply recipient context; message omitted"));
  }
}

// Scheduled reports are kept separately so they can never get ahead of
// a reply to a button or command. A newer scheduled report replaces an
// older unsent one instead of building up a stale backlog.
void queueVKReportMessage(const String& message) {
  if (!vkConfigured()) {
    Serial.println(F("VK: token/IDs not configured; report not queued"));
    return;
  }
  if (ownerState.ownerId == 0) {
    Serial.println(F("VK: public owner is not assigned; scheduled report omitted"));
    return;
  }
  pendingVKReport = message;
}

// Постоянная клавиатура: только статус и изменение интервала.
// Кнопки типа text отправляют подпись в историю диалога VK.
static const char VK_KEYBOARD_JSON[] =
  "{\"one_time\":false,\"inline\":false,\"buttons\":["
  "[{\"action\":{\"type\":\"text\",\"label\":\"📊 Статус\"},\"color\":\"primary\"},"
  "{\"action\":{\"type\":\"text\",\"label\":\"⏱ Интервал\"},\"color\":\"positive\"}]]}";

// Отправка в произвольный peer. Для скрытой команды /test клавиатура не нужна.
bool sendVKMessageToPeer(uint32_t peerId, const String& message, bool withKeyboard, uint32_t* messageIdOut = nullptr) {
  if (messageIdOut) *messageIdOut = 0;
  if (peerId == 0) return false;

  uint32_t randomId = (uint32_t)random(1, 2147483647L);
  String params;
  params.reserve(message.length() * 3 + sizeof(VK_KEYBOARD_JSON) * 3 + 160);
  params = "&peer_id=";
  params += String(peerId);
  params += "&random_id=";
  params += String(randomId);
  params += "&message=";
  params += urlEncode(message);
  if (withKeyboard) {
    params += "&keyboard=";
    params += urlEncode(String(VK_KEYBOARD_JSON));
  }

  String response;
  bool httpOk = vkPost("messages.send", params, response);
  Serial.print(F("VK response: "));
  Serial.println(response);

  if (!httpOk) return false;
  if (response.indexOf("\"error\"") >= 0) return false;

  int responseKey = response.indexOf("\"response\"");
  if (responseKey < 0) return false;
  int colon = response.indexOf(':', responseKey);
  if (colon < 0) return false;
  int valuePos = colon + 1;
  while (valuePos < (int)response.length() &&
         (response[valuePos] == ' ' || response[valuePos] == '\t' ||
          response[valuePos] == '\r' || response[valuePos] == '\n')) {
    valuePos++;
  }
  char* endPtr = nullptr;
  unsigned long parsedId = strtoul(response.c_str() + valuePos, &endPtr, 10);
  if (endPtr == response.c_str() + valuePos || parsedId == 0 || parsedId > 0xFFFFFFFFUL) return false;
  if (messageIdOut) *messageIdOut = (uint32_t)parsedId;
  Serial.print(F("VK: messages.send accepted message_id="));
  Serial.print((uint32_t)parsedId);
  Serial.print(F(" for peer_id="));
  Serial.println(peerId);
  return true;
}

void serviceVKQueue() {
  if (WiFi.status() != WL_CONNECTED) return;

  // Admin hidden-command replies, owner command replies, then scheduled report.
  const bool sendingAdminReply = pendingVKAdminReply.length() > 0;
  const bool sendingOwnerReply = !sendingAdminReply && pendingVKOwnerReply.length() > 0;
  const bool sendingReply = sendingAdminReply || sendingOwnerReply;
  if (!sendingReply && pendingVKReport.length() == 0) return;

  uint32_t now = millis();
  bool retryDelayActive = vkAttempted && (uint32_t)(now - lastVKAttempt) < VK_RETRY_MS;
  if (retryDelayActive && !(sendingReply && !lastVKFailureWasReply)) return;

  vkAttempted = true;
  lastVKAttempt = now;

  uint32_t targetPeer = sendingAdminReply ? VK_ALLOWED_USER_ID : ownerState.ownerId;
  String messageToSend = sendingAdminReply ? pendingVKAdminReply :
                         (sendingOwnerReply ? pendingVKOwnerReply : pendingVKReport);
  bool withKeyboard = (targetPeer == ownerState.ownerId);

  if (sendVKMessageToPeer(targetPeer, messageToSend, withKeyboard)) {
    if (sendingAdminReply) {
      pendingVKAdminReply = "";
      Serial.println(F("VK: admin hidden-command reply sent"));
    } else if (sendingOwnerReply) {
      pendingVKOwnerReply = "";
      Serial.println(F("VK: owner command reply sent"));
    } else {
      pendingVKReport = "";
      Serial.println(F("VK: scheduled report sent to owner"));
    }
    vkAttempted = false;
    lastVKFailureWasReply = false;
  } else {
    lastVKFailureWasReply = sendingReply;
    Serial.println(F("VK: send failed; retry in 60 seconds"));
  }
}

// ============================================================
// STATUS / COMMAND HELPERS
// ============================================================

String makeSensorMessage() {
  char t[16], h[16];
  dtostrf(lastTemperature, 1, 2, t);
  dtostrf(lastHumidity, 1, 2, h);

  String message;
  message.reserve(100);
  message = "🌡 Температура\n";
  message += t;
  message += " °C\n\n💧 Влажность\n";
  message += h;
  message += " %";
  return message;
}

void requestStatus() {
  Serial.println(F("VK: status command; reading sensor for command reply"));
  float temperature, humidity;
  if (readSHT(temperature, humidity)) {
    lastTemperature = temperature;
    lastHumidity = humidity;
    haveReading = true;
    queueVKMessage(makeSensorMessage());
  } else if (haveReading) {
    // If one immediate read fails, return the most recent valid measurement.
    queueVKMessage(makeSensorMessage());
  } else {
    queueVKMessage("⚠️ Датчик SHT40/41 пока не отдал показания.");
  }
}

void sendHelp() {
  queueVKMessage(
    "Управление метеостанцией:\n"
    "📊 Статус — получить температуру и влажность.\n"
    "⏱ Интервал — посмотреть или изменить частоту отправки.\n\n"
    "Также можно написать /status или /interval 10.\n"
    "Допустимый интервал: от 1 до 1440 минут."
  );
}

// ============================================================
// HIDDEN HTTP/HTTPS FIRMWARE UPDATE: /update http://host/path.bin
// Only VK_ALLOWED_USER_ID may invoke this hidden maintenance command.
// ESP8266HTTPUpdate validates image header, available flash space and MD5
// when the server supplies the x-MD5 header.
// ============================================================

bool validFirmwareUrl(const String& url) {
  if (url.length() < 10 || url.length() > 700) return false;

  // Accept ordinary HTTP servers as well as HTTPS. Scheme is case-insensitive.
  String scheme = url.substring(0, 7);
  scheme.toLowerCase();
  int hostStart = 0;
  if (scheme == "http://") {
    hostStart = 7;
  } else {
    String secureScheme = url.substring(0, 8);
    secureScheme.toLowerCase();
    if (secureScheme != "https://") return false;
    hostStart = 8;
  }

  for (size_t i = 0; i < url.length(); i++) {
    char c = url[i];
    if (c <= 32 || c == '\\' || c == '"' || c == '<' || c == '>') return false;
  }

  int hostEnd = url.length();
  int slash = url.indexOf('/', hostStart);
  int query = url.indexOf('?', hostStart);
  int fragment = url.indexOf('#', hostStart);
  if (slash >= 0 && slash < hostEnd) hostEnd = slash;
  if (query >= 0 && query < hostEnd) hostEnd = query;
  if (fragment >= 0 && fragment < hostEnd) hostEnd = fragment;

  if (hostEnd <= hostStart) return false;
  String host = url.substring(hostStart, hostEnd);
  if (host.indexOf('@') >= 0 || host.indexOf('\\') >= 0) return false;
  return true;
}

void performFirmwareUpdate(const String& url, uint32_t notifyPeerId) {
  if (firmwareUpdateRunning) {
    sendVKMessageToPeer(notifyPeerId, "⚠️ Обновление уже выполняется.", false);
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    sendVKMessageToPeer(notifyPeerId, "❌ Обновление отменено: нет подключения к Wi-Fi.", false);
    return;
  }

  firmwareUpdateRunning = true;
  waitingForIntervalInput = false;

  Serial.println(F("OTA: URL received from authorized VK user"));
  sendVKMessageToPeer(notifyPeerId, "📥 Ссылка на прошивку получена. Проверяю формат HTTP/HTTPS-ссылки...", false);

  if (!validFirmwareUrl(url)) {
    sendVKMessageToPeer(notifyPeerId, "❌ Ссылка отклонена. Нужна полная HTTP- или HTTPS-ссылка без пробелов, например http://example.com/firmware.bin", false);
    Serial.println(F("OTA: URL validation failed"));
    firmwareUpdateRunning = false;
    return;
  }

  String urlScheme = url.substring(0, 8);
  urlScheme.toLowerCase();
  const bool useHttps = (urlScheme == "https://");
  if (useHttps) {
    sendVKMessageToPeer(notifyPeerId, "✅ Формат HTTPS-ссылки принят. Проверяю сервер и прошивку...", false);
    Serial.println(F("OTA: starting HTTPS firmware download and validation"));
  } else {
    sendVKMessageToPeer(notifyPeerId, "⚠️ Формат HTTP-ссылки принят. Соединение не шифруется; прошивку можно подменить по пути передачи. Проверяю сервер и прошивку...", false);
    Serial.println(F("OTA: starting UNENCRYPTED HTTP firmware download and validation"));
  }

  ESPhttpUpdate.setClientTimeout(15000);
  ESPhttpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  ESPhttpUpdate.rebootOnUpdate(false);
  ESPhttpUpdate.onStart([]() {
    Serial.println(F("OTA: server returned firmware; download/write started"));
  });
  ESPhttpUpdate.onProgress([](int current, int total) {
    static int lastPercent = -10;
    int percent = total > 0 ? (int)((uint32_t)current * 100UL / (uint32_t)total) : 0;
    if (percent >= lastPercent + 10 || current == total) {
      lastPercent = percent;
      Serial.printf("OTA: %d%% (%d/%d bytes)\n", percent, current, total);
    }
  });
  ESPhttpUpdate.onEnd([]() {
    Serial.println(F("OTA: image written and Update.end passed"));
  });
  ESPhttpUpdate.onError([](int error) {
    Serial.printf("OTA: updater error %d\n", error);
  });

  // HTTP uses a plain WiFiClient. HTTPS uses WiFiClientSecure.
  // The updater checks the response, image header, available flash space and
  // x-MD5 (when supplied by the server), then writes the new sketch.
  t_httpUpdate_return result;
  if (useHttps) {
    WiFiClientSecure updateClient;
    updateClient.setInsecure(); // TLS is encrypted, but certificate is not verified.
    result = ESPhttpUpdate.update(updateClient, url);
  } else {
    WiFiClient updateClient;
    result = ESPhttpUpdate.update(updateClient, url);
  }

  if (result == HTTP_UPDATE_OK) {
    Serial.println(F("OTA: update successful; rebooting"));
    sendVKMessageToPeer(notifyPeerId, "✅ Проверка прошивки пройдена. Образ записан во flash без ошибки. Устройство перезагружается...", false);
    delay(1200);
    ESP.restart();
    return;
  }

  if (result == HTTP_UPDATE_NO_UPDATES) {
    sendVKMessageToPeer(notifyPeerId, "ℹ️ Сервер сообщил, что обновлений нет.", false);
  } else {
    String error = ESPhttpUpdate.getLastErrorString();
    if (error.length() == 0) error = "код " + String(ESPhttpUpdate.getLastError());
    Serial.print(F("OTA failed: "));
    Serial.println(error);
    sendVKMessageToPeer(notifyPeerId, "❌ Обновление не выполнено. Проверка/загрузка/запись завершилась ошибкой: " + error, false);
  }

  firmwareUpdateRunning = false;
}

bool parseUnsignedMinutes(const String& arg, uint16_t& value) {
  if (arg.length() == 0) return false;
  for (size_t i = 0; i < arg.length(); i++) {
    if (arg[i] < '0' || arg[i] > '9') return false;
  }
  unsigned long parsed = strtoul(arg.c_str(), nullptr, 10);
  if (parsed > 65535UL) return false;
  value = (uint16_t)parsed;
  return true;
}

void setSendInterval(uint16_t minutes) {
  settings.intervalMin = minutes;
  saveSettings();
  nextSendAt = millis() + (uint32_t)minutes * 60000UL;
  scheduleStarted = true;
  waitingForIntervalInput = false;
  queueVKMessage("✅ Интервал отправки VK установлен: " + String(minutes) + " мин.");
}

void askForInterval() {
  waitingForIntervalInput = true;
  queueVKMessage(
    "⏱ Текущий интервал: " + String(settings.intervalMin) + " мин.\n"
    "Отправь новое значение в минутах (от 1 до 1440).\n"
    "Например: 10"
  );
}

void sendPersonalReport(uint32_t recipientId) {
  float temperature, humidity;
  if (readSHT(temperature, humidity)) {
    lastTemperature = temperature;
    lastHumidity = humidity;
    haveReading = true;
  }

  String reportText;
  if (haveReading) {
    reportText = makeSensorMessage();
  } else {
    reportText = "⚠️ Датчик SHT40/41 пока не отдал показания.";
  }

  Serial.print(F("VK: personal report target peer_id="));
  Serial.print(recipientId);
  Serial.print(F(", payload bytes="));
  Serial.println(reportText.length());
  Serial.print(F("VK: personal report payload: "));
  Serial.println(reportText);

  uint32_t sentMessageId = 0;
  if (!sendVKMessageToPeer(recipientId, reportText, false, &sentMessageId)) {
    Serial.print(F("VK: personal report API send failed for peer_id "));
    Serial.println(recipientId);
    return;
  }

  Serial.print(F("VK: VK API accepted personal report, message_id="));
  Serial.print(sentMessageId);
  Serial.print(F(", peer_id="));
  Serial.println(recipientId);

  // Do not treat an API response as proof that the message is in the intended
  // dialog. Verify it via the next existing history poll, without a new TLS call.
  pendingVKVerifyPeerId = recipientId;
  pendingVKVerifyMessageId = sentMessageId;
}


bool parsePeerId(const String& value, uint32_t& peerId) {
  if (value.length() == 0) return false;
  for (size_t i = 0; i < value.length(); i++) {
    if (value[i] < '0' || value[i] > '9') return false;
  }
  unsigned long parsed = strtoul(value.c_str(), nullptr, 10);
  if (parsed == 0 || parsed > 2147483647UL) return false;
  peerId = (uint32_t)parsed;
  return true;
}

void handleVKCommand(String messageText, uint32_t senderId) {
  const bool isAdmin = senderId == VK_ALLOWED_USER_ID;
  const bool isOwner = senderId == ownerState.ownerId;
  if (!isAdmin && !isOwner) return;

  activeCommandPeerId = senderId;
  messageText.trim();
  if (messageText.length() == 0) return;

  // Text buttons are handled through messages.getHistory.
  if (messageText == "📊 Статус") messageText = "/status";
  else if (messageText == "⏱ Интервал") messageText = "/interval";

  // Lowercase only the command token. URL arguments stay case-sensitive.
  int separator = messageText.indexOf(' ');
  String command = separator >= 0 ? messageText.substring(0, separator) : messageText;
  command.toLowerCase();
  String argument = separator >= 0 ? messageText.substring(separator + 1) : "";
  argument.trim();

  // Hidden owner assignment: available only to the fixed administrator.
  if (command == "/owner") {
    waitingForIntervalInput = false;
    if (!isAdmin) {
      queueVKMessage("⛔ Назначать владельца может только администратор.");
      return;
    }

    uint32_t newOwnerId = 0;
    if (!parsePeerId(argument, newOwnerId) || newOwnerId == VK_ALLOWED_USER_ID) {
      queueVKMessage("Формат: /owner ID_пользователя\nID владельца должен быть числом и отличаться от ID администратора 34027667.");
      return;
    }

    uint32_t previousOwnerId = ownerState.ownerId;
    uint32_t previousOwnerCursor = ownerState.lastProcessedMessageId;
    uint32_t previousCursorInitialized = ownerState.cursorInitialized;
    ownerState.ownerId = newOwnerId;
    ownerState.lastProcessedMessageId = 0;
    ownerState.cursorInitialized = 0;
    pendingVKOwnerReply = "";
    pendingVKReport = "";
    waitingForIntervalInput = false;
    if (!saveOwnerState()) {
      ownerState.ownerId = previousOwnerId;
      ownerState.lastProcessedMessageId = previousOwnerCursor;
      ownerState.cursorInitialized = previousCursorInitialized;
      queueVKMessage("❌ Не удалось сохранить нового владельца в EEPROM. Старый владелец оставлен без изменений.");
      return;
    }

    // Send a personal welcome message with the public keyboard only after
    // pollVKPeer() has released its JSON document and HTTP stream.
    pendingOwnerWelcomePeerId = newOwnerId;
    queueVKMessage("✅ Права владельца переданы пользователю VK ID " + String(newOwnerId) + ". Отправляю ему личное уведомление с инструкцией и кнопками.");
    Serial.print(F("VK: owner changed to "));
    Serial.println(newOwnerId);
    return;
  }

  // Hidden personal report: only the fixed administrator may call it;
  // the report itself is delivered directly and only to that sender.
  if (command == "/myreport") {
    waitingForIntervalInput = false;
    if (!isAdmin) {
      queueVKMessage("⛔ Эта скрытая команда доступна только администратору.");
      return;
    }
    if (pendingPersonalReportPeerId != 0) {
      queueVKMessage("⚠️ Запрос /myreport уже ожидает отправки. Повтори команду позже.");
      return;
    }
    pendingPersonalReportPeerId = senderId;
    Serial.println(F("VK: /myreport queued; will send after history JSON is released"));
    return;
  }

  // The fixed administrator alone may use hidden maintenance commands.
  if (command == "/update") {
    waitingForIntervalInput = false;
    if (!isAdmin) {
      queueVKMessage("⛔ Обновление прошивки доступно только администратору.");
      return;
    }
    if (argument.length() == 0) {
      queueVKMessage("Формат скрытой команды: /update http://адрес/firmware.bin или /update https://адрес/firmware.bin");
      return;
    }
    if (pendingFirmwareUpdatePeerId != 0) {
      queueVKMessage("⚠️ Другая команда /update уже ожидает выполнения.");
      return;
    }
    pendingFirmwareUpdateUrl = argument;
    pendingFirmwareUpdatePeerId = senderId;
    Serial.println(F("OTA: update command queued until VK history parsing is finished"));
    return;
  }

  if (command == "/test") {
    waitingForIntervalInput = false;
    if (!isAdmin) {
      queueVKMessage("⛔ Эта скрытая команда доступна только администратору.");
      return;
    }

    uint32_t targetPeer = 0;
    if (!parsePeerId(argument, targetPeer)) {
      queueVKMessage("Формат скрытой команды: /test ID_получателя");
      return;
    }

    if (pendingTestTargetPeerId != 0) {
      queueVKMessage("⚠️ Другая команда /test уже ожидает отправки.");
      return;
    }
    pendingTestTargetPeerId = targetPeer;
    pendingTestCommandPeerId = senderId;
    Serial.print(F("VK hidden test queued for peer_id "));
    Serial.println(targetPeer);
    return;
  }

  // Administrator is deliberately excluded from public controls once a
  // different owner is assigned. It keeps access only to hidden commands.
  if (!isOwner) {
    if (messageText.startsWith("/")) {
      if (ownerState.ownerId == 0) {
        queueVKMessage("Владелец ещё не назначен. Назначь его скрытой командой /owner ID. Администратору доступны только скрытые команды.");
      } else {
        queueVKMessage("Обычное управление передано владельцу ID " + String(ownerState.ownerId) + ". Администратору доступны только скрытые команды.");
      }
    }
    return;
  }

  // After pressing Interval, a plain number is accepted as the new value.
  if (waitingForIntervalInput) {
    uint16_t minutes;
    if (parseUnsignedMinutes(messageText, minutes)) {
      if (minutes < MIN_INTERVAL_MIN || minutes > MAX_INTERVAL_MIN) {
        queueVKMessage("❌ Допустимый интервал: от " + String(MIN_INTERVAL_MIN) + " до " + String(MAX_INTERVAL_MIN) + " минут.");
        return;
      }
      setSendInterval(minutes);
      return;
    }
  }

  if (command == "/help" || command == "help" || command == "/start" || command == "start") {
    waitingForIntervalInput = false;
    sendHelp();
    return;
  }

  if (command == "/status" || command == "status") {
    waitingForIntervalInput = false;
    requestStatus();
    return;
  }

  if (command == "/interval" || command == "interval") {
    if (argument.length() == 0) {
      askForInterval();
      return;
    }

    uint16_t minutes;
    if (!parseUnsignedMinutes(argument, minutes) || minutes < MIN_INTERVAL_MIN || minutes > MAX_INTERVAL_MIN) {
      queueVKMessage("❌ Допустимый интервал: от " + String(MIN_INTERVAL_MIN) + " до " + String(MAX_INTERVAL_MIN) + " минут. Пример: /interval 10");
      return;
    }

    setSendInterval(minutes);
    return;
  }

  if (messageText.startsWith("/")) {
    waitingForIntervalInput = false;
    queueVKMessage("Неизвестная команда. Используй кнопки «Статус» и «Интервал» или /help.");
  }
}

// ============================================================
// VK INCOMING COMMAND POLLING
// ============================================================

void pollVKPeer(uint32_t peerId, uint32_t& cursor, bool isAdminPeer, bool isOwnerPeer) {
  if (peerId == 0) return;

  // Read history directly from the HTTPS stream. The old getString() path
  // temporarily held two copies of a ~5 KB response (HTTPClient payload and
  // response String), then allocated an 8 KB JSON document. On ESP8266 this
  // could truncate the body and produce EmptyInput/IncompleteInput.
  WiFiClientSecure tls;
  tls.setInsecure();

  HTTPClient http;
  http.setTimeout(VK_HTTP_TIMEOUT_MS);
  http.useHTTP10(true); // Connection: close; no persistent TLS socket reuse.

  String url = String(VK_API_BASE) + "messages.getHistory";
  if (!http.begin(tls, url)) {
    Serial.println(F("VK: getHistory HTTP begin failed"));
    return;
  }

  http.addHeader(F("Content-Type"), F("application/x-www-form-urlencoded"));

  String body;
  body.reserve(300);
  body = "access_token=";
  body += urlEncode(VK_TOKEN);
  body += "&v=";
  body += VK_API_VERSION;
  body += "&group_id=";
  body += String(VK_GROUP_ID);
  body += "&peer_id=";
  body += String(peerId);
  body += "&count=5"; // Small batch keeps response and heap usage bounded.
  body += "&rev=";
  body += String(VK_HISTORY_REV);

  int httpCode = http.POST(body);
  Serial.print(F("VK messages.getHistory HTTP: "));
  Serial.println(httpCode);
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    Serial.print(F("VK: getHistory request failed for peer_id "));
    Serial.println(peerId);
    return;
  }

  int contentLength = http.getSize();
  Serial.print(F("VK getHistory Content-Length: "));
  Serial.print(contentLength);
  Serial.print(F("; free heap before JSON: "));
  Serial.println(ESP.getFreeHeap());

  // Retain only the four fields actually used below. The filter is tiny and
  // also prevents unused VK profile/attachment data consuming the JSON pool.
  StaticJsonDocument<256> filter;
  filter["error"]["error_msg"] = true;
  filter["error"]["error_code"] = true;
  filter["response"]["items"][0]["id"] = true;
  filter["response"]["items"][0]["from_id"] = true;
  filter["response"]["items"][0]["peer_id"] = true;
  filter["response"]["items"][0]["text"] = true;

  // Allocate a smaller document only after the TLS connection is established.
  DynamicJsonDocument doc(4096);
  WiFiClient& responseStream = http.getStream();
  // Arduino Stream defaults to a 1-second read timeout. VK can pause while
  // sending a response, so explicitly use the configured HTTP timeout here;
  // otherwise ArduinoJson may report EmptyInput/IncompleteInput mid-response.
  responseStream.setTimeout(VK_HTTP_TIMEOUT_MS);
  DeserializationError jsonError = deserializeJson(
    doc, responseStream, DeserializationOption::Filter(filter));
  http.end(); // Release TLS/socket before processing the parsed messages.

  if (jsonError) {
    Serial.print(F("VK JSON parse error: "));
    Serial.print(jsonError.c_str());
    Serial.print(F("; Content-Length: "));
    Serial.print(contentLength);
    Serial.print(F("; free heap after parse: "));
    Serial.print(ESP.getFreeHeap());
    Serial.println(F("; cursor unchanged"));
    return;
  }
  if (doc.overflowed()) {
    Serial.println(F("VK JSON document overflow; cursor unchanged"));
    return;
  }

  if (doc.containsKey("error")) {
    Serial.print(F("VK API error for peer_id "));
    Serial.print(peerId);
    Serial.print(F(": "));
    Serial.println((const char*)(doc["error"]["error_msg"] | "unknown"));
    return;
  }

  JsonArray items = doc["response"]["items"].as<JsonArray>();
  bool cursorInitialized = isAdminPeer ? (settings.reserved1 != 0) : (ownerState.cursorInitialized != 0);
  if (items.isNull() || items.size() == 0) {
    if (!cursorInitialized) {
      if (isAdminPeer) {
        settings.reserved1 = 1;
        saveSettings();
      }
      if (isOwnerPeer) {
        ownerState.cursorInitialized = 1;
        saveOwnerState();
      }
      Serial.print(F("VK: empty history initialized for peer_id "));
      Serial.println(peerId);
    }
    return;
  }

  // A newly assigned owner's history cursor starts at zero. Baseline the
  // existing history, but execute the newest command if it is a public command.
  if (cursor == 0 && !cursorInitialized) {
    uint32_t newestId = 0;
    int32_t newestFromId = 0;
    String firstText;
    for (JsonObject item : items) {
      uint32_t id = item["id"] | 0UL;
      if (id > newestId) {
        newestId = id;
        newestFromId = item["from_id"] | 0L;
        firstText = String((const char*)(item["text"] | ""));
      }
    }
    if (newestId > 0) {
      cursor = newestId;
      if (isAdminPeer) {
        settings.reserved1 = 1;
        saveSettings();
      }
      if (isOwnerPeer) {
        ownerState.cursorInitialized = 1;
        saveOwnerState();
      }
      Serial.print(F("VK: initial cursor for peer_id "));
      Serial.print(peerId);
      Serial.print(F(" set to "));
      Serial.println(newestId);

      firstText.trim();
      // If this is the first message from the new owner, allow a public
      // command/button to work immediately instead of requiring a dummy message.
      bool isPublicStart = firstText == "/start" || firstText == "start" ||
                           firstText == "/status" || firstText == "status" ||
                           firstText == "📊 Статус" || firstText == "⏱ Интервал" ||
                           firstText == "/interval" || firstText == "interval" ||
                           firstText.startsWith("/interval ") ||
                           firstText == "/help" || firstText == "help";
      if (isOwnerPeer && (uint32_t)newestFromId == peerId && isPublicStart) {
        activeCommandPeerId = peerId;
        handleVKCommand(firstText, peerId);
        activeCommandPeerId = 0;
      }
    }
    return;
  }

  bool cursorChanged = false;
  // rev=0 gives newest-first; process newly arrived messages oldest-first.
  for (int i = (int)items.size() - 1; i >= 0; i--) {
    JsonObject item = items[i];
    uint32_t messageId = item["id"] | 0UL;
    int32_t fromId = item["from_id"] | 0L;

    // Check the report ID before cursor filtering: this message may already
    // have been seen when the normal history cursor advanced.
    if (pendingVKVerifyMessageId != 0 &&
        peerId == pendingVKVerifyPeerId &&
        messageId == pendingVKVerifyMessageId) {
      const char* verifiedText = item["text"] | "";
      int32_t itemPeerId = item["peer_id"] | 0L;
      Serial.print(F("VK SEND VERIFY: message_id="));
      Serial.print(messageId);
      Serial.print(F(", requested_peer="));
      Serial.print(pendingVKVerifyPeerId);
      Serial.print(F(", item_peer="));
      Serial.print(itemPeerId);
      Serial.print(F(", from_id="));
      Serial.print(fromId);
      Serial.print(F(", text="));
      Serial.println(verifiedText);
      if (itemPeerId == (int32_t)pendingVKVerifyPeerId) {
        Serial.println(F("VK SEND VERIFY: message is present in the requested conversation history"));
      } else {
        Serial.println(F("VK SEND VERIFY: WARNING - VK returned a different peer_id"));
      }
      pendingVKVerifyMessageId = 0;
      pendingVKVerifyPeerId = 0;
    }

    if (messageId == 0 || messageId <= cursor) continue;

    if ((uint32_t)fromId == peerId) {
      const char* text = item["text"] | "";
      Serial.print(F("VK message from peer_id "));
      Serial.print(peerId);
      Serial.print(F(": "));
      Serial.println(text);
      activeCommandPeerId = peerId;
      handleVKCommand(String(text), peerId);
      activeCommandPeerId = 0;
    } else {
      // Outgoing community messages are returned in this same peer history.
      // This confirms whether VK actually stored the message in the intended dialog.
      const char* outgoingText = item["text"] | "";
      int32_t itemPeerId = item["peer_id"] | 0L;
      Serial.print(F("VK history outgoing/other record: requested_peer="));
      Serial.print(peerId);
      Serial.print(F(", item_peer="));
      Serial.print(itemPeerId);
      Serial.print(F(", from_id="));
      Serial.print(fromId);
      Serial.print(F(", message_id="));
      Serial.print(messageId);
      Serial.print(F(", text="));
      Serial.println(outgoingText);
    }

    cursor = messageId;
    cursorChanged = true;
  }

  if (cursorChanged) {
    if (isAdminPeer) saveSettings();
    if (isOwnerPeer) saveOwnerState();
  }
}

void pollVKMessages() {
  if (!vkConfigured() || WiFi.status() != WL_CONNECTED) return;

  uint32_t now = millis();
  if (lastVKPoll != 0 && (uint32_t)(now - lastVKPoll) < VK_POLL_MS) return;
  lastVKPoll = now;

  const uint32_t adminId = VK_ALLOWED_USER_ID;
  const uint32_t ownerId = ownerState.ownerId;

  if (ownerId == adminId) {
    // Before a separate owner is assigned, the original peer serves both roles.
    pollVKPeer(adminId, settings.lastProcessedMessageId, true, true);
    return;
  }

  pollVKPeer(adminId, settings.lastProcessedMessageId, true, false);

  // The administrator may have changed the owner while its history was being
  // processed. Never apply the old owner's message cursor to the new owner.
  const uint32_t currentOwnerId = ownerState.ownerId;
  if (currentOwnerId == adminId) return;
  if (currentOwnerId != ownerId) {
    pollVKPeer(currentOwnerId, ownerState.lastProcessedMessageId, false, true);
    return;
  }
  pollVKPeer(currentOwnerId, ownerState.lastProcessedMessageId, false, true);
}

// Execute commands that open another TLS connection only after pollVKPeer()
// has returned. This frees the history response buffer and JSON document first.
void serviceDeferredVKCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (pendingOwnerWelcomePeerId != 0) {
    uint32_t newOwnerPeerId = pendingOwnerWelcomePeerId;
    pendingOwnerWelcomePeerId = 0;

    String welcome;
    welcome.reserve(520);
    welcome = "🎉 Поздравляем!\n\n";
    welcome += "Вам переданы права владельца метеостанции SHT40/41.\n\n";
    welcome += "Теперь вы можете самостоятельно управлять устройством:\n";
    welcome += "📊 «Статус» — получить текущую температуру и влажность.\n";
    welcome += "⏱ «Интервал» — изменить частоту автоматических отчётов.\n";
    welcome += "📨 Автоматические отчёты теперь будут приходить именно вам.\n\n";
    welcome += "Для начала работы откройте диалог с сообществом и нажмите «Начать», если ещё не сделали этого. Кнопки управления появятся под сообщениями сообщества.\n\n";
    welcome += "Приятного пользования! 🌡️";

    Serial.print(F("VK: sending owner welcome to peer_id "));
    Serial.println(newOwnerPeerId);
    bool welcomeSent = sendVKMessageToPeer(newOwnerPeerId, welcome, true);

    // Report delivery status back to the fixed administrator.
    activeCommandPeerId = VK_ALLOWED_USER_ID;
    if (welcomeSent) {
      queueVKMessage("✅ Личное приветствие с клавиатурой успешно отправлено новому владельцу (VK ID " + String(newOwnerPeerId) + ").");
      Serial.println(F("VK: owner welcome message sent"));
    } else {
      queueVKMessage("⚠️ Права владельца сохранены, но VK не разрешил отправить ему приветствие. Пусть новый владелец откроет диалог с сообществом и нажмёт «Начать», затем повтори команду /owner " + String(newOwnerPeerId) + ".");
      Serial.println(F("VK: owner welcome send failed; owner must start the community dialog"));
    }
    activeCommandPeerId = 0;
  }

  if (pendingTestTargetPeerId != 0) {
    uint32_t targetPeer = pendingTestTargetPeerId;
    uint32_t commandPeer = pendingTestCommandPeerId;
    pendingTestTargetPeerId = 0;
    pendingTestCommandPeerId = 0;

    Serial.print(F("VK hidden test: sending to peer_id "));
    Serial.println(targetPeer);
    bool sent = sendVKMessageToPeer(targetPeer, "тест", false);
    activeCommandPeerId = commandPeer;
    if (sent) {
      queueVKMessage("Тестовое сообщение отправлено в VK peer_id " + String(targetPeer) + ".");
    } else {
      queueVKMessage("Не удалось отправить тестовое сообщение в peer_id " + String(targetPeer) + ". Проверь, что получатель открыл диалог с сообществом.");
    }
    activeCommandPeerId = 0;
  }

  if (pendingPersonalReportPeerId != 0) {
    uint32_t recipientId = pendingPersonalReportPeerId;
    pendingPersonalReportPeerId = 0;
    Serial.print(F("VK: executing deferred personal report for peer_id "));
    Serial.println(recipientId);
    sendPersonalReport(recipientId);
  }

  if (pendingFirmwareUpdatePeerId != 0) {
    uint32_t notifyPeerId = pendingFirmwareUpdatePeerId;
    pendingFirmwareUpdatePeerId = 0;
    Serial.println(F("OTA: executing deferred firmware update after VK history parsing"));
    performFirmwareUpdate(pendingFirmwareUpdateUrl, notifyPeerId);
    pendingFirmwareUpdateUrl = "";
  }
}

// ============================================================
// WIFI MANAGEMENT
// ============================================================

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttempt = millis();
  Serial.println(F("Wi-Fi connecting..."));
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      if (!scheduleStarted) {
        // Do not send an unsolicited report immediately after boot/Wi-Fi connect.
        // The first periodic report is due only after the configured interval.
        nextSendAt = millis() + (uint32_t)settings.intervalMin * 60000UL;
        scheduleStarted = true;
        Serial.print(F("First VK report scheduled in "));
        Serial.print(settings.intervalMin);
        Serial.println(F(" minutes"));
      }

      Serial.println();
      Serial.println(F("Wi-Fi CONNECTED"));
      Serial.print(F("IP: "));
      Serial.println(WiFi.localIP());
      Serial.print(F("RSSI: "));
      Serial.print(WiFi.RSSI());
      Serial.println(F(" dBm"));

    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println(F("Wi-Fi LOST"));
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastWiFiAttempt) >= WIFI_RETRY_MS) {
    lastWiFiAttempt = now;
    Serial.println(F("Wi-Fi reconnect attempt..."));
    if (!WiFi.reconnect()) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

// ============================================================
// SENSOR SAMPLING AND VK REPORT SCHEDULE
// ============================================================

void performSensorUpdate() {
  Serial.println();
  Serial.println(F("========== SENSOR UPDATE =========="));

  float temperature, humidity;
  if (!readSHT(temperature, humidity)) {
    Serial.println(F("SHT: READ FAILED; retry in 30 seconds"));
    nextSendAt = millis() + SENSOR_RETRY_MS;
    Serial.println(F("===================================="));
    return;
  }

  lastTemperature = temperature;
  lastHumidity = humidity;
  haveReading = true;

  Serial.print(F("Temperature: "));
  Serial.print(temperature, 2);
  Serial.println(F(" C"));
  Serial.print(F("Humidity: "));
  Serial.print(humidity, 2);
  Serial.println(F(" %"));

  // The scheduled VK notification contains only temperature and humidity.
  queueVKReportMessage(makeSensorMessage());

  nextSendAt = millis() + (uint32_t)settings.intervalMin * 60000UL;

  Serial.print(F("Next VK report in "));
  Serial.print(settings.intervalMin);
  Serial.println(F(" minutes"));
  Serial.println(F("===================================="));
}

void serviceSchedules() {
  if (WiFi.status() == WL_CONNECTED && scheduleStarted && timeReached(nextSendAt)) {
    performSensorUpdate();
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println(F("===================================="));
  Serial.println(F(" SHT40/41 + VK MONITOR"));
  Serial.println(F("BUILD: VK_OWNER_WELCOME_2026-11-23_K"));
  Serial.println(F("===================================="));

  randomSeed(ESP.getChipId() ^ micros());
  loadSettings();
  loadOwnerState();

  Serial.print(F("VK report interval: "));
  Serial.print(settings.intervalMin);
  Serial.println(F(" min"));
  Serial.print(F("VK report owner ID: "));
  Serial.println(ownerState.ownerId);
  Serial.print(F("VK hidden-command admin ID: "));
  Serial.println(VK_ALLOWED_USER_ID);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // I2C pins from Config.h
  Wire.setClock(I2C_CLOCK_HZ);

  Wire.beginTransmission(SHT_ADDRESS);
  if (Wire.endTransmission() == 0) Serial.println(F("SHT40/41 detected"));
  else Serial.println(F("WARNING: SHT40/41 not found"));

  if (!vkConfigured()) Serial.println(F("VK is not configured: check token and IDs"));

  setupWiFi();
  Serial.println(F("System initialized"));
}

void loop() {
  maintainWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    pollVKMessages();
    serviceDeferredVKCommands();
    serviceSchedules();
    serviceVKQueue();
  }

  delay(10);
}
