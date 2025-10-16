#include <Arduino.h>
#include <WiFi.h>

// ---------- WiFi helpers ----------
static int scanWiFi() {
  int n = WiFi.scanNetworks();
  Serial.println("Scan done");
  if (n == 0) {
    Serial.println("No networks found");
  } else {
    for (int i = 0; i < n; ++i) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "" : " *");
      delay(10);
    }
  }
  return n;
}

static bool initWiFi(String ssid, String password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
    if (++i >= 30 || Serial.available()) {
      (void)Serial.readString();
      Serial.println("\nConnection failed");
      return false;
    }
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  return true;
}

void setupWiFiFromConsole() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(500);
  bool connected = false;

  while (!connected) {
    int n = scanWiFi();
    Serial.println("Enter SSID number:");
    while (!Serial.available()) {}
    int pos = Serial.readString().toInt();
    if (pos > 0 && pos <= n) {
      String ssid = WiFi.SSID(pos - 1);
      Serial.println("Selected: " + ssid);
      Serial.println("Enter password:");
      while (!Serial.available()) {}
      String password = Serial.readString();
      password.trim();
      ssid.trim();
      Serial.print("Connecting with password: ");
      for (int i = 0; i < password.length(); i++) Serial.print("*");
      Serial.println();
      connected = initWiFi(ssid, password);
    } else {
      Serial.println("Invalid selection. Try again.");
    }
  }
}

void waitForNTP() {
  struct tm timeinfo;
  Serial.print("Waiting for NTP");
  while (!getLocalTime(&timeinfo, 5000)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Time sync complete");
}