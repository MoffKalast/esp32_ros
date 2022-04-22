#include "esp_camera.h"

#include <WiFi.h>
#include <WebSocketsServer.h>

#include "Motors.h"
#include "Sockets.h"
#include "Lidar.h"
#include "Camera.h"

const char* ssid = "ESP32Car";
const char* password = "tokyodrift";

//In Arduino IDE Select: ESP32 Wrover Module, partition scheme: Huge APP 3MB

void startCameraServer();

int range = 2000;

void setup(){
	setupCamera();

	WiFi.mode(WIFI_AP);
	/**
	 * Set up an access point
	 * @param ssid              Pointer to the SSID (max 63 char).
	 * @param passphrase        (for WPA2 min 8 char, for open use NULL)
	 * @param channel           WiFi channel number, 1 - 13.
	 * @param ssid_hidden       Network cloaking (0 = broadcast SSID, 1 = hide SSID)
	 * @param max_connection    Max simultaneous connected clients, 1 - 4.
	*/
	WiFi.softAP(ssid, password, 6, 0, 2);

	initSockets();
	delay(50);
	
	initMotors();
	delay(50);

	initLidar();
	delay(50);

	startCameraServer();

	digitalWrite(4, HIGH);
	delay(1);
	digitalWrite(4, LOW);

}
 
void loop(){
	updateLidar();	
	updateSockets();
	updateMotors();
	delay(5);
}
