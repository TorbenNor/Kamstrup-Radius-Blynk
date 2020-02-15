
/*
	Name:       Kamstrup.ino
	Created:	04-02-2020 21:00:00
	Author:     Torben
    Board LOLIN D32
    Blynk http://docs.blynk.cc/
    VPin 5: Export-Import
    VPin 6: Export-Import L1
    VPin 7: Export-Import L2 
    VPin 8: Export-Import L3
    VPin 40: RSSi Wireless sinal 
    VPin 101: Version

    Pin 16  :RXD2 Seriel in

*/

#include <BlynkSimpleEsp32.h>
#include "mbusparserkam.h"
#include <mbedtls/gcm.h>

const String Version = "  1.0";
const char* ssid = "my-SSID";
const char* password = "my-SSID paasword";
IPAddress local_IP(192, 1, 1, 150);
IPAddress gateway(192, 1, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(193, 162, 153, 164);
IPAddress secondaryDNS(194, 239, 134, 83);

const char auth[] = "*********"; //Blynk Key
const char conf_key[33] = "********"; //Meter Key
const char conf_authkey[33] = "*******"; //Meter Key

const size_t headersize = 11;
const size_t footersize = 3;
uint8_t encryption_key[16];
uint8_t authentication_key[16];
uint8_t receiveBuffer[500];
uint8_t decryptedFrameBuffer[500];
VectorView decryptedFrame(decryptedFrameBuffer, 0);
MbusStreamParser streamParser(receiveBuffer, sizeof(receiveBuffer));
mbedtls_gcm_context m_ctx;

#define BLYNK_PRINT Serial
#define RXD2 16
#define TXD2 17


void setup() {
	Serial.begin(115200);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("STA Failed to configure");
    }

    // Connect to Wi-Fi network with SSID and password
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    // Setup Blynk
    Blynk.config(auth);
    while (Blynk.connect() == false) {    }
    Blynk.syncAll();
    Blynk.virtualWrite(V101, Version);
    Blynk.virtualWrite(V9, 255);
    long rssi = WiFi.RSSI();
    Serial.println(rssi);

    Serial2.begin(2400, SERIAL_8N1, RXD2, TXD2);
    hexStr2bArr(encryption_key, conf_key, sizeof(encryption_key));
    hexStr2bArr(authentication_key, conf_authkey, sizeof(authentication_key));
    
    Serial.println("Setup completed");
}

void loop() {
    Blynk.run();

    while (Serial2.available() > 0) {
        if (streamParser.pushData(Serial2.read())) {
             VectorView frame = streamParser.getFrame();
                      if (streamParser.getContentType() == MbusStreamParser::COMPLETE_FRAME) {
                          Serial.println("Frame complete");
                          if (!decrypt(frame))
                          {
                    Serial.println("Decryption failed");
                    return;
                }
                MeterData md = parseMbusFrame(decryptedFrame);
                sendData(md);
            }
        }
    }
 // Do never ever use delay more than 20ms when using wireless functions
}

void sendData(MeterData md) {
    if (md.activePowerPlusValid)
        Serial.println("/power/activePowerPlus " + String(md.activePowerPlus));
     if (md.activePowerMinusValid)
        Serial.println(  "/power/activePowerMinus " + String(md.activePowerMinus));
  Blynk.virtualWrite(V5, md.activePowerPlus - md.activePowerMinus);
    if (md.activePowerPlusValidL1)
        Serial.println(  "/power/activePowerPlusL1 " + String(md.activePowerPlusL1));
    if (md.activePowerMinusValidL1)
        Serial.println(  "/power/activePowerMinusL1 "+ String(md.activePowerMinusL1));
  Blynk.virtualWrite(V6, md.activePowerPlusL1 - md.activePowerMinusL1);
    if (md.activePowerPlusValidL2)
        Serial.println(  "/power/activePowerPlusL2 "+ String(md.activePowerPlusL2));
    if (md.activePowerMinusValidL2)
        Serial.println(  "/power/activePowerMinusL2 "+ String(md.activePowerMinusL2));
  Blynk.virtualWrite(V7, md.activePowerPlusL2 - md.activePowerMinusL2);
    if (md.activePowerPlusValidL3)
        Serial.println(  "/power/activePowerPlusL3 "+ String(md.activePowerPlusL3));
    if (md.activePowerMinusValidL3)
        Serial.println(  "/power/activePowerMinusL3 "+ String(md.activePowerMinusL3));
  Blynk.virtualWrite(V8, md.activePowerPlusL3 - md.activePowerMinusL3);
    if (md.reactivePowerPlusValid)
        Serial.println(  "/power/reactivePowerPlus "+ String(md.reactivePowerPlus));
    if (md.reactivePowerMinusValid)
        Serial.println(  "/power/reactivePowerMinus "+ String(md.reactivePowerMinus));
    if (md.powerFactorValidL1)
        Serial.println(  "/power/powerFactorL1 "+ String(md.powerFactorL1));
    if (md.powerFactorValidL2)
        Serial.println(  "/power/powerFactorL2 "+ String(md.powerFactorL2));
    if (md.powerFactorValidL3)
        Serial.println(  "/power/powerFactorL3 "+ String(md.powerFactorL3));
    if (md.powerFactorTotalValid)
        Serial.println(  "/power/powerFactorTotal "+ String(md.powerFactorTotal));
    if (md.voltageL1Valid)
        Serial.println(  "/voltage/L1 "+ String(md.voltageL1));
    if (md.voltageL2Valid)
        Serial.println(  "/voltage/L2 "+ String(md.voltageL2));
    if (md.voltageL3Valid)
        Serial.println(  "/voltage/L3 "+ String(md.voltageL3));
    if (md.centiAmpereL1Valid)
        Serial.println(  "/current/L1 "+ String(md.centiAmpereL1 / 100.));
    if (md.centiAmpereL2Valid)
        Serial.println(  "/current/L2 "+ String(md.centiAmpereL2 / 100.));
    if (md.centiAmpereL3Valid)
        Serial.println(  "/current/L3 "+ String(md.centiAmpereL3 / 100.));
    if (md.activeImportWhValid)
        Serial.println(  "/energy/activeImportKWh "+ String(md.activeImportWh / 1000.));
    if (md.activeExportWhValid)
        Serial.println(  "/energy/activeExportKWh "+ String(md.activeExportWh / 1000.));
    if (md.activeImportWhValidL1)
        Serial.println(  "/energy/activeImportKWhL1 "+ String(md.activeImportWhL1 / 1000.));
    if (md.activeExportWhValidL1)
        Serial.println(  "/energy/activeExportKWhL1 "+ String(md.activeExportWhL1 / 1000.));
    if (md.activeImportWhValidL2)
        Serial.println(  "/energy/activeImportKWhL2 "+ String(md.activeImportWhL2 / 1000.));
    if (md.activeExportWhValidL2)
        Serial.println(  "/energy/activeExportKWhL2 "+ String(md.activeExportWhL2 / 1000.));
    if (md.activeImportWhValidL3)
        Serial.println(  "/energy/activeImportKWhL3 "+ String(md.activeImportWhL3 / 1000.));
    if (md.activeExportWhValidL3)
        Serial.println(  "/energy/activeExportKWhL3 "+ String(md.activeExportWhL3 / 1000.));
    if (md.reactiveImportWhValid)
        Serial.println(  "/energy/reactiveImportKWh "+ String(md.reactiveImportWh / 1000.));
    if (md.reactiveExportWhValid)
        Serial.println(  "/energy/reactiveExportKWh "+ String(md.reactiveExportWh / 1000.));
    Blynk.virtualWrite(V40, WiFi.RSSI());
       
}

void printHex(const unsigned char* data, const size_t length) {
    for (int i = 0; i < length; i++) {
        Serial.printf("%02X", data[i]);
    }
}

void printHex(const VectorView& frame) {
    for (int i = 0; i < frame.size(); i++) {
        Serial.printf("%02X", frame[i]);
    }
}

bool decrypt(const VectorView& frame) {

    if (frame.size() < headersize + footersize + 12 + 18) {
        Serial.println("Invalid frame size.");
    }

    memcpy(decryptedFrameBuffer, &frame.front(), frame.size());

    uint8_t system_title[8];
    memcpy(system_title, decryptedFrameBuffer + headersize + 2, 8);

    uint8_t initialization_vector[12];
    memcpy(initialization_vector, system_title, 8);
    memcpy(initialization_vector + 8, decryptedFrameBuffer + headersize + 14, 4);

    uint8_t additional_authenticated_data[17];
    memcpy(additional_authenticated_data, decryptedFrameBuffer + headersize + 13, 1);
    memcpy(additional_authenticated_data + 1, authentication_key, 16);

    uint8_t authentication_tag[12];
    memcpy(authentication_tag, decryptedFrameBuffer + headersize + frame.size() - headersize - footersize - 12, 12);

    uint8_t cipher_text[frame.size() - headersize - footersize - 18 - 12];
    memcpy(cipher_text, decryptedFrameBuffer + headersize + 18, frame.size() - headersize - footersize - 12 - 18);

    uint8_t plaintext[sizeof(cipher_text)];

    mbedtls_gcm_init(&m_ctx);
    int success = mbedtls_gcm_setkey(&m_ctx, MBEDTLS_CIPHER_ID_AES, encryption_key, sizeof(encryption_key) * 8);
    if (0 != success) {
        Serial.println("Setkey failed: " + String(success));
        return false;
    }
    success = mbedtls_gcm_auth_decrypt(&m_ctx, sizeof(cipher_text), initialization_vector, sizeof(initialization_vector),
        additional_authenticated_data, sizeof(additional_authenticated_data), authentication_tag, sizeof(authentication_tag),
        cipher_text, plaintext);
    if (0 != success) {
        Serial.println("authdecrypt failed: " + String(success));
        return false;
    }
    mbedtls_gcm_free(&m_ctx);

    //copy replace encrypted data with decrypted for mbusparser library. Checksum not updated. Hopefully not needed
    memcpy(decryptedFrameBuffer + headersize + 18, plaintext, sizeof(plaintext));
    decryptedFrame = VectorView(decryptedFrameBuffer, frame.size());

    return true;
}

void hexStr2bArr(uint8_t* dest, const char* source, int bytes_n)
{
    uint8_t* dst = dest;
    uint8_t* end = dest + sizeof(bytes_n);
    unsigned int u;

    while (dest < end && sscanf(source, "%2x", &u) == 1)
    {
        *dst++ = u;
        source += 2;
    }
}
