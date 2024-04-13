#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "Pulse.h"

#define BUTTON A0
#define LONG_PRESS_DURATION 2000
#define BLUETOOTH_ENABLE_PIN 3 // Pin de contrôle de l'alimentation du module Bluetooth
#define SYSTEM_RUNNING_ADDR 0 // Adresse dans la mémoire EEPROM pour stocker l'état du système

MAX30105 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // pour le capteur de température

long lastBeat = 0;
int  beatAvg;
int  SPO2;

// La table est organisée de manière à ce que chaque indice représente un pourcentage de SpO2,
// stockage dans la mémoire flash du microcontrôleur au lieu de la mémoire RAM (économise la mémoire)
const uint8_t spo2_table[184] PROGMEM =
    { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
      99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
      97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
      90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
      80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
      66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
      49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
      28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
      3, 2, 1 };

bool isSystemRunning = false; // variable that control the state of the systeme

// function that send data to bluetooth module
void sendVitalSign() {
  Serial.print("Hz="); Serial.print(beatAvg); Serial.print(":");
  Serial.print("Spo2="); Serial.print(SPO2); Serial.print(":");
  Serial.print("Temp="); Serial.print(mlx.readObjectTempC());
  Serial.print("\n"); // marque the end of sharing data
}

void vitalSigns() {
  sensor.check();
    
  if (!sensor.available()) return;
  uint32_t irValue = sensor.getIR();
  uint32_t redValue = sensor.getRed();
  
  sensor.nextSample();
  
  if (irValue < 5000) {
      
  } else {
      int16_t IR_signal, Red_signal;
      bool beatRed, beatIR;
      
      IR_signal = pulseIR.dc_filter(irValue);
      Red_signal = pulseRed.dc_filter(redValue);
      beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
      beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));
      
      if (beatRed || beatIR) {
          long btpm = 60000 / (millis() - lastBeat);
          if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
          lastBeat = millis();
          
          // Calcul du ratio SpO2
          long numerator = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
          long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
          int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;
         
          // À partir de la table
          if ((RX100 >= 0) && (RX100 < 184))
              SPO2 = pgm_read_byte(&spo2_table[RX100]);

          delay(500);
          sendVitalSign(); // send vitalSign all 0,5 secondes 
      }
  }  
}

// Function to power off components
void powerOffComponents() {
  digitalWrite(BLUETOOTH_ENABLE_PIN, LOW);
  Wire.end(); // Mettre fin à la communication I2C
}

// Function to power on components
void powerOnComponents() {
  digitalWrite(BLUETOOTH_ENABLE_PIN, HIGH); 
  Wire.begin(); // Début de la communication I2C
}

unsigned long buttonPressStartTime = 0; // Variable pour stocker le temps de début de l'appui sur le bouton

// Fonction appelée lors de l'appui sur le bouton
void checkButton() {
  static bool previousButtonState = HIGH; 
  bool currentButtonState = digitalRead(BUTTON);
  
  // Si l'état actuel du bouton est bas (bouton enfoncé) et l'état précédent était haut (bouton relâché)
  if (currentButtonState == LOW && previousButtonState == HIGH) {
    buttonPressStartTime = millis(); // Enregistrer le temps de début de l'appui sur le bouton
  }

  // Si l'état actuel du bouton est haut (bouton relâché) et l'état précédent était bas (bouton enfoncé)
  if (currentButtonState == HIGH && previousButtonState == LOW) {
    unsigned long buttonPressDuration = millis() - buttonPressStartTime; // Calculer la durée de l'appui sur le bouton

    // Si la durée de l'appui sur le bouton est supérieure ou égale à la durée requise pour un appui long
    if (buttonPressDuration >= LONG_PRESS_DURATION) {
      isSystemRunning = !isSystemRunning; // Inverser l'état du système
      EEPROM.write(SYSTEM_RUNNING_ADDR, isSystemRunning); // Enregistrer l'état du système dans la mémoire EEPROM
      
      // Si le système est démarré, mettre sous tension les composants
      if (isSystemRunning) {
        powerOnComponents();
      } else { // Sinon, mettre hors tension les composants
        powerOffComponents();
      }
    }
  }

  // Mettre à jour l'état précédent du bouton
  previousButtonState = currentButtonState;
}

// setup function for initialisation
void setup() {
    Serial.begin(9600); // Initialisation of serial monitor
    
    pinMode(BUTTON, INPUT_PULLUP);
    pinMode(BLUETOOTH_ENABLE_PIN, OUTPUT); 

    // Lire l'état du système depuis la mémoire EEPROM
    isSystemRunning = EEPROM.read(SYSTEM_RUNNING_ADDR);
    
    if (!sensor.begin()) {
        // dont send data and close the systeme in 1 munite
        
        while (1);
    }

    if(!mlx.begin()) {
      // dont send data and close the systeme in 1 munite
      
      while (1);
    }
    
    sensor.setup();
}

// loop function
void loop() {
  checkButton();
  
  if (isSystemRunning){ 
    vitalSigns();
  }
}
