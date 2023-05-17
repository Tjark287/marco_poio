float GlobalLatitude;
float GlobalLongitude;
int GlobalSatellite;

String receivedData; // Variable for store the received data

void setup() {
  Serial.begin(9600); 
}

void loop() {
  
  if (Serial.available()) {
    char c = Serial.read(); // Read a single character from the serial interface
    receivedData += c; // Add received data to receivedData

    //receivedData = "$GPGGA,145509.000,4813.4212,N,01622.9795,E,6,2,99.99,251.0,M,43.4,M,,*6E";
    // Check if a complete Data-set is received 
    if (c == '\n') {
      // Check if its a GPGGA Data
      if (receivedData.startsWith("$GPGGA")) {

        firstTryToGetVaribalen(receivedData);
        secondTryTosafeGPGGA(receivedData);
        receivedData = "";
      }
      if (receivedData.startsWith("$GPGSA")) {
        // check 2D or 3D Mode
      }
    }

  }
}

// should safe Latitude, Longitude, Satellite Anzahl 
void firstTryToGetVaribalen(String gpggaStr ){

        // Latitude 
        String latitudeStr = gpggaStr.substring(18, 28);
        float latitude = latitudeStr.toFloat(); 
        // Longitude
        String longitudeStr = gpggaStr.substring(30, 42);
        float longitude = longitudeStr.toFloat(); 
        // Satellite
        String satelliteStr = gpggaStr.substring(45, 46);
        int satellite = satelliteStr.toInt(); 

        // safe variable global
        float GlobalLatitude = latitude;
        float GlobalLongitude = longitude;
        int GlobalSatellite = satellite;

        // serial Monitor
        Serial.print("Latitude: ");
        Serial.println(latitude, 4);

        Serial.print("Longitude: ");
        Serial.println(longitude, 4);

        Serial.print("Satellite: ");
        Serial.println(satellite); 
}

//  safe all Elemenets of the String in an Array
void secondTryTosafeGPGGA(String gpggaStr){
 
  
  String gpggaArray[15]; // Array zur Speicherung der Elemente

  int startIndex = 0;
  int elementIndex = 0;
  int commaIndex;

  // Schleife, um die Elemente zu extrahieren
  while ((commaIndex = gpggaStr.indexOf(',', startIndex)) != -1) {
    String element = gpggaStr.substring(startIndex, commaIndex); // Extrahiere das Element
    gpggaArray[elementIndex] = element; // Speichere das Element im Array
    elementIndex++;

    startIndex = commaIndex + 1;
  }

  // Letztes Element speichern
  String lastElement = gpggaStr.substring(startIndex); 
  gpggaArray[elementIndex] = lastElement;

  // Ausgabe der gespeicherten Elemente
  for (int i = 0; i < 15; i++) {
    Serial.print("Element ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(gpggaArray[i]);
  }



}






// Ohne Serial read
//________________________________________________________________________________________________//



float GlobalLatitude;
float GlobalLongitude;
int GlobalSatellite;

String receivedData; // Variable for store the received data

void setup() {
  Serial.begin(9600); 
}

void loop() {
  
receivedData = "$GPGGA,145509.000,4813.4212,N,01622.9795,E,6,2,99.99,251.0,M,43.4,M,,*6E";

      if (receivedData.startsWith("$GPGGA")) {

        firstTryToGetVaribalen(receivedData);
        secondTryTosafeGPGGA(receivedData);
        receivedData = "";


  }
}

// should safe Latitude, Longitude, Satellite Anzahl 
void firstTryToGetVaribalen(String gpggaStr ){

        // Latitude 
        String latitudeStr = gpggaStr.substring(18, 27);
        float latitude = latitudeStr.toFloat(); 
        // Longitude
        String longitudeStr = gpggaStr.substring(30, 42);
        float longitude = longitudeStr.toFloat(); 
        // Satellite
        String satelliteStr = gpggaStr.substring(45, 46);
        int satellite = satelliteStr.toInt(); 

        // safe variable global
        float GlobalLatitude = latitude;
        float GlobalLongitude = longitude;
        int GlobalSatellite = satellite;

        // serial Monitor
        Serial.print("Latitude: ");
        Serial.println(latitude, 4); //  6 Dezimalstellen begrenzen

        Serial.print("Longitude: ");
        Serial.println(longitude, 4); // 6 Dezimalstellen begrenzen

        Serial.print("Satellite: ");
        Serial.println(satellite); 

}

// should safe all Elemenets out of the String 
void secondTryTosafeGPGGA(String gpggaStr){
 
  
  String gpggaArray[15]; // Array zur Speicherung der Elemente

  int startIndex = 0;
  int elementIndex = 0;
  int commaIndex;

  // Schleife, um die Elemente zu extrahieren
  while ((commaIndex = gpggaStr.indexOf(',', startIndex)) != -1) {
    String element = gpggaStr.substring(startIndex, commaIndex); // Extrahiere das Element
    gpggaArray[elementIndex] = element; // Speichere das Element im Array
    elementIndex++;

    startIndex = commaIndex + 1;
  }

  // Letztes Element speichern
  String lastElement = gpggaStr.substring(startIndex); 
  gpggaArray[elementIndex] = lastElement;

  // Ausgabe der gespeicherten Elemente
  for (int i = 0; i < 15; i++) {
    Serial.print("Element ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(gpggaArray[i]);
  }

}

