/*********
 * DCCWebServer.cpp
*********/

// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "RTAX999";
//const char* ssid = "RTAX999_EXT";
//const char* password = "LiDoDa[959285]";
//const char* password = "LiDoDa(959285)";
//const char* password = "LiDoDa#959285$";

//const char* ssid     = "HUAWEI P30";
//const char* password = "6381bf07b666";

//const char* ssid     = "ASUS_98_2G";
const char* password = "LiDoDa#959285$";


// Set web server port number to 80
WiFiServer server(80);


// Current time
unsigned long currentTime = 0;
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setupDCCWebServer() {
//    Serial.begin(115200);
    // Initialize the output variables as outputs
//    pinMode(output26, OUTPUT);
//    pinMode(output27, OUTPUT);
    // Set outputs to LOW
//    digitalWrite(output26, LOW);
//    digitalWrite(output27, LOW);

    // Connect to Wi-Fi network with SSID and password
    Serial.print("Esp32 DCC controller Simple V1.0 ");

    Serial.print("Connecting to ");
    Serial.println(ssid);
    Serial.println(password);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // Print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    currentTime = millis();

    server.begin();
}

void loopDCCWebServer(std::string& lastWebCommand) {
    WiFiClient client = server.available();   // listen for incoming clients

    if (client) {                             // if you get a client,
        Serial.println("#New Client.");           // print a message out the serial port
        String currentLine = "";                // make a String to hold incoming data from the client
        String commandLine = "";                // make a String to hold command
        while (client.connected()) {            // loop while the client's connected
            if (client.available()) {             // if there's bytes to read from the client,
                char c = client.read();             // read a byte, then
                Serial.write(c);                    // print it out the serial monitor
                if (c == '\n') {                    // if the byte is a newline character
                    if (commandLine.length() == 0) {
                        commandLine = currentLine;
                        Serial.print(F(" #######currentLine:"));
                        Serial.print(currentLine);
                        Serial.print(F(" currentLine length:"));
                        Serial.print(currentLine.length());
                        if (currentLine.length() > 14) {
                            lastWebCommand = currentLine.substring(5, currentLine.length() - 13).c_str();

                            Serial.print(F(" lastWebCommand length:"));
                            Serial.print(lastWebCommand.length());
                            Serial.print(F(" lastWebCommand:"));
                            Serial.println(lastWebCommand.c_str());
                        }
                    }
                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so send a response:
                    if (currentLine.length() == 0) {
                        // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                        // and a content-type so the client knows what's coming, then a blank line:
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println();

                        // the content of the HTTP response follows the header:
                        /*
                        if (ledOn) {
                            client.print("LED=On<br>");
                        }
                        else
                            client.print("LED=Off<br>");
                        client.print("LED <a href=\"/H\">On</a>.<br>");
                        client.print("LED <a href=\"/L\">Off</a>.<br>");
                        */
                        client.print("Command=");
                        client.print(commandLine);
                        client.print("<br>");

                        //GetNodeStatus(client);

                        // The HTTP response ends with another blank line:
                        client.println();
                        // break out of the while loop:
                        break;
                    }
                    else {    // if you got a newline, then clear currentLine:
                        currentLine = "";
                    }
                }
                else if (c != '\r') {  // if you got anything else but a carriage return character,
                    currentLine += c;      // add it to the end of the currentLine
                }

                /*
                if (currentLine.endsWith("GET /H")) {
                    digitalWrite(LED, HIGH);               
                    ledOn = true;
                }
                if (currentLine.endsWith("GET /L")) {
                    digitalWrite(LED, LOW);                
                    ledOn = false;
                }
                */
            }
        }
        // close the connection:
        delay(10);
        client.stop();
        Serial.println("#Stop Client.");
    }
}
