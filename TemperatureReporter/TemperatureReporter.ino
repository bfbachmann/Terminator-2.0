#include <Wire.h>

char command = '\n';

void setup() {
	Wire.begin(8);
	Wire.onRequest(handleRequest);
	Wire.onReceive(acceptCommand);
	
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
}

void loop() {
	delay(100);
}

void handleRequest() {		
	if (command == 't') {
		// digitalWrite(13, HIGH);
		Wire.write(analogRead(0));
	}
	
	command = '\n';
}

void acceptCommand(int bytes) {
	digitalWrite(13, HIGH);
	while (Wire.available() > 0) {
		command = Wire.read();
	}
}
