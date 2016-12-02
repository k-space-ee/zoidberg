// enable debug messages on uart0
//#define DEBUG_SET

// packet timeout in ms
#define PACKET_TIMEOUT 100

#define MOTOR1_PWM 5
#define MOTOR1_ENABLE 4
#define MOTOR1_DIRECTION 3


#define MOTOR2_PWM 11
#define MOTOR2_ENABLE 10
#define MOTOR2_DIRECTION 9

#define MOTOR3_PWM 6
#define MOTOR3_ENABLE 7
#define MOTOR3_DIRECTION 8

#define GRABBER_SENSOR 13
#define GRABBER_MOTOR 12
#define KICKER 17

#define KICKER_RECHARGE_TIME 500
#define KICKER_DISCHARGE_TIME 100

#ifdef DEBUG_SET
#define DEBUG(x) Serial.print (x)
#define DEBUGF(x, f) Serial.print (x, f)
#endif
#ifndef DEBUG_SET
#define DEBUG(x)
#define DEBUGF(x, f)
#endif

#define PSIZE 6
uint8_t packet[PSIZE-2] = {0};
uint8_t pcount = 0;
uint8_t checksum = 0;
bool pready = false;
uint32_t lastpt = 0;
uint32_t lastkick = 0;

void setup()
{
	Serial.begin(9600);
	for (int i=2; i < 18; i++) {
		pinMode(i, OUTPUT);
	}
	pinMode(GRABBER_SENSOR, INPUT);
	lastpt = millis();
	lastkick = millis();
}

void kick() {
	if (lastkick + KICKER_RECHARGE_TIME < millis()) {
		lastkick = millis();
		digitalWrite(KICKER, 1);
	}

}

void loop()
{
	if (pready) {
		pready = false;
		lastpt = millis();
		//Serial.println("READY!");
		/*for (int i; i < PSIZE-2; i++) {
			Serial.print(packet[i], HEX);
			Serial.print(" ");
		}
		Serial.println();*/
		analogWrite(MOTOR1_PWM, packet[0]);
		analogWrite(MOTOR2_PWM, packet[1]);
		analogWrite(MOTOR3_PWM, packet[2]);
		digitalWrite(MOTOR1_ENABLE, bitRead(packet[3], 0));
		digitalWrite(MOTOR2_ENABLE, bitRead(packet[3], 1));
		digitalWrite(MOTOR3_ENABLE, bitRead(packet[3], 2));
		digitalWrite(MOTOR1_DIRECTION, bitRead(packet[3], 3));
		digitalWrite(MOTOR2_DIRECTION, bitRead(packet[3], 4));
		digitalWrite(MOTOR3_DIRECTION, bitRead(packet[3], 5));
		digitalWrite(GRABBER_MOTOR, bitRead(packet[3], 6));
		if (bitRead(packet[3], 7)) {
			kick();
		}
	}

	//
	if (lastkick + KICKER_DISCHARGE_TIME < millis()) {
		digitalWrite(KICKER, 0);
	}

	// Reset everything on packet timeout
	if (millis() - lastpt > PACKET_TIMEOUT) {
		checksum = 0;
		pcount = 0;
		pready = false;
		analogWrite(MOTOR1_PWM, 0);
		analogWrite(MOTOR2_PWM, 0);
		analogWrite(MOTOR3_PWM, 0);
		digitalWrite(MOTOR1_ENABLE, 0);
		digitalWrite(MOTOR2_ENABLE, 0);
		digitalWrite(MOTOR3_ENABLE, 0);
		digitalWrite(MOTOR1_DIRECTION, 0);
		digitalWrite(MOTOR2_DIRECTION, 0);
		digitalWrite(MOTOR3_DIRECTION, 0);
		digitalWrite(GRABBER_MOTOR, 0);
		digitalWrite(KICKER, 0);
	}

}


void serialEvent(){
	while(Serial.available()) {
		uint8_t data = Serial.read();
		// find start of a packet with two 0xAA bytes
        if (data == 0xAA && pcount == 0) {
			DEBUG("DATA 1\r\n");
			pcount++;
			return;
        } else if (data == 0xAA && pcount == 1) {
			DEBUG("DATA 2\r\n");
			pcount++;
			return;
        }

		if (pcount > 1 && pcount < PSIZE) {
            packet[pcount-2] = data;
            DEBUGF(data, HEX);
			DEBUG(" ");
			DEBUG(pcount);
			DEBUG("\r\n");
            checksum += data;
			pcount++;
            return;
        } else if (pcount == PSIZE){
            if (checksum == data) {
                DEBUG("Checksum OK ");
				DEBUGF(checksum, HEX);
				DEBUG("\r\n");
                checksum = 0;
				pcount = 0;
				pready = true;
                return;
            } else {
                DEBUG("Checksum ERR ");
				DEBUGF(checksum, HEX);
				DEBUG(" ");
				DEBUGF(data, HEX);
				DEBUG("\r\n");
                checksum = 0;
				pcount = 0;
                return;
            }
        } else {
            DEBUG("Packet ERR\r\n");
            checksum = 0;
			pcount = 0;
            return;
        }
	}
}
