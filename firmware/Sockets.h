WebSocketsServer webSocket = WebSocketsServer(8080);

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
	switch(type) {

		case WStype_BIN:{
			setMotor(*(payload), *(payload+1));
			setSteer(*(payload+2), *(payload+3));			
			break;
		}
			
		default:
			break;
	}
}

void initSockets(){	
	webSocket.begin();
	webSocket.onEvent(onWebSocketEvent);
}

void updateSockets(){
	webSocket.loop();
}

void stopSockets(){	
	webSocket.close();
}
