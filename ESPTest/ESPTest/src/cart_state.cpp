#include "cart_state.hpp"
#include "configure.hpp"


typedef struct{
	 uint16_t start;
	 int16_t  steer;
	 int16_t  speed;
	 uint16_t checksum;
} SerialCommand;


String cart_state_to_json(CartState &state){
	return "{\"speed_set\": "+ String(state.speed_set) + 
			", \"speed_actual\": " + String(state.speed_actual) +
			", \"drive_mode\": " + String(state.drive_mode) +
			", \"baby_mode\": " + String(state.baby_mode) +
			", \"axes\": [" +
				axis_state_to_json(state.front_axis) + "," +
				axis_state_to_json(state.rear_axis) +
			"]}";
}


String axis_state_to_json(AxisState &state){
	return "{\"left_rpm\": " + String(state.Feedback.speedL_meas) + 
			", \"right_rpm\": " + String(state.Feedback.speedR_meas) +
			", \"temperature\": " + String(state.Feedback.boardTemp) +
			", \"voltage\": " + String(state.Feedback.batVoltage) +
			"}";
}


AxisState get_axis_state(CartState &state, Axis axis) {
	if (axis == FRONT) {
		return state.front_axis;
	}
	else {
		return state.rear_axis;
	}
}


void read_axis_serial(HardwareSerial &axis_serial, HardwareSerial &debug_serial, AxisState &axis) {
	// Check for new data availability in the Serial buffer
	if (axis_serial.available()) {
		axis.incomingByte = axis_serial.read();																											 // Read the incoming byte
		axis.bufStartFrame = ((uint16_t)(axis.incomingByte) << 8) | axis.incomingBytePrev; // Construct the start frame
	}
	else{
		return;
	}

	// Copy received data
	if (axis.bufStartFrame == START_FRAME)
	{ // Initialize if new data is detected
		axis.p = (byte *)&axis.NewFeedback;
		*(axis.p)++ = axis.incomingBytePrev;
		*(axis.p)++ = axis.incomingByte;
		axis.idx = 2;
	}
	else if (axis.idx >= 2 && axis.idx < sizeof(SerialFeedback))
	{ // Save the new received data
		*(axis.p)++ = axis.incomingByte;
		axis.idx++;
	}

	// Check if we reached the end of the package
	if (axis.idx == sizeof(SerialFeedback))
	{
		uint16_t checksum;
		checksum = (uint16_t)(axis.NewFeedback.start ^ axis.NewFeedback.cmd1 ^ axis.NewFeedback.cmd2 ^ axis.NewFeedback.speedR_meas ^ axis.NewFeedback.speedL_meas ^ axis.NewFeedback.batVoltage ^ axis.NewFeedback.boardTemp ^ axis.NewFeedback.cmdLed);

		// Check validity of the new data
		if (axis.NewFeedback.start == START_FRAME && checksum == axis.NewFeedback.checksum)
		{
			// Copy the new data
			memcpy(&axis.Feedback, &axis.NewFeedback, sizeof(SerialFeedback));
		}
		axis.idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
	}

	// Update previous states
	axis.incomingBytePrev = axis.incomingByte;
}


void send_hover_command(HardwareSerial& hover_serial, int16_t uSpeed, int16_t uSteer) {
	SerialCommand Command;

	Command.start    = (uint16_t) START_FRAME;
	Command.steer    = (int16_t)  uSteer;
	Command.speed    = (int16_t)  uSpeed;
	Command.checksum = (uint16_t) (Command.start ^ Command.steer ^ Command.speed);

	hover_serial.write((uint8_t *) &Command, sizeof(Command)); 
}