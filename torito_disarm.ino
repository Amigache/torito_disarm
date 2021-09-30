#include <mavlink.h>
#include <SoftwareSerial.h>

#define DEBUG

// Aircraft identifier
#define AIRCRAFT_SYSTEM_ID      1   // Value of SYSID_THISMAV param of your aircraft
#define AIRCRAFT_COMPONENT_ID   0   //

 // Arduino identifier
#define ARDUINO_SYSTEM_ID       2   //
#define ARDUINO_COMPONENT_ID    200 //

// Cond values
#define COND_ALTITUDE 200000 //cm

SoftwareSerial softSerial(9,10);   // RX, TX

/* STREAMS that can be requested
 * MAV_DATA_STREAM_ALL=0, // Enable all data streams
 * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
 * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
 * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
 * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
 * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
 * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
 * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
 * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
 * MAV_DATA_STREAM_ENUM_END=13,
 * 
 * Data in PixHawk available in:
 *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
 *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
 */
const uint8_t mav_streams[] = {MAV_DATA_STREAM_ALL, MAV_DATA_STREAM_POSITION,MAV_DATA_STREAM_EXTRA3};
const uint16_t mav_rates[] = {0x00,0x02,0x01};
const uint8_t n_streams =  sizeof(mav_streams)/sizeof(mav_streams[0]);

// Mavlink variables
uint32_t previous_millis = 0;     
uint32_t millis_interval = 1000; 
const int max_request_freq = 60;      // Request period = millis_interval x max_request_freq milliseconds
int request_count = max_request_freq;

// Flight modes
enum flightModes {
  FLIGHT_MODE_AUTO  = 10,
  FLIGHT_MODE_RTL   = 11,
  FLIGHT_MODE_QLAND = 20,
  FLIGHT_MODE_QRTL  = 21
};

// Condition vars
uint8_t cond_mode  = 0;
uint8_t cond_armed = 0; 
uint8_t cond_alt = 0;

// Handled messages
mavlink_heartbeat_t heartbeat;
mavlink_global_position_int_t global_position;
//mavlink_hil_controls_t hil_controls;
//mavlink_attitude_t attitude;
      
void setup() {
  // MAVLink interface start
  softSerial.begin(57600);
  // Debug Serial 
  Serial.begin(57600);
  Serial.println("MAVLink starting.");
}

void loop() {
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(ARDUINO_SYSTEM_ID, ARDUINO_COMPONENT_ID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
 
  // Fill buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // send/recieve messages every millis_interval milliseconds
  uint32_t current_millis = millis();
  if (current_millis - previous_millis >= millis_interval) {
    previous_millis = current_millis;
    // Send buffer (heratbeat of this arduino)
    softSerial.write(buf,len);

    // Request streams from flight controller
    request_count++;
    if(request_count >= max_request_freq) {
      // Request streams from flight controller
      Serial.println("Streams requested!");
      request_mavlink_messages();
      request_count = 0;
    }
    
  }

  // Recieve telemtry
  parse_mavlink_telemetry();

  // Check disarm condition
  if(cond_armed == 1 && cond_mode == 1 && cond_alt == 1) {
    //Disarm
    Serial.println("Disarm");
    disarm(0);
    cond_armed = 0;
  }
}

void request_mavlink_messages()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  for (int i=0; i < n_streams; i++) {
    mavlink_msg_request_data_stream_pack(ARDUINO_SYSTEM_ID, ARDUINO_COMPONENT_ID, &msg, AIRCRAFT_SYSTEM_ID, AIRCRAFT_COMPONENT_ID, mav_streams[i], mav_rates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    softSerial.write(buf,len);
  }
  
}

void parse_mavlink_telemetry() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(softSerial.available()>0) {
    uint8_t c = softSerial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);

            if(msg.sysid == AIRCRAFT_SYSTEM_ID) {
              cond_mode = (heartbeat.custom_mode ==  FLIGHT_MODE_AUTO || heartbeat.custom_mode == FLIGHT_MODE_RTL || heartbeat.custom_mode == FLIGHT_MODE_QLAND || heartbeat.custom_mode == FLIGHT_MODE_QRTL)?1:0;
              cond_armed = bitRead(heartbeat.base_mode, 7);

            
#ifdef DEBUG
              Serial.print("systemid: ");
              Serial.print(msg.sysid);
              Serial.print(" base_mode: ");
              Serial.print(heartbeat.base_mode);
              Serial.print(" cond_armed: ");
              Serial.print(cond_armed);
              Serial.print(" custom_mode: ");
              Serial.print(heartbeat.custom_mode);
              Serial.print(" cond_mode: ");
              Serial.print(cond_mode);
              Serial.print(" altitude (cm): ");
              Serial.print(global_position.alt);
              Serial.print(" cond_alt:");
              Serial.println(cond_alt);
#endif
            }
          }
          break;

//          case  132: // #132: MAVLINK_MSG_ID_DISTANCE_SENSOR:
//          {
//            Serial.println("####");
//          }
//          break;

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
              mavlink_msg_global_position_int_decode(&msg, &global_position);
              cond_alt = (global_position.alt < COND_ALTITUDE)?1:0;            
          }
          break;
//
//        case MAVLINK_MSG_ID_HIL_CONTROLS:  
//          {
//        
//            mavlink_msg_hil_controls_decode(&msg, &hil_controls);
//#ifdef DEBUG
//            Serial.print("mode: ");
//            Serial.println(hil_controls.mode);
//#endif
//          }
//          break;


//        case MAVLINK_MSG_ID_ATTITUDE:  // #30
//          {
//            /* Message decoding: PRIMITIVE
//             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
//             */
//            
//            mavlink_msg_attitude_decode(&msg, &attitude);
//#ifdef DEBUG
//
//#endif
//          }
//          break;

        
       default:
//#ifdef DEBUG
//          Serial.print("--- Otros: ");
//          Serial.print("sysid: ");
//          Serial.print(msg.sysid);
//          Serial.print(" [ID: ");
//          Serial.print(msg.msgid);
//          Serial.print("], [seq: ");
//          Serial.print(msg.seq);
//          Serial.println("]");
//#endif
          break;
      }
    }
  }
}

int disarm(int state)
{
  // Format the data:
  mavlink_command_long_t armed = {0};
  armed.target_system = AIRCRAFT_SYSTEM_ID;
  armed.target_component = AIRCRAFT_COMPONENT_ID;
  armed.command = MAV_CMD_COMPONENT_ARM_DISARM; //400
  armed.confirmation = true;
  armed.param1 = (int) state;
  
  // Encode:
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_encode(ARDUINO_SYSTEM_ID, ARDUINO_COMPONENT_ID, &msg, &armed);
    
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  softSerial.write(buf,len);
}
