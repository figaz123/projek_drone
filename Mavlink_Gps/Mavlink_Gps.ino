#include <mavlink.h>
#include <SoftwareSerial.h>
SoftwareSerial _MavLinkSerial(0, 1); // PIN 9=Telemetry TX->Pixhawk RX, PIN 10=Telemetry RX->Pixhawk TX

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 5;// # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.

int num_hbs_pasados = num_hbs;

void setup() {
  // MAVLink interface start
  _MavLinkSerial.begin(57600);

  Serial.begin(57600);
  Serial.println("MAVLink starting.");
}

void loop() {

  // MAVLink
  /* The default UART header for your MCU */
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_HEXAROTOR;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
  {
    // Record last HB update
    previousMillisMAVLink = currentMillisMAVLink;


    //Mav_Request_Data();
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }

  }

  // Check reception buffer
  comm_receive();
}


void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_POSITION};
  const uint16_t MAVRates[maxStreams] = {0x02};

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(6, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _MavLinkSerial.write(buf, len);
  }
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (_MavLinkSerial.available() > 0) {
    uint8_t c = _MavLinkSerial.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      Serial.print("DEBUG msgid:"); Serial.println(msg.msgid);
      // Handle message
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            mavlink_heartbeat_t variabel;
            Serial.println("PX HB");
            Serial.println("heart beat: ");
            Serial.print("custom mode: ");
            Serial.println(variabel.custom_mode);
            Serial.print("type: ");
            Serial.println(variabel.type);
            Serial.print("autopilot: ");
            Serial.println(variabel.autopilot);
            Serial.print("base_mode: ");
            Serial.println(variabel.base_mode);
            Serial.print("mavlink version: ");
            Serial.println(variabel.mavlink_version);
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t datagps;
            mavlink_msg_gps_raw_int_decode (&msg, &datagps);
            //Serial.println("PX HB");
            Serial.println("GPS Data ");
            //Serial.print("time usec: ");
            //Serial.println(datagps.time_usec);
            Serial.print("lat: ");
            Serial.println(datagps.lat);
            Serial.print("lon: ");
            Serial.println(datagps.lon);
            Serial.print("alt: ");
            Serial.println(datagps.alt);
            Serial.print("Sattelite visible: ");
            Serial.println(datagps.satellites_visible);
            //Serial.println(datagps.eph);
            //Serial.println(datagps.epv);
            }
          break;
          
/*          case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
          {
            mavlink_gps_global_origin_t datagps;
            mavlink_msg_gps_global_origin_decode (&msg, &datagps);
            //Serial.println("PX HB");
            Serial.println("GPS Data ");
            //Serial.print("time usec: ");
            //Serial.println(datagps.time_usec);
            Serial.print("lat: ");
            Serial.println(datagps.latitude);
            Serial.print("lon: ");
            Serial.println(datagps.longitude);
            Serial.print("alt: ");
            Serial.println(datagps.altitude);

            }
          break;
        */
      }
    }
  }
} 
