// #include <WiFi.h>
// #include "func.h"
// #include "conf.h"
// #include "battery.h"

// // Constants
// const uint16_t groundstation_port = GROUNDSTATION_PORT;
// int16_t system_mode = MAV_MODE_PREFLIGHT;
// int16_t control_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
// const int16_t system_id = 1;
// const int16_t component_id = 1;
// unsigned long previousTimeoutMillis = 0;



// void send_mavlink(mavlink_message_t *mavlink_message, const char *groundstation_host)
// {
//   uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
//   uint16_t mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, mavlink_message);

//   udp.beginPacket(groundstation_host, groundstation_port);
//   udp.write(mavlink_message_buffer, mavlink_message_length);
//   udp.endPacket();

//   Serial.write(mavlink_message_buffer, mavlink_message_length);
// }

// void send_heartbeat(IPAddress targetIP)
// {
//   mavlink_message_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//   uint8_t system_id = 21;
//   uint8_t component_id = 13;
//   uint32_t custom_mode = 0;
//   bool armed = false;

//   uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
//   base_mode |= (armed ? MAV_MODE_MANUAL_ARMED : MAV_MODE_MANUAL_DISARMED);

//   mavlink_msg_heartbeat_pack(
//       system_id,
//       component_id,
//       &msg,
//       MAV_TYPE_QUADROTOR,
//       MAV_AUTOPILOT_GENERIC,
//       base_mode,
//       custom_mode,
//       MAV_STATE_ACTIVE);

//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   sendUDP(buf, len, targetIP);
//   Serial.printf("[INFO] Heartbeat sent to %s\n", targetIP.toString().c_str());
// }

// void send_systemstatus(const char *groundstation_host)
// {
//   mavlink_sys_status_t sys_status;
//   mavlink_message_t mvl_tx_message;

//   sys_status.onboard_control_sensors_present = 0;
//   sys_status.onboard_control_sensors_enabled = 0;
//   sys_status.onboard_control_sensors_health = 0;

//   // generate random values for system status
//   sys_status.load = random(500, 600);
//   // Panasonic 18650: 3.6V nominal, 3.4Ah capacity
//   sys_status.voltage_battery = random(3600, 4200);  // 3.6V-4.2V range
//   sys_status.current_battery = random(0, 340);      // 0-3.4A range
//   sys_status.battery_remaining = random(80, 100);   // 80-100% range

//   // Update battery information
//   updateBatteryInfo(sys_status.voltage_battery, sys_status.current_battery, sys_status.battery_remaining);

//   sys_status.drop_rate_comm = 0;
//   sys_status.errors_comm = 0;
//   sys_status.errors_count1 = 0;
//   sys_status.errors_count2 = 0;
//   sys_status.errors_count3 = 0;
//   sys_status.errors_count4 = 0;

//   mavlink_msg_sys_status_encode(system_id, component_id, &mvl_tx_message, &sys_status);
//   send_mavlink(&mvl_tx_message, groundstation_host);
// }

// void send_radiostatus(const char *groundstation_host)
// {
//   mavlink_radio_status_t radio_status;
//   mavlink_message_t mvl_tx_message;

//   radio_status.remrssi = WiFi.RSSI();
//   radio_status.rssi = WiFi.RSSI();

//   mavlink_msg_radio_status_encode(system_id, component_id, &mvl_tx_message, &radio_status);
//   send_mavlink(&mvl_tx_message, groundstation_host);
// }

// void send_followme(float lat_deg, float lon_deg, float alt_m, IPAddress targetIP)
// {
//   mavlink_message_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//   mavlink_set_position_target_global_int_t target{};
//   target.time_boot_ms = millis();
//   target.target_system = 1;    // Your system ID
//   target.target_component = 0; // Usually 0 = autopilot
//   target.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

//   target.lat_int = lat_deg * 1e7; // Degrees to int
//   target.lon_int = lon_deg * 1e7;
//   target.alt = alt_m;

//   mavlink_msg_set_position_target_global_int_encode(1, 1, &msg, &target);
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//   sendUDP(buf, len, targetIP);
// }

// void send_position(IPAddress targetIP)
// {
//   char ipBuf[16]; // enough for "xxx.xxx.xxx.xxx"

//   targetIP.toString().toCharArray(ipBuf, sizeof(ipBuf));
//   mavlink_global_position_int_t position;
//   mavlink_message_t mvl_tx_message;

//   // generate spoofed coordinates
//   position.time_boot_ms = millis();
//   position.lat = 557515000;
//   position.lon = 376158000;
//   position.alt = 602900;
//   position.hdg = 36000;

//   mavlink_msg_global_position_int_encode(system_id, component_id, &mvl_tx_message, &position);
//   send_mavlink(&mvl_tx_message, ipBuf);
// }

// void handle_message_command_long(mavlink_message_t *msg, const char *groundstation_host)
// {
//   mavlink_command_long_t command_long;
//   mavlink_command_ack_t mvl_command_ack;
//   mavlink_message_t mvl_tx_message;

//   mvl_command_ack.result = MAV_RESULT_FAILED;
//   uint8_t result = MAV_RESULT_UNSUPPORTED;

//   mavlink_msg_command_long_decode(msg, &command_long);

//   switch (command_long.command)
//   {
//   case MAV_CMD_COMPONENT_ARM_DISARM: // LONG CMD ID 400 - https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
//     mvl_command_ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
//     mvl_command_ack.result = MAV_RESULT_ACCEPTED;

//     (command_long.param1 == 1) ? armed = 1 : armed = 0;

//     mavlink_msg_command_ack_encode(system_id, component_id, &mvl_tx_message, &mvl_command_ack);
//     send_mavlink(&mvl_tx_message, groundstation_host);
//     break;

//   default:
//     break;
//   }
// }

// void handle_mission_manual_control(mavlink_message_t *msg)
// {
//   mavlink_manual_control_t manual_control;

//   mavlink_msg_manual_control_decode(msg, &manual_control);
//   // pass &manual_control to motor control logic
// }

// void parse_mavlink(uint8_t parsing_byte, const char *groundstation_host)
// {
//   mavlink_message_t mvl_rx_message;
//   mavlink_status_t status;

//   if (mavlink_parse_char(MAVLINK_COMM_0, parsing_byte, &mvl_rx_message, &status))
//   {
//     switch (mvl_rx_message.msgid)
//     {
//     case MAVLINK_MSG_ID_HEARTBEAT: // MSG ID 1 - https://mavlink.io/en/messages/common.html#HEARTBEAT
//       previousTimeoutMillis = millis();
//       break;

//     case MAVLINK_MSG_ID_MANUAL_CONTROL: // MSG ID 69 - https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
//       handle_mission_manual_control(&mvl_rx_message);
//       break;

//     case MAVLINK_MSG_ID_COMMAND_LONG: // MSG ID 76 - https://mavlink.io/en/messages/common.html#COMMAND_LONG
//       handle_message_command_long(&mvl_rx_message, groundstation_host);
//       break;

//     default:
//       break;
//     }
//   }
// }