/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ADSB peripheral simulator class
*/

#pragma once

#include <AP_HAL/utility/Socket.h>
#include <bitset>
#include "SIM_Aircraft.h"


namespace SITL {
/*
  a class for individual simulated vehicles
*/
class ADSB_Vehicle {
    friend class ADSB;
public:
  uint32_t get_ICAO_address(){ return this->ICAO_address; }

  void set_security_parameters(uint8_t n, uint8_t r, uint32_t k, uint32_t s, float session_distance, uint32_t* waitTimes);

  struct prover_response{
    std::bitset<1> bit_response;
    double time_response_us;
  };

  struct json_param{
    uint32_t ICAO;
    double maxError;
    double velocity;
    double tp;
    uint32_t waitT;
    double vel_error;
    double pos_error;
  };
  double randnum (double a, double b);
  prover_response challenge(std::bitset<1> challenge_bit);
  json_param getNextVehicle();
  double dist;
  double session_dist;
  uint32_t last_challenge;
  int round_number;
  uint32_t wTime;
  uint32_t* myWaitTimes;
  double* mydistances;
  double my_processing_time;
  double errorMax;
  Vector3f db_velocity_ef;
  double myerrors[48];
  double velocity_error;
  double position_error;
  bool attacker = true;

private:
    void update(float delta_t);

    Vector3f position; // NED from origin
    Vector3f velocity_ef; // NED
    char callsign[9];
    uint32_t ICAO_address;
    bool initialised = false;

    std::string _config_path = "/home/zachary/Desktop/ardupilot/libraries/AP_DistanceBound/Configurations";
    int challenge_count = 16;

    std::bitset<256> R0;
    std::bitset<256> R1;
};

class ADSB {
public:
    ADSB(const struct sitl_fdm &_fdm, const char *home_str);
    void update(void);

private:
    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5762;
    Location home;
    uint8_t num_vehicles = 0;

    static const uint8_t num_vehicles_MAX = 200;
    ADSB_Vehicle vehicles[num_vehicles_MAX];

    // reporting period in ms
    const float reporting_period_ms = 1000;
    uint32_t last_report_us = 0;
    uint32_t last_update_us = 0;
    uint32_t last_tx_report_ms = 0;

    uint32_t last_heartbeat_ms = 0;
    bool seen_heartbeat = false;
    uint8_t vehicle_system_id;
    uint8_t vehicle_component_id;

    SocketAPM mav_socket { false };
    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink {};

    void send_report(void);
};

}  // namespace SITL
