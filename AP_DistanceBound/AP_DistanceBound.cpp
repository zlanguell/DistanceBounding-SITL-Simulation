/*
When our drone receives an ADSB message, it should create an instance of this class on a separate thread.

It needs our information (verifier), the other craft information (prover), the ADSB Message,
and the Crypto-system information

Logs using rapidjson
*/

#include "AP_DistanceBound.h"
#include <AP_Common/AP_Common.h>
#include <AP_ADSB/AP_ADSB.h>
#include "SITL/SIM_Aircraft.h"
#include "SITL/SIM_ADSB.h"
#include "SITL/SITL.h"
#include <stdio.h>
#include <string>
#include <sstream>
#include <bitset>
#include <fstream>
#include <ctime>
#include <cmath>


//Json includes
#include "/home/zachary/Desktop/ardupilot/Tools/RapidJSON/rapidjson/include/rapidjson/stringbuffer.h"
#include "/home/zachary/Desktop/ardupilot/Tools/RapidJSON/rapidjson/include/rapidjson/prettywriter.h"


//Constructor
AP_DistanceBound::AP_DistanceBound(AP_ADSB::adsb_vehicle_t new_vehicle)
{
    _sitl = AP::sitl();
    _adsb_msg_time = AP_HAL::micros();
    srand(time(NULL));

    //Populate security parameters to send to prover
    _sec_params.n = rounds_per_challenge;
    _sec_params.r = num_of_challenges;
    _sec_params.k = secret_key_size;
    _sec_params.s = nonce_size;

    //Get sim vehicles from sitl ONLY NEEDED FOR SIMULATION VEHICLES
    if (_sitl != NULL) {
        SITL::ADSB_Vehicle* vehicle_list = (SITL::ADSB_Vehicle*) _sitl->adsb_vehicle_list;
        uint8_t num_vehicles = _sitl->adsb_plane_count;
        for (int i = 0; i < num_vehicles; i++) {
            if (new_vehicle.info.ICAO_address == vehicle_list[i].get_ICAO_address()) {
                _other_vehicle = vehicle_list[i];
            }
        }

        //Extract vehicle configuration for simulation
        processing_time = _other_vehicle.my_processing_time;
        tp_noise_error = _other_vehicle.errorMax;
        max_wait_time = _other_vehicle.wTime;
        adsb_velocity_error = _other_vehicle.velocity_error;
        adsb_position_error = _other_vehicle.position_error;
    }
    else {   // Not a simulation
        processing_time = 0;
        tp_noise_error = 0;
        max_wait_time = 0;
        adsb_velocity_error = 0;
        adsb_position_error = 0;
    }
}

void AP_DistanceBound::setup(AP_ADSB::adsb_vehicle_t new_vehicle)
{
    _vehicle = new_vehicle;

    //get our location
    if (!AP::ahrs().get_position(_my_loc)) {
        _my_loc.zero();
    }

    //get other vehicle location and time
    Location_Class vehicle_loc = Location_Class(
                                     _vehicle.info.lat,
                                     _vehicle.info.lon,
                                     _vehicle.info.altitude * 0.1f,
                                     Location_Class::ALT_FRAME_ABSOLUTE);


    //Determine max processing time based on distance
    float my_loc_distance_to_vehicle = _my_loc.get_distance(vehicle_loc);
    _estimated_distance[0] = my_loc_distance_to_vehicle; //Saved for later

    //Generate random wait times between rounds in microseconds (us)
    for (int i = 0; i < num_of_challenges; i++) {
        _random_wait_times[i] = max_wait_time;
    }

    //Send security parameters to other vehicle
    _other_vehicle.set_security_parameters(_sec_params.n, _sec_params.r, _sec_params.k, _sec_params.s, my_loc_distance_to_vehicle, _random_wait_times);

    // Get Nonce and Generate R0/R1
    random_hash_function(_S, _S, _K);

    //Generate the challenges
    for (int i = 0; i < (total_rounds); i++) {
        _challenges[i] = rand() % 2;
    }

    //Get the start time and start the fast cycle
    _fast_round_start_time = AP_HAL::micros();
    distanceBounding();

    //Determine if successful and log results
    finalize();
}

void AP_DistanceBound::distanceBounding()
{
    std::bitset<1> current_challenge;
    for (int i = 0; i < num_of_challenges; i++) {
        _round_start_time[i] = AP_HAL::micros();
        for (int j = 0; j < rounds_per_challenge; j++) {
            current_challenge[0] = _challenges[(i*rounds_per_challenge) + j];

            SITL::ADSB_Vehicle::prover_response p_response = _other_vehicle.challenge(current_challenge);
            _responses[(i*rounds_per_challenge) + j] = p_response.bit_response[0];
            _timestamps_for_responses[(i*rounds_per_challenge) + j] = p_response.time_response_us;
        }
    }
}

void AP_DistanceBound::finalize()
{
    double sum_of_challenges[num_of_challenges] = {0};
    double estimated_tp[num_of_challenges]= {0};
    double calculated_distance[num_of_challenges]= {0};
    double wrong_answers[num_of_challenges] = {0};

    //Generate correct answers and calculate average times for each challenge
    std::bitset<total_rounds> correct_answers;
    int wrong_answer_count = 0;
    int current_round = 0;
    double averageTP = 0;
    double estimated_mean_tp = 0;
    //double estimated_dev_tp = 0;

    //Predicted future locations from ADSB message
    uint32_t time_between_msg_strt = 0; //(fast_round_start_time * 1.0e-3f) - adsb_msg_time;
    _estimated_distance[0] += distance_after_time(_vehicle, ((_random_wait_times[0] + time_between_msg_strt) * 1.0e-6));
    for (int i = 1; i < num_of_challenges; i++) {
        _estimated_distance[i] = _estimated_distance[i-1] + distance_after_time(_vehicle, _random_wait_times[i] * 1.0e-6);
    }

    //main loop for checking correctness and adding values
    for (int i = 0; i < total_rounds; i++) {

        current_round = i / rounds_per_challenge;

        //set the correct answer
        if (!_challenges.test(i)) {
            correct_answers[i] = _R0[i];
        }
        else {
            correct_answers[i] = _R1[i];
        }

        double time_for_distance = (1.0e6 * ((2 * abs(_estimated_distance[current_round])) / c_ms));

        //check for correct answer
        if (correct_answers[i] == _responses[i]) {
            sum_of_challenges[current_round] += _timestamps_for_responses[i];
            estimated_tp[current_round] += (_timestamps_for_responses[i] -  time_for_distance);
        }
        else {
            wrong_answer_count++;
            wrong_answers[current_round] += 1;
        }
    }//end checking loop

    if (wrong_answer_count < fail_threshold) {
        for (int i = 0; i < num_of_challenges; i++) {
            sum_of_challenges[i] = sum_of_challenges[i] / (rounds_per_challenge - wrong_answers[i]);
            estimated_tp[i] = estimated_tp[i] / (rounds_per_challenge - wrong_answers[i]);
            averageTP += estimated_tp[i];
        }
        averageTP = averageTP/num_of_challenges;

        //Calculate Estimated Mean/StDev TP
        estimated_mean_tp = (tp_noise_error/2) + processing_time;
        double sum_adsb_error = 0.0;
        for (int i = 0; i < num_of_challenges; i++) {
            sum_adsb_error += (adsb_position_error * adsb_position_error) + ((adsb_velocity_error * adsb_velocity_error) * (_round_start_time[i] - _adsb_msg_time));
        }
        sum_adsb_error *= ((4)/((num_of_challenges*num_of_challenges) * (c_ms*c_ms)));
        //estimated_dev_tp = sqrt(((tp_noise_error * tp_noise_error)/(12*total_rounds)) + sum_adsb_error);
        //tp_upperbound = estimated_mean_tp + (tp_stdev_limit * estimated_dev_tp);
        tp_upperbound = estimated_mean_tp + 0.02; //adds 20ns

        //Calculated Distances that we make from distance bounding
        for (int i = 0; i < num_of_challenges; i++) {
            calculated_distance[i] = (((c_ms / 2))  * ((sum_of_challenges[i] - averageTP) *1.0e-6));
        }
    }
    else {   // Failed Threshold

        sum_of_challenges[0] = 9999999;
        sum_of_challenges[1] = 9999999;
        sum_of_challenges[2] = 9999999;
    }

    //Log results
    if (LOGGING_TO_FILE) {
        createJsonLog(calculated_distance, averageTP);
    }
}

double AP_DistanceBound::distance_after_time(AP_ADSB::adsb_vehicle_t vehi, float time_passed_s)
{
    float velx = (vehi.info.hor_velocity * 1.0e-2f) * cos((vehi.info.heading * 1.0e-1f) / (180 * M_PI));
    float vely = (vehi.info.hor_velocity * 1.0e-2f) * sin((vehi.info.heading * 1.0e-1f) / (180 * M_PI));
    float velz = (vehi.info.ver_velocity * 1.0e-2f);
    double tempx = velx * time_passed_s;
    double tempy = vely * time_passed_s;
    double tempz = velz * time_passed_s;
    return sqrt((tempx * tempx) + (tempy * tempy) + (tempz * tempz));
}

void AP_DistanceBound::random_hash_function(unsigned char* nonceA, unsigned char* nonceB, unsigned char* k)
{
    // load the R sequences with 1's if simulation
    if (_sitl != NULL) {
        for (int i = 0; i < bit_sequence_size; i++) {
            _R0[i] = 1;
            _R1[i] = 1;
        }
    }
}

void AP_DistanceBound::createJsonLog(double *calculated_distance, double averageTimeP)  //rapidjson
{
    std::stringstream ss;
    ss<<_other_vehicle.get_ICAO_address();
    std::string str;
    ss>>str;

    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);
    writer.StartObject();
    writer.Key("ICAO");
    writer.String(str.c_str());
    writer.Key("Processing Time");
    writer.Double(this->processing_time);
    writer.Key("Processing Error");
    writer.Double(this->tp_noise_error);
    writer.Key("Average TP");
    writer.Double(averageTimeP);
    writer.Key("Estimated TP Upperbound");
    writer.Double(this->tp_upperbound);
    writer.Key("Initial Position");
    writer.Double(_other_vehicle.mydistances[0]);
    writer.Key("Velocity");
    writer.StartArray();
    writer.Double(_other_vehicle.db_velocity_ef.x);
    writer.Double(_other_vehicle.db_velocity_ef.y);
    writer.Double(_other_vehicle.db_velocity_ef.z);
    writer.EndArray();
    writer.Key("Actual Distances");
    writer.StartArray();
    for (int i = 0; i < num_of_challenges; i++) {
        writer.Double(_other_vehicle.mydistances[i+1]);
    }
    writer.EndArray();
    writer.Key("Random Wait Times");
    writer.StartArray();
    for (int i = 0; i < num_of_challenges; i++) {
        writer.Uint(_random_wait_times[i]);
    }
    writer.EndArray();
    writer.Key("Error Values");
    writer.StartArray();
    for (int i = 0; i < total_rounds; i++) {
        writer.Double(_other_vehicle.myerrors[i]);
    }
    writer.EndArray();
    writer.Key("Sigmas");
    writer.StartArray();
    for (int i = 0; i < total_rounds; i++) {
        writer.Double(_timestamps_for_responses[i]);
    }
    writer.EndArray();
    writer.Key("Calculated Distances");
    writer.StartArray();
    for (int i = 0; i < num_of_challenges; i++) {
        writer.Double(calculated_distance[i]);
    }
    writer.EndArray();
    writer.Key("Estimated Distances");
    writer.StartArray();
    for (int i = 0; i < num_of_challenges; i++) {
        writer.Double(_estimated_distance[i]);
    }
    writer.EndArray();
    writer.EndObject();

    std::string filepath = _log_path + "/log_" + str + ".json";
    std::ofstream ofs(filepath.c_str(), std::ios_base::out | std::ios_base::app);
    if (!ofs.bad()) {
        ofs << s.GetString() << '\n';
        ofs.close();
    }
    else {
        std::printf("ofs failed");
    }
}
