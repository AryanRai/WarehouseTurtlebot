// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: DeliveryStructures.hpp
// Author(s): Aryan Rai
//
// Description: Data structures for delivery robot operations.

#ifndef DELIVERY_STRUCTURES_HPP
#define DELIVERY_STRUCTURES_HPP

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

class DeliveryData
{
    public:
        // DeliveryZone - Represents a delivery zone in the warehouse
        struct DeliveryZone
        {
                std::string name;
                geometry_msgs::msg::Point position;
                std::string description;

                DeliveryZone() = default;
                DeliveryZone(const std::string &n,
                             const geometry_msgs::msg::Point &p,
                             const std::string &d = "")
                    : name(n), position(p), description(d)
                {
                }
        };

        // DeliveryRequest - Request to deliver an item from one zone to another
        struct DeliveryRequest
        {
            std::string id;
            std::string from_zone;
            std::string to_zone;
            std::string item_description;
            int priority; // 1 = highest, 5 = lowest
            bool completed;

            DeliveryRequest() : priority(3), completed(false) {}
        };

        // DeliveryRecord - Log entry for completed delivery
        struct DeliveryRecord
        {
            std::string timestamp;
            std::string request_id;
            std::string from_zone;
            std::string to_zone;
            double distance_traveled;
            double time_taken_seconds;
            bool success;
            std::string notes;

            std::string toLogString() const
            {
                return timestamp + "," + request_id + "," + from_zone +
                        "," + to_zone + "," +
                        std::to_string(distance_traveled) + "," +
                        std::to_string(time_taken_seconds) + "," +
                        (success ? "SUCCESS" : "FAILED") + "," + notes;
            }
        };
};

#endif // DELIVERY_STRUCTURES_HPP