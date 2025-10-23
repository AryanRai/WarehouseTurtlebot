// MTRX3760 2025 Project 2: Warehouse Robot 
// File: priority_queue.hpp
// Author(s): Aryan Rai
//
// Priority queue implementation for A* pathfinding

#ifndef PRIORITY_QUEUE_HPP
#define PRIORITY_QUEUE_HPP

#include "slam_types.hpp"
#include <queue>
#include <vector>

namespace slam {

class PriorityQueue {
public:
    PriorityQueue() = default;
    
    void put(const GridCell& item, double priority);
    GridCell get();
    bool empty() const;
    size_t size() const;

private:
    struct QueueItem {
        GridCell cell;
        double priority;
        
        QueueItem(const GridCell& c, double p) : cell(c), priority(p) {}
        
        // For min-heap (lower priority values have higher precedence)
        bool operator>(const QueueItem& other) const {
            return priority > other.priority;
        }
    };
    
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue_;
};

} // namespace slam

#endif // PRIORITY_QUEUE_HPP