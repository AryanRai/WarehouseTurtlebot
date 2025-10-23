// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: priority_queue.cpp
// Author(s): Dylan George
//
// Priority queue implementation for A* pathfinding

#include "priority_queue.hpp"

namespace slam {

void PriorityQueue::put(const GridCell& item, double priority) {
    queue_.emplace(item, priority);
}

GridCell PriorityQueue::get() {
    if (queue_.empty()) {
        return {-1, -1}; // Invalid cell
    }
    
    QueueItem top = queue_.top();
    queue_.pop();
    return top.cell;
}

bool PriorityQueue::empty() const {
    return queue_.empty();
}

size_t PriorityQueue::size() const {
    return queue_.size();
}

} // namespace slam