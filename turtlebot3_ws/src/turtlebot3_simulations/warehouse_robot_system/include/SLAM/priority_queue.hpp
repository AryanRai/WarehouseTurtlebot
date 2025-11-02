// ============================================================================
// MTRX3760 Project 2 - 
// File: priority_queue.hpp
// Description: Priority queue implementation for A* pathfinding algorithm.
//              Provides efficient priority-based node selection for optimal
//              path planning in warehouse navigation systems.
// Author(s): Dylan George
// Last Edited: 2025-11-02
// ============================================================================

#ifndef PRIORITY_QUEUE_HPP
#define PRIORITY_QUEUE_HPP

#include <queue>
#include <vector>
#include <utility>
#include <functional>

template<typename T>
class PriorityQueue {
public:
    void put(const T& item, double priority) {
        pq_.push(std::make_pair(priority, item));
    }

    T get() {
        T item = pq_.top().second;
        pq_.pop();
        return item;
    }

    bool empty() const {
        return pq_.empty();
    }

    size_t size() const {
        return pq_.size();
    }

private:
    std::priority_queue<
        std::pair<double, T>,
        std::vector<std::pair<double, T>>,
        std::greater<std::pair<double, T>>
    > pq_;
};

#endif // PRIORITY_QUEUE_HPP