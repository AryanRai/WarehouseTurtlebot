// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: priority_queue.hpp
// Author(s): Aryan Rai
//
// Description: generic priority queue implementation.

#ifndef PRIORITY_QUEUE_HPP
#define PRIORITY_QUEUE_HPP

#include <functional>
#include <queue>
#include <utility>
#include <vector>

template <typename T> class PriorityQueue
{
    public:
        void put(const T &item, double priority)
        {
            pq_.push(std::make_pair(priority, item));
        }

        T get()
        {
            T item = pq_.top().second;
            pq_.pop();
            return item;
        }

        bool empty() const { return pq_.empty(); }

        size_t size() const { return pq_.size(); }

    private:
        std::priority_queue<std::pair<double, T>,
                            std::vector<std::pair<double, T>>,
                            std::greater<std::pair<double, T>>>
            pq_;
};

#endif // PRIORITY_QUEUE_HPP