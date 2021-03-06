/* Copyright (c) 2016-2017, Mohamed-Ghaith Kaabi <mohamedghaith.kaabi@gmail.com>
 *
 * This file is part of LunchBox
 *  https://github.com/Eyescale/Lunchbox/blob/master/lunchbox/threadPool.cpp
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "threadPool.h"

namespace brion
{
ThreadPool& ThreadPool::getInstance()
{
    static ThreadPool pool(std::thread::hardware_concurrency());
    return pool;
}

ThreadPool::ThreadPool(const size_t size)
    : _stop(false)
{
    for (size_t i = 0; i < size; ++i)
        _threads.emplace_back([this] { this->work(); });
}

ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _stop = true;
        _condition.notify_all();
    }
    joinAll();
}

size_t ThreadPool::getSize() const
{
    return _threads.size();
}

bool ThreadPool::hasPendingJobs() const
{
    std::unique_lock<std::mutex> lock(_mutex);
    return !_tasks.empty();
}

void ThreadPool::joinAll()
{
    for (auto& thread : _threads)
        thread.join();
}

void ThreadPool::work()
{
    for (;;)
    {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _condition.wait(lock, [this] { return _stop || !_tasks.empty(); });
            if (_stop)
                return;
            task = std::move(_tasks.front());
            _tasks.pop();
        }
        task();
    }
}
}
