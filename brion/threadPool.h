/* Copyright (c) 2016-2017, Mohamed-Ghaith Kaabi <mohamedghaith.kaabi@gmail.com>
 *
 * This file is part of LunchBox
 *  https://github.com/Eyescale/Lunchbox/blob/master/lunchbox/threadPool.h
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

#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <condition_variable> // member
#include <functional>
#include <future> // inline return value
#include <queue>  // member
#include <thread> // member
#include <vector> // member

namespace brion
{
class ThreadPool
{
public:
    static ThreadPool& getInstance();

    ThreadPool(const size_t size);
    ~ThreadPool();

    size_t getSize() const;

    template <typename F>
    inline std::future<typename std::result_of<F()>::type> post(F&& f);

    template <typename F>
    inline void postDetached(F&& f);

    /** @return true if there are pending tasks to be executed. */
    bool hasPendingJobs() const;

private:
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    void joinAll();
    void work();

    std::vector<std::thread> _threads;
    std::queue<std::function<void()> > _tasks;
    mutable std::mutex _mutex;
    std::condition_variable _condition;
    bool _stop;
};

template <typename F>
std::future<typename std::result_of<F()>::type> ThreadPool::post(F&& f)
{
    using ReturnType = typename std::result_of<F()>::type;

    auto task =
        std::make_shared<std::packaged_task<ReturnType()> >(std::forward<F>(f));

    auto res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _tasks.emplace([task]() { (*task)(); });
    }
    _condition.notify_one();
    return res;
}

template <typename F>
void ThreadPool::postDetached(F&& f)
{
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _tasks.emplace(f);
    }
    _condition.notify_one();
}
}

#endif // THREADPOOL_H
