#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <tbb/concurrent_priority_queue.h>

void test_push_single(const std::size_t iterations)
{
    std::priority_queue<std::size_t> test_queue;

    std::mutex m;
    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t i = 0 ; i < iterations ; ++i) {
        std::unique_lock<std::mutex> l(m);
        test_queue.push(i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test push: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
}

void test_push_tbb_single(const std::size_t iterations)
{
    tbb::concurrent_priority_queue<std::size_t> test_queue;
    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t i = 0 ; i < iterations ; ++i) {
        test_queue.push(i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test push: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
}

void test_push_dual(const std::size_t iterations)
{
    std::priority_queue<std::size_t> test_queue;
    std::mutex m;

    auto loop = [&test_queue, &m, iterations] () {
        for(std::size_t i = 0 ; i < iterations ; ++i) {
            std::unique_lock<std::mutex> l(m);
            test_queue.push(i);
        }
    };

    auto start = std::chrono::high_resolution_clock::now();
    std::thread t1 = std::thread(loop);
    std::thread t2 = std::thread(loop);
    t1.join();
    t2.join();
    auto end = std::chrono::high_resolution_clock::now();
    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test push: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

//    while(!test_queue.empty()) {
//        std::cout << test_queue.top() << std::endl;
//        test_queue.pop();
//    }


}

void test_push_tbb_dual(const std::size_t iterations)
{
    tbb::concurrent_priority_queue<std::size_t> test_queue;
    auto loop = [&test_queue, iterations] () {
        for(std::size_t i = 0 ; i < iterations ; ++i) {
            test_queue.push(i);
        }
    };

    auto start = std::chrono::high_resolution_clock::now();
    std::thread t1 = std::thread(loop);
    std::thread t2 = std::thread(loop);
    t1.join();
    t2.join();
    auto end = std::chrono::high_resolution_clock::now();
    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test push: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
//    std::size_t item;
//    while(!test_queue.try_pop(item)) {
//        std::cout << item << std::endl;
//    }
}



int main(int argc, char *argv[])
{

    const std::size_t iterations = 1000000;
    test_push_single(iterations);
    test_push_tbb_single(iterations);
    test_push_dual(iterations);
    test_push_tbb_dual(iterations);

    return 0;
}
