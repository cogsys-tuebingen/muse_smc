#include <chrono>
#include <functional>
#include <iostream>
#include "function.h"

#include <muse_mcl/utils/delegate.hpp>

#define FUNC_NO_EXCEPTIONS
#define FUNC_NO_RTTI

template<typename T>
struct Type
{
    T type;
};



struct Test {
    std::size_t blob;
    void test(const std::size_t blub)
    {
        blob = blub;
    }
};

std::size_t blob;
void test(const std::size_t blub)
{
    blob = blub;
}

using member_function = void (Test::*)(const std::size_t);
using global_function = void (*)(const std::size_t);
using functional = std::function<void(const std::size_t)>;
using function = func::function<void(const std::size_t)>;


void member_test_function_pointer(const std::size_t iterations)
{
    Test test;
    member_function test_member = &Test::test;

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        (test.*test_member)(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test member: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
}

void global_test_function_pointer(const std::size_t iterations)
{
    global_function test_global = &test;

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        (*test_global)(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test global: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
}

void member_test_lambda(const std::size_t iterations)
{
    Test test;
    auto test_member = [&test](const std::size_t i){test.test(i);};
    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_member(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test lambda member: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;


}

void global_test_lambda(const std::size_t iterations)
{
    auto test_global = [](const std::size_t i){test(i);};

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_global(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test lambda global: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

void member_test_functional(const std::size_t iterations)
{
    Test test;
    functional test_member = [&test](const std::size_t i){test.test(i);};
    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_member(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test functional member: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

void global_test_functional(const std::size_t iterations)
{
    functional test_global = [](const std::size_t i){test(i);};

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_global(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test functional global: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

void member_test_function(const std::size_t iterations)
{
    Test test;
    function test_member =  [&test](const std::size_t i){test.test(i);};

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_member(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test function sm member: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

void global_test_function(const std::size_t iterations)
{
    function test_member =  [](const std::size_t i){test(i);};;

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_member(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test function sm global: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

void member_test_delegate(const std::size_t iterations)
{
    Test test;
    delegate<void(const std::size_t)> test_member = delegate<void(const std::size_t)> ::from<Test, &Test::test>(&test);
    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_member(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test delegate member: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

void global_test_delegate(const std::size_t iterations)
{
    delegate<void(const std::size_t)> test_global = delegate<void(const std::size_t)>::from<test>();

    auto start = std::chrono::high_resolution_clock::now();
    for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
        test_global(iteration);
    }
    auto end = std::chrono::high_resolution_clock::now();

    long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cerr << "test delegate global: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

}

int main(int argc, char *argv[])
{

    const std::size_t iterations = 10000000;
    member_test_function_pointer(iterations);
    global_test_function_pointer(iterations);

    member_test_lambda(iterations);
    global_test_lambda(iterations);

    member_test_functional(iterations);
    global_test_functional(iterations);

    member_test_function(iterations);
    global_test_function(iterations);

    member_test_delegate(iterations);
    global_test_delegate(iterations);



    return 0;
}
