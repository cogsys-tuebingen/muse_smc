#include <vector>
#include <chrono>
#include <iostream>

int main(int argc, char *argv[])
{
    std::vector<double> a(100);
    std::vector<double> b(100);

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    for(std::size_t i = 0 ; i < 10000 ; ++i) {
        std::swap(a,b);
    }
    std::chrono::duration<double> dur1 = std::chrono::system_clock::now() - start;

    start = std::chrono::system_clock::now();
    for(std::size_t i = 0 ; i < 10000 ; ++i) {
        a.assign(b.begin(), b.end());
    }
    std::chrono::duration<double> dur2 = std::chrono::system_clock::now() - start;


    std::cout << dur1.count() * 1000.0 << " ms" << std::endl;
    std::cout << dur2.count() * 1000.0 << " ms" << std::endl;

    return 0;
}
