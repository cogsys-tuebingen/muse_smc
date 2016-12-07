#include <muse_amcl/utils/signals.hpp>

#include <iostream>

void triggered(int a, int b)
{
    std::cout << a << " " << b << std::endl;
}

int main(int argc, char *argv[])
{
    typedef Signal<std::function<void(int,int)>> Test;

    Test things;
    Test::Connection::Ptr c = things.connect(triggered);
    things(1,2);

    return 0;
}
