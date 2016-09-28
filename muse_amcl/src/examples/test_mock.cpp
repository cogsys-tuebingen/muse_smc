#include <muse_amcl/functions/update_function_factory.h>

int main(int argc, char *argv[])
{
    muse_amcl::UpdateFunctionFactory uf;

    std::unique_ptr<muse_amcl::Update> u = uf.create("muse_amcl::MockUpdate");


    return 0;
}
