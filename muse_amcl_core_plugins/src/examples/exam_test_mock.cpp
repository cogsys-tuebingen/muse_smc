#include <muse_amcl/plugins/update_function_factory.h>
#include <muse_amcl/plugins/propagation_function_factory.h>

int main(int argc, char *argv[])
{
    muse_amcl::UpdateFunctionFactory uf;
    muse_amcl::PropagationFunctionFactory pf;

    std::shared_ptr<muse_amcl::Update>      u = uf.create("muse_amcl::MockUpdate");
    std::shared_ptr<muse_amcl::Propagation> p = pf.create("muse_amcl::MockPropagation");

    muse_amcl::ParticleSet set(1);

    u->apply(set.getWeights());
    p->apply(set.getPoses());

    return 0;
}
