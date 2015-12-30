#include <muse_amcl/sensor/sensor_model_factory.h>
#include <iostream>

using namespace muse_amcl;

int main(int argc, char *argv[])
{
    SensorModelFactory factory;

    std::shared_ptr<SensorModel> model = factory.create("muse_amcl::MockSensorModel");

    std::cout << "model says: " << model->mock() << std::endl;

    assert(1 == model->mock());

    return 0;
}
