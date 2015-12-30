#include "mock_sensor_model.h"

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockSensorModel, muse_amcl::SensorModel)

using namespace muse_amcl;

MockSensorModel::MockSensorModel()
{

}

int MockSensorModel::mock()
{
    return 1 /* MAGIC NUMBER BECAUSE WE CAN */;
}
