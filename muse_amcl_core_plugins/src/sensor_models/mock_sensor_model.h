#ifndef MOCKSENSORMODEL_H
#define MOCKSENSORMODEL_H

#include <muse_amcl/sensor/sensor_model.h>

namespace muse_amcl {
class MockSensorModel : public SensorModel
{
public:
    MockSensorModel();

    virtual int mock() override;

};
}

#endif // MOCKSENSORMODEL_H
