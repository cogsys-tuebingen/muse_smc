#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

namespace muse_amcl {

class SensorModel {
public:
    virtual ~SensorModel();

    /// testing inheritance
    virtual int mock() = 0;

protected:
    SensorModel();


};


}

#endif // SENSOR_MODEL_H
