#include <muse_amcl/sensor/sensor_model_factory.h>
#include <iostream>
#include <vector>

using namespace muse_amcl;

class MotionModel {

};

class MotionModelFactory {

};


struct Particle {
    /// pose (position / quaternion)
    /// speed
};

struct ParticleSet {
    std::vector<double>   sample_weights;
    std::vector<Particle> samples;
};

struct Scheduler {
    std::vector<std::unique_ptr<SensorModel>>  &sensor_models;
    MotionModel                                &motion_model;

    Scheduler(std::vector<std::unique_ptr<SensorModel>>  &sensor_models,
              MotionModel                                &motion_model) :
        sensor_models(sensor_models),
        motion_model(motion_model)
    {
    }

    virtual ~Scheduler() = default;

    virtual void perform() = 0;
};


struct SynchronousScheduler : public Scheduler
{
    SynchronousScheduler(std::vector<std::unique_ptr<SensorModel>>  &sensor_models,
                         MotionModel                                &motion_model) :
        Scheduler(sensor_models,
                  motion_model)
    {
    }


    void perform() override
    {
        std::cout << "foo bar baz" << std::endl;
    }
};

struct AsynchronousScheduler : public Scheduler
{
    AsynchronousScheduler(std::vector<std::unique_ptr<SensorModel>>  &sensor_models,
                          MotionModel                                &motion_model) :
        Scheduler(sensor_models,
                  motion_model)
    {
    }

    void perform() override
    {
        std::cout << "foo bar baz" << std::endl;
    }
};


class SchedulerFactory {
public:

    std::unique_ptr<Scheduler> create(std::vector<std::unique_ptr<SensorModel>>  &sensor_models,
                                      MotionModel                                &motion_model) {
        return std::unique_ptr<Scheduler>(new SynchronousScheduler(sensor_models,
                                                                   motion_model));
    }

};

struct ParticleFilter {
    SchedulerFactory                          &scheduler_factory;
    SensorModelFactory                        &sensor_model_factory;
    MotionModelFactory                        &motion_model_factory;

    std::vector<std::unique_ptr<SensorModel>>  sensor_models;
    std::unique_ptr<MotionModel>               motion_model;
    std::unique_ptr<Scheduler>                 scheduler;

    ParticleSet set;

    ParticleFilter(SensorModelFactory &sensor_model_factory,
                   MotionModelFactory &motion_model_factory,
                   SchedulerFactory   &scheduler_factory) :
        scheduler_factory(scheduler_factory),
        sensor_model_factory(sensor_model_factory),
        motion_model_factory(motion_model_factory)
    {
        sensor_models.emplace_back(sensor_model_factory.create("muse_amcl::MockSensorModel"));
        scheduler = scheduler_factory.create(sensor_models,
                                             *motion_model);
    }

    void run()  {
        scheduler->perform();

        for(auto &model : sensor_models) {
            assert(1 == model->mock());
            std::cout << "model says: " << model->mock() << std::endl;
        }

    }


};


int main(int argc, char *argv[])
{
    SensorModelFactory sensor_model_factory;
    MotionModelFactory motion_model_factory;
    SchedulerFactory   scheduler_factory;

    ///

    ParticleFilter particle_filter(sensor_model_factory,
                                   motion_model_factory,
                                   scheduler_factory);


    particle_filter.run();

    return 0;
}
