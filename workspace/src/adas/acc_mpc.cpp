#include <thread>

#include <atom/dds/msgs.hpp>
#include <atom/dds/car.hpp>
#include <atom/dds/acc.hpp>
#include <atom/dds/diagnostic.hpp>
#include <boost/program_options.hpp>
#include <dds/dds.hpp>

#include "utils.hpp"
#include "types_converter.hpp"
#include "../control/acc_controller.hpp"
#include "../control/maintaining_speed_controller.hpp"
#include "../planning/vehicle_model.hpp"

namespace ctl = atom::control;
namespace po = boost::program_options;
namespace pln = atom::planning;

int main(int argc, char *argv[]) {

    constexpr double ms2kmh = 3.6;
    constexpr double kmh2ms = 1 / ms2kmh;

    double wheel_base = 2.6985, vehicle_length = 4.5415, vehicle_rear_padding = 0.8952;
    double vehicle_left_size = 1.0511, vehicle_right_size = 1.0281;
    double rho_min = 5;

    std::uint32_t domain_id = 0;

    double tau = 0.4;
    double ts = 0.1;
    double dc = 5.5;
    double desired_speed = 45;
    double time_gap = 2;

    bool acc_diff = true;
    bool cipv_off = true;

    constexpr double kp = 0.75;     // proportional coeff

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Produce help message")
        ("domain-id,d", po::value(&domain_id)->default_value(domain_id), "DDS domain ID")
        ("tau", po::value(&tau)->default_value(tau), "Time lag of ACC system, seconds")
        ("ts", po::value(&ts)->default_value(ts), "Sampling period of ACC System, seconds")
        ("dc", po::value(&dc)->default_value(dc), "Minimum safe distance, meters")
        ("desired_speed,v", po::value(&desired_speed)->default_value(desired_speed), "Desired speed value, kilometers per hour")
        ("time_gap,g", po::value(&time_gap)->default_value(time_gap), "Desired time gap (for distance level), seconds")
        ("acc_diff,a", po::bool_switch(&acc_diff), "Enable discrete differentiation of acceleration")
        ("cipv_turn_off,c", po::bool_switch(&cipv_off), "Disable CIPV accounting")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << "Usage: " << argv[0] << " [options]" << '\n' << desc << "\n";
        return 1;
    }

    pln::vehicle_model vehicle{
        vehicle_length - vehicle_rear_padding,
        vehicle_rear_padding,
        vehicle_left_size,
        vehicle_right_size,
        wheel_base,
        rho_min
    };

    dds::domain::DomainParticipant participant{domain_id};

    auto u_writer = atom::make_datawriter<AccelerationCommand>(participant, "car/acceleration_control");
    auto logs_writer = atom::make_datawriter<AccLogs2>(participant, "acc/logs2");
    auto state_writer = atom::make_datawriter<ModuleState>(participant, "acc/state");

    auto speed_reader = atom::make_datareader<Speed>(participant, "car/odometry/speed");
    auto accel_reader = atom::make_datareader<Acceleration>(participant, "car/odometry/acceleration");
    auto cipv_reader = atom::make_datareader<RadarOutput>(participant, "cipv_out");
    auto ds_reader = atom::make_datareader<Speed>(participant, "desired_speed");
    auto control_state_reader = atom::make_datareader<CcuControlState>(participant, "car/ccu_control_state");

    desired_speed *= kmh2ms;

    ctl::acc_controller controller{tau, ts, dc, desired_speed, time_gap};

    Eigen::Vector<double, 5> x = {1000, 0, 0, 0, 0};
    Eigen::Vector<double, 5> prev_x = x;

    dds::sub::SampleRef<Speed> prev_speed_sample;
    dds::sub::SampleRef<Acceleration> prev_accel_sample;
    dds::sub::SampleRef<RadarOutput> prev_cipv_sample;

    auto take_data = [&state_writer] (auto &reader, auto &sample, const char *name) {
        bool new_data = false;
        for (auto new_sample : reader.take()) {
            if (new_sample.info().valid()) {
                sample = new_sample;
                new_data = true;
            }
        }
        return new_data;
    };

    const auto total_duration = std::chrono::duration<double>(ts);
    auto current_duration = total_duration;

    dds::sub::SampleRef<Speed> speed_sample;
    dds::sub::SampleRef<Acceleration> accel_sample;
    dds::sub::SampleRef<RadarOutput> cipv_sample;

    for (;;) {
        std::this_thread::sleep_for(current_duration);

        const auto start = std::chrono::steady_clock::now();
        current_duration = total_duration;

        const auto speed_data = take_data(speed_reader, speed_sample, "speed");
        const auto accel_data = acc_diff || take_data(accel_reader, accel_sample, "acceleration");
        const auto cipv_data = take_data(cipv_reader, cipv_sample, "cipv");


        if (!speed_sample.info().valid() || (!acc_diff && !accel_sample.info().valid()) || !cipv_sample.info().valid())
            continue;

        for (auto sample : ds_reader.take()) {
            if (sample.info().valid()) {
                desired_speed = sample.data().value() * kmh2ms;
                controller.update_desired_speed(desired_speed);
            }
        }

        prev_x = x;

        x[1] = speed_sample.data().value() * kmh2ms;

        double elapsed_ts = 0;
        double speed_elapsed_ts = prev_speed_sample.info().valid()
                ? speed_sample.info().timestamp().to_secs() - prev_speed_sample.info().timestamp().to_secs()
                : 0;

        if (acc_diff) {
            elapsed_ts = speed_elapsed_ts;

            x[3] = (elapsed_ts > 0)
                ? (x[1] - prev_x[1]) / elapsed_ts
                : 0;

        } else {
            elapsed_ts = prev_accel_sample.info().valid()
                ? accel_sample.info().timestamp().to_secs() - prev_accel_sample.info().timestamp().to_secs()
                : 0;

            x[3] = accel_sample.data().longitudinal();
        }

        x[4] = (elapsed_ts > 0)
            ? (x[3] - prev_x[3]) / elapsed_ts
            : 0;

        prev_speed_sample = speed_sample;
        prev_cipv_sample = cipv_sample;
        if (!acc_diff)
            prev_accel_sample = accel_sample;

        auto ccu_state = ControlState::CCU_STATE_UNKNOWN;
        bool driver_interaction = false;

        for (auto sample : control_state_reader.read()) {
            if (sample.info().valid()) {
                const auto &state = sample.data();
                ccu_state = state.control_state();

                if ((state.control_state() != ControlState::CCU_STATE_ENABLED) || state.pedal_intervention())
                    driver_interaction = true;
            }
        }

        auto solver_status = OsqpEigen::Status::Solved;

        const auto &cipv = cipv_sample.data().radarout()[0];

        x[0] = (cipv.x() == 1000 || cipv_off) ? 1000 : cipv.x() - (vehicle.front - vehicle.wheel_base);
        x[2] = cipv_off ? 0 : cipv.vx();

        auto target = (cipv.x() == 1000 || cipv_off) ? std::optional<atom::radar_box>{} : atom::convert_from_dds(cipv);
        const auto res = controller(x, target, driver_interaction);
        if (res.solver_status != OsqpEigen::Status::Solved) {
            std::cerr << "control not solved! solver_status=" << static_cast<int8_t>(solver_status) << "\n";
        }

        u_writer.write(AccelerationCommand{static_cast<float>(res.u), false});
        state_writer.write(ModuleState("ACC", ModuleStateEnum::STATE_ENABLE, {}));

        const double desired_dist = cipv.x() == 1000 ? 1000 : dc + time_gap * x[1];
        logs_writer.write(AccLogs2{desired_speed * ms2kmh, desired_dist, cipv.obj_id(),
            x[0], x[1] * ms2kmh, x[2], x[3], x[4], res.u, static_cast<int8_t>(solver_status),
            res.acc_mpc_reset, res.cc_mpc_reset, static_cast<uint8_t>(res.mode), static_cast<uint8_t>(ccu_state)});

        current_duration -= (std::chrono::steady_clock::now() - start);
    }

    return 0;
}