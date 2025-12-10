#ifndef ADAS_CRUISE_CONTROLLER_HPP
#define ADAS_CRUISE_CONTROLLER_HPP


#include "adas/controller.hpp"


class CruiseController : public Controller {
private:

public:
    CruiseController();

    std::tuple<double> calculate_control(const double ref_velocity, const double velocity, const double acceleration);
};


#endif  // ADAS_CRUISE_CONTROLLER_HPP
