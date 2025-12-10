#ifndef ADAS_CONTROLLER_HPP
#define ADAS_CONTROLLER_HPP


#include <tuple>


class Controller {
public:
    virtual ~Controller() = default;

    virtual std::tuple<double> calculate_control() = 0;
};


#endif  // ADAS_CONTROLLER_HPP
