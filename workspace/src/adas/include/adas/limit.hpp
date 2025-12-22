#ifndef ADAS_LIMIT_HPP
#define ADAS_LIMIT_HPP


struct Limit {
    double min;
    double max;

    Limit() = default;
    Limit(double min, double max) : min{min}, max{max} {}
};


#endif  // ADAS_LIMIT_HPP
