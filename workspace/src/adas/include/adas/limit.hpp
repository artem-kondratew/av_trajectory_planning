#ifndef ADAS_LIMIT_HPP
#define ADAS_LIMIT_HPP


struct Limit {
    double min;
    double max;

    static constexpr double ms2kmh = 3.6;
    static constexpr double kmh2ms = 1 / ms2kmh;

    Limit() = default;
    Limit(double min, double max) : min{min}, max{max} {}

    static Limit vectorToLimit(const std::vector<double>& vec) {
        return Limit{vec[0], vec[1]};
    }

    void scale(double scale_factor) {
        min *= scale_factor;
        max *= scale_factor;
    }
};


#endif  // ADAS_LIMIT_HPP
