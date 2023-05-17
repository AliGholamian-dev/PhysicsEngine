#pragma once


class CircleHelper final {
    public:
        CircleHelper() = default;
        ~CircleHelper()  = default;

        static double getArea(const double& radius);
        static double getArea(const double& mass, const double& density);
        static double getMassByArea(const double& area, const double& density);
        static double getMassByRadius(const double& radius, const double& density);
        static double getDensityByArea(const double& mass, const double& area);
        static double getDensityByRadius(const double& mass, const double& radius);
        static double getMomentOfInertia(const double& mass, const double& radius);
    private:
        static void checkCircleSpecs(const double& radius) ;
};
