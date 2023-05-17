#include "CircleHelper.h"
#include "Exceptions.h"
#include "MathUtils.h"
#include <numbers>

void CircleHelper::checkCircleSpecs(const double& radius) {
    if(MathUtils::checkForEquality(radius, 0) ||
       radius < 0)
    {
        throw WrongParametersException();
    }
}

double CircleHelper::getArea(const double& radius) {
    checkCircleSpecs(radius);
//    return std::numbers::pi * radius * radius;
	return 3.1415 * radius * radius;
}

double CircleHelper::getArea(const double& mass, const double& density) {
    if(MathUtils::checkForEquality(density, 0) ||
       MathUtils::checkForEquality(mass, 0) ||
       density < 0 ||
       mass < 0)
    {
        throw WrongParametersException();
    }
    return mass / density;
}

double CircleHelper::getMassByArea(const double& area, const double& density) {
    if(MathUtils::checkForEquality(area, 0) ||
       MathUtils::checkForEquality(density, 0) ||
       area < 0 ||
       density < 0)
    {
        throw WrongParametersException();
    }
    return area * density;
}

double CircleHelper::getMassByRadius(const double& radius, const double& density) {
    checkCircleSpecs(radius);
    if(MathUtils::checkForEquality(density, 0) ||
       density < 0)
    {
        throw WrongParametersException();
    }
    return getMassByArea(getArea(radius), density);
}

double CircleHelper::getDensityByArea(const double& mass, const double& area) {
    if(MathUtils::checkForEquality(area, 0) ||
       MathUtils::checkForEquality(mass, 0) ||
       area < 0 ||
       mass < 0)
    {
        throw WrongParametersException();
    }
    return mass / area;
}

double CircleHelper::getDensityByRadius(const double& mass, const double& radius) {
    checkCircleSpecs(radius);
    if(MathUtils::checkForEquality(mass, 0) ||
       mass < 0)
    {
        throw WrongParametersException();
    }
    return getDensityByArea(mass, getArea(radius));
}

double CircleHelper::getMomentOfInertia(const double& mass, const double& radius) {
    checkCircleSpecs(radius);
    if(MathUtils::checkForEquality(mass, 0) ||
       mass < 0)
    {
        throw WrongParametersException();
    }
    return mass * radius * radius / 2.0;
}

