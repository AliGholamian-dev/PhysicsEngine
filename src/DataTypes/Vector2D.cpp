#include "Vector2D.h"
#include <cmath>

Vector2D::Vector2D() : x {0}, y{0} {}

Vector2D::Vector2D(const double& x, const double& y) : x {x}, y{y} {}

Vector2D::Vector2D(const Vector2D& other) : Vector2D(other.x, other.y) {}

Vector2D::Vector2D(Vector2D&& other) noexcept : Vector2D(other.x, other.y) {}

const double& Vector2D::getX() const {
    return x;
}

const double& Vector2D::getY() const {
    return y;
}

double Vector2D::getSquaredMagnitude() const {
    return (x * x) + (y * y);
}

double Vector2D::getMagnitude() const {
    return std::sqrt(getSquaredMagnitude());
}


Vector2D Vector2D::getNormalizedVector() const {
    Vector2D normalizedVector(x, y);
    double magnitude = getMagnitude();
    normalizedVector /= magnitude;
    return normalizedVector;
}

double Vector2D::getDotProductTo(const Vector2D& other) const {
    return (x * other.getX()) +  (y * other.getY());
}

double Vector2D::getCrossProductTo(const Vector2D& other) const {
    return (x * other.getY()) - (y * other.getX());
}

double Vector2D::getDistanceTo(const Vector2D& other) const {
    Vector2D distanceVector{*this - other};
    return distanceVector.getMagnitude();
}

void Vector2D::setX(double newX) {
    x = newX;
}

void Vector2D::setY(double newY) {
    y = newY;
}

void Vector2D::setToNewVector(const Vector2D& newVector) {
    setX(newVector.getX());
    setY(newVector.getY());
}

void Vector2D::rotate(double angleInRadian) {
    double rotatedX = (x * std::cos(angleInRadian)) - (y * std::sin(angleInRadian));
    double rotatedY = (x * std::sin(angleInRadian)) + (y * std::cos(angleInRadian));
    setToNewVector({rotatedX, rotatedY});
}

Vector2D& Vector2D::operator=(const Vector2D& other) {
    if (this == &other)
        return *this;
    setToNewVector(other);
    return *this;
}

Vector2D& Vector2D::operator=(Vector2D&& other) noexcept {
    if (this == &other)
        return *this;
    setToNewVector(other);
    return *this;
}

Vector2D Vector2D::operator+(const Vector2D& other) const {
    double sumX = other.getX() + x;
    double sumY = other.getY() + y;
    return {sumX, sumY};
}

Vector2D& Vector2D::operator+=(const Vector2D& other) {
    x += other.getX();
    y += other.getY();
    return *this;
}

Vector2D Vector2D::operator-(const Vector2D& other) const {
    double subX = x - other.getX();
    double subY = y - other.getY();
    return {subX, subY};
}

Vector2D& Vector2D::operator-=(const Vector2D& other) {
    x -= other.getX();
    y -= other.getY();
    return *this;
}

Vector2D Vector2D::operator*(double scalar) const {
    double multipliedX = x * scalar;
    double multipliedY = y * scalar;
    return {multipliedX, multipliedY};
}

Vector2D& Vector2D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2D Vector2D::operator/(double scalar) const {
    double dividedX = x / scalar;
    double dividedY = y / scalar;
    return {dividedX, dividedY};
}

Vector2D& Vector2D::operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    return *this;
}

Vector2D operator*(double scalar, const Vector2D& toBeMultipliedVector) {
    return toBeMultipliedVector * scalar;
}
