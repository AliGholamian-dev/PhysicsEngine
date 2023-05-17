#pragma once


class Vector2D final {
    public:
        Vector2D();
        Vector2D(const double& x, const double& y);
        Vector2D(const Vector2D& other);
        Vector2D(Vector2D&& other) noexcept ;
        ~Vector2D() = default;

        [[nodiscard]] const double& getX() const;
        [[nodiscard]] const double& getY() const;
        [[nodiscard]] double getSquaredMagnitude() const;
        [[nodiscard]] double getMagnitude() const;
        [[nodiscard]] Vector2D getNormalizedVector() const;
        [[nodiscard]] double getDotProductTo(const Vector2D& other) const;
        [[nodiscard]] double getCrossProductTo(const Vector2D& other) const;
        [[nodiscard]] double getDistanceTo(const Vector2D& other) const;

        void setX(double newX);
        void setY(double newY);
        void setToNewVector(const Vector2D& newVector);
        void rotate(double angleInRadian);

        Vector2D& operator=(const Vector2D& other);
        Vector2D& operator=(Vector2D&& other) noexcept;
        Vector2D  operator+(const Vector2D& other) const;
        Vector2D& operator+=(const Vector2D& other);
        Vector2D  operator-(const Vector2D& other) const;
        Vector2D& operator-=(const Vector2D& other);
        Vector2D  operator*(double scalar) const;
        Vector2D& operator*=(double scalar);
        Vector2D  operator/(double scalar) const;
        Vector2D& operator/=(double scalar);

        friend Vector2D operator*(double scalar, const Vector2D& toBeMultipliedVector);

    private:
        double x;
        double y;
};
