#pragma once


class Constraint {
    public:
        Constraint(const Constraint& other) = delete;
        Constraint(Constraint& other) = delete;
        Constraint(Constraint&& other) = delete;
        Constraint() = default;
        virtual ~Constraint() = default;

        virtual void updateConstraint (const double& deltaTime) = 0;
};