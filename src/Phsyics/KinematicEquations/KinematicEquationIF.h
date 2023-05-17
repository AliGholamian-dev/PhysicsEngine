#pragma once


#include "KinematicInfo.h"

class KinematicEquationIF {
    public:
        KinematicEquationIF() = default;
        virtual ~KinematicEquationIF() = default;

        [[nodiscard]] virtual KinematicInfo getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                                    const double& deltaTime) const = 0;
};
