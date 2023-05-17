#pragma once


#include "ID.h"
#include "Vector2D.h"
#include "KinematicInfo.h"
#include "DampingInfo.h"
#include "PointerHelper.h"

class Body;
class KinematicEquationIF;
class BodyPointerHolderIF;

class KinematicBody {
    public:
        KinematicBody() = delete;
        KinematicBody(const KinematicBody& other) = delete;
        KinematicBody(KinematicBody&& other) noexcept  = delete;
        KinematicBody(ID id,
                      Body* bodyToManipulate,
                      const sp<KinematicEquationIF>& kinematicEquation,
                      const KinematicInfo& initialKinematicInfo,
                      const DampingInfo& dampingInfo,
                      const wp<BodyPointerHolderIF>& bodyPointerHolder);
        virtual ~KinematicBody();
        [[nodiscard]] const ID& getKinematicBodyID() const;

        void setDampingInfo(const DampingInfo& newDampingInfo);
        void setKinematicInfo(const KinematicInfo& kinematicInfo);
        void addKinematicInfo(const KinematicInfo& kinematicInfo);
        void manipulateBody(const Vector2D& calculatedLinearAcceleration,
                            const double& calculatedAngularAcceleration,
                            const double& deltaTime);

        [[nodiscard]] const KinematicInfo& getLastKinematicInfo() const;
        [[nodiscard]] const DampingInfo& getDampingInfo() const;

    private:
        void checkInputParameters() const;
        void registerBody();
        void removeBody();
        KinematicInfo getInitialState(const Vector2D& calculatedLinearAcceleration,
                                      const double& calculatedAngularAcceleration);
        void addUserAddedKinematicInfoTo(KinematicInfo& kinematicInfo) const;
        void moveBody(const KinematicInfo& finalState);
        void updateLastKinematicInfo(const KinematicInfo& finalState);
        void damp(const double& deltaTime);
        void resetAdditionalKinematicInfo();

        const ID id;
        Body* bodyToManipulate;
        sp<KinematicEquationIF> kinematicEquation;
        KinematicInfo lastKinematicInfo;
        DampingInfo dampingInfo;
        wp<BodyPointerHolderIF> bodyPointerHolder;
        optional<KinematicInfo> additionalKinematicInfo;
};
