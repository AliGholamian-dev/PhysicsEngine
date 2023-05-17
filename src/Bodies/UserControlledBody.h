#pragma once


#include "PointerHelper.h"
#include "ID.h"
class BodyPointerHolderIF;

class UserControlledBody {
    public:
        UserControlledBody(ID id, const wp<BodyPointerHolderIF>& bodyPointerHolder);
        virtual ~UserControlledBody();
        [[nodiscard]] const ID& getUserControlledBodyID() const;

        virtual void doUserTasks(const double& deltaTime) = 0;

    private:
        void registerBody();
        void removeBody();

        const ID id;
        wp<BodyPointerHolderIF> bodyPointerHolder;
};
