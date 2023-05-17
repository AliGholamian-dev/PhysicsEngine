#include "UserControlledBody.h"
#include "BodyPointerHolderIF.h"

UserControlledBody::UserControlledBody(ID id, const wp<BodyPointerHolderIF>& bodyPointerHolder) :
        id(id),
        bodyPointerHolder(bodyPointerHolder)
{
    registerBody();
}

UserControlledBody::~UserControlledBody() {
    removeBody();
}

const ID& UserControlledBody::getUserControlledBodyID() const {
    return id;
}

void UserControlledBody::registerBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->registerUserControlledBody(id, this);
    }
}

void UserControlledBody::removeBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->removeUserControlledBody(id);
    }
}
