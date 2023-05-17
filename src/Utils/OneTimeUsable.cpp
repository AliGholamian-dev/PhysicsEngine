#include "OneTimeUsable.h"
#include "Exceptions.h"

void OneTimeUsable::ensureOneTimeUsage()
{
    if (isUsed)
        throw ItemIsUsedBeforeException();
    isUsed = true;
}
