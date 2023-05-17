#include "ConditionalDebug.h"
#include "CustomWorld.h"

int main() {
    ConditionalDebug debug(true);
    debug << "World Started"  << std::endl;
    CustomWorld::getInstance()->simulate();
    debug << "World Ended"  << std::endl;
    return 0;
}
