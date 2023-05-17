#pragma once

#include "FlagClass.h"
FlagClass LayerMask {
        enum Value {
            layer1 = 0b0000000000000001,
            layer2 = 0b0000000000000010,
            layer3 = 0b0000000000000100,
            layer4 = 0b0000000000001000,
            layer5 = 0b0000000000010000,
            layer6 = 0b0000000000100000,
            layer7 = 0b0000000001000000,
            layer8 = 0b0000000010000000,
            layer9 = 0b0000000100000000,
            layer10 =0b0000001000000000,
            layer11 =0b0000010000000000,
            layer12 =0b0000100000000000,
            layer13 =0b0001000000000000,
            layer14 =0b0010000000000000,
            layer15 =0b0100000000000000,
            layer16 =0b1000000000000000
        };
        Flag_Class(LayerMask)

        [[nodiscard]] bool isLayerMarked(const Value& layer) const {
            return isFlagSet(layer);
        }
};
