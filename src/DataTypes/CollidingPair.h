#pragma once


class CollidingBody;
struct CollidingPair final {
    CollidingBody* first;
    CollidingBody* second;
};
