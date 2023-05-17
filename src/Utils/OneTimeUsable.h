#pragma once

class OneTimeUsable
{
    public:
        OneTimeUsable() = default;
        virtual ~OneTimeUsable() = default;

    protected:
        void ensureOneTimeUsage();

    private:
        bool isUsed = false;
};

