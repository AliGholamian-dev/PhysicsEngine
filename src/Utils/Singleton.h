#pragma once


template <typename T>
class Singleton
{
    protected:
        explicit Singleton() = default;
        virtual ~Singleton() = default;
    public:
        static T* getInstance() {
            if (!instance)
                instance = new T;
            return instance;
        }

    protected:
        static T* instance;
};

template <typename T>
T* Singleton<T>::instance = nullptr;

