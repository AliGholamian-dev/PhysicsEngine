#pragma once

#include "FileHolder.h"
#include "PointerHelper.h"
#include <iostream>
#include <mutex>

class ConditionalDebug final {
    public:
        ConditionalDebug(const ConditionalDebug& other) = delete;
        explicit ConditionalDebug(bool enable);
        ConditionalDebug(ConditionalDebug&& other)  noexcept;
        ~ConditionalDebug();

        ConditionalDebug& noSpace();
        ConditionalDebug& withSpace();

        template<class T>
        ConditionalDebug& operator<< (const T& input) {
            putMessageInConsole(input);
            putMessageInFile(input);
            return *this;
        }

        ConditionalDebug& operator<<(std::ostream& (*functionPointer)( std::ostream& )) {
            noSpace();
            putMessageInConsole(functionPointer);
            putMessageInFile(functionPointer);
            withSpace();
            return *this;
        }


    private:
        void prepareDebugFile() const;
        template<class T>
        void putMessageInConsole(const T& input) {
            if (debugInConsole) {
                std::cout << input;
                if (space) {
                    std::cout << " ";
                }
            }
        }

        template<class T>
        void putMessageInFile(const T& input) {
            if (debugInFile && fileHolder) {
                fileHolder->file << input;
                if (space) {
                    fileHolder->file << " ";
                }
            }
        }

        bool debugEnabled;
        bool debugInFile;
        bool debugInConsole;
        bool space{true};
        static up<FileHolder> fileHolder;
        static std::mutex mutex;
        static constexpr uint64_t maxFileSize {10000000};
};
