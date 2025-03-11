#pragma once
#include <string>
#include <iostream>
#include <cstdarg>
#include <sstream>

namespace vision {
class Logger {
    private:
        std::string component_name;
        std::string component_id;
    public:
        enum class LogLevel {
            DEBUG,
            INFO,
            WARN,
            ERROR
        };

        Logger(std::string component_name, std::string component_id=""):
        component_name(component_name),
        component_id(component_id) {}

        virtual ~Logger() {}

        // Pure virtual method to log a message with a specified log level.
        #include <cstdarg>
        #include <cstdio>

        // Variadic logger methods supporting printf-style formatting.
        virtual void vlog(LogLevel level, const char* format, va_list args) {
            constexpr size_t bufferSize = 1024;
            char buffer[bufferSize];
            vsnprintf(buffer, bufferSize, format, args);

            std::cout << "[" << component_name << "]";

            if (!component_id.empty()) {
                std::cout << "[" << component_id << "]";
            }

            switch (level) {
                case LogLevel::DEBUG:
                    std::cout << "[DEBUG] ";
                    break;
                case LogLevel::INFO:
                    std::cout << "[INFO] ";
                    break;
                case LogLevel::WARN:
                    std::cout << "[WARN] ";
                    break;
                case LogLevel::ERROR:
                    std::cout << "[ERROR] ";
                    break;
            }
            std::cout << buffer << std::endl;
        }

        virtual void log(const char* format, LogLevel level = LogLevel::INFO, ...) {
            va_list args;
            va_start(args, level);
            vlog(level, format, args);
            va_end(args);
        }

        virtual void LogDebug(const char* format, ...) {
            va_list args;
            va_start(args, format);
            vlog(LogLevel::DEBUG, format, args);
            va_end(args);
        }

        virtual void LogDebug(const std::string& message) {
            log(message.c_str(), LogLevel::DEBUG);
        }

        template<typename T>
        void LogDebug(const T& value) {
            std::stringstream ss;
            ss << value;
            LogDebug(ss.str());
        }

        template<typename T>
        Logger& operator<<(const T& value) {
            LogDebug(value);
            return *this;
        }

        virtual void LogInfo(const char* format, ...) {
            va_list args;
            va_start(args, format);
            vlog(LogLevel::INFO, format, args);
            va_end(args);
        }

        virtual void LogWarning(const char* format, ...) {
            va_list args;
            va_start(args, format);
            vlog(LogLevel::WARN, format, args);
            va_end(args);
        }

        virtual void LogError(const char* format, ...) {
            va_list args;
            va_start(args, format);
            vlog(LogLevel::ERROR, format, args);
            va_end(args);
        }
};

}