#ifndef CSV_H
#define CSV_H

#include <iostream>
#include <numeric>
#include <sstream>

struct CommaSeparatedReader {
    template <typename... Args>
    static std::istream& read(std::istream& is, Args&... args) {
        std::string temp;
        ((std::getline(is, temp, ','), std::istringstream(temp) >> args), ...);
        is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return is;
    }
};

struct  CommaSeparatedWriter {
    template <typename... Args>
    static std::ostream& write(std::ostream& os, const Args&... args) {
        ((os << args << ","), ...);
        os << "\n";
        return os;
    }
};

#endif //CSV_H
