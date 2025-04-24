#ifndef MA_BUF_H
#define MA_BUF_H

template <typename T>
class MA_Buf {
    std::deque<T> m_buffer;
    size_t m_size;
public:
    MA_Buf(size_t size) : m_size(size) {}

    void push_back(const T& value) {
        m_buffer.push_back(value);
        if (m_buffer.size() > m_size) {
            m_buffer.pop_front();
        }
    }

    T get_average() const {
        T average;
        for (const auto& value : m_buffer) {
            average += value;
        }
        average *= (1.0 / m_buffer.size());
        return average;
    }

    size_t size() const {
        return m_buffer.size();
    }
};

#endif //MA_BUF_H
