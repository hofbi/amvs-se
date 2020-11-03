#ifndef SINGLE_ENCODER_BUFFER_UTILS_H
#define SINGLE_ENCODER_BUFFER_UTILS_H

#include <algorithm>
#include <deque>
#include <vector>

namespace lmt
{
namespace mux
{
template <typename T>
using BufferList = std::vector<std::deque<T>>;
template <typename T>
std::vector<T> getFirstOfEveryBuffer(const BufferList<T>& bufferList) noexcept
{
    std::vector<T> bufferFronts;

    for (const auto& buffer : bufferList)
    {
        bufferFronts.push_back(buffer.front());
    }

    return bufferFronts;
}

template <typename T>
void popFirstOfEveryBuffer(BufferList<T>& bufferList) noexcept
{
    std::for_each(bufferList.begin(), bufferList.end(), [](auto& buffer) { buffer.pop_front(); });
}
}  // namespace mux
}  // namespace lmt

#endif  // SINGLE_ENCODER_BUFFER_UTILS_H
