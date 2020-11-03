#include "buffer_utils.h"

#include <gtest/gtest.h>

using namespace lmt::mux;

class BufferUtilsMuxTest : public ::testing::Test
{
  protected:
    template <typename T>
    bool allBuffersEmpty(const BufferList<T>& bufferList) const
    {
        return std::all_of(bufferList.begin(), bufferList.end(), [](const auto& buffer) { return buffer.empty(); });
    }
};

TEST_F(BufferUtilsMuxTest, getFirstOfEveryBuffer_oneBufferInList_sizeIsOne)
{
    BufferList<int> bufferList{{3, 2, 1}};

    const auto actualResult = getFirstOfEveryBuffer(bufferList);

    EXPECT_EQ(1, actualResult.size());
    EXPECT_EQ(3, actualResult.front());
}

TEST_F(BufferUtilsMuxTest, getFirstOfEveryBuffer_twoBufferInList_sizeIsTwo)
{
    BufferList<int> bufferList{{3, 2, 1}, {2}};

    const auto actualResult = getFirstOfEveryBuffer(bufferList);

    EXPECT_EQ(2, actualResult.size());
    EXPECT_EQ(3, actualResult.front());
    EXPECT_EQ(2, actualResult.back());
}

TEST_F(BufferUtilsMuxTest, getFirstOfEveryBuffer_emptyList_empty)
{
    BufferList<int> bufferList{};

    const auto actualResult = getFirstOfEveryBuffer(bufferList);

    EXPECT_TRUE(bufferList.empty());
}

TEST_F(BufferUtilsMuxTest, popFirstOfEveryBuffer_TwoBufferInListWithOneElement_allBuffersEmpty)
{
    BufferList<int> bufferList{{3}, {2}};

    popFirstOfEveryBuffer(bufferList);

    allBuffersEmpty(bufferList);
}

TEST_F(BufferUtilsMuxTest, popFirstOfEveryBuffer_emptyList_empty)
{
    BufferList<int> bufferList{};

    popFirstOfEveryBuffer(bufferList);

    EXPECT_TRUE(bufferList.empty());
}
