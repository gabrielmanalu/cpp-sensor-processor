#include <gtest/gtest.h>
#include <fstream>
#include <cmath>
#include "sensor_processor/core/PointCloud.hpp"
#include "sensor_processor/core/PointTypes.hpp"
#include "sensor_processor/io/CsvReader.hpp"
#include "sensor_processor/processing/Processor.hpp"
#include "sensor_processor/processing/ThreadPool.hpp"

using namespace sensor;

// ---------------------------------------------------------------------------
// PointCloud
// ---------------------------------------------------------------------------

TEST(PointCloudTest, StartsEmpty) {
    core::PointCloud<core::PointXYZ> cloud;
    EXPECT_TRUE(cloud.empty());
    EXPECT_EQ(cloud.size(), 0u);
}

TEST(PointCloudTest, AddAndRetrievePoints) {
    core::PointCloud<core::PointXYZ> cloud;
    cloud.addPoint({1.0f, 2.0f, 3.0f});
    cloud.addPoint({4.0f, 5.0f, 6.0f});
    EXPECT_EQ(cloud.size(), 2u);
    EXPECT_FLOAT_EQ(cloud.points()[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points()[1].z, 6.0f);
}

TEST(PointCloudTest, Clear) {
    core::PointCloud<core::PointXYZ> cloud;
    cloud.addPoint({1.0f, 2.0f, 3.0f});
    cloud.clear();
    EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudTest, RGBPoint) {
    core::PointCloud<core::PointXYZRGB> cloud;
    cloud.addPoint(core::PointXYZRGB{1.0f, 0.0f, 0.0f, 255, 0, 0});
    EXPECT_EQ(cloud.points()[0].r, 255);
}

// ---------------------------------------------------------------------------
// Type traits
// ---------------------------------------------------------------------------

TEST(PointTypesTest, HasRgbTrait) {
    EXPECT_FALSE(core::has_rgb_v<core::PointXYZ>);
    EXPECT_TRUE(core::has_rgb_v<core::PointXYZRGB>);
}

// ---------------------------------------------------------------------------
// ThreadPool
// ---------------------------------------------------------------------------

TEST(ThreadPoolTest, ExecutesTasks) {
    proc::ThreadPool pool(2);
    auto f1 = pool.enqueue([] { return 21; });
    auto f2 = pool.enqueue([] { return 21; });
    EXPECT_EQ(f1.get() + f2.get(), 42);
}

TEST(ThreadPoolTest, ReportsSize) {
    proc::ThreadPool pool(3);
    EXPECT_EQ(pool.size(), 3u);
}

// ---------------------------------------------------------------------------
// CsvReader
// ---------------------------------------------------------------------------

static std::string writeTempCsv(const std::string& content) {
    const std::string path = "test_tmp.csv";
    std::ofstream f(path);
    f << content;
    return path;
}

TEST(CsvReaderTest, ReadsXYZChunk) {
    auto path = writeTempCsv(
        "x,y,z\n"
        "1.0,2.0,3.0\n"
        "4.0,5.0,6.0\n");

    io::CsvReader<core::PointXYZ> reader(path);
    auto chunk = reader.readChunk(10);
    ASSERT_NE(chunk, nullptr);
    EXPECT_EQ(chunk->size(), 2u);
    EXPECT_FLOAT_EQ(chunk->points()[0].x, 1.0f);
    EXPECT_FLOAT_EQ(chunk->points()[1].z, 6.0f);
}

TEST(CsvReaderTest, ReturnsNullptrWhenExhausted) {
    auto path = writeTempCsv("x,y,z\n1.0,2.0,3.0\n");
    io::CsvReader<core::PointXYZ> reader(path);
    reader.readChunk(10);                       // consume all
    auto chunk = reader.readChunk(10);
    EXPECT_EQ(chunk, nullptr);
}

TEST(CsvReaderTest, ReadsInChunks) {
    std::string csv = "x,y,z\n";
    for (int i = 0; i < 10; ++i)
        csv += std::to_string(i) + ",0.0,0.0\n";

    auto path = writeTempCsv(csv);
    io::CsvReader<core::PointXYZ> reader(path);

    auto c1 = reader.readChunk(4);
    ASSERT_NE(c1, nullptr);
    EXPECT_EQ(c1->size(), 4u);

    auto c2 = reader.readChunk(4);
    ASSERT_NE(c2, nullptr);
    EXPECT_EQ(c2->size(), 4u);

    auto c3 = reader.readChunk(4);
    ASSERT_NE(c3, nullptr);
    EXPECT_EQ(c3->size(), 2u);   // only 2 left
}

TEST(CsvReaderTest, ReadsXYZRGBChunk) {
    auto path = writeTempCsv(
        "x,y,z,r,g,b\n"
        "1.0,2.0,3.0,255,128,0\n");

    io::CsvReader<core::PointXYZRGB> reader(path);
    auto chunk = reader.readChunk(10);
    ASSERT_NE(chunk, nullptr);
    EXPECT_EQ(chunk->points()[0].r, 255);
    EXPECT_EQ(chunk->points()[0].g, 128);
}

TEST(CsvReaderTest, ThrowsOnMissingFile) {
    EXPECT_THROW(io::CsvReader<core::PointXYZ>("no_such_file.csv"),
                 std::runtime_error);
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

static core::PointCloud<core::PointXYZ> makeSymmetricCloud(int n) {
    core::PointCloud<core::PointXYZ> cloud;
    for (int i = 1; i <= n; ++i) {
        const float v = static_cast<float>(i);
        cloud.addPoint({v, v * 2.0f, v * 3.0f});
        cloud.addPoint({-v, -v * 2.0f, -v * 3.0f});
    }
    return cloud;
}

TEST(ProcessorTest, EmptyCloudReturnsZeroStats) {
    proc::Processor<core::PointXYZ> proc(2);
    core::PointCloud<core::PointXYZ> empty;
    auto s = proc.computeParallelStats(empty);
    EXPECT_EQ(s.count, 0u);
}

TEST(ProcessorTest, CorrectMeanForSymmetricCloud) {
    proc::Processor<core::PointXYZ> proc(2);
    auto cloud = makeSymmetricCloud(50);
    auto s     = proc.computeParallelStats(cloud);

    EXPECT_EQ(s.count, 100u);
    EXPECT_NEAR(s.mean_x, 0.0f, 1e-4f);
    EXPECT_NEAR(s.mean_y, 0.0f, 1e-4f);
    EXPECT_NEAR(s.mean_z, 0.0f, 1e-4f);
}

TEST(ProcessorTest, CorrectMinMaxForSymmetricCloud) {
    proc::Processor<core::PointXYZ> proc(4);
    auto cloud = makeSymmetricCloud(10);   // x in [-10, 10]
    auto s     = proc.computeParallelStats(cloud);

    EXPECT_FLOAT_EQ(s.min_x, -10.0f);
    EXPECT_FLOAT_EQ(s.max_x,  10.0f);
}

TEST(ProcessorTest, RGBStatsAreComputed) {
    core::PointCloud<core::PointXYZRGB> cloud;
    cloud.addPoint(core::PointXYZRGB{0.0f, 0.0f, 0.0f, 100, 200, 50});
    cloud.addPoint(core::PointXYZRGB{1.0f, 1.0f, 1.0f, 200, 100, 150});

    proc::Processor<core::PointXYZRGB> proc(2);
    auto s = proc.computeParallelStats(cloud);

    EXPECT_NEAR(s.avg_r, 150.0, 1e-9);
    EXPECT_NEAR(s.avg_g, 150.0, 1e-9);
    EXPECT_NEAR(s.avg_b, 100.0, 1e-9);
}

TEST(ProcessorTest, ObserverIsNotified) {
    struct CountingObserver : proc::IStatsObserver {
        int calls{0};
        void onStats(const proc::Stats&) override { ++calls; }
    };

    proc::Processor<core::PointXYZ> p(2);
    auto obs = std::make_shared<CountingObserver>();
    p.addObserver(obs);

    auto cloud = makeSymmetricCloud(5);
    p.computeParallelStats(cloud);
    EXPECT_EQ(obs->calls, 1);
}
