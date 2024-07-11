#include "Butterworth.hpp"
#include <gtest/gtest.h>
#include <ros/package.h>
#include <rosneuro_filters/rosneuro_filters_utilities.hpp>

namespace rosneuro {

class ButterworthTestSuite : public ::testing::Test {
    public:
        ButterworthTestSuite() {};
        ~ButterworthTestSuite() {};
        void SetUp() override {
            bw_filter = new Butterworth<double>();
        }
        void TearDown() override {
            delete bw_filter;
        }
        Butterworth <double>* bw_filter;
};

TEST_F(ButterworthTestSuite, Constructor){
    EXPECT_EQ(bw_filter->name_, "unknown");
    EXPECT_EQ(bw_filter->samplerate_, 0.0f);
    EXPECT_EQ(bw_filter->type_, static_cast<int>(rosneuro::ButterType::Unknown));
    EXPECT_EQ(bw_filter->order_, 0);
    EXPECT_EQ(bw_filter->cutoff_, 0.0f);
    EXPECT_EQ(bw_filter->filter_, nullptr);
    EXPECT_EQ(bw_filter->is_filter_configured_, false);
    EXPECT_EQ(bw_filter->is_filter_set_, false);
}

TEST_F(ButterworthTestSuite, ConstructorWithParameters){
    bw_filter = new Butterworth<double>(ButterType::LowPass, 2, 1.0, 2.0);
    EXPECT_EQ(bw_filter->name_, "lowpass");
    EXPECT_EQ(bw_filter->samplerate_, 2.0);
    EXPECT_EQ(bw_filter->type_, static_cast<int>(ButterType::LowPass));
    EXPECT_EQ(bw_filter->order_, 2);
    EXPECT_EQ(bw_filter->cutoff_, 1.0);
    EXPECT_EQ(bw_filter->filter_, nullptr);
    EXPECT_EQ(bw_filter->is_filter_configured_, true);
    EXPECT_EQ(bw_filter->is_filter_set_, false);

    bw_filter = new Butterworth<double>(ButterType::HighPass, 2, 1.0, 2.0);
    EXPECT_EQ(bw_filter->name_, "highpass");

    bw_filter = new Butterworth<double>(ButterType::Unknown, 2, 1.0, 2.0);
    EXPECT_EQ(bw_filter->name_, "unknown");
}

TEST_F(ButterworthTestSuite, Setters){
    bw_filter = new Butterworth<double>(ButterType::LowPass, 2, 1.0, 2.0);
    EXPECT_TRUE(bw_filter->create_butterworth_filter(2));
}

TEST_F(ButterworthTestSuite, Get){
    EXPECT_EQ(bw_filter->type(), -1);
    EXPECT_EQ(bw_filter->order(), 0);
    EXPECT_EQ(bw_filter->cutoff(), 0);
    EXPECT_EQ(bw_filter->samplerate(), 0);
}

TEST_F(ButterworthTestSuite, DumpTest) {
    testing::internal::CaptureStdout();
    bw_filter->dump();
    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_NE(output.find("Filter configuration"), std::string::npos);
    EXPECT_NE(output.find("Samplerate"), std::string::npos);
    EXPECT_NE(output.find("Filter type"), std::string::npos);
    EXPECT_NE(output.find("Filter order"), std::string::npos);
    EXPECT_NE(output.find("Filter cutoff"), std::string::npos);
}

TEST_F(ButterworthTestSuite, Configure){
    bw_filter->params_["order"] = XmlRpc::XmlRpcValue(2);
    bw_filter->params_["samplerate"] = XmlRpc::XmlRpcValue(2.0);
    bw_filter->params_["cutoff"] = XmlRpc::XmlRpcValue(1.0);

    bw_filter->params_["type"] = XmlRpc::XmlRpcValue("lowpass");
    EXPECT_TRUE(bw_filter->configure());
    EXPECT_TRUE(bw_filter->is_filter_configured_);

    bw_filter->params_["type"] = XmlRpc::XmlRpcValue("highpass");
    EXPECT_TRUE(bw_filter->configure());
    EXPECT_TRUE(bw_filter->is_filter_configured_);

    bw_filter->params_["type"] = XmlRpc::XmlRpcValue("unknown");
    EXPECT_FALSE(bw_filter->configure());
}

TEST_F(ButterworthTestSuite, Apply){
    DynamicMatrix<double> in(3, 5);
    in << 1, 2, 3, 4, 5,
          6, 7, 8, 9, 10,
          11, 12, 13, 14, 15;

    bw_filter->is_filter_configured_ = true;
    EXPECT_EQ(bw_filter->apply(in), in);

    bw_filter->is_filter_configured_ = false;
    EXPECT_THROW(bw_filter->apply(in), std::runtime_error);
}

TEST_F(ButterworthTestSuite, Integration){
    int frame_size = 32;
    double sample_rate = 512;
    int order_lp = 4;
    int order_hp = 4;
    double cutoff_lp = 10;
    double cutoff_hp = 1;

    std::string base_path = ros::package::getPath("rosneuro_filters_butterworth");
    const std::string input_path  = base_path + "/test/rawdata.csv";
    const std::string path_expected_lp  = base_path + "/test/expected_lp.csv";
    const std::string path_expected_hp  = base_path + "/test/expected_hp.csv";

    DynamicMatrix<double> input = readCSV<double>(input_path);
    DynamicMatrix<double> expected_lp = readCSV<double>(path_expected_lp);
    DynamicMatrix<double> expected_hp = readCSV<double>(path_expected_hp);

    int nsamples  = input.rows();
    int nchannels = input.cols();

    DynamicMatrix<double> output_lp = DynamicMatrix<double>::Zero(nsamples, nchannels);
    DynamicMatrix<double> output_hp = DynamicMatrix<double>::Zero(nsamples, nchannels);

    Butterworth<double> butter_lp(ButterType::LowPass,  order_lp, cutoff_lp, sample_rate);
    Butterworth<double> butter_hp(ButterType::HighPass, order_hp, cutoff_hp, sample_rate);

    DynamicMatrix<double> frame = DynamicMatrix<double>::Zero(frame_size, nchannels);

    for(auto i = 0; i<nsamples; i = i+frame_size) {
        frame = input.middleRows(i, frame_size);
        output_lp.middleRows(i, frame_size) = butter_lp.apply(frame);
        output_hp.middleRows(i, frame_size) = butter_hp.apply(frame);
    }

    ASSERT_TRUE(expected_hp.isApprox(output_hp, 1e-6));
    ASSERT_TRUE(expected_lp.isApprox(output_lp, 1e-6));
}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_butterworth");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}