//
// Created by tipakorng on 6/6/16.
//

#include <gtest/gtest.h>
#include <cwru_opencv_common/image_label.h>
#include <cwru_opencv_common/manual_labeling.h>

TEST(ManualLabelingTest, testManualLabelingCallback) {
    // TODO: Implement tests which require neither roscore nor human interaction
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "gtest_manual_labeling");
    return RUN_ALL_TESTS();
}