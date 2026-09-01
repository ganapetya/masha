#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <utility>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "proud_up/camera_frame.hpp"
#include "proud_up/load_dotenv.hpp"
#include "proud_up/mailer.hpp"
#include "proud_up/pose_commander.hpp"

TEST(CameraFrame, UniqueThenSharedOwnership) {
  const cv::Mat img(24, 32, CV_8UC3, cv::Scalar(0, 0, 255));
  std::unique_ptr<proud_up::CameraFrame> unique_frame = proud_up::encode_jpeg(img, 80);
  ASSERT_NE(unique_frame, nullptr);
  EXPECT_EQ(unique_frame->width, 32);
  EXPECT_EQ(unique_frame->height, 24);
  ASSERT_GE(unique_frame->jpeg.size(), 32u);
  EXPECT_EQ(unique_frame->jpeg[0], 0xFF);
  EXPECT_EQ(unique_frame->jpeg[1], 0xD8);

  std::unique_ptr<proud_up::CameraFrame> moved = std::move(unique_frame);
  EXPECT_EQ(unique_frame, nullptr);
  ASSERT_NE(moved, nullptr);

  std::shared_ptr<const proud_up::CameraFrame> shared = std::move(moved);
  EXPECT_EQ(moved, nullptr);
  EXPECT_EQ(shared.use_count(), 1);

  const auto disk_consumer = shared;
  const auto smtp_consumer = shared;
  EXPECT_EQ(shared.use_count(), 3);
  EXPECT_EQ(disk_consumer->jpeg.size(), smtp_consumer->jpeg.size());
}

TEST(CameraFrame, EmptyMatYieldsNullUniquePtr) {
  EXPECT_EQ(proud_up::encode_jpeg(cv::Mat(), 80), nullptr);
}

TEST(PoseCommander, EmitsArmServos19To24) {
  proud_up::ArmPulses pose;
  pose.id20 = 810.0f;
  pose.duration_s = 1.5;
  const auto msg = proud_up::make_arm_command(pose);
  EXPECT_EQ(msg.position_unit, "pulse");
  EXPECT_DOUBLE_EQ(msg.duration, 1.5);
  ASSERT_EQ(msg.position.size(), 6u);
  EXPECT_EQ(msg.position[0].id, 19);
  EXPECT_EQ(msg.position[1].id, 20);
  EXPECT_FLOAT_EQ(msg.position[1].position, 810.0f);
  EXPECT_EQ(msg.position[5].id, 24);
}

TEST(Mailer, UnconfiguredWithoutHostOrTo) {
  proud_up::SmtpConfig cfg;
  proud_up::Mailer mailer(cfg);
  EXPECT_FALSE(mailer.configured());

  cfg.host = "smtp.example.com";
  cfg.to = "peter@example.com";
  proud_up::Mailer ready(cfg);
  EXPECT_TRUE(ready.configured());
}

TEST(Dotenv, ParsesQuotedAndExportLines) {
  proud_up::EnvAssignment assignment;
  EXPECT_FALSE(proud_up::parse_dotenv_line("# comment", assignment));
  EXPECT_FALSE(proud_up::parse_dotenv_line("   ", assignment));
  ASSERT_TRUE(proud_up::parse_dotenv_line("export MASHA_SMTP_HOST=smtp.example.com", assignment));
  EXPECT_EQ(assignment.key, "MASHA_SMTP_HOST");
  EXPECT_EQ(assignment.value, "smtp.example.com");
  ASSERT_TRUE(proud_up::parse_dotenv_line("MASHA_SMTP_PASSWORD='replace-me'", assignment));
  EXPECT_EQ(assignment.key, "MASHA_SMTP_PASSWORD");
  EXPECT_EQ(assignment.value, "replace-me");
}

TEST(Dotenv, LoadsFileWithoutOverridingExisting) {
  const std::string path = "/tmp/proud_up_test.env";
  std::ofstream out(path);
  ASSERT_TRUE(out);
  out << "PROUD_UP_TEST_NEW=from-file\n";
  out << "PROUD_UP_TEST_KEEP=from-file\n";
  out.close();

  setenv("PROUD_UP_TEST_KEEP", "already-set", 1);
  unsetenv("PROUD_UP_TEST_NEW");
  ASSERT_EQ(proud_up::load_dotenv_file(path), 2);
  EXPECT_STREQ(std::getenv("PROUD_UP_TEST_NEW"), "from-file");
  EXPECT_STREQ(std::getenv("PROUD_UP_TEST_KEEP"), "already-set");
  unlink(path.c_str());
}
