#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "proud_up/hunter.hpp"
#include "proud_up/target_source.hpp"

using proud_up::errors_from_pose;
using proud_up::Hunter;
using proud_up::HunterConfig;
using proud_up::HunterPhase;
using proud_up::kHunterPi;
using proud_up::LegCommandKind;
using proud_up::lidar_front;
using proud_up::pick_target;
using proud_up::PoseInBase;
using proud_up::SaveliSource;
using proud_up::ScanView;
using proud_up::TargetHit;
using proud_up::twist_is_zero;

namespace {

TargetHit saveli_at(double x, double y, double t = 0.0) {
  (void)t;
  TargetHit h;
  h.id = "saveli";
  h.display_name = "Saveli";
  h.spoken_wav = "saveli.wav";
  h.pose.x = x;
  h.pose.y = y;
  h.pose_in_base = true;
  h.pose.range_ok = true;
  return h;
}

HunterConfig walk_cfg() {
  HunterConfig c;
  c.enable_walk = true;
  c.enable_crab = false;
  c.enable_wander = false;
  c.follow_max_s = 60.0;
  c.lost_timeout = 0.5;
  c.search_timeout_s = 20.0;
  c.name_timeout_s = 0.2;
  c.control_dt = 0.05;
  return c;
}

}  // namespace

TEST(Hunter, BodyHeadingTagOnLeftIsPositive) {
  PoseInBase p;
  p.x = 1.0;
  p.y = 0.20;
  const auto e = errors_from_pose(p, 0.80);
  EXPECT_GT(e.theta, 0.0);
  EXPECT_NEAR(e.r, std::hypot(1.0, 0.20), 1e-9);
}

TEST(Hunter, DeadbandHaltsNotZeroTwist) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, saveli_at(0.80, 0.0), 2.0, true);
  h.notify_name_done();
  // Inside all deadbands at the 0.80 m standoff, on the nose.
  const auto o = h.tick(0.3, saveli_at(0.80, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
  EXPECT_TRUE(twist_is_zero(o.twist) || o.legs != LegCommandKind::Twist);
}

TEST(Hunter, RangeErrorCommandsForwardTwist) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.3, saveli_at(1.20, 0.0), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, saveli_at(1.20, 0.0), 2.0, true);
  }
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_EQ(o.legs, LegCommandKind::Twist);
  EXPECT_GT(o.twist.vx, 0.0);
  EXPECT_FALSE(twist_is_zero(o.twist));
}

TEST(Hunter, LidarHysteresisLatchesStop) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.3, saveli_at(1.20, 0.0), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, saveli_at(1.20, 0.0), 2.0, true);
  }
  o = h.tick(0.40, saveli_at(1.20, 0.0), 0.40, true);
  EXPECT_EQ(o.phase, HunterPhase::Stopped);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
  o = h.tick(0.45, saveli_at(1.20, 0.0), 0.60, true);
  EXPECT_EQ(o.phase, HunterPhase::Stopped);
  o = h.tick(0.50, saveli_at(1.20, 0.0), 0.75, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
}

TEST(Hunter, CrabGateUsesYNotThetaAlone) {
  auto cfg = walk_cfg();
  cfg.enable_crab = true;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, saveli_at(0.80, 0.12), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.3, saveli_at(0.80, 0.12), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, saveli_at(0.80, 0.12), 2.0, true);
  }
  EXPECT_TRUE(o.crab);
  EXPECT_GT(std::fabs(o.twist.vy), 0.0);

  // 40° off to the side: yaw first, no crab.
  const double x = 0.80 * std::cos(40.0 * kHunterPi / 180.0);
  const double y = 0.80 * std::sin(40.0 * kHunterPi / 180.0);
  Hunter h2(cfg);
  h2.start();
  h2.tick(0.0, saveli_at(x, y), 2.0, true);
  h2.notify_name_done();
  auto o2 = h2.tick(0.3, saveli_at(x, y), 2.0, true);
  if (o2.legs == LegCommandKind::SelectGait15) {
    o2 = h2.tick(0.35, saveli_at(x, y), 2.0, true);
  }
  EXPECT_FALSE(o2.crab);
  EXPECT_NEAR(o2.twist.vy, 0.0, 1e-12);
  EXPECT_GT(std::fabs(o2.twist.wz), 0.0);
}

TEST(Hunter, NameOnceThenSkipWithinSearchTimeout) {
  Hunter h(walk_cfg());
  h.start();
  auto o = h.tick(1.0, saveli_at(1.0, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_TRUE(o.play_name);
  h.notify_name_done();
  o = h.tick(1.3, saveli_at(1.0, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
  o = h.tick(2.0, std::nullopt, 2.0, true);  // lost
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  o = h.tick(2.6, saveli_at(1.0, 0.0), 2.0, true);
  EXPECT_NE(o.phase, HunterPhase::Name);
  EXPECT_FALSE(o.play_name);
}

TEST(Hunter, FollowMaxGoesToHuntEvenIfStillTracking) {
  auto cfg = walk_cfg();
  cfg.follow_max_s = 60.0;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, saveli_at(1.0, 0.0), 2.0, true);
  h.notify_name_done();
  h.tick(1.0, saveli_at(1.0, 0.0), 2.0, true);
  const auto o = h.tick(61.5, saveli_at(1.0, 0.0), 2.0, false);
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
}

TEST(Hunter, EnableWalkFalseNeverPublishesTwist) {
  auto cfg = walk_cfg();
  cfg.enable_walk = false;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  const auto o = h.tick(0.3, saveli_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_NE(o.legs, LegCommandKind::Twist);
  EXPECT_NE(o.legs, LegCommandKind::SelectGait15);
}

TEST(Hunter, WanderOpenWalksWallHaltsThenTurns) {
  auto cfg = walk_cfg();
  cfg.enable_wander = true;
  Hunter h(cfg);
  h.start();
  auto o = h.tick(0.0, std::nullopt, 2.0, true, 3.0, 3.0);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.05, std::nullopt, 2.0, true, 3.0, 3.0);
  }
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_EQ(o.legs, LegCommandKind::Twist);
  EXPECT_GT(o.twist.vx, 0.0);

  o = h.tick(0.10, std::nullopt, 0.40, true, 2.0, 0.5);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
  EXPECT_FALSE(twist_is_zero(o.twist) && o.legs == LegCommandKind::Twist);
  o = h.tick(0.15, std::nullopt, 0.40, true, 2.0, 0.5);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.20, std::nullopt, 0.40, true, 2.0, 0.5);
  }
  EXPECT_EQ(o.legs, LegCommandKind::Twist);
  EXPECT_GT(o.twist.wz, 0.0);
}

TEST(Hunter, WanderOffNeverWalksInHunt) {
  auto cfg = walk_cfg();
  cfg.enable_wander = false;
  Hunter h(cfg);
  h.start();
  const auto o = h.tick(0.0, std::nullopt, 3.0, true, 4.0, 4.0);
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_NE(o.legs, LegCommandKind::Twist);
}

TEST(Hunter, LidarFrontTransformsToBaseLink) {
  // One hit dead ahead at 0.40 m in lidar frame. lidar_x = 0.10 → ~0.50 m
  // from base_link, not "lidar metres".
  const float ranges[] = {0.40f};
  ScanView scan;
  scan.ranges = ranges;
  scan.n = 1;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_max = 10.0;
  HunterConfig cfg;
  cfg.lidar_x = 0.10;
  cfg.lidar_y = 0.0;
  cfg.lidar_sector_rad = kHunterPi;
  cfg.lidar_ignore_below = 0.20;
  const auto s = lidar_front(scan, cfg);
  EXPECT_TRUE(s.have_hit);
  EXPECT_NEAR(s.d_min, 0.50, 1e-6);
}

TEST(Hunter, SaveliPlugNeedsMatchingTag) {
  SaveliSource src(0, "saveli.wav");
  proud_up::DetectInput in;
  in.have_saveli_pose = true;
  in.tag_id = 0;
  in.saveli_pose.x = 1.0;
  auto hit = src.detect(in);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->id, "saveli");
  in.tag_id = 7;
  EXPECT_FALSE(src.detect(in).has_value());
}

TEST(Hunter, PickPrefersStickyId) {
  TargetHit a = saveli_at(1.0, 0.0);
  TargetHit b;
  b.id = "cat";
  std::vector<std::optional<TargetHit>> hits{a, b};
  const auto p = pick_target(hits, "cat", {"saveli", "cat"});
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->id, "cat");
}
