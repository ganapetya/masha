// Policy tests — no ROS, no robot. Each TEST constructs a Hunter, calls
// start() then tick() with hand-made TargetHit poses, and checks the
// phase / LegCommandKind / Twist. If a comment in hunter.cpp and a test
// here disagree, the test is the contract.
//
// Typical sequence a test uses (mirrors the live node):
//   h.start()                         Idle → Hunt
//   h.tick(t, saveli_at(x,y), d_min)  Hunt → Name (play_name)
//   h.notify_name_done()              Name → Follow on the next tick
//   h.tick(...)                       Follow: SelectGait15 then Twist
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "proud_up/hunter.hpp"
#include "proud_up/reach_greet.hpp"
#include "proud_up/target_source.hpp"

using proud_up::CatConfirmState;
using proud_up::confirm_cat_hit;
using proud_up::errors_from_pose;
using proud_up::Hunter;
using proud_up::HunterConfig;
using proud_up::HunterPhase;
using proud_up::kCatConfirmTicks;
using proud_up::kHunterPi;
using proud_up::LegCommandKind;
using proud_up::legs_for_metric_target;
using proud_up::lidar_front;
using proud_up::pick_target;
using proud_up::PoseInBase;
using proud_up::SaveliSource;
using proud_up::ScanView;
using proud_up::advance_reach_greet;
using proud_up::kGripperClose;
using proud_up::kGripperOpen;
using proud_up::ReachArm;
using proud_up::ReachGreet;
using proud_up::ReachGreetOutput;
using proud_up::ReachPhase;
using proud_up::tag_target_policy;
using proud_up::TargetHit;
using proud_up::TargetPolicy;
using proud_up::twist_is_zero;
using proud_up::vision_target_policy;

namespace {

TargetHit saveli_at(double x, double y, double t = 0.0) {
  (void)t;
  TargetHit h;
  h.id = "saveli";
  h.display_name = "Saveli";
  h.spoken_wav = "saveli.wav";
  h.policy = tag_target_policy();
  h.pose.x = x;
  h.pose.y = y;
  h.pose_in_base = true;
  h.pose.range_ok = true;
  return h;
}

TargetHit cat_at(double x, double y, bool metric = true) {
  TargetHit h = saveli_at(x, y);
  h.id = "cat";
  h.display_name = "cat";
  h.spoken_wav = "call-kitten.mp3";
  h.policy = vision_target_policy();
  h.pose.has_pixel = true;
  h.pose.u = 200.0;
  h.pose.v = 180.0;
  h.pose_in_base = metric;
  h.pose.range_ok = metric;
  return h;
}

HunterConfig walk_cfg() {
  HunterConfig c;
  c.enable_walk = true;
  c.enable_crab = false;
  c.enable_wander = false;
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

// The old 60 s cap left Follow and forced one pan while the tag was
// still in view, so a later plug could be discovered by looking away.
// A held lock stays in Follow. The pan starts only after lost_timeout.
TEST(Hunter, FollowStaysWhileTargetIsHeld) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, saveli_at(1.0, 0.0), 2.0, true);
  h.notify_name_done();
  h.tick(1.0, saveli_at(1.0, 0.0), 2.0, true);
  const auto o = h.tick(120.0, saveli_at(1.0, 0.0), 2.0, false);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.pan_head);
  EXPECT_EQ(o.sticky_id, "saveli");
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
  EXPECT_NEAR(s.d_min_bearing, 0.0, 1e-6);
}

TEST(Hunter, LidarFrontKeepsASideWallInsideTheFrontWedge) {
  // Default sector is ±45° from the body, not from the lidar puck.
  // A ray at 50° in the lidar frame, 0.47 m out, lands near 42° from
  // base_link because the puck sits 10 cm forward. That is still
  // "in front", so a wall beside her can be the stop.
  const float side[] = {0.47f};
  ScanView scan;
  scan.ranges = side;
  scan.n = 1;
  scan.angle_min = 50.0 * kHunterPi / 180.0;
  scan.angle_increment = 0.0;
  scan.range_max = 10.0;
  HunterConfig cfg;
  cfg.lidar_x = 0.102;
  cfg.lidar_ignore_below = 0.20;
  const auto kept = lidar_front(scan, cfg);
  EXPECT_TRUE(kept.have_hit);
  EXPECT_LT(kept.d_min, 0.55);
  EXPECT_GT(kept.d_min_bearing, 35.0 * kHunterPi / 180.0);
  EXPECT_LT(kept.d_min_bearing, 45.0 * kHunterPi / 180.0);

  // Farther around, past the wedge, the same range is not a stop.
  scan.angle_min = 80.0 * kHunterPi / 180.0;
  const auto dropped = lidar_front(scan, cfg);
  EXPECT_FALSE(dropped.have_hit);
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
  EXPECT_EQ(hit->policy.preempt_rank, 0);
  EXPECT_FALSE(hit->policy.exclusive_coast);
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

TEST(Hunter, FollowMissCoastsTwistUntilLostTimeout) {
  auto cfg = walk_cfg();
  cfg.enable_crab = true;
  cfg.lost_timeout = 1.5;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, saveli_at(0.80, 0.12), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.3, saveli_at(0.80, 0.12), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, saveli_at(0.80, 0.12), 2.0, true);
  }
  ASSERT_EQ(o.phase, HunterPhase::Follow);
  ASSERT_EQ(o.legs, LegCommandKind::Twist);
  const double vy = o.twist.vy;
  EXPECT_NE(vy, 0.0);

  // 0.2 s miss: stay in Follow and keep the last crab Twist so the
  // walk watchdog does not stand the legs.
  o = h.tick(0.55, std::nullopt, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_EQ(o.legs, LegCommandKind::Twist);
  EXPECT_NEAR(o.twist.vy, vy, 1e-12);
  EXPECT_FALSE(o.pan_head);

  o = h.tick(1.90, std::nullopt, 2.0, true);  // 1.55 s since last hit
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
  EXPECT_TRUE(o.pan_head);
}

TEST(Hunter, NotifyHaltedReselectsGait) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.3, saveli_at(1.20, 0.0), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, saveli_at(1.20, 0.0), 2.0, true);
  }
  ASSERT_EQ(o.legs, LegCommandKind::Twist);
  h.notify_halted();
  o = h.tick(0.40, saveli_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_EQ(o.legs, LegCommandKind::SelectGait15);
}

TEST(Hunter, CatBeatsStickySaveli) {
  TargetHit saveli = saveli_at(1.0, 0.0);
  TargetHit cat = cat_at(1.1, 0.1);
  std::vector<std::optional<TargetHit>> hits{saveli, cat};
  const auto p = pick_target(hits, "saveli", {"saveli", "cat"}, tag_target_policy());
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->id, "cat");
}

TEST(Hunter, CatStickyHidesSaveliUntilReleased) {
  std::vector<std::optional<TargetHit>> hits{saveli_at(1.0, 0.0), std::nullopt};
  // The hold is the vision policy, not the spelling of the id.
  EXPECT_FALSE(
      pick_target(hits, "cat", {"saveli", "cat"}, vision_target_policy()).has_value());
}

TEST(Hunter, CatTakesOverSaveliAndNames) {
  Hunter h(walk_cfg());
  h.start();
  auto o = h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_TRUE(o.play_name);
  h.notify_name_done();
  o = h.tick(0.30, saveli_at(1.20, 0.0), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, saveli_at(1.20, 0.0), 2.0, true);
  }
  ASSERT_EQ(o.phase, HunterPhase::Follow);

  o = h.tick(0.50, cat_at(1.10, 0.10), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_TRUE(o.play_name);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
  EXPECT_EQ(o.sticky_id, "cat");

  h.notify_name_done();
  o = h.tick(0.80, cat_at(1.10, 0.10), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
  EXPECT_EQ(o.sticky_id, "cat");
}

TEST(Hunter, SaveliDoesNotCutInDuringCatCoast) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, cat_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.30, cat_at(1.20, 0.0), 2.0, true);
  if (o.legs == LegCommandKind::SelectGait15) {
    o = h.tick(0.35, cat_at(1.20, 0.0), 2.0, true);
  }
  ASSERT_EQ(o.phase, HunterPhase::Follow);
  ASSERT_EQ(o.sticky_id, "cat");

  // walk_cfg lost_timeout is 0.5 s. This Saveli pose is inside the coast.
  o = h.tick(0.50, saveli_at(0.90, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_EQ(o.sticky_id, "cat");
  EXPECT_FALSE(o.play_name);

  o = h.tick(1.00, saveli_at(0.90, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_TRUE(o.sticky_id.empty());

  o = h.tick(1.10, saveli_at(0.90, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_TRUE(o.play_name);
  EXPECT_EQ(o.sticky_id, "saveli");
}

TEST(Hunter, CatReacquireInsideQuietWindowDoesNotRename) {
  Hunter h(walk_cfg());
  h.start();
  auto o = h.tick(0.0, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_TRUE(o.play_name);
  h.notify_name_done();
  h.tick(0.30, cat_at(1.20, 0.0), 2.0, true);
  o = h.tick(1.00, std::nullopt, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_TRUE(o.sticky_id.empty());
  o = h.tick(1.10, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
  EXPECT_EQ(o.sticky_id, "cat");
}

TEST(Hunter, PixelCatStillNamesButDoesNotWalk) {
  Hunter h(walk_cfg());
  h.start();
  const auto o = h.tick(0.0, cat_at(0.0, 0.0, false), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_TRUE(o.play_name);

  TargetHit guessed = cat_at(1.2, 0.0, false);
  EXPECT_EQ(legs_for_metric_target(LegCommandKind::Twist, guessed), LegCommandKind::None);
  EXPECT_EQ(legs_for_metric_target(LegCommandKind::SelectGait15, guessed), LegCommandKind::None);
  EXPECT_EQ(legs_for_metric_target(LegCommandKind::Halt, guessed), LegCommandKind::Halt);
  guessed.pose_in_base = true;
  guessed.pose.range_ok = false;
  EXPECT_EQ(legs_for_metric_target(LegCommandKind::Twist, guessed), LegCommandKind::None);
  guessed.pose.range_ok = true;
  EXPECT_EQ(legs_for_metric_target(LegCommandKind::Twist, guessed), LegCommandKind::Twist);
  EXPECT_EQ(legs_for_metric_target(LegCommandKind::Twist, saveli_at(1.2, 0.0)),
            LegCommandKind::Twist);
}

TEST(Hunter, ConfirmCatNeedsTwoCentresAndKeepsOneMiss) {
  CatConfirmState st;
  const TargetHit cat = cat_at(1.0, 0.0);
  for (int i = 0; i < kCatConfirmTicks - 1; ++i) {
    EXPECT_FALSE(confirm_cat_hit(st, cat, 40.0).has_value());
  }
  const auto ok = confirm_cat_hit(st, cat, 40.0);
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(ok->id, "cat");

  // One empty frame keeps the streak. The next centre is still a lock.
  EXPECT_FALSE(confirm_cat_hit(st, std::nullopt, 40.0).has_value());
  const auto still = confirm_cat_hit(st, cat, 40.0);
  ASSERT_TRUE(still.has_value());

  // Two empty frames clear it, so a single later centre does not lock.
  EXPECT_FALSE(confirm_cat_hit(st, std::nullopt, 40.0).has_value());
  EXPECT_FALSE(confirm_cat_hit(st, std::nullopt, 40.0).has_value());
  EXPECT_FALSE(confirm_cat_hit(st, cat, 40.0).has_value());
  EXPECT_TRUE(confirm_cat_hit(st, cat, 40.0).has_value());

  CatConfirmState jumped;
  EXPECT_FALSE(confirm_cat_hit(jumped, cat, 40.0).has_value());
  TargetHit far = cat;
  far.pose.u = 400.0;
  EXPECT_FALSE(confirm_cat_hit(jumped, far, 40.0).has_value());
  for (int i = 0; i < kCatConfirmTicks - 2; ++i) {
    EXPECT_FALSE(confirm_cat_hit(jumped, far, 40.0).has_value());
  }
  EXPECT_TRUE(confirm_cat_hit(jumped, far, 40.0).has_value());
}

// call-kitten again after 5 s of quiet, while this sighting lasts.
// The repeat stays in Follow: legs are not forced back through Name.
TEST(Hunter, CatGreetingLoopsAfterFiveSecondsOfSilence) {
  auto cfg = walk_cfg();
  cfg.cat_greet_silence_s = 5.0;
  Hunter h(cfg);
  h.start();
  auto o = h.tick(0.0, cat_at(1.20, 0.0), 2.0, true);
  ASSERT_EQ(o.phase, HunterPhase::Name);
  ASSERT_TRUE(o.play_name);
  h.notify_name_done();

  o = h.tick(4.9, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);

  o = h.tick(5.0, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_TRUE(o.play_name);
  EXPECT_EQ(o.legs, LegCommandKind::Twist);

  o = h.tick(5.05, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_FALSE(o.play_name);

  h.notify_name_done();
  o = h.tick(10.0, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_FALSE(o.play_name);
  o = h.tick(10.05, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_TRUE(o.play_name);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
}

TEST(Hunter, CatGreetingSurvivesAShortMiss) {
  auto cfg = walk_cfg();
  cfg.lost_timeout = 1.5;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, cat_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  h.tick(4.0, cat_at(1.20, 0.0), 2.0, true);
  auto o = h.tick(4.4, std::nullopt, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
  o = h.tick(5.0, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_TRUE(o.play_name);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
}

TEST(Hunter, CatGreetingStopsAfterTheSightingIsLost) {
  auto cfg = walk_cfg();
  cfg.lost_timeout = 1.5;
  cfg.search_timeout_s = 20.0;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, cat_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  h.tick(0.30, cat_at(1.20, 0.0), 2.0, true);
  auto o = h.tick(2.0, std::nullopt, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  o = h.tick(8.0, cat_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
}

// A dog is not a new branch in tick. Same vision policy, different id.
// A second tag robot is tag_target_policy with its own id: it does not
// steal a vision coast, and it does not repeat its name clip.
TEST(Hunter, DogAndOtherTagUsePolicyNotANewBranch) {
  TargetHit dog = cat_at(1.10, 0.10);
  dog.id = "dog";
  dog.display_name = "dog";
  dog.spoken_wav = "call-dog.mp3";
  TargetHit mira = saveli_at(0.90, 0.0);
  mira.id = "mira";
  mira.display_name = "Mira";

  const auto picked =
      pick_target({saveli_at(1.0, 0.0), dog}, "saveli", {"saveli", "dog"}, tag_target_policy());
  ASSERT_TRUE(picked.has_value());
  EXPECT_EQ(picked->id, "dog");
  EXPECT_FALSE(pick_target({mira, std::nullopt}, "dog", {"mira", "dog"}, vision_target_policy())
                   .has_value());

  auto cfg = walk_cfg();
  cfg.lost_timeout = 1.5;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.30, dog, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_TRUE(o.play_name);
  EXPECT_EQ(o.sticky_id, "dog");

  h.notify_name_done();
  o = h.tick(0.50, dog, 2.0, true);
  ASSERT_EQ(o.phase, HunterPhase::Follow);
  o = h.tick(0.70, mira, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_EQ(o.sticky_id, "dog");
  EXPECT_FALSE(o.play_name);

  o = h.tick(2.20, mira, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Hunt);
  EXPECT_TRUE(o.sticky_id.empty());
  o = h.tick(2.30, mira, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Name);
  EXPECT_EQ(o.sticky_id, "mira");
  h.notify_name_done();
  o = h.tick(12.0, mira, 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
}

// 15 cm is the follow standoff. A LiDAR return on the target does not
// count as a wall, so she is allowed to arrive. A return well in front
// of a far target still latches Stopped, and that stop does not greet.
TEST(Hunter, StandoffIsFifteenCentimetresNotTheWall) {
  auto cfg = walk_cfg();
  cfg.r_target = 0.15;
  cfg.d_stop = 0.55;
  cfg.d_go = 0.70;
  Hunter h(cfg);
  h.start();
  h.tick(0.0, saveli_at(0.15, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(0.30, saveli_at(0.15, 0.0), 0.14, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_TRUE(o.at_standoff);
  EXPECT_EQ(o.legs, LegCommandKind::Halt);
  EXPECT_FALSE(o.pan_head);

  o = h.tick(0.40, saveli_at(1.20, 0.0), 0.40, true);
  EXPECT_EQ(o.phase, HunterPhase::Stopped);
  EXPECT_FALSE(o.at_standoff);
}

TEST(Hunter, ReachGreetGrabsTwiceThenReturnsHome) {
  ReachArm home;
  home.id19 = 500.0f;
  home.id20 = 810.0f;
  home.id22 = 117.0f;
  home.id24 = 500.0f;
  ReachGreet g;
  int closes = 0;
  bool saw_reach = false;
  bool saw_home = false;
  ReachGreetOutput step;
  for (int i = 0; i <= 40; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    step = advance_reach_greet(g, true, t, home, 520.0f);
    if (!step.publish) {
      continue;
    }
    if (step.arm.id20 > home.id20 && step.arm.id24 == kGripperOpen && !saw_reach) {
      saw_reach = true;
      EXPECT_FLOAT_EQ(step.arm.id19, 520.0f);
    }
    if (step.arm.id24 == kGripperClose) {
      closes += 1;
    }
    if (step.arm.id20 == home.id20 && step.arm.id24 == home.id24 &&
        step.arm.id22 == home.id22) {
      saw_home = true;
    }
  }
  EXPECT_TRUE(saw_reach);
  EXPECT_EQ(closes, 2);
  EXPECT_TRUE(saw_home);
  EXPECT_EQ(g.phase, ReachPhase::Hold);
  EXPECT_TRUE(step.own_arm);

  const auto left = advance_reach_greet(g, false, 10.0, home, 500.0f);
  EXPECT_FALSE(left.own_arm);
  EXPECT_EQ(g.phase, ReachPhase::Idle);
}

TEST(Hunter, SaveliGreetingDoesNotLoop) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, saveli_at(1.20, 0.0), 2.0, true);
  h.notify_name_done();
  auto o = h.tick(10.0, saveli_at(1.20, 0.0), 2.0, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
}

TEST(Hunter, CatGreetingLoopsWhileStandingClose) {
  Hunter h(walk_cfg());
  h.start();
  h.tick(0.0, cat_at(0.40, 0.0), 0.40, true);
  h.notify_name_done();
  // The return matches the cat, so it is not a wall. She stays in Follow.
  // The name clip still repeats while she is there.
  auto o = h.tick(0.30, cat_at(0.40, 0.0), 0.40, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_FALSE(o.play_name);
  o = h.tick(5.0, cat_at(0.40, 0.0), 0.40, true);
  EXPECT_EQ(o.phase, HunterPhase::Follow);
  EXPECT_TRUE(o.play_name);
}
