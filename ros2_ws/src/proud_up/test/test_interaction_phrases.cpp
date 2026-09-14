#include <gtest/gtest.h>

#include "proud_up/interaction_phrases.hpp"

using proud_up::is_master_query;
using proud_up::is_stop_command;
using proud_up::kMasterQueryCanonical;
using proud_up::normalize_speech;

// ASR already publishes this string. Exact match is the live path.
TEST(InteractionPhrases, CanonicalMasterQuery) {
  EXPECT_TRUE(is_master_query(kMasterQueryCanonical));
  EXPECT_TRUE(is_master_query("who is your master"));
}

// extract_command strips punctuation before publish, so "who's" is "who s".
TEST(InteractionPhrases, WhosContraction) {
  EXPECT_EQ(normalize_speech("who's your master"), "who s your master");
  EXPECT_TRUE(is_master_query("who's your master"));
  EXPECT_TRUE(is_master_query("who s your master"));
}

// Spoken protocol includes her name as address, not as a new wake word.
TEST(InteractionPhrases, LeadingMasha) {
  EXPECT_TRUE(is_master_query("Masha, who is your master"));
  EXPECT_TRUE(is_master_query("masha who is your master"));
}

TEST(InteractionPhrases, OtherAliases) {
  EXPECT_TRUE(is_master_query("who your master"));
  EXPECT_TRUE(is_master_query("who is ur master"));
  EXPECT_TRUE(is_master_query("whose your master"));
}

TEST(InteractionPhrases, WalkAndDanceAreNotMaster) {
  EXPECT_FALSE(is_master_query("go forward"));
  EXPECT_FALSE(is_master_query("dance"));
  EXPECT_FALSE(is_master_query("stop"));
  EXPECT_FALSE(is_master_query(""));
  EXPECT_FALSE(is_master_query("hello masha"));
}

TEST(InteractionPhrases, StopCanonicalAndWord) {
  EXPECT_TRUE(is_stop_command("stop"));
  EXPECT_TRUE(is_stop_command("Stop!"));
  EXPECT_TRUE(is_stop_command("please stop"));
  EXPECT_FALSE(is_stop_command("who is your master"));
  EXPECT_FALSE(is_stop_command("go forward"));
}
