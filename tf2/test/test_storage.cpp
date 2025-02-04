// Copyright (c) 2024 Dexory. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/time.hpp"
#include "tf2/transform_storage.hpp"

class TransformStorageTest : public ::testing::Test
{
protected:
  tf2::TransformStorage createTransformStorage()
  {
    const tf2::CompactFrameID frame_id(0);
    const tf2::CompactFrameID child_frame_id(1);
    const tf2::TimePoint stamp(tf2::TimePointZero);
    const tf2::Quaternion rotation(0.0, 0.0, 0.0, 1.0);
    const tf2::Vector3 translation(0.0, 0.0, 0.0);
    return tf2::TransformStorage(stamp, rotation, translation, frame_id, child_frame_id);
  }
};

TEST_F(TransformStorageTest, EqualityOperator) {
  // Create a dummy storage, set to identity
  tf2::TransformStorage transformStorage1 = createTransformStorage();

  // tf2::Quaternion rotation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.rotation_.setValue(1.0, 0.0, 0.0, 0.0);
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // tf2::Vector3 translation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // TimePoint stamp_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.stamp_ = tf2::TimePoint(tf2::durationFromSec(1.0));
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // CompactFrameID frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.frame_id_ = 55;
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // CompactFrameID child_frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
}

TEST_F(TransformStorageTest, InequalityOperator) {
  // Create a dummy storage, set to identity
  tf2::TransformStorage transformStorage1 = createTransformStorage();

  // tf2::Quaternion rotation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.rotation_.setValue(1.0, 0.0, 0.0, 0.0);
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // tf2::Vector3 translation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // TimePoint stamp_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.stamp_ = tf2::TimePoint(tf2::durationFromSec(1.0));
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // CompactFrameID frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.frame_id_ = 55;
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // CompactFrameID child_frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
}
