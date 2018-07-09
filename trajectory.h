#pragma once
#include "linalg.h"
#include <track.h>
#include <vector>

namespace kartik_ns
{
class Segment
{
 public:
  Segment(v2d const &l, v2d const &r, double length, double alpha, int type)
      : left_(l), right_(r), length_(length), type_(type), alpha_(alpha)
  {
    updateMid();
  }

  double getAlpha() { return alpha_; }

  void setAlpha(double alpha)
  {
    alpha_ = alpha;
    updateMid();
  }

  v2d const &getMid() { return mid_; }

  v2d const left_, right_;
  double const length_;
  int const type_;

 private:
  void updateMid() { mid_ = alpha_ * right_ + (1 - alpha_) * left_; }

  v2d mid_;
  double alpha_;
};

class Trajectory
{
 public:
  void processTrack(tTrack const *trk, tdble min_alpha, tdble max_alpha);
  v2d getPoint(int seg_id, tdble length_along_segment);

 private:
  double gradient(double v, double alpha, v2d const &l, v2d const &r,
                  v2d const &P0, v2d const &P2, v2d const &P3, double s1,
                  double s2, double s3, double orig_alpha);

  std::vector<Segment> segment_data_;
  int nseg_;
};
} // namespace kartik_ns
