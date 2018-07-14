#include "trajectory.h"

namespace kartik_ns
{
void Trajectory::processTrack(tTrack const *trk, tdble min_alpha,
                              tdble max_alpha)
{
  nseg_ = trk->nseg;
  printf("min_alpha: %f, max_alpha: %f\n", min_alpha, max_alpha);

  std::vector<tTrackSeg const *> segments;
  {
    tTrackSeg *seg = trk->seg;
    // Switch seg to seg 0 for sure.
    while(seg->id != 0)
    {
      seg = seg->prev;
    }

    segments.reserve(nseg_);
    for(int seg_idx = 0; seg_idx < nseg_; ++seg_idx, seg = seg->next)
    {
      if(seg->id != seg_idx)
        printf("********** seg->id != seg_idx **********\n");
      segments.push_back(seg);
    }
  }
  segment_data_.clear();
  segment_data_.reserve(nseg_);
  std::vector<double> orig_alphas;
  orig_alphas.reserve(nseg_);
  for(int i = 0; i < nseg_; ++i)
  {
    tTrackSeg const *seg = segments[i];
    tdble alpha = 0.5;
    if(seg->type == 3) // Straight
    {
      // Find next turn
      for(int j = 0; j < nseg_; ++j)
      {
        int idx_pos = (i + j) % nseg_;
        int idx_neg = (i - j) % nseg_;
        if(idx_neg < 0)
          idx_neg += nseg_;
        if(segments[idx_pos]->type == 1 ||
           segments[idx_neg]->type == 1) // next/prev is right
        {
          alpha = min_alpha;
          break;
        }
        else if(segments[idx_pos]->type == 2 ||
                segments[idx_neg]->type == 2) // next/prev is left
        {
          alpha = max_alpha;
          break;
        }
      }
    }
#if 0
    else if(seg->type == 1) // Right
      alpha = min_alpha;
    else if(seg->type == 2) // Left
      alpha = max_alpha;
#endif

    v2d left(seg->vertex[TR_SL].x, seg->vertex[TR_SL].y);
    v2d right(seg->vertex[TR_SR].x, seg->vertex[TR_SR].y);
    Segment s(left, right, seg->length, alpha, seg->type);
    segment_data_.push_back(s);
    orig_alphas.push_back(alpha);
  }
  printf("num_segs: %d\n", nseg_);

  const unsigned int NUM_TRACK_ITER = 100;
  const unsigned int NUM_OPT_ITER = 20;
  for(unsigned int i = 0; i < NUM_TRACK_ITER * nseg_; ++i)
  {
    //printf("i: %d\n", i);
    int idx0 = (i - 1) % nseg_;
    if(idx0 < 0)
      idx0 += nseg_;
    int idx1 = i % nseg_;
    int idx2 = (i + 1) % nseg_;
    int idx3 = (i + 2) % nseg_;
    double s1 = segment_data_[idx0].length_;
    double s2 = s1 + segment_data_[idx1].length_;
    double s3 = s2 + segment_data_[idx2].length_;

    double g_min = 1e100;
    double alpha = segment_data_[idx1].getAlpha();
    for(unsigned int j = 0; j < NUM_OPT_ITER; ++j)
    {
      v2d const &left = segment_data_[idx1].left_;
      v2d const &right = segment_data_[idx1].right_;
      v2d const &P0 = segment_data_[idx0].getMid();
      v2d const &P2 = segment_data_[idx2].getMid();
      v2d const &P3 = segment_data_[idx3].getMid();
      double const g = gradient(1.0, alpha, left, right, P0, P2, P3, s1, s2, s3,
                                orig_alphas[idx1]);

      if(g_min > std::abs(g))
        g_min = std::abs(g);

      //printf("g: %f\n", g);
      //printf("g_min: %f\n", g_min);
      //printf("std::abs(g_min / g): %f\n", std::abs(g_min / g));
      if(std::abs(g) > 0)// && std::abs(g_min / g) > 1e-3)
      {
        alpha -= 0.001 * (g_min / g);
      }
      else
        break;

      if(alpha < min_alpha)
      {
        alpha = min_alpha;
        break;
      }
      else if(alpha > max_alpha)
      {
        alpha = max_alpha;
        break;
      }
    }
    segment_data_[idx1].setAlpha(alpha);
  }

  printf("alpha: [");
  for(int i = 0; i < nseg_; ++i)
  {
    printf("%f, ", segment_data_[i].getAlpha());
  }
  printf("]\n");
}

// From https://stackoverflow.com/a/33454406
template <typename T>
T powInt(T x, unsigned int n)
{
  if(n == 0)
    return T{1};
  else if(n == 1)
    return x;
  else if(n == 2)
    return x * x;

  auto y = T{1};
  while(n > 1)
  {
    if(n % 2 == 1)
      y *= x;
    x *= x;
    n /= 2;
  }
  return x * y;
}

double Trajectory::gradient(double v, double alpha, v2d const &l, v2d const &r,
                            v2d const &P0, v2d const &P2, v2d const &P3,
                            double s1, double s2, double s3, double orig_alpha)
{
  v2d const P1 = l + alpha * (r - l);
  double const l_x = l.x, l_y = l.y, r_x = r.x, r_y = r.y;
  double const P0_x = P0.x, P0_y = P0.y;
  double const P1_x = P1.x, P1_y = P1.y;
  double const P2_x = P2.x, P2_y = P2.y;
  double const P3_x = P3.x, P3_y = P3.y;

  double const dacc_sq_dalpha_x =
      8 * powInt(v, 4) * (l_x - r_x) * (s1 - 2 * s3) *
      (2 * P0_x *
           (powInt(s1, 2) * s2 - powInt(s1, 2) * s3 - s1 * powInt(s2, 2) +
            s1 * powInt(s3, 2) + powInt(s2, 2) * s3 - s2 * powInt(s3, 2)) +
       P1_x * s2 * (s1 * s2 - s1 * s3 - 2 * s2 * s3 + 2 * powInt(s3, 2)) -
       P2_x * (powInt(s1, 2) * s2 - 2 * powInt(s1, 2) * s3 - s1 * s2 * s3 +
               2 * s1 * powInt(s3, 2)) -
       P3_x * (powInt(s1, 2) * s2 - s1 * powInt(s2, 2)));
  double const dacc_sq_dalpha_y =
      8 * powInt(v, 4) * (l_y - r_y) * (s1 - 2 * s3) *
      (2 * P0_y *
           (powInt(s1, 2) * s2 - powInt(s1, 2) * s3 - s1 * powInt(s2, 2) +
            s1 * powInt(s3, 2) + powInt(s2, 2) * s3 - s2 * powInt(s3, 2)) +
       P1_y * s2 * (s1 * s2 - s1 * s3 - 2 * s2 * s3 + 2 * powInt(s3, 2)) -
       P2_y * (powInt(s1, 2) * s2 - 2 * powInt(s1, 2) * s3 - s1 * s2 * s3 +
               2 * s1 * powInt(s3, 2)) -
       P3_y * (powInt(s1, 2) * s2 - s1 * powInt(s2, 2)));

  double const d_alpha = alpha - orig_alpha;
  double const K = 0;
  double const exp_term = std::exp(K * powInt(d_alpha, 2)) * 2 * K * d_alpha;
  return dacc_sq_dalpha_x + dacc_sq_dalpha_y + exp_term;
}

v2d Trajectory::getPoint(int seg_id, tdble length_along_segment)
{
  int idx0 = (seg_id - 1) % nseg_;
  if(idx0 < 0)
    idx0 += nseg_;
  int idx1 = seg_id % nseg_;
  int idx2 = (seg_id + 1) % nseg_;
  int idx3 = (seg_id + 2) % nseg_;

  // printf("indices: %d, %d, %d, %d\n", idx0, idx1, idx2, idx3);
  tdble const t0 = 0;
  tdble const t1 = t0 + segment_data_[idx0].length_;
  tdble const t2 = t1 + segment_data_[idx1].length_;
  tdble const t3 = t2 + segment_data_[idx2].length_;

  tdble t = t1 + length_along_segment;

  v2d const &P0 = segment_data_[idx0].getMid();
  v2d const &P1 = segment_data_[idx1].getMid();
  v2d const &P2 = segment_data_[idx2].getMid();
  v2d const &P3 = segment_data_[idx3].getMid();

  v2d A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1;
  v2d A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2;
  v2d A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3;
  v2d B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2;
  v2d B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3;

  v2d C = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2;
  return C;
}

#if 0
dacc_sq_dalpha_x = 8*v0**4*(l_x - r_x)*(s1 - 2*s3)*(
    2*P0_x*(s1**2*s2 - s1**2*s3 - s1*s2**2 + s1*s3**2 + s2**2*s3 - s2*s3**2) -
    P2_x*(s1**2*s2 - 2*s1**2*s3 - s1*s2*s3 + 2*s1*s3**2) -
    P3_x*(s1**2*s2 - s1*s2**2) +
    (l_x + alpha*(r_x - l_x))*s2*(s1*s2 - s1*s3 - 2*s2*s3 + 2*s3**2))
#endif

} // namespace kartik_ns
