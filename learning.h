#pragma once

#include <robot.h>
#include <vector>

namespace kartik_ns
{
class TrackSegment
{
 public:
  TrackSegment() : allowed_speed_factor_(1) {}
  tdble getAllowedSpeedFactor() const { return allowed_speed_factor_; }
  void changeAllowedSpeedFactor(tdble factor)
  {
    allowed_speed_factor_ *= factor;
  }

 private:
  tdble allowed_speed_factor_;
};

class Track
{
 public:
  void init(tTrack const *track);
  static Track load(char const *fname, tTrack const *track);
  void save(char const *fname);
  TrackSegment getSegment(int segment_id) const;
  tdble getAllowedSpeedFactor(int segment_id) const;
  void changeAllowedSpeedFactor(int segment_id, tdble factor);

 private:
  std::vector<TrackSegment> segments_;
};

class TrainingData
{
 public:
  void addData(tdble bankangle, tdble length, tdble arc, tdble width,
               tdble mass, tdble speed);
  void save(char const *fname);

 private:
  struct Data
  {
    tdble bankangle;
    tdble length;
    tdble arc;
    tdble width;
    tdble mass;
    tdble speed;
  };
  std::vector<Data> data_;
};
}
