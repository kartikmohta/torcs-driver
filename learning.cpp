#include "learning.h"
#include <fstream>
//#include <robottools.h>

namespace kartik_ns
{
void Track::init(tTrack const *track)
{
  segments_.reserve(track->nseg);

  tTrackSeg const *seg = track->seg;
  // Switch seg to seg 0 for sure.
  while(seg->id != 0)
  {
    seg = seg->prev;
  }
  // Fill in the segments
  do
  {
    segments_.push_back(TrackSegment());
    seg = seg->next;
  } while(seg->id != 0);
}

Track Track::load(char const *fname, tTrack const *track)
{
  Track t;

  std::ifstream fin(fname,std::ios::binary);
  if(!fin.is_open())
  {
    t.init(track);
    return t;
  }

  float f;
  t.segments_.clear();
  while(fin >> f)
  {
    TrackSegment tseg;
    tseg.changeAllowedSpeedFactor(f);
    t.segments_.push_back(tseg);
  }
  fin.close();

  // Make sure we have the same number of segments
  if(t.segments_.size() != track->nseg)
    t.init(track);

  return t;
}

void Track::save(char const *fname)
{
  std::ofstream fout(fname, std::ios::binary);

  for(size_t i = 0; i < segments_.size(); ++i)
  {
    fout << segments_[i].getAllowedSpeedFactor() << " ";
  }
  fout.close();
}

TrackSegment Track::getSegment(int segment_id) const
{
  return segments_.at(segment_id);
}

tdble Track::getAllowedSpeedFactor(int segment_id) const
{
  return segments_.at(segment_id).getAllowedSpeedFactor();
}

void Track::changeAllowedSpeedFactor(int segment_id, tdble factor)
{
  segments_.at(segment_id).changeAllowedSpeedFactor(factor);
}


void TrainingData::addData(tdble bankangle, tdble length, tdble arc,
                           tdble width, tdble mass, tdble speed)
{
  if(speed > 100)
    speed = 100;
  const tdble pi = M_PI;
  // Normalize the data
  TrainingData::Data new_data = {
      bankangle / (pi / 2), length / 2000, arc / pi,
      width / 20,           mass / 2000,   speed / 100};
  data_.push_back(new_data);
}

void TrainingData::save(char const *fname)
{
  const size_t num_data = data_.size();

  std::ofstream fout(fname, std::ios::binary);

  fout << num_data << " " << 5 /* inputs */ << " "
       << 1 /* outputs */ << std::endl;

  for(size_t i = 0; i < data_.size(); ++i)
  {
    fout << data_[i].bankangle << " " << data_[i].length << " " << data_[i].arc
         << " " << data_[i].width << " " << data_[i].mass << std::endl;
    fout << data_[i].speed << std::endl;
  }
  fout.close();
}
}
