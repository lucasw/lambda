//  COPYRIGHT NOTES
// -----------------
//  Copyright (C) 2006 Simon Ahrens, Matthias Blau, IHA Oldenburg
//  Copyright (C) 2009 Marco Ruhland, Matthias Blau, IHA Oldenburg
//  All rights reserved.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//  or visit http://www.gnu.org/licenses/gpl.html

#ifndef LAMBDA_H
#define LAMBDA_H

#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <memory>
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <sys/stat.h>
#include <time.h>

// This struct contains all the simulation config data.
struct simConfig {
  // reset is just for restarting the current sim
  void reset();
  int n = 0;         // actual iteration #
#if 0
  float cTube;   // sound speed in the tubes in meters/sec.
  float lTube;   // tube length in meters
  float rho;     // air density in kg/m^3, needed for velocity sources
  float tSample; // tsample=1/fsample, sampling period
  float fSample; // sampling frequency in Hz
  int nSamples;  // number of embedded samples for sources
  double t0;     // last time, used to check render speed
#endif
};

// group all data here that tends to be accessed in the sim update step,
// so that it is all close together in memory.
// Initially just the filter data.
struct DirectionalFilter {
  DirectionalFilter();
  void resetPressure();
  void resetEnvironment();
  // whether to use a filter at this location at all
  bool filt_ = false;

  static const size_t length_ = 4;
  int numcoeffs_ = 0;  // number of filter coeffs for filters, has to be <= length_ above
  // Constant array size for now, later try to make it more dynamic
  // TODO(lucasw) need a small array optimization if size 0..4 or some other
  // low value use a std::array, but still allow special large filters
  // that use dynamic memory
  // which should be fine if not on too many nodes.
  std::array<float, length_> oldx_;
  std::array<float, length_> oldy_;
  std::array<float, length_> coeffsA_;
  std::array<float, length_> coeffsB_;

  void print();

  // past, present, future incident pressure
  std::array<float, 3> inci_;

  // array containing the actual velocity of velo sources from four directions
  float velo_;
};

// TODO(lucasw) instead of the below being all independent arrays (and therefore widely
// spaced in memory), make them all in the same data structure per node.
struct Node {
  void resetPressure();
  void resetEnvironment();
  // TODO(lucasw) maybe this is bad if frequently sparse filters
  std::array<DirectionalFilter, 4> filter_;

  // TODO(lucasw) maybe put these in DirectionalFilter
  #if USE_WRAP
  std::array<int, 4> neighbors_ = {-1, -1, -1, -1};
  #endif

  void print();

  // TODO(lucasw) keeping these performance notes around for future reference
  // std::vector about 10% slower than float** or unique_ptr of of unique_ptr float*
  // std::vector<std::vector<float> > oldx_;          // filter non-recursive memory for filters
  // these seem almost as fast as float**
  //std::unique_ptr<std::unique_ptr<float[]>[]> oldx_;          // filter non-recursive memory for filters
};

#if 0
// Source data.
struct simSource {
  float y;     // y-position of source
  float x;     // x-position of source
  float type;  // type of source
  float amp;   // amplitude of source in Pascal or in m/s for velo sources
  float freq;  // frequency of source
  float phase; // phase angle of source
};
#endif

// Angular preemphasis identifiers.
typedef enum { kHorizontal = 0, kVertical, kNone } simAngularType;

class Lambda {
public:
  Lambda(const size_t width, const size_t height);

  // TODO(lucasw) 3D add up and down?
  // static constexpr std::array<char[7], 4> dirs_ = {"left", "right", "top", "bottom"};
  const std::vector<std::string> dirs_ = {"left", "right", "top", "bottom"};

  typedef enum {PAST, PRES, FUTU, NUM_TIMES} times;
  typedef enum {LEFT, TOP, RIGHT, BOTTOM, NUM_DIRS} dir;
  simAngularType dirToPreemphasis(const int d)
  {
    if ((d == LEFT) || (d == RIGHT))
      return kHorizontal;
    if ((d == TOP) || (d == BOTTOM))
      return kVertical;
    return kNone;
  }

  //   Prepares variables needed for simulation.
  // RETURN VALUE
  //   return true if no error occured, false identfier otherwise.
  bool init();
  //   Resets variables and arrays used directly for simulation purposes.
  virtual void resetPressure();
  virtual void resetEnvironment();

  void setVel(const int& srcxy, const float& magnitude, const float& alpha);
  //   Processes the next simulation iteration.
  void processSim();

  void print(const size_t x, const size_t y);
  void getFilterImage(cv::Mat& image, const int d, const std::string type, const int i=0);

  void setPressure(const size_t x, const size_t y, const float value);
  void addPressure(const size_t x, const size_t y, const float value);
  void getPressure(cv::Mat& image)
  {
    const int idx = ((config.n + 1) % 3); // present index
    image = pressure_[idx];
  }
  float getPressure(const size_t x, const size_t y);

  bool setWall(const size_t x, const size_t y, const float value,
      const float base_pressure=0.0);
  void getEnvironment(cv::Mat& image)
  {
    image = envi_;
  }

#if USE_WRAP
  // have the waves wrap left-right and top-bottom -
  // corresponds to a torus, not really that physical but neat
  // to experiment with.
  bool wrap_ = true;
#endif

  const size_t width_;
  const size_t height_;
  const size_t num_;

private:
  //   This function intializes all the important variables, arrays and
  //   matrices. Sets pointers to NULL. Called only one single time at startup.
  void initVariables();

  void initEnvironment();

  // Not sure if this ought to be dynamic, modifiable?
  static const size_t num_tmp_filters_ = 1;
  std::array<DirectionalFilter, num_tmp_filters_> tmp_filters_;
  std::array<size_t, num_tmp_filters_> tmp_filtid_;         // temporary filter ID array

  void addFilter(const int x, const int y, const int d,
    float envi, float angle);
  void addFilter(const int pos, const int d,
    float envi, float angle);
  void addNeighborFilter(const int x, const int y,
    const float envi, const float angle);

  //   Creates a new digital filter for a given real-valued reflexion factor and
  //   preemphases the filter coefficients due to a given sonic incidence angle
  //   and horizontal/vertical alignment of the filter.
  // INPUT
  //   float r    	: desired real-valued reflexion factor
  //   float alpha	: angle of incidence for the desired filter, needed for
  //   preemphasis simAngularType direction : sets whether the filter is used in
  //   horizontal or vertical tubes, needed for preemphasis
  //
  // OUTPUT
  //   int& dest_numcoeffs    : number of filter coefficients for the calculated
  //   filter float*& dest_coeffsA   : array containig the new computed
  //   a-Filter-coefficients float*& dest_coeffsB   : array containig the new
  //   computed b-Filter-coefficients
  virtual void adaptreflexionfactor(int &dest_numcoeffs,
                                    std::array<float, 4>& dest_coeffsA,
                                    std::array<float, 4>& dest_coeffsB,
                                    float r, float alpha,
                                    simAngularType direction);

  // PURPOSE
  //   Creates a new digital filter from a given digital filter ID and
  //   preemphases the filter coefficients due to a given sonic incidence angle
  //	 and horizontal/vertical alignment of the filter.
  //
  // INPUT
  //	 int* src_id    		: pointer on source filter ID array
  //   int* src_numcoeffs 	: pointer on source filter numcoeffs array
  //   float** src_coeffsA	: pointer on 2-Dim filter a-coeff Matrix
  //   float** src_coeffsB	: pointer on 2-Dim filter b-coeff Matrix
  //   int src_numfilters     : number of filters in the srcfilter-Arrays
  //   int id    				: desired filter ID
  //   float alpha			: angle of incidence for the desired filter, needed for
  //   preemphasis simAngularType direction : sets whether the filter is used in
  //   horizontal or vertical tubes, needed for preemphasis
  //
  // OUTPUT
  //   int& dest_numcoeffs    : number of filter coefficients for the calculated
  //   filter float*& dest_coeffsA   : array containig the new computed
  //   a-Filter-coefficients float*& dest_coeffsB   : array containig the new
  //   computed b-Filter-coefficients
  // TODO(lucasw) make the 4 a template parameter
  virtual void adaptfilter(int &dest_numcoeffs,
                           std::array<float, 4>& dest_coeffsA,
                           std::array<float, 4>& dest_coeffsB,
                           const std::array<size_t, num_tmp_filters_>& src_id,
                           const std::array<DirectionalFilter, num_tmp_filters_>& src_filters_,
                           const int id,
                           const float alpha, simAngularType direction);

  simConfig config;

  static constexpr float rad_per_deg = M_PI / 180.f;

  void processWalls();
  bool processWall(const size_t x, const size_t y);
  // the location of these in memory is not that important because
  // they aren't accessed during the sim update.
  // use cv::Mat 32F for these?  Do a speed comparison before and after
  // the conversion.
  cv::Mat envi_;       // simulation environment - the walls
  cv::Mat angle_;      // angle matrix as loaded from the .sim-file

  // these are important to keep together.
  // if there a lot of dead nodes, then better to kepep them in own array,
  // otherwise move into node above
  std::unique_ptr<bool[]> deadnode;    // array indicating "dead" nodes
  // same here- if many boundaries then move into Node
  // seems much faster outside of node (but maybe didn't test correctly).
  std::unique_ptr<bool[]> boundary;    // array indicating boundary nodes
  std::array<cv::Mat, 3> pressure_;  // array containing the actual node pressure distribution
  // TODO(lucasw) test smart pointer to array vs vector
  std::unique_ptr<Node[]> nodes_;
};

#endif
