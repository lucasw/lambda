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
  // clear is for starting from scratch, also resets
  void clear();
  // reset is just for restarting the current sim
  void reset();
  int n;         // actual iteration #
  int nX;        // number of nodes in X-direction
  int nY;        // number of nodes in Y-direction
  int nNodes;    // total number of nodes
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
  void clear();
  void reset();
  // whether to use a filter at this location at all
  bool filt_ = false;

  // TODO(lucasw)
  // maybe best to come up with a fixed max filter size and always allocate that
  // but for now keep to with the dynamic memory

  int numcoeffs_ = 0;    // number of filter coeffs for filters

  // Constant array size for now, later try to make it more dynamic
  std::array<float, 4> oldx_;
  std::array<float, 4> oldy_;
  std::array<float, 4> coeffsA_;
  std::array<float, 4> coeffsB_;

#if 0
  std::unique_ptr<float[]> oldx_;          // filter non-recursive memory for filters
  std::unique_ptr<float[]> oldy_;          // filter non-recursive memory for filters
  float *coeffsA_ = nullptr;   // recursive filter coeffs for filters
  float *coeffsB_ = nullptr;   // non-recursive filter coeffs for filters
#endif

  void print();

  // array containing the actual velocity of velo sources from four directions
  float velo_;
};

// TODO(lucasw) instead of the below being all independent arrays (and therefore widely
// spaced in memory), make them all in the same data structure per node.
struct Node {
  void clear();
  void reset();
  // TODO(lucasw) maybe this is bad if frequently sparse filters
  std::array<DirectionalFilter, 4> filter_;

  // TODO(lucasw) keeping these performance notes around for future reference
  // about 10% slower than float**
  // std::vector<std::vector<float> > oldx_;          // filter non-recursive memory for filters
  // these seem almost as fast as float**
  //std::unique_ptr<std::unique_ptr<float[]>[]> oldx_;          // filter non-recursive memory for filters
};

// TODO(lucasw) make an array of SimDatas of length n Nodes instead
// of many many individual arrays - or will that hurt performance?
// This struct contains specific data about the simulation environment.
struct SimData {
  SimData();
  void clear(const size_t num_nodes);
  void reset(const size_t num_nodes);
  // the location of these in memory is not that important because
  // they aren't accessed during the sim update.
  // use cv::Mat 32F for these?  Do a speed comparison before and after
  // the conversion.
  cv::Mat envi;       // simulation environment - the walls
  cv::Mat angle;      // angle matrix as loaded from the .sim-file

  // these are important to keep together.
  // if there a lot of dead nodes, then better to kepep them in own array,
  // otherwise move into node above
  bool *deadnode;    // array indicating "dead" nodes
  // same here- if many boundaries then move into Node
  bool *boundary;    // array indicating boundary nodes
  // definitely move into Node
  float *inci;       // array containing the incident node pressures
  cv::Mat pressure_[3];  // array containing the actual node pressure distribution
  // TODO(lucasw) test smart pointer to array vs vector
  std::unique_ptr<Node[]> nodes_;
};

// Indices used during simulation process.
struct Inci {
  float *past_;
  float *pres_;
  float *futu_;
  float *idxI[3];
};

struct simIndex {
  void clear();
  void reset();
  float *presPast, *presPres, *presFutu;
  // TODO(lwalter) replace with array with integers for LEFT, RIGHT etc.
  const std::vector<std::string> dirs_ = {"left", "right", "top", "bottom"};
  std::map<std::string, Inci> inci_;
  float *idxP[3];
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
  Lambda();

  // TODO(lucasw) 3D add up and down?
  // static constexpr std::array<char[7], 4> dirs_ = {"left", "right", "top", "bottom"};
  const std::vector<std::string> dirs_ = {"left", "right", "top", "bottom"};

  typedef enum {LEFT, TOP, RIGHT, BOTTOM, NUM_DIRS} dir;

  //   Prepares variables needed for simulation.
  // RETURN VALUE
  //   return true if no error occured, false identfier otherwise.
  bool initSimulationPre();
  void initEnvironmentSetup();
  bool initSimulation();
  void setVel(const int& srcxy, const float& magnitude, const float& alpha);
  //   Processes the next simulation iteration.
  void processSim();

  void setPressure(const size_t x, const size_t y, const float value);
  void addPressure(const size_t x, const size_t y, const float value);
  void getPressure(cv::Mat& image)
  {
    const int idx = ((config.n + 1) % 3); // present index
    image = data.pressure_[idx];
  }
  float getPressure(const size_t x, const size_t y);

  void setWall(const size_t x, const size_t y, const float value);

  //   Function template. This function should be used whenever one of the key
  //   variables of the lambda class is changed. It performs the necessary
  //   checks before changing one of those variables and makes necessary
  //   additional changes after changing. For example, the number of elements in
  //   X-direction should not just be changed. Instead, you should use
  //   set("nX",41);. This will check wether the second argument is a valid
  //   value for nX
  //   (>0) and will update nNodes (which should always be nX*nY at any time)
  //   and dispSizeX automatically.
  // TODO(lucasw) size_t
  bool setNX(const int value);
  bool setNY(const int value);
  bool setNNodes(const int value);

private:
  //   This function intializes all the important variables, arrays and
  //   matrices. Sets pointers to NULL. Called only one single time at startup.
  void initVariables();

  void initEnvironment();

  void setupOldXY(const size_t d, const int pos);

  int *tmp_filtid = NULL;         // temporary filter ID array
  int *tmp_filtnumcoeffs = NULL;  // temporary filter numcoeffs array
  float **tmp_filtcoeffsA = NULL; // temporary filter a-coeffs array
  float **tmp_filtcoeffsB = NULL; // temporary filter b-coeffs array
  int tmp_numfilters;             // temporary number of filters

  void processWall(const int x, const int y);

  //   Resets important variables, arrays and matrices to zero, e.g. before
  //   starting new simulation runs.
  virtual void clear();

  //   Resets variables and arrays used directly for simulation purposes.
  virtual void reset();

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
                           int *src_id,
                           int *src_numcoeffs, float **src_coeffsA,
                           float **src_coeffsB, int src_numfilters, int id,
                           float alpha, simAngularType direction);

  simConfig config;
  SimData data;
  simIndex index;

  static constexpr float rad_per_deg = M_PI / 180.f;
};

#endif
