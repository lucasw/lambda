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
  int n;         // actual iteration #
  int nX;        // number of nodes in X-direction
  int nY;        // number of nodes in Y-direction
  int nN;        // total number of iterations
  int nRec;      // number of receivers
  int nSrc;      // number of sources
  int nNodes;    // total number of nodes
  float cTube;   // sound speed in the tubes in meters/sec.
  float lTube;   // tube length in meters
  float rho;     // air density in kg/m^3, needed for velocity sources
  float tSample; // tsample=1/fsample, sampling period
  float fSample; // sampling frequency in Hz
  int nSamples;  // number of embedded samples for sources
  double t0;     // last time, used to check render speed
};

// Sample Data
struct simSample {
  int id;
  int sr;
  int nsamples;
  float *data;
};

struct DirData {
  // TODO(lucasw) need to init these to nullptr?

  bool *filt_;            // array indicating filters at the nodes for this direction
  // std::vector<std::vector<float> > oldx_;          // filter non-recursive memory for filters
  // std::vector<std::vector<float> > oldy_;          // filter non-recursive memory for filters
  std::unique_ptr<std::unique_ptr<float[]>[]> oldx_;          // filter non-recursive memory for filters
  std::unique_ptr<std::unique_ptr<float[]>[]> oldy_;          // filter non-recursive memory for filters
  int *filtnumcoeffs_;    // number of filter coeffs for filters
  float **filtcoeffsA_;   // recursive filter coeffs for filters
  float **filtcoeffsB_;   // non-recursive filter coeffs for filters

  void print(const size_t pos);

  // array containing the actual velocity of velo sources from four directions
  cv::Mat velo_;
};

// TODO(lucasw) make an array of SimDatas of length nNodes instead
// of many many individual arrays - or will that hurt performance?
// This struct contains specific data about the simulation environment.
struct SimData {
  SimData();

  // use cv::Mat 32F for these?  Do a speed comparison before and after
  // the conversion.
  cv::Mat envi;       // simulation environment - the walls
  cv::Mat angle;      // angle matrix as loaded from the .sim-file
  float *srcs;       // array containing the sources
  cv::Mat pressure_[3];  // array containing the actual node pressure distribution
  float *inci;       // array containing the incident node pressures
  float *recs;       // array containing the receivers
  bool *boundary;    // array indicating boundary nodes
  bool *deadnode;    // array indicating "dead" nodes
  int *recIdx;       // array containing the receiver positions
  float *record; // in case of recording into rco file, this array contains the
                 // recorded data
  std::map<std::string, DirData> dir_data_;

  float
      *mem; // array for sources to use for storing information between samples.
  simSample **samples; // contains sample data from soundfiles embedded in the
                       // simulation
};

// Indices used during simulation process.
struct Inci {
  float *past_;
  float *pres_;
  float *futu_;
  float *idxI[3];
};

struct simIndex {
  float *presPast, *presPres, *presFutu;
  std::map<std::string, Inci> inci_;
  float *idxP[3];
};

// Files used in the program.
struct simFiles {
  std::string lastFileName;
  std::ofstream aviFile;
  std::ofstream rceFile;
  std::ofstream rcoFile;
};

// Source data.
struct simSource {
  float y;     // y-position of source
  float x;     // x-position of source
  float type;  // type of source
  float amp;   // amplitude of source in Pascal or in m/s for velo sources
  float freq;  // frequency of source
  float phase; // phase angle of source
};

// Error identifiers.
typedef enum {
  NONE = 0,
  FILE_BAD,
  FILE_SIZE_BAD,
  FILE_HEADER_BAD_OR_WRONG_VERSION,
  FILE_DEF_BLOCK_BAD,
  FILE_ENV_BLOCK_BAD,
  FILE_ANG_BLOCK_BAD,
  FILE_FLT_BLOCK_BAD,
  FILE_SRC_BLOCK_BAD,
  TUBE_SPEED_BAD,
  TUBE_LENGTH_BAD,
  RHO_BAD,
  SRC_COORDS_BAD,
  SRC_FREQ_BAD,
  SRC_TYPE_BAD,
  NO_SAMPLES,
  NO_NODES,
  NO_SOURCES,
} simError;

// Angular preemphasis identifiers.
typedef enum { kHorizontal = 0, kVertical, kNone } simAngularType;

class Lambda {
public:
  Lambda();

  // TODO(lucasw) 3D add up and down?
  const std::vector<std::string> dirs_ = {"left", "right", "top", "bottom"};

  typedef enum {LEFT, RIGHT, TOP, BOTTOM} dir;

  //   Prepares variables needed for simulation.
  // RETURN VALUE
  //   simError: NONE if no error occured, error identfier otherwise.
  simError initSimulationPre();
  void initEnvironmentSetup();
  simError initSimulation();
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
  template <class T> simError set(const std::string what, const T value);

private:
  //   starts or quits receiver. The
  //   receiver stores sound pressure data at user specified receiver pixels to
  //   a file.
  void rce();

  //   QT Slot connected to the Walls/Showbounds checkbox. Updates the
  //   visualization window when checkbox is clicked.
  void showbounds();

  void setContrast();
  void setZoom();
  void setSkip();
  void setColormap();
  void setSamples();

  //   This function intializes all the important variables, arrays and
  //   matrices. Sets pointers to NULL. Called only one single time at startup.
  void initVariables();

  void initEnvironment();

  //   Processes input parameters and sets internal variables accordingly.
  // INPUT
  //   int argc     : Number of elements in input vector argv
  //   char *argv[] : Array of variable input parameters
  void handleParameters(int argc, char *argv[]);

  //   Adds a source to the array of sources (data.srcs) after performing a few
  //   checks on the data.
  // INPUT
  //   const unsigned int idx   : An integer defining the source's index in the
  //   array. const simSource *srcData : Pointer to the source to be added.
  // RETURN VALUE
  //   simError: NONE if source was added successfully, error identfier
  //   otherwise.
  virtual simError defineSource(const int idx, const simSource *srcData);

  // PURPOSE
  //   Tries to load a recorded playback file.
  // RETURN VALUE
  //   simError: NONE if file was opened successfully, error identfier
  //   otherwise.
  virtual simError loadRecord(const std::string fileName);

  int *tmp_filtid = NULL;         // temporary filter ID array
  int *tmp_filtnumcoeffs = NULL;  // temporary filter numcoeffs array
  float **tmp_filtcoeffsA = NULL; // temporary filter a-coeffs array
  float **tmp_filtcoeffsB = NULL; // temporary filter b-coeffs array
  int tmp_numfilters;             // temporary number of filters

  void processWall(const int x, const int y);

  // load a simulation file.
  // RETURN VALUE
  //   simError: NONE if file was opened successfully, error identfier
  //   otherwise.
  virtual simError loadSimulation(const std::string fileName);

  //   Resets important variables, arrays and matrices to zero, e.g. before
  //   starting new simulation runs.
  virtual void resetAll();

  //   Resets variables and arrays used directly for simulation purposes.
  virtual void resetSimulation();

  //   Processes the receiver output after each calculated sim iteration if Rce
  //   is switched on.
  virtual void processRce();

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
  virtual void adaptreflexionfactor(int &dest_numcoeffs, float *&dest_coeffsA,
                                    float *&dest_coeffsB, float r, float alpha,
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
  virtual void adaptfilter(int &dest_numcoeffs, float *&dest_coeffsA,
                           float *&dest_coeffsB, int *src_id,
                           int *src_numcoeffs, float **src_coeffsA,
                           float **src_coeffsB, int src_numfilters, int id,
                           float alpha, simAngularType direction);

  simConfig config;
  SimData data;
  simIndex index;
  simFiles files;
  int MEMSRC;

  static constexpr float rad_per_deg = M_PI / 180.f;
};

#endif
