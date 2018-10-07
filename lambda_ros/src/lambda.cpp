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

#include <lambda_ros/lambda.h>

simSample **new_simSample_array(int n) {
  simSample **out = new simSample *[n];
  simSample *sample;
  for (int i = 0; i < n; i++) {
    sample = new simSample;
    sample->data = NULL;
    sample->sr = 0;
    sample->id = 0;
    sample->nsamples = 0;
    out[i] = sample;
  }
  return out;
}

SimData::SimData()
    : // Initialize receiver/recorder pointers
      // TODO(lucasw) simData constructor can handle this
      recs(NULL), record(NULL), recIdx(NULL),
      // Initialize simulation environment data pointers
      srcs(NULL), boundary(NULL), deadnode(NULL),
      inci(NULL),
      // Initialize velocity source pointers
      mem(NULL), samples(NULL) {}

///////////////////////////////////////////////////////////////////////////////
//   Constructor for the program's main class, initializes program and builds up
Lambda::Lambda()
    : MEMSRC(20) {
  initVariables();
  // TODO(lucasw) let something else determine whether or not to randomize like
  // this
  srand(time(NULL));
}

//   This function intializes all the important variables, arrays and matrices.
//   Sets pointers to NULL. Called only one single time at startup.
void Lambda::initVariables() {
  // Initialize graphics properties & colours
  resetAll();
}

//   Resets important variables, arrays and matrices to zero, e.g. before
//   starting new simulation runs.
void Lambda::resetAll() {
  resetSimulation();

  // reset important simfield variables
  config.n = 0;
  config.nX = 0;
  config.nY = 0;
  config.nN = 0;
  config.nRec = 0;
  config.nSrc = 0;
  config.nNodes = 0;
  config.cTube = 0;
  config.lTube = 0;
  config.rho = 0;
  // the time of simulation step- doesn't change simulation update, just
  // measurement of the passing of time with each update.
  // int ms = config.n * config.tSample * 1E3;
  config.tSample = 0;
  config.fSample = 0;
  config.t0 = 0;

  // delete simulation environment data pointers
  for (size_t i = 0; i < 3; ++i)
    data.pressure_[i] = cv::Mat();
  data.envi = cv::Mat();
  data.angle = cv::Mat();
  if (data.srcs != NULL) {
    delete[] data.srcs;
    data.srcs = NULL;
  }
  if (data.mem != NULL) {
    delete[] data.mem;
    data.mem = NULL;
  }
  if (data.dir_data_["left"].filt_ != NULL) {
    delete[] data.dir_data_["left"].filt_;
    data.dir_data_["left"].filt_ = NULL;
  }
  if (data.dir_data_["top"].filt_ != NULL) {
    delete[] data.dir_data_["top"].filt_;
    data.dir_data_["top"].filt_ = NULL;
  }
  if (data.dir_data_["right"].filt_ != NULL) {
    delete[] data.dir_data_["right"].filt_;
    data.dir_data_["right"].filt_ = NULL;
  }
  if (data.dir_data_["bottom"].filt_ != NULL) {
    delete[] data.dir_data_["bottom"].filt_;
    data.dir_data_["bottom"].filt_ = NULL;
  }
  if (data.deadnode != NULL) {
    delete[] data.deadnode;
    data.deadnode = NULL;
  }
  if (data.boundary != NULL) {
    delete[] data.boundary;
    data.boundary = NULL;
  }
  if (data.samples != NULL) {
    for (int n = 0; n < config.nSamples; n++) {
      if (data.samples[n]->data != NULL) {
        delete[] data.samples[n]->data;
      }
      delete[] data.samples[n];
    }
    delete[] data.samples;
    data.samples = NULL;
  }
  config.nSamples = 0;

  // delete receiver pointers
  if (data.recIdx != NULL) {
    delete[] data.recIdx;
    data.recIdx = NULL;
  }

  for (const std::string& dir : dirs_) {
    // TODO(lucasw) make a function to eliminate redundant code-
    // make it a simData method.
    // delete A filter coefficient arrays
    if (data.dir_data_[dir].filtcoeffsA_ != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.dir_data_[dir].filtcoeffsA_[n] != NULL)
          delete[] data.dir_data_[dir].filtcoeffsA_[n];
        data.dir_data_[dir].filtcoeffsA_[n] = NULL;
      }
      delete[] data.dir_data_[dir].filtcoeffsA_;
      data.dir_data_[dir].filtcoeffsA_ = NULL;
    }
    // delete B filter coefficient arrays
    if (data.dir_data_[dir].filtcoeffsB_ != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.dir_data_[dir].filtcoeffsB_[n] != NULL)
          delete[] data.dir_data_[dir].filtcoeffsB_[n];
        data.dir_data_[dir].filtcoeffsB_[n] = NULL;
      }
      delete[] data.dir_data_[dir].filtcoeffsB_;
      data.dir_data_[dir].filtcoeffsB_ = NULL;
    }

    // delete filter number-of-coefficients arrays
    if (data.dir_data_[dir].filtnumcoeffs_ != NULL) {
      delete[] data.dir_data_[dir].filtnumcoeffs_;
      data.dir_data_[dir].filtnumcoeffs_ = NULL;
    }
  }  // loop through directions
}

//   Resets variables and arrays used directly for simulation purposes.
void Lambda::resetSimulation() {
  // Delete simulation environment data memory
  if (data.inci != NULL) {
    delete[] data.inci;
    data.inci = NULL;
  }
  // Reset pressure index pointers
  index.presPast = NULL;
  index.presPres = NULL;
  index.presFutu = NULL;

  for (int x = 0; x < 3; x++) {
    index.idxP[x] = 0;
  }

  for (const std::string& dir : dirs_) {
    // Reset incident pressure index pointers (past)
    index.inci_[dir].past_ = NULL;
    // Reset incident pressure index pointers (present)
    index.inci_[dir].pres_ = NULL;
    // Reset incident pressure index pointers (future)
    index.inci_[dir].pres_ = NULL;
    // Reset indices used during calculation
    for (int x = 0; x < 3; x++) {
      index.inci_[dir].idxI[x] = 0;
    }

    // Delete bottom filter non-recursive memory
    if (data.dir_data_[dir].oldx_ != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.dir_data_[dir].oldx_[n] != NULL)
          delete[] data.dir_data_[dir].oldx_[n];
        data.dir_data_[dir].oldx_[n] = NULL;
      }
      delete[] data.dir_data_[dir].oldx_;
      data.dir_data_[dir].oldx_ = NULL;
    }
    // Delete bottom filter recursive memory
    if (data.dir_data_[dir].oldy_ != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.dir_data_[dir].oldy_[n] != NULL)
          delete[] data.dir_data_[dir].oldy_[n];
        data.dir_data_[dir].oldy_[n] = NULL;
      }
      delete[] data.dir_data_[dir].oldy_;
      data.dir_data_[dir].oldy_ = NULL;
    }
  }
  // Delete velocity source memory
  // TODO(lucasw) for key in velo keys, velo[key] = cv::Mat()
}

//   Processes input parameters and sets internal variables accordingly.
void Lambda::handleParameters(int argc, char *argv[]) {
  std::string argument;
  int arg;
  // Order of parameters is random, but rce, rco and vis must be
  // activated after loading a file, if requested. If a file is requested,
  // the last step must be starting the simulation.
  // To achieve this order of processing, the parameters are checked and read in
  // a first step, notifying necessary further steps through theses bool
  // variables. Theses steps will then be processed in the right order later.

  // Parse array of input arguments
  for (arg = 0; arg < argc; arg++) {
    argument = argv[arg];

    if ((argument == "-file") || (argument == "-File") ||
        (argument == "-FILE") || (argument == "/file") ||
        (argument == "/File") || (argument == "/FILE")) {
      // if this argument is "-file" and there is another argument following
      // this one, try to load the file named like that following parameter.
      if (arg < argc - 1) {
        std::string fileName = argv[arg + 1];
        if (loadSimulation(fileName) != NONE) {
        }
      }
    } else if ((argument == "-help") || (argument == "--help") ||
               (argument == "/help")) {
      std::cout
          << "lambda [options]\n"
             "\n"
             "-file simfile   : open the .sim file\n"
             "-vis            : activate visualization\n"
             "-rce            : activate recording at receivers, if defined\n"
             "-walls          : show walls, if defined\n"
             "-exit           : exit the program after finishing (good for "
             "batch processes)\n"
             "\n"

          ;
      exit(0);
    }
  }
}

#if 0
void Lambda::stop() {
  resetSimulation();
  // Reset display, if vis was on and simulation was reset manually
  // (don't change picture if simulation ended automatically)
  config.n = 0;
}
#endif

//   Processes the receiver output after each calculated sim iteration if Rce is
//   switched on.
void Lambda::processRce() {
  double *dummy = new double; // double pointer needed for float2double cast
  for (int rec = 0; rec < config.nRec; rec++) {
    // For each receiver in the simulation environment, cast
    // its sound pressure value to double and append it to the rce file.
    *dummy = (double)*(index.presPres + *(data.recIdx + rec));
    files.rceFile.write((char *)dummy, sizeof(double));
  }
  delete dummy;
}

// TODO(lucasw) split this up into inidividual setters
// PURPOSE
//   Function template. This function should be used whenever one of the key
//   variables of the lambda class is changed. It performs the necessary checks
//   before changing one of those variables and makes necessary additional
//   changes after changing. For example, the number of elements in X-direction
//   should not just be changed. Instead, you should use set("nX",41);. This
//   will check wether the second argument is a valid value for nX
//   (>0) and will update nNodes (which should always be nX*nY at any time) and
//   dispSizeX automatically.
template <class T> simError Lambda::set(const std::string what, const T value) {
  if (what == "nX") {
    // If nX is to be set, check if new X-size is greater than 0.
    if ((int)value < 1)
      return NO_NODES;
    config.nX = (int)value;
    // Adjust nNodes and dispSizeX to fit the new setting
    set("nNodes", config.nX * config.nY);
    return NONE;
  }
  if (what == "nY") {
    // If nY is to be set, check if new Y-size is greater than 0.
    if ((int)value < 1)
      return NO_NODES;
    config.nY = (int)value;
    // Adjust nNodes and dispSizeY to fit the new setting
    set("nNodes", config.nX * config.nY);
    return NONE;
  }
  if (what == "nN") {
    // If nN is to be set, check if new number of nodes is greater than 0.
    if ((int)value < 0)
      return NO_SAMPLES;
    config.nN = (int)value;
    return NONE;
  }
  if (what == "cTube") {
    // If cTube is to be set, check if new tube speed is greater than 0.
    if (value <= 0)
      return TUBE_SPEED_BAD;
    config.cTube = value;
    set("fSample", config.cTube / config.lTube);
    return NONE;
  }
  if (what == "lTube") {
    // If lTube is to be set, check if new tube length is greater than 0.
    if (value <= 0)
      return TUBE_LENGTH_BAD;
    config.lTube = value;
    set("fSample", config.cTube / config.lTube);
    return NONE;
  }
  if (what == "rho") {
    // If rho is to be set, check if new rho is greater than 0.
    if (value <= 0)
      return RHO_BAD;
    config.rho = value;
    return NONE;
  }
  if (what == "nRec") {
    config.nRec = (int)value;
    return NONE;
  }
  if (what == "nSrc") {
    config.nSrc = (int)value;
    return NONE;
  }
  if (what == "nNodes") {
    config.nNodes = (int)value;
    return NONE;
  }
  if (what == "nSamples") {
    config.nSamples = (int)value;
    return NONE;
  }
  if (what == "tSample") {
    config.tSample = value;
    return NONE;
  }
  if (what == "fSample") {
    config.fSample = value;
    // Adjust sample duration
    set("tSample", 1 / config.fSample);
    return NONE;
  }
  return NONE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Lambda::defineSource()
//
// PURPOSE
//   Adds a source to the array of sources (data.srcs) after performing a few
//   checks on the data.
//
// INPUT
//   const unsigned int idx   : An integer defining the source's index in the
//   array. const simSource *srcData : Pointer to the source to be added.
//
// OUTPUT
//   None
//
// RETURN VALUE
//   simError: NONE if source was added successfully, error identfier otherwise.
//
//	AUTHOR		CHANGES
// DATE	VERSION 	S. Ahrens 	First build
// 05/06	1.0 	M. Ruhland 	added velocity sources handling
// 05/09	2.0
//
simError Lambda::defineSource(const int idx, const simSource *srcData) {
  // Check if coordinates are in range of environment size
  if ((srcData->y < 0) || (srcData->y >= config.nY) || (srcData->x < 0) ||
      (srcData->x >= config.nX))
    return SRC_COORDS_BAD;
  // Check if source has legal source function type
  if ((srcData->type < 1) || (srcData->type > 100))
    return SRC_TYPE_BAD;
  // Check for positive frequency
  // if (srcData->freq<=0) return SRC_FREQ_BAD;
  // Add data to source array
  data.srcs[idx * 6 + 0] = srcData->y * config.nX;
  data.srcs[idx * 6 + 1] = srcData->x;
  data.srcs[idx * 6 + 2] = srcData->type;
  data.srcs[idx * 6 + 3] = srcData->amp;
  data.srcs[idx * 6 + 4] = srcData->freq;
  data.srcs[idx * 6 + 5] = srcData->phase;
  int srcxy;
  srcxy = (int)data.srcs[idx * 6 + 0] + (int)data.srcs[idx * 6 + 1];
  float alpha;
  // get the desired incidence angle out of angle matrix
  alpha = data.angle.ptr<float>(0)[srcxy];
  // angle out of bounds?
  if ((alpha < 0.f) || (alpha >= 360.f))
    alpha = 0.f;
  if ((srcData->type >= 6) && (srcData->type <= 10)) {
    if ((alpha >= 0.f) && (alpha < 90.f)) {
      // set up a left- and top- filter if angle of incidence is
      // between 0 and 90 degrees
      data.boundary[srcxy] = true;
      data.dir_data_["left"].filt_[srcxy] = true;
      data.dir_data_["top"].filt_[srcxy] = true;
      adaptreflexionfactor(
          data.dir_data_["left"].filtnumcoeffs_[srcxy],
          data.dir_data_["left"].filtcoeffsA_[srcxy],
          data.dir_data_["left"].filtcoeffsB_[srcxy], 1.f, 180.f, kHorizontal);
      adaptreflexionfactor(data.dir_data_["top"].filtnumcoeffs_[srcxy],
                           data.dir_data_["top"].filtcoeffsA_[srcxy],
                           data.dir_data_["top"].filtcoeffsB_[srcxy], 1.f, 270.f, kVertical);
    } else if ((alpha >= 90.f) && (alpha < 180.f)) {
      // set up a top- and right- filter if angle of incidence is
      // between 90 and 180 degrees
      data.boundary[srcxy] = true;
      data.dir_data_["top"].filt_[srcxy] = true;
      data.dir_data_["right"].filt_[srcxy] = true;
      adaptreflexionfactor(data.dir_data_["top"].filtnumcoeffs_[srcxy],
                           data.dir_data_["top"].filtcoeffsA_[srcxy],
                           data.dir_data_["top"].filtcoeffsB_[srcxy], 1.f, 270.f, kVertical);
      adaptreflexionfactor(
          data.dir_data_["right"].filtnumcoeffs_[srcxy],
          data.dir_data_["right"].filtcoeffsA_[srcxy],
          data.dir_data_["right"].filtcoeffsB_[srcxy], 1.f, 0.f, kHorizontal);
    } else if ((alpha >= 180.f) && (alpha < 270.f)) {
      // set up a right- and bottom- filter if angle of incidence is
      // between 180 and 270 degrees
      data.boundary[srcxy] = true;
      data.dir_data_["right"].filt_[srcxy] = true;
      data.dir_data_["bottom"].filt_[srcxy] = true;
      adaptreflexionfactor(
          data.dir_data_["right"].filtnumcoeffs_[srcxy],
          data.dir_data_["right"].filtcoeffsA_[srcxy],
          data.dir_data_["right"].filtcoeffsB_[srcxy], 1.f, 0.f, kHorizontal);
      adaptreflexionfactor(
          data.dir_data_["bottom"].filtnumcoeffs_[srcxy],
          data.dir_data_["bottom"].filtcoeffsA_[srcxy],
          data.dir_data_["bottom"].filtcoeffsB_[srcxy], 1.f, 90.f, kVertical);
    } else if ((alpha >= 270.f) && (alpha < 360.f)) {
      // set up a bottom- and left- filter if angle of incidence is
      // between 270 and 360 degrees
      data.boundary[srcxy] = true;
      data.dir_data_["bottom"].filt_[srcxy] = true;
      data.dir_data_["left"].filt_[srcxy] = true;
      adaptreflexionfactor(
          data.dir_data_["bottom"].filtnumcoeffs_[srcxy],
          data.dir_data_["bottom"].filtcoeffsA_[srcxy],
          data.dir_data_["bottom"].filtcoeffsB_[srcxy], 1.f, 90.f, kVertical);
      adaptreflexionfactor(
          data.dir_data_["left"].filtnumcoeffs_[srcxy],
          data.dir_data_["left"].filtcoeffsA_[srcxy],
          data.dir_data_["left"].filtcoeffsB_[srcxy], 1.f, 180.f, kHorizontal);
    }
  }
  return NONE;
}

//  ----- PREPROCESSING OF THE ENVIRONMENT -----
void Lambda::initEnvironment(int *&tmp_filtid, int *&tmp_filtnumcoeffs,
                             float **&tmp_filtcoeffsA, float **&tmp_filtcoeffsB,
                             int &tmp_numfilters) {
  data.boundary = new bool[config.nNodes]; // mem for boundary indicator
  for (int pos = 0; pos < config.nNodes; pos++) {
    data.boundary[pos] = false;
  }
  // data.deadnode=new bool[config.nNodes];              // mem for deadnode
  // indicator
  for (const std::string& dir : dirs_)
  {
    data.dir_data_[dir].filt_ = new bool[config.nNodes];   // mem for filter left indicator
    // mem for number of filter coeffs
    data.dir_data_[dir].filtnumcoeffs_ = new int[config.nNodes];
    // mem for filter a-coeffs
    data.dir_data_[dir].filtcoeffsA_ = new float *[config.nNodes];
    // mem for filter b-coeffs
    data.dir_data_[dir].filtcoeffsB_ = new float *[config.nNodes];
    // Initialize all these memories
    // data.deadnode[pos]=false;
    for (int pos = 0; pos < config.nNodes; pos++) {
      data.dir_data_[dir].filt_[pos] = false;
      data.dir_data_[dir].filtnumcoeffs_[pos] = 0;
      data.dir_data_[dir].filtcoeffsA_[pos] = NULL;
      data.dir_data_[dir].filtcoeffsB_[pos] = NULL;
    }
  }

  int actrec = 0;

  // work through all nodes in the environment
  for (int y = 0; y < config.nY; y++) {
    for (int x = 0; x < config.nX; x++) {
      int pos = y * config.nX + x;
      if ((y == 0) || (x == 0) || (y == config.nY - 1) ||
          (x == config.nX - 1)) // is this a simfield border node?
      {
        data.boundary[pos] = true;
        if (x == 0) // left simfield border
        {
          data.dir_data_["left"].filt_[pos] = true;
          // apply a zero-reflection-filter to left border node
          adaptreflexionfactor(
              data.dir_data_["left"].filtnumcoeffs_[pos],
              data.dir_data_["left"].filtcoeffsA_[pos],
              data.dir_data_["left"].filtcoeffsB_[pos], 0.f, 180.f, kHorizontal);
        }
        if (y == 0) // top simfield border
        {
          data.dir_data_["top"].filt_[pos] = true;
          // apply a zero-reflection-filter to top border node
          adaptreflexionfactor(
              data.dir_data_["top"].filtnumcoeffs_[pos], data.dir_data_["top"].filtcoeffsA_[pos],
              data.dir_data_["top"].filtcoeffsB_[pos], 0.f, 270.f, kVertical);
        }
        if (x == config.nX - 1) // right simfield border
        {
          data.dir_data_["right"].filt_[pos] = true;
          // apply a zero-reflection-filter to right border node
          adaptreflexionfactor(
              data.dir_data_["right"].filtnumcoeffs_[pos],
              data.dir_data_["right"].filtcoeffsA_[pos],
              data.dir_data_["right"].filtcoeffsB_[pos], 0.f, 0.f, kHorizontal);
        }
        if (y == config.nY - 1) // bottom simfield border
        {
          data.dir_data_["bottom"].filt_[pos] = true;
          // apply a zero-reflection-filter to bottom border node
          adaptreflexionfactor(
              data.dir_data_["bottom"].filtnumcoeffs_[pos],
              data.dir_data_["bottom"].filtcoeffsA_[pos],
              data.dir_data_["bottom"].filtcoeffsB_[pos], 0.f, 90.f, kVertical);
        }
      }
      // is the actual node a real-valued-reflecting node?
      const float envi = data.envi.ptr<float>(0)[pos];
      const float angle = data.angle.ptr<float>(0)[pos];
      if ((envi >= -1.0) && (envi != 0.0) && (envi <= 1.0)) {
        data.boundary[pos] = true;
        data.dir_data_["left"].filt_[pos] = true;
        data.dir_data_["top"].filt_[pos] = true;
        data.dir_data_["right"].filt_[pos] = true;
        data.dir_data_["bottom"].filt_[pos] = true;
        // apply a left filter with correspondig reflection factor to it
        adaptreflexionfactor(data.dir_data_["left"].filtnumcoeffs_[pos],
                             data.dir_data_["left"].filtcoeffsA_[pos],
                             data.dir_data_["left"].filtcoeffsB_[pos], envi,
                             angle, kHorizontal);
        // apply a top filter with correspondig reflection factor to it
        adaptreflexionfactor(data.dir_data_["top"].filtnumcoeffs_[pos],
                             data.dir_data_["top"].filtcoeffsA_[pos],
                             data.dir_data_["top"].filtcoeffsB_[pos], envi,
                             angle, kVertical);
        // apply a right filter with correspondig reflection factor to it
        adaptreflexionfactor(data.dir_data_["right"].filtnumcoeffs_[pos],
                             data.dir_data_["right"].filtcoeffsA_[pos],
                             data.dir_data_["right"].filtcoeffsB_[pos], envi,
                             angle, kHorizontal);
        // apply a bottom filter with correspondig reflection factor to it
        adaptreflexionfactor(data.dir_data_["bottom"].filtnumcoeffs_[pos],
                             data.dir_data_["bottom"].filtcoeffsA_[pos],
                             data.dir_data_["bottom"].filtcoeffsB_[pos], envi,
                             angle, kVertical);
        if (x <
            config.nX - 1) // apply a left filter to its right neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + 1] = true;
          data.dir_data_["left"].filt_[pos + 1] = true;
          adaptreflexionfactor(data.dir_data_["left"].filtnumcoeffs_[pos + 1],
                               data.dir_data_["left"].filtcoeffsA_[pos + 1],
                               data.dir_data_["left"].filtcoeffsB_[pos + 1], envi,
                               angle, kHorizontal);
        }
        if (y <
            config.nY - 1) // apply a top filter to its bottom neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + config.nX] = true;
          data.dir_data_["top"].filt_[pos + config.nX] = true;
          adaptreflexionfactor(data.dir_data_["top"].filtnumcoeffs_[pos + config.nX],
                               data.dir_data_["top"].filtcoeffsA_[pos + config.nX],
                               data.dir_data_["top"].filtcoeffsB_[pos + config.nX],
                               envi, angle, kVertical);
        }
        if (x > 0) // apply a right filter to its left neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - 1] = true;
          data.dir_data_["right"].filt_[pos - 1] = true;
          adaptreflexionfactor(data.dir_data_["right"].filtnumcoeffs_[pos - 1],
                               data.dir_data_["right"].filtcoeffsA_[pos - 1],
                               data.dir_data_["right"].filtcoeffsB_[pos - 1], envi,
                               angle, kHorizontal);
        }
        if (y > 0) // apply a bottom filter to its top neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - config.nX] = true;
          data.dir_data_["bottom"].filt_[pos - config.nX] = true;
          adaptreflexionfactor(data.dir_data_["bottom"].filtnumcoeffs_[pos - config.nX],
                               data.dir_data_["bottom"].filtcoeffsA_[pos - config.nX],
                               data.dir_data_["bottom"].filtcoeffsB_[pos - config.nX],
                               envi, angle, kVertical);
        }
      // is the actual node a filter-node?
      } else if ((envi > 1.0) && (envi <= 1000.0)) {
        data.boundary[pos] = true;
        data.dir_data_["left"].filt_[pos] = true;
        data.dir_data_["top"].filt_[pos] = true;
        data.dir_data_["right"].filt_[pos] = true;
        data.dir_data_["bottom"].filt_[pos] = true;
        // apply the left filter with the correspondig ID to it
        adaptfilter(data.dir_data_["left"].filtnumcoeffs_[pos],
                    data.dir_data_["left"].filtcoeffsA_[pos],
                    data.dir_data_["left"].filtcoeffsB_[pos], tmp_filtid, tmp_filtnumcoeffs,
                    tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                    (int)envi, angle, kHorizontal);
        // apply the top filter with the correspondig ID to it
        adaptfilter(data.dir_data_["top"].filtnumcoeffs_[pos],
                    data.dir_data_["top"].filtcoeffsA_[pos],
                    data.dir_data_["top"].filtcoeffsB_[pos], tmp_filtid, tmp_filtnumcoeffs,
                    tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                    (int)envi, angle, kVertical);
        // apply the right filter with the correspondig ID to it
        adaptfilter(data.dir_data_["right"].filtnumcoeffs_[pos],
                    data.dir_data_["right"].filtcoeffsA_[pos],
                    data.dir_data_["right"].filtcoeffsB_[pos], tmp_filtid, tmp_filtnumcoeffs,
                    tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                    (int)envi, angle, kHorizontal);
        // apply the bottom filter with the correspondig ID to it
        adaptfilter(data.dir_data_["bottom"].filtnumcoeffs_[pos],
                    data.dir_data_["bottom"].filtcoeffsA_[pos],
                    data.dir_data_["bottom"].filtcoeffsB_[pos],
                    tmp_filtid, tmp_filtnumcoeffs, tmp_filtcoeffsA,
                    tmp_filtcoeffsB, tmp_numfilters, (int)envi,
                    angle, kVertical);
        if (x <
            config.nX - 1) // apply a left filter to its right neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + 1] = true;
          data.dir_data_["left"].filt_[pos + 1] = true;
          adaptfilter(
              data.dir_data_["left"].filtnumcoeffs_[pos + 1],
              data.dir_data_["left"].filtcoeffsA_[pos + 1],
              data.dir_data_["left"].filtcoeffsB_[pos + 1], tmp_filtid, tmp_filtnumcoeffs,
              tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
              (int)envi, angle, kHorizontal);
        }
        if (y <
            config.nY - 1) // apply a top filter to its bottom neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + config.nX] = true;
          data.dir_data_["top"].filt_[pos + config.nX] = true;
          adaptfilter(data.dir_data_["top"].filtnumcoeffs_[pos + config.nX],
                      data.dir_data_["top"].filtcoeffsA_[pos + config.nX],
                      data.dir_data_["top"].filtcoeffsB_[pos + config.nX], tmp_filtid,
                      tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                      tmp_numfilters, (int)envi, angle,
                      kVertical);
        }
        if (x > 0) // apply a right filter to its left neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - 1] = true;
          data.dir_data_["right"].filt_[pos - 1] = true;
          adaptfilter(data.dir_data_["right"].filtnumcoeffs_[pos - 1],
                      data.dir_data_["right"].filtcoeffsA_[pos - 1],
                      data.dir_data_["right"].filtcoeffsB_[pos - 1], tmp_filtid,
                      tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                      tmp_numfilters, (int)envi, angle,
                      kHorizontal);
        }
        if (y > 0) // apply a bottom filter to its top neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - config.nX] = true;
          data.dir_data_["bottom"].filt_[pos - config.nX] = true;
          adaptfilter(data.dir_data_["bottom"].filtnumcoeffs_[pos - config.nX],
                      data.dir_data_["bottom"].filtcoeffsA_[pos - config.nX],
                      data.dir_data_["bottom"].filtcoeffsB_[pos - config.nX], tmp_filtid,
                      tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                      tmp_numfilters, (int)envi, angle,
                      kVertical);
        }
      } else if (envi < -1.0) // is the actual node a receiver-node?
      {
        if (actrec < config.nRec) {
          data.recIdx[actrec] =
              pos;  // yes, add the receiver's position into the
          actrec++; // recIdx-Array
        }
      }
    } // x-loop
  }   // y-loop
}

// RETURN VALUE
//   simError: NONE if file was opened successfully, error identfier otherwise.
simError Lambda::loadSimulation(const std::string fileName) {
  // some variables needed during the read-in process
  struct stat results;
  char *pblockid;
  double *pdummy;
  simError error = NONE;
  simSource curSource;
  bool donotreadnextblockid = false;
  float value;

  // check the filename
  if (stat((char *)fileName.c_str(), &results) != 0)
    return FILE_BAD;
  // reset all variables
  resetAll();
  // open the simfile
  std::ifstream simFile((char *)fileName.c_str(),
                        std::ios::in | std::ios::binary);

  //  ------ HEADER ------
  //  read the simfile header. it consists of the std::string "LAMBDASIM200".
  // otherwise, the file is corrupt or wrong version
  pblockid = new char[12];
  simFile.read(pblockid, sizeof(char) * 12);
  if (strncmp(pblockid, "LAMBDASIM200", 12) != 0) {
    simFile.close();
    delete[] pblockid;
    return FILE_HEADER_BAD_OR_WRONG_VERSION;
  }
  delete[] pblockid;

  pblockid = new char[3];
  //  ----- DEFINITIONS -----
  // read in the important simulation definitions.
  simFile.read(pblockid, sizeof(char) * 3);
  if (strncmp(pblockid, "DEF", 3) == 0) // DEF-Chunk exists?
  {
    std::cout << "parsing DEF chunk\n";
    pdummy = new double[6];
    simFile.read((char *)pdummy, sizeof(double) * 6); // read the data
    if (error == NONE)
      error = set("nY", (int)pdummy[0]); // simulation Y-size
    if (error == NONE)
      error = set("nX", (int)pdummy[1]); // simulation X-size
    if (error == NONE)
      error = set("nN", (int)pdummy[2]); // number of iterations
    if (error == NONE)
      error = set(
          "cTube",
          M_SQRT2 *
              (float)pdummy[3]); // Speed in tubes is sqrt(2)*c_{free field}!
    if (error == NONE)
      error = set("lTube", (float)pdummy[4]); // tube length
    if (error == NONE)
      error = set("rho", (float)pdummy[5]); // air density in kg/m^3
    delete[] pdummy;
    if (error != NONE) // if there was an error, close the file and exit
    {
      simFile.close();
      delete[] pblockid;
      return FILE_DEF_BLOCK_BAD;
    }
  } else // if no DEF-Chunk exists, close the file and exit
  {
    std::cout << "no DEF-Chunk exists, close the file and exit\n";
    simFile.close();
    delete[] pblockid;
    return FILE_DEF_BLOCK_BAD;
  }

  //  ----- ENVIRONMENT -----
  // read in the simulation environment matrix.
  simFile.read(pblockid, sizeof(char) * 3);
  if (strncmp(pblockid, "ENV", 3) == 0) // ENV-Chunk exists?
  {
    std::cout << "parsing ENV chunk\n";
    pdummy =
        new double[config.nNodes]; // reserve memory for env-data in simfile
    data.envi = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(0));
    data.deadnode = new bool[config.nNodes]; // reserve mem for deadnode matrix
    simFile.read((char *)pdummy,
                 sizeof(double) * config.nNodes); // read envi-matrix
    for (int pos = 0; pos < config.nNodes;
         pos++) // and cast it from double to float
    {
      value = (float)pdummy[pos];
      data.deadnode[pos] = false;
      if (value == -999) {         // is it a dead node?
        value = 0;                 // turn it into empty
        data.deadnode[pos] = true; // mark it as dead
      }
      data.envi.ptr<float>(0)[pos] = (float)value; // (all nodes)
    }
    delete[] pdummy;
  } else // ENV-Chunk does not exist -> close file and exit
  {
    simFile.close();
    delete[] pblockid;
    return FILE_ENV_BLOCK_BAD;
  }

  //  ----- ANGLES -----
  // read in the angle-matrix
  simFile.read(pblockid, sizeof(char) * 3);
  if (strncmp(pblockid, "ANG", 3) == 0) // ANG-Chunk exists?
  {
    std::cout << "parsing ANG chunk\n";
    pdummy =
        new double[config.nNodes]; // reserve memory for ang-data in simfile
    data.angle = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(0));
    simFile.read((char *)pdummy,
                 sizeof(double) * config.nNodes); // read angle-matrix
    for (int pos = 0; pos < config.nNodes;
         pos++)                             // and cast it from double to float
      data.angle.ptr<float>(0)[pos] = (float)pdummy[pos]; // (all nodes)
    delete[] pdummy;
    donotreadnextblockid = false; // make shure that the next chunk will be read
  } else if ((strncmp(pblockid, "FLT", 3) == 0) ||
             (strncmp(pblockid, "SRC", 3) == 0) ||
             (strncmp(pblockid, "SMP", 3) ==
              0)) { // if angle-matrix does not exist and the next Chunk is FLT
                    // or SMP or SRC
    std::cout
        << "angle-matrix does not exist and the next Chunk is FLT or SMP or "
           "SRC\n";
    // 400.0f this means no preemphasis is done on the nodes)
    data.angle = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(400.0f));
    donotreadnextblockid =
        true; // do not read the next Chunk ID, because we have it already
  } else      // if angle-matrix does not exist and no valid chunk is following
  {
    std::cout
        << "angle matrix does not exist and no valid chunk is following\n";
    simFile.close();
    delete[] pblockid;
    return FILE_ANG_BLOCK_BAD;
  }

  //  ----- FILTERS -----
  //  read in the filter data
  int tmp_numfilters;             // temporary number of filters
  int *tmp_filtid = NULL;         // temporary filter ID array
  int *tmp_filtnumcoeffs = NULL;  // temporary filter numcoeffs array
  float **tmp_filtcoeffsA = NULL; // temporary filter a-coeffs array
  float **tmp_filtcoeffsB = NULL; // temporary filter b-coeffs array
  tmp_numfilters = 1;             // at least one filter must exist!
  if (!donotreadnextblockid) // do not read the next Chunk-ID if ANG-Chunk was
                             // left out
    simFile.read(pblockid, sizeof(char) * 3);
  if (strncmp(pblockid, "FLT", 3) == 0) // FLT-Chunk exists?
  {
    std::cout << "parsing FLT chunk\n";
    pdummy = new double;
    simFile.read((char *)pdummy,
                 sizeof(double)); // yes, read in number of filters in chunk
    tmp_numfilters = ((int)*pdummy) + 1; // one more for the standard 0 filter
    delete pdummy;

    tmp_filtid = new int[tmp_numfilters]; // reserve memory for filter IDs
    tmp_filtnumcoeffs = new int[tmp_numfilters]; // reserve memory for numcoeffs
    tmp_filtcoeffsA =
        new float *[tmp_numfilters]; // reserve memory for filter a-coeffs
    tmp_filtcoeffsB =
        new float *[tmp_numfilters]; // reserve memory for filter b-coeffs

    for (int n = 1; n < tmp_numfilters;
         n++) // work through all the filters, except for
    {         // the 0 filter
      int numcoeffsA;
      int numcoeffsB;

      pdummy = new double;
      simFile.read((char *)pdummy, sizeof(double)); // read next filter ID
      tmp_filtid[n] = (int)*pdummy;
      simFile.read((char *)pdummy, sizeof(double)); // read number of a-coeffs
      numcoeffsA = (int)*pdummy;
      simFile.read((char *)pdummy, sizeof(double)); // read number of b-coeffs
      numcoeffsB = (int)*pdummy;
      delete pdummy;

      tmp_filtnumcoeffs[n] =
          std::max(numcoeffsA, numcoeffsB); // numcoeffs=maximum

      pdummy = new double[numcoeffsA];
      simFile.read((char *)pdummy,
                   sizeof(double) * numcoeffsA); // read the a-coeffs
      tmp_filtcoeffsA[n] =
          new float[tmp_filtnumcoeffs[n]]; // reserve mem for a-coeffs
      for (int k = 0; k < tmp_filtnumcoeffs[n]; k++)
        tmp_filtcoeffsA[n][k] = 0.f; // initialize the a-coeffs array
      for (int k = 0; k < numcoeffsA; k++)
        tmp_filtcoeffsA[n][k] =
            (float)pdummy[k]; // and cast the a-coeffs to float
      delete[] pdummy;

      pdummy = new double[numcoeffsB];
      simFile.read((char *)pdummy,
                   sizeof(double) * numcoeffsB); // read the b-coeffs
      tmp_filtcoeffsB[n] =
          new float[tmp_filtnumcoeffs[n]]; // reserve mem for b-coeffs
      for (int k = 0; k < tmp_filtnumcoeffs[n]; k++)
        tmp_filtcoeffsB[n][k] = 0.f; // initialize the b-coeffs array
      for (int k = 0; k < numcoeffsB; k++)
        tmp_filtcoeffsB[n][k] =
            (float)pdummy[k]; // and cast the b-coeffs to float
      delete[] pdummy;
    }
    tmp_filtid[0] = 0;                 // set up the 0 filter
    tmp_filtnumcoeffs[0] = 1;          // set up the 0 filter
    tmp_filtcoeffsA[0] = new float[1]; // set up the 0 filter
    tmp_filtcoeffsB[0] = new float[1]; // set up the 0 filter
    tmp_filtcoeffsA[0][0] = 1.f;       // set up the 0 filter
    tmp_filtcoeffsB[0][0] = 0.f;       // set up the 0 filter
    donotreadnextblockid = false;      // make sure that the next chunk is read
  } else if ((strncmp(pblockid, "SRC", 3) == 0) ||
             (strncmp(pblockid, "SMP", 3) ==
              0)) // the FLT-chunk is missing, is this a valid chunk?
  {
    std::cout
        << "the FLT-chunk is missing, this a valid chunk, initialize filters "
           "to 0\n";
    tmp_numfilters = 1; // if yes, initialize only the 0 filter
    tmp_filtid = new int[tmp_numfilters]; // reserve memory for the 0 filter
    tmp_filtnumcoeffs =
        new int[tmp_numfilters]; // reserve memory for the 0 filter
    tmp_filtcoeffsA =
        new float *[tmp_numfilters]; // reserve memory for the 0 filter
    tmp_filtcoeffsB =
        new float *[tmp_numfilters];   // reserve memory for the 0 filter
    tmp_filtid[0] = 0;                 // set up the 0 filter
    tmp_filtnumcoeffs[0] = 1;          // set up the 0 filter
    tmp_filtcoeffsA[0] = new float[1]; // set up the 0 filter
    tmp_filtcoeffsB[0] = new float[1]; // set up the 0 filter
    tmp_filtcoeffsA[0][0] = 1.f;       // set up the 0 filter
    tmp_filtcoeffsB[0][0] = 0.f;       // set up the 0 filter
    donotreadnextblockid = true;       // and don't read the next chunk-header,
                                       // because we already have it
  } else // if FLT-chunk does not exist and no valid chunk is following
  {
    std::cout << "FLT-chunk does not exist and no valid chunk is following\n";
    simFile.close();
    delete[] pblockid;
    return FILE_FLT_BLOCK_BAD;
  }

  //  ----- count the receivers -----
  set("nRec", 0);
  for (int pos = 0; pos < config.nNodes; pos++)
    if (data.envi.ptr(0)[pos] < -1.0)
      set("nRec", config.nRec + 1);
  // and reserve memory for the receivers
  if (config.nRec > 0) {
    if (data.recIdx != NULL)
      delete[] data.recIdx;
    data.recIdx = new int[config.nRec];
  }

  initEnvironment(tmp_filtid, tmp_filtnumcoeffs, tmp_filtcoeffsA,
                  tmp_filtcoeffsB, tmp_numfilters);

  // read samples
  if (!donotreadnextblockid) // read the chunk header if it is required (see
                             // above)
    simFile.read(pblockid, sizeof(char) * 3);
  set("nSamples", 0);
  simSample *sample = NULL;
  if (strncmp(pblockid, "SMP", 3) == 0) // is it a SMP-chuck
  {
    std::cout << "parsing SMP\n";
    pdummy = new double;
    simFile.read((char *)pdummy, sizeof(double));
    set("nSamples", (int)*pdummy);
    delete[] pdummy;
    std::cout << "found " << config.nSamples << " sources\n";
    data.samples = new_simSample_array(config.nSamples);
    for (int n = 0; n < config.nSamples; n++) // read all the samples
    {
      std::cout << "reading source " << n << "\n";
      pdummy = new double;
      sample = data.samples[n];
      simFile.read((char *)pdummy, sizeof(double)); // read sample ID
      sample->id = (int)*pdummy;
      std::cout << "IDX: " << sample->id << "\n";
      simFile.read((char *)pdummy, sizeof(double)); // read sample SR
      sample->sr = (int)*pdummy;
      std::cout << "SR: " << sample->sr << "\n";
      simFile.read((char *)pdummy, sizeof(double)); // read numsamples
      sample->nsamples = (int)*pdummy;
      std::cout << "nsamples: " << sample->nsamples << "\n";
      sample->data = new float[sample->nsamples]; // allocate memory
      delete[] pdummy;

      pdummy = new double[sample->nsamples];
      std::cout << "reading sample data\n";
      simFile.read((char *)pdummy, sizeof(double) * sample->nsamples);
      std::cout << "finished reading sample data\n";
      for (int pos = 0; pos < sample->nsamples; pos++) // convert it to float
      {
        sample->data[pos] = (float)pdummy[pos];
      }
      delete[] pdummy;
      std::cout << "\nfinished reading samples\n";
    }
    std::cout << "finished reading all samples\n";
    donotreadnextblockid = false;
  }

  //  ----- SOURCES -----
  // read in the sources
  bool *isvelosource;
  isvelosource =
      new bool[config.nNodes]; // temporary array, indicating velocity sources
  for (int pos = 0; pos < config.nNodes; pos++)
    isvelosource[pos] = false; // initialize the temp array
  set("nSrc", 0);

  if (!donotreadnextblockid) // read the chunk header if it is required (see
                             // above)
  {
    std::cout << "reading block ID\n";
    simFile.read(pblockid, sizeof(char) * 3);
    std::cout << "found: " << pblockid << "\n";
  } else
    std::cout << "skipping reading blockid for SRC chunk\n";
  if (strncmp(pblockid, "SRC", 3) == 0) // is it a SRC-chunk?
  {
    std::cout << "parsing SRC chunk\n";
    pdummy = new double;
    simFile.read((char *)pdummy,
                 sizeof(double)); // yes, read in the number of sources
    set("nSrc", (int)*pdummy);
    data.srcs = new float[config.nSrc * 6]; // reserve memory for the sources
    data.mem = new float[config.nSrc * MEMSRC]; // sources extra data
    for (int n = 0; n < config.nSrc * MEMSRC; n++)
      data.mem[n] = 0.0;
    for (int n = 0; n < config.nSrc; n++) {
      simFile.read((char *)pdummy, sizeof(double)); // read src y-position
      curSource.y = (int)*pdummy - 1;
      simFile.read((char *)pdummy, sizeof(double)); // read src x-position
      curSource.x = (int)*pdummy - 1;
      simFile.read((char *)pdummy, sizeof(double)); // read src type
      curSource.type = (float)*pdummy;
      simFile.read((char *)pdummy, sizeof(double)); // read src amplitude
      curSource.amp = (float)*pdummy;
      simFile.read((char *)pdummy, sizeof(double)); // read src frequency
      curSource.freq = (float)*pdummy;
      simFile.read((char *)pdummy, sizeof(double)); // read src phase angle
      curSource.phase = (float)*pdummy;
      // if the source type is between 6 and 10, then it is a velocity source:
      if ((curSource.type >= 6) && (curSource.type <= 10))
        isvelosource[(int)curSource.y * (int)config.nX + (int)curSource.x] =
            true;
      // a sample source
      if (curSource.type == 30) {
        sample = data.samples[(int)curSource.freq];
        std::cout << "sample source: IDX=" << sample->id << " SR=" << sample->sr
                  << " NSAMPLES=" << sample->nsamples
                  << " DURATION(ms)=" << (sample->nsamples * 1000) / sample->sr
                  << "\n";
      }
      defineSource(n, &curSource); // and add the source to the simulation
      std::cout << "finished defining source of type " << curSource.type
                << "\n";
    }
    delete pdummy;
  } else // no SRC-chunk found --> delete all our work we've done so far
  {
    std::cout << "no SRC-chunk found, found instead " << pblockid << "\n";
    std::cout
        << "no SRC-chunk found --> delete all our work we've done so far\n";
    simFile.close();
    delete[] pblockid;
    delete[] isvelosource;
    for (int n = 0; n < tmp_numfilters; n++)
      delete[] tmp_filtcoeffsB[n];
    delete[] tmp_filtcoeffsB;
    for (int n = 0; n < tmp_numfilters; n++)
      delete[] tmp_filtcoeffsA[n];
    delete[] tmp_filtcoeffsA;
    delete[] tmp_filtnumcoeffs;
    delete[] tmp_filtid;
    return FILE_SRC_BLOCK_BAD;
  }

  delete[] pblockid;

  //  ----- DEACTIVATE BOUNDARIES BETWEEN ADJACENT VELOCITY SOURCES
  // this is important for the stability of the simulation. all boundaries
  // between adjacent velocity source nodes must be removed again.
  for (int y = 0; y < config.nY; y++) {
    for (int x = 0; x < config.nX; x++) {
      if (isvelosource[y * config.nX +
                       x]) // is actual node a velocity source node?
      {
        if (x > 0)
          if (isvelosource[y * config.nX + x -
                           1]) // is left neighbour a velo src?
            data.dir_data_["left"].filt_[y * config.nX + x] =
                false; // yes, disable left filter
        if (y > 0)
          if (isvelosource[(y - 1) * config.nX +
                           x]) // is top neighbour a velo src?
            data.dir_data_["top"].filt_[y * config.nX + x] = false; // yes, disable top filter
        if (x < config.nX - 1)
          if (isvelosource[y * config.nX + x +
                           1]) // is right neighbour a velo src?
            data.dir_data_["right"].filt_[y * config.nX + x] =
                false; // yes, disable right filter
        if (y < config.nY - 1)
          if (isvelosource[(y + 1) * config.nX +
                           x]) // is bottom neighbour a velo src?
            data.dir_data_["bottom"].filt_[y * config.nX + x] =
                false; // yes, disable bottom filter
      }
    }
  }
  delete[] isvelosource; // the temporary isvelosource-array is now not needed
                         // any longer.

  //  ----- DEADNODE SCANNER -----
  // search the environment for "dead" nodes. a dead node is a node which is
  // surrounded by filters on all four sides, so that their pressure always must
  // be 0. For example, every normal filter node is such a dead node. these
  // nodes are indicated in the deadnode-array, to spare them out of the
  // simulation process. this speeds up the simulation.

  // work through all nodes
  for (int pos = 0; pos < config.nNodes; pos++) {
    // four walls around the node?
    if ((data.dir_data_["left"].filt_[pos]) &&
        (data.dir_data_["top"].filt_[pos]) &&
        (data.dir_data_["right"].filt_[pos]) &&
        (data.dir_data_["bottom"].filt_[pos])) {
      data.deadnode[pos] = true; // ---> yes, it's a deadnode
    }
  }

  for (int n = 0; n < tmp_numfilters;
       n++) // delete all the temporary filter stuff
    delete[] tmp_filtcoeffsB[n];
  delete[] tmp_filtcoeffsB;
  for (int n = 0; n < tmp_numfilters;
       n++) // delete all the temporary filter stuff
    delete[] tmp_filtcoeffsA[n];
  delete[] tmp_filtcoeffsA;   // delete all the temporary filter stuff
  delete[] tmp_filtnumcoeffs; // delete all the temporary filter stuff
  delete[] tmp_filtid;        // delete all the temporary filter stuff

  simFile.close();               // close the simfile
  files.lastFileName = fileName; // and remember its name
  return NONE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Lambda::loadRecord
//
// PURPOSE
//   Tries to load a recorded playback file.
//
// INPUT
//   const std::string fileName : name of the file to be opened.
//
// OUTPUT
//   None
//
// RETURN VALUE
//   simError: NONE if file was opened successfully, error identfier otherwise.
//
//	AUTHOR		CHANGES
// DATE	VERSION 	S. Ahrens 	First build
// 05/06	1.0 	M. Ruhland 	no changes
// 05/09	2.0
//
simError Lambda::loadRecord(const std::string fileName) {
  struct stat results; // File info
  // Try to get file info, return error if file could not be opened
  if (stat((char *)fileName.c_str(), &results) != 0)
    return FILE_BAD;
  // Each record file should at least contain 2 doubles: nY and nX.
  if ((int)results.st_size <= 2 * (int)sizeof(double))
    return FILE_SIZE_BAD;
  resetAll();
  // open recfile and create pointer
  std::ifstream recFile((char *)fileName.c_str(),
                        std::ios::in | std::ios::binary);
  // read nY and nX into the dim array
  double *dim = new double[2];
  recFile.read((char *)dim, sizeof(double) * 2);
  // Make sure that nY and nX (dim[0] and dim[1] are greater than 0
  // and check if the rest of the file is dividable by nY*nX doubles. If not,
  // the file must be corrupt. If the file is okay, behind the two dim doubles
  // there is an unknown number of frames, each consisting of nY*nX floats.
  if (((int)dim[0] * (int)dim[1] <= 0) ||
      ((results.st_size - sizeof(double) * 2) %
           ((int)dim[0] * (int)dim[1] * sizeof(float)) !=
       0)) {
    recFile.close();
    delete[] dim;
    return FILE_SIZE_BAD;
  }
  // Set nX and nY to the new values
  set("nX", (int)dim[0]);
  set("nY", (int)dim[1]);
  delete[] dim;
  // The number of frames is the file size minus the two dim doubles divided
  // by the size of one single frame.
  set("nN",
      (results.st_size - 2 * sizeof(double)) / (config.nNodes * sizeof(float)));
  // Make sure that the user cannot choose to watch more frames than available
  // Prepare recorded data array
  data.record = new float[config.nNodes * config.nN];
  // read recorded data
  recFile.read((char *)data.record, sizeof(float) * config.nN * config.nNodes);
  recFile.close();
  files.lastFileName = fileName;
  return NONE;
}

float **new_array2(int n) {
  // allocate a new array of array, without allocating the sub-arrays
  // these will be allocated if filters are defined
  // important: set each slot to NULL
  float **out = new float *[n];
  for (int i = 0; i < n; i++) {
    out[i] = NULL;
  }
  return out;
}

//   Prepares variables needed for simulation.
// RETURN VALUE
//   simError: NONE if no error occured, error identfier otherwise.
simError Lambda::initSimulationPre() {
  // Check one more time
  if (config.nNodes < 1)
    return NO_NODES;

  data.envi = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(0));
  data.deadnode = new bool[config.nNodes]; // reserve mem for deadnode matrix
  for (int pos = 0; pos < config.nNodes; pos++) {
    data.deadnode[pos] = false;
  }

  // 400.0f means no preemphasis is done on the nodes)
  data.angle = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(400.0f));

  return NONE;
}

void Lambda::initEnvironmentSetup() {
  int *tmp_filtid = NULL;         // temporary filter ID array
  int *tmp_filtnumcoeffs = NULL;  // temporary filter numcoeffs array
  float **tmp_filtcoeffsA = NULL; // temporary filter a-coeffs array
  float **tmp_filtcoeffsB = NULL; // temporary filter b-coeffs array
  int tmp_numfilters;             // temporary number of filters

  tmp_numfilters = 1;                   // if yes, initialize only the 0 filter
  tmp_filtid = new int[tmp_numfilters]; // reserve memory for the 0 filter
  tmp_filtnumcoeffs =
      new int[tmp_numfilters]; // reserve memory for the 0 filter
  tmp_filtcoeffsA =
      new float *[tmp_numfilters]; // reserve memory for the 0 filter
  tmp_filtcoeffsB =
      new float *[tmp_numfilters];   // reserve memory for the 0 filter
  tmp_filtid[0] = 0;                 // set up the 0 filter
  tmp_filtnumcoeffs[0] = 1;          // set up the 0 filter
  tmp_filtcoeffsA[0] = new float[1]; // set up the 0 filter
  tmp_filtcoeffsB[0] = new float[1]; // set up the 0 filter
  tmp_filtcoeffsA[0][0] = 1.f;       // set up the 0 filter
  tmp_filtcoeffsB[0][0] = 0.f;       // set up the 0 filter

  initEnvironment(tmp_filtid, tmp_filtnumcoeffs, tmp_filtcoeffsA,
                  tmp_filtcoeffsB, tmp_numfilters);
}

simError Lambda::initSimulation() {
  config.n = 0;
  // incident pressure pulses
  data.inci = new float[12 * config.nNodes];
  for (int pos = 0; pos < 12 * config.nNodes; pos++) {
    data.inci[pos] = 0;
  }
  // set up indices needed for the simulation
  for (int x = 0; x < 3; x++) {
    data.pressure_[x] = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(0));
    index.idxP[x] = data.pressure_[x].ptr<float>(0);

    index.inci_["left"].idxI[x] = data.inci + (x * 4 + 0) * config.nNodes;
    index.inci_["top"].idxI[x] = data.inci + (x * 4 + 1) * config.nNodes;
    index.inci_["right"].idxI[x] = data.inci + (x * 4 + 2) * config.nNodes;
    index.inci_["bottom"].idxI[x] = data.inci + (x * 4 + 3) * config.nNodes;
  }

  // velocity sources
  for (const std::string& dir : dirs_) {
    data.dir_data_[dir].velo_ = cv::Mat(cv::Size(config.nX, config.nY), CV_32FC1, cv::Scalar::all(0.0));

    // reserve memory for the filter memories
    // data.oldx_left=new float*[config.nNodes];
    data.dir_data_[dir].oldx_ = new_array2(config.nNodes);
    data.dir_data_[dir].oldy_ = new_array2(config.nNodes);

    for (int pos = 0; pos < config.nNodes; pos++) {
      if (data.dir_data_[dir].filtnumcoeffs_[pos] >= 1) {
        // reserve+initialize recursive and non-recursive memory for left filters
        int memorycnt = data.dir_data_[dir].filtnumcoeffs_[pos] - 1;
        if (memorycnt == 0) // to ensure that even 0th order filters have
          memorycnt = 1; // memory; this spares an if-condition in the algorithm
        data.dir_data_[dir].oldx_[pos] = new float[memorycnt];
        data.dir_data_[dir].oldy_[pos] = new float[memorycnt];
        for (int k = 0; k < memorycnt; k++) {
          data.dir_data_[dir].oldx_[pos][k] = 0.f;
          data.dir_data_[dir].oldy_[pos][k] = 0.f;
        }
      }
    }
  }

  return NONE;
}

void Lambda::setPressure(const size_t x, const size_t y, const float value)
{
  if (x >= config.nX)
    return;
  if (y >= config.nY)
    return;
  int idx = ((config.n + 1) % 3); // current index
  data.pressure_[idx].at<float>(y, x) = value;
}

void Lambda::addPressure(const size_t x, const size_t y, const float value)
{
  if (x >= config.nX)
    return;
  if (y >= config.nY)
    return;
  int idx = ((config.n + 1) % 3); // current index
  data.pressure_[idx].at<float>(y, x) += value;
}

// this has to be done before initSimulation
void Lambda::setWall(const size_t x, const size_t y, const float value)
{
  if (x >= config.nX)
    return;
  if (y >= config.nY)
    return;
  data.envi.at<float>(y, x) = value;
}

// alpha is angle
void Lambda::setVel(const int& srcxy, const float& magnitude, const float& alpha) {
  if ((alpha >= 0.f) && (alpha < 90.f)) {
    // alpha between 0 and 90 degrees? -> left and top incidence
    const float rad = alpha * rad_per_deg;
    data.dir_data_["left"].velo_.ptr<float>(0)[srcxy] = cos(rad) * magnitude;
    data.dir_data_["top"].velo_.ptr<float>(0)[srcxy] = sin(rad) * magnitude;
  } else if ((alpha >= 90.f) && (alpha < 180.f)) {
    // alpha between 90 and 180 degrees? -> top and right incidence
    const float rad = (alpha - 90.f) * rad_per_deg;
    data.dir_data_["top"].velo_.ptr<float>(0)[srcxy] = cos(rad) * magnitude;
    data.dir_data_["right"].velo_.ptr<float>(0)[srcxy] = sin(rad) * magnitude;
  } else if ((alpha >= 180.f) && (alpha < 270.f)) {
    // alpha between 180 and 270 degrees? -> right and bottom incidence
    const float rad = (alpha - 180.f) * rad_per_deg;
    data.dir_data_["right"].velo_.ptr<float>(0)[srcxy] = cos(rad) * magnitude;
    data.dir_data_["bottom"].velo_.ptr<float>(0)[srcxy] = sin(rad) * magnitude;
  } else if ((alpha >= 270.f) && (alpha < 360.f)) {
    // alpha between 270 and 360 degrees? -> bottom and left incidence
    const float rad = (alpha - 270.f) * rad_per_deg;
    data.dir_data_["bottom"].velo_.ptr<float>(0)[srcxy] = cos(rad) * magnitude;
    data.dir_data_["left"].velo_.ptr<float>(0)[srcxy] = sin(rad) * magnitude;
  }
}

//   Processes the next simulation iteration.
void Lambda::processSim() {
  {
    // periodic cycling of time indices
    int idxPast = ((config.n + 0) % 3); // past index
    int idxPres = ((config.n + 1) % 3); // present index
    int idxFutu = ((config.n + 2) % 3); // future index
    // Set these pointers at the beginning of the iteration.
    // They would have to be calculated several times during the process
    // otherwise.
    index.presPast = index.idxP[idxPast]; // past pressure index
    index.presPres = index.idxP[idxPres]; // present pressure index
    index.presFutu = index.idxP[idxFutu]; // future pressure index
    // std::cout << index.presPast[0] << " " << index.presPres[1]
    //    << " " << index.presFutu[2] << std::endl;
    float *index_presFutu = index.presFutu;
    float *presPres = index.presPres;

    for (const std::string& dir : dirs_) {
      index.inci_[dir].past_ =
          index.inci_[dir].idxI[idxPast]; // past left incident pressure index
      index.inci_[dir].pres_ =
          index.inci_[dir].idxI[idxPres]; // present left incident pressure index
      index.inci_[dir].futu_ =
          index.inci_[dir].idxI[idxFutu]; // future left incident pressure index
    }

    // TODO(lucasw) map lookups are slow
    // future+present scattered pressure in each direction
    std::map<std::string, float> scatFutu;
    std::map<std::string, float> scatPres;

    // TODO(lucasw) is there anything special about these sources vs.
    // feeding in the source externally?
    // Work through all the sources. First init all the sources
    for (int src = 0; src < config.nSrc; src++) {
      int srcpos = src * 6;
      int srcxy = (int)data.srcs[srcpos] +
                  (int)data.srcs[srcpos + 1]; // get actual source pos.
      presPres[srcxy] = 0;
    }
    // then calculate press for each.
    // this allows pressure sources to share the same node
    for (int src = 0; src < config.nSrc; src++) {
      register int srcpos = src * 6;
      int srcxy = (int)data.srcs[srcpos] +
                  (int)data.srcs[srcpos + 1]; // get actual source pos.
      int type = (int)data.srcs[srcpos + 2];  // get actual source type
      float amp = data.srcs[srcpos + 3];      // get actual source amplitude
      float freq = data.srcs[srcpos + 4];     // get actual source frequency
      float phi = data.srcs[srcpos + 5];      // get actual source phase offset
      float alpha = data.angle.ptr<float>(0)[srcxy]; // get actual source angle of incidence
      if ((alpha < 0.f) || (alpha >= 360.f)) // angle out of bounds?
        alpha = 0.f;
      float t = (float)config.n / (float)config.fSample;
      float twopi = 6.2831853f;
      float T = 1.f / freq;
      float hann;
      float magnitude = 0.f;
      float twopi_freq = twopi * freq;
      float tmp0, tmp1;
      // float onepi_phi_180 = onepi*phi/180.f;
      float onepi_phi_180 = phi * 0.034906585f;
      float b0, b1, b2, white;
      int samplepos, mempos;
      simSample *sample;
      switch (type) {
      case 1: // sinusoidal source
        // index.presPres[srcxy]=amp*sin(twopi*freq*t+onepi*phi/180.f);
        presPres[srcxy] += amp * sin(twopi_freq * t + onepi_phi_180);
        break;
      case 2: // rectangular source
        if ((int)(2.f * (freq * t + phi / 360.f)) % 2 == 0)
          presPres[srcxy] += amp;
        else
          presPres[srcxy] -= amp;
        break;
      case 3: // delta-pulse source
        if (config.n == 0)
          presPres[srcxy] += amp;
        break;
      case 4: // exponential decay source (not working correctly yet!!!)
        presPres[srcxy] += amp * exp(-(float)config.n);
        break;
      case 5: // hann-windowed sinusoidal source
        if (t < 0.f)
          hann = 0.f;
        else if (t > (T / 2.f))
          hann = 1.f;
        else {
          hann =
              (float)cos(M_PI * (t + T / 2.f) / T); // compute hann window
          hann *= hann;
        }
        presPres[srcxy] +=
            hann * amp * (float)sin(twopi_freq * t + onepi_phi_180);
        break;
      case 6: // sinusoidal velocity source
        magnitude = config.rho * config.cTube * amp *
                    sin(twopi_freq * t + onepi_phi_180);
        // TODO(lucasw) all this is duplicate code for the next few cases
        // only the magnitude changes
        break;
      case 7: // rectangular velocity source
        if ((int)(2.f * (freq * t + phi / 360.f)) % 2 == 0)
          magnitude = config.rho * config.cTube * amp;
        else
          magnitude = -config.rho * config.cTube * amp;
        setVel(srcxy, magnitude, alpha);
        break;
      case 8: // delta-pulse velocity source
        if (config.n == 0)
          magnitude = config.rho * config.cTube * amp;
        setVel(srcxy, magnitude, alpha);
        break;
      case 9: // exponential decay velocity source (not working correctly
              // yet!!!)
        magnitude = config.rho * config.cTube * amp * exp(-(float)config.n);
        setVel(srcxy, magnitude, alpha);
        break;
      case 10: // hann-windowed sinusoidal velocity source
        if (t < 0.f)
          hann = 0.f;
        else if (t > (T / 2.f))
          hann = 1.f;
        else {
          hann =
              (float)cos(M_PI * (t + T / 2.f) / T); // compute hann window
          hann *= hann;
        }
        magnitude = config.rho * config.cTube * hann * amp *
                    (float)sin(twopi * freq * t + M_PI * phi / 180.f);
        setVel(srcxy, magnitude, alpha);
        break;
      case 20: // white-noise
        presPres[srcxy] += amp * ((rand() % 32767) / 32767.f * 2.f - 1.f);
        break;
      case 21: // pink noise
        mempos = src * MEMSRC;
        white = amp * ((rand() % 32767) / 32767.f * 2.f - 1.f);
        b0 = data.mem[mempos];
        b1 = data.mem[mempos + 1];
        b2 = data.mem[mempos + 2];
        b0 = 0.99765f * b0 + white * 0.0990460f;
        b1 = 0.96300f * b1 + white * 0.2965164f;
        b2 = 0.57000f * b2 + white * 1.0526913f;
        presPres[srcxy] += b0 + b1 + b2 + white * 0.1848;
        data.mem[mempos] = b0;
        data.mem[mempos + 1] = b1;
        data.mem[mempos + 2] = b2;
        break;
      case 30: // sample
        mempos = src * MEMSRC;
        sample = data.samples[(int)freq];  // freq contains the sample id
        samplepos = (int)data.mem[mempos]; // read position
        presPres[srcxy] = sample->data[samplepos] *
                          amp; // read individual sample, scale it to amp
        data.mem[mempos] =
            (float)((samplepos + 1) %
                    sample->nsamples); // advance read position, loop if at end
      }
    }

    ////////////////////////////////////////////////////////////////////////
    // it's expensive to do a lot of map lookups in the big loop before,
    // they don't get optimized out, so set up these temp arrays instead.
    std::vector<DirData*> dir_datas;
    std::vector<Inci*> incis;
    std::vector<int> dirs;
    for (const std::string& dir : dirs_) {
      dir_datas.push_back(&data.dir_data_[dir]);
      incis.push_back(&index.inci_[dir]);
      if (dir == "left")
        dirs.push_back(LEFT);
      if (dir == "right")
        dirs.push_back(RIGHT);
      if (dir == "top")
        dirs.push_back(TOP);
      if (dir == "bottom")
        dirs.push_back(BOTTOM);
    }
    int n;    // counter variable
    float yn; // filter output
    int config_nX = config.nX;
    // Work through all the nodes in the environment
    for (int pos = 0; pos < config.nNodes; pos++) {
      index_presFutu[pos] = 0.f;
      if (data.deadnode[pos]) // deadnode? --> no calculation needed!
        continue;

      if (data.boundary[pos]) // boundary? --> no standard propagation!
      {
        for (size_t d = 0; d < 4; ++d) {
          // TODO(lucasw) clean this up by make this a method of DirData?
          //  filter
          if (dir_datas[d]->filt_[pos]) {
            // calculate filter input
            const float scat_futu = presPres[pos] - incis[d]->pres_[pos];
            // calculate the digital filter
            yn = scat_futu * dir_datas[d]->filtcoeffsB_[pos][0];
            for (n = 1; n < dir_datas[d]->filtnumcoeffs_[pos]; n++) {
              yn += dir_datas[d]->oldx_[pos][n - 1] *
                  dir_datas[d]->filtcoeffsB_[pos][n];
              yn -= dir_datas[d]->oldy_[pos][n - 1] *
                  dir_datas[d]->filtcoeffsA_[pos][n];
            }
            // add magnitude of a possible velocity source
            yn += dir_datas[d]->velo_.ptr<float>(0)[pos];
            // rotate the filter memories
            for (n = dir_datas[d]->filtnumcoeffs_[pos] - 2; n > 0; n--) {
              dir_datas[d]->oldx_[pos][n] = dir_datas[d]->oldx_[pos][n - 1];
              dir_datas[d]->oldy_[pos][n] = dir_datas[d]->oldy_[pos][n - 1];
            }
            dir_datas[d]->oldx_[pos][0] = scat_futu;
            dir_datas[d]->oldy_[pos][0] = yn;
            // and write the filter output into the pressure matrix
            incis[d]->futu_[pos] = yn;
            index_presFutu[pos] += incis[d]->futu_[pos];
          } else {
            // no filter in this direction
            const float scat_pres = index.presPast[pos] - incis[d]->past_[pos];

            float incis_futu = 0.0;
            if (dirs[d] == LEFT)
              incis_futu = presPres[pos - 1] - scat_pres;
            else if (dirs[d] == TOP)
              incis_futu = presPres[pos - config_nX] - scat_pres;
            else if (dirs[d] == RIGHT)
              incis_futu = presPres[pos + 1] - scat_pres;
            else if (dirs[d] == BOTTOM)
              incis_futu = presPres[pos + config_nX] - scat_pres;

            incis[d]->futu_[pos] = incis_futu;
            index_presFutu[pos] += incis_futu;
          }
          index_presFutu[pos] *= 0.5f;
        } // dir loop
      } else {
        // no boundary node: do the fast standard propagation
        index_presFutu[pos] =
            (presPres[pos - 1] + presPres[pos - config_nX] +
             presPres[pos + 1] + presPres[pos + config_nX]) *
                0.5f -
            index.presPast[pos];
      }  // boundary
    }  // loop through all nodes
    config.n++;
  }  // unneeded bracketing, delete it later
}  // processSim

//   Creates a new digital filter for a given real-valued reflexion factor and
//   preemphases the filter coefficients due to a given sonic incidence angle
//   and horizontal/vertical alignment of the filter.
//
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
void Lambda::adaptreflexionfactor(int &dest_numcoeffs, float *&dest_coeffsA,
                                  float *&dest_coeffsB, float r, float alpha,
                                  simAngularType direction) {
  // new filter has got only one a- and one b-coefficient
  dest_numcoeffs = 1;
  // if a destination filter is already existing, then delete it
  if (dest_coeffsA != NULL)
    delete[] dest_coeffsA;
  if (dest_coeffsB != NULL)
    delete[] dest_coeffsB;
  // reserve memory for the a- and b-coefficient
  dest_coeffsA = new float[1];
  dest_coeffsB = new float[1];
  // is alpha out of range? [-360..+360 degrees] -> yes, no preemphasis
  if ((alpha <= -360.f) || (alpha >= 360.f))
    direction = kNone;
  float a, b, anew, bnew;
  // set the temporary filter coeffs (a0=1, b0=reflexion factor)
  a = 1.f;
  b = r;
  if (direction == kHorizontal) {
    // do horizontal preemphasis
    anew =
        (a + b) + M_SQRT2 * abs(cos(alpha * rad_per_deg)) * (a - b);
    bnew =
        (a + b) - M_SQRT2 * abs(cos(alpha * rad_per_deg)) * (a - b);
  } else if (direction == kVertical) {
    // do vertical preemphasis
    anew =
        (a + b) + M_SQRT2 * abs(sin(alpha * rad_per_deg)) * (a - b);
    bnew =
        (a + b) - M_SQRT2 * abs(sin(alpha * rad_per_deg)) * (a - b);
  } else {
    // no preemphasis
    anew = a;
    bnew = b;
  }
  // normalize the filter coefficients
  dest_coeffsA[0] = 1.f;
  dest_coeffsB[0] = bnew / anew;
}

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
//   float alpha			: angle of incidence for the desired filter,
//   needed for preemphasis simAngularType direction : sets whether the filter
//   is used in horizontal or vertical tubes, needed for preemphasis
//
// OUTPUT
//   int& dest_numcoeffs    : number of filter coefficients for the calculated
//   filter float*& dest_coeffsA   : array containig the new computed
//   a-Filter-coefficients float*& dest_coeffsB   : array containig the new
//   computed b-Filter-coefficients
void Lambda::adaptfilter(int &dest_numcoeffs, float *&dest_coeffsA,
                         float *&dest_coeffsB, int *src_id, int *src_numcoeffs,
                         float **src_coeffsA, float **src_coeffsB,
                         int src_numfilters, int id, float alpha,
                         simAngularType direction) {
  // search for the filter ID in the source filter array
  // (if no matching ID is found, set ID to 0 which is the standard 0 filter)
  int actnum = 0;
  for (int kk = 0; kk < src_numfilters; kk++)
    if (src_id[kk] == id)
      actnum = kk;
  // get number of filter coefficients
  dest_numcoeffs = src_numcoeffs[actnum];
  // if a destination filter is already existing, then delete it
  if (dest_coeffsA != NULL)
    delete[] dest_coeffsA;
  if (dest_coeffsB != NULL)
    delete[] dest_coeffsB;
  // reserve memory for the a- and b-coefficients
  dest_coeffsA = new float[src_numcoeffs[actnum]];
  dest_coeffsB = new float[src_numcoeffs[actnum]];
  // is alpha out of range? [-360..+360 degrees] -> yes, no preemphasis
  if ((alpha <= -360.f) || (alpha >= 360.f))
    direction = kNone;
  // preemphase all filter coefficients
  for (int kk = 0; kk < src_numcoeffs[actnum]; kk++) {
    float a, b, anew, bnew;
    // get one pair of the temporary filter coeffs
    a = src_coeffsA[actnum][kk];
    b = src_coeffsB[actnum][kk];
    if (direction == kHorizontal) {
      // do horizontal preemphasis
      const float ca = M_SQRT2 * abs(cos(alpha * rad_per_deg)) * (a - b);
      anew = (a + b) + ca;
      bnew = (a + b) - ca;
    } else if (direction == kVertical) {
      // do vertical preemphasis
      const float sa = M_SQRT2 * abs(sin(alpha * rad_per_deg)) * (a - b);
      anew = (a + b) + sa;
      bnew = (a + b) - sa;
    } else {
      // no preemphasis
      anew = a;
      bnew = b;
    }
    dest_coeffsA[kk] = anew;
    dest_coeffsB[kk] = bnew;
  }
  // normalize the filter coefficients
  float a0 = dest_coeffsA[0];
  for (int kk = 0; kk < src_numcoeffs[actnum]; kk++) {
    dest_coeffsA[kk] /= a0;
    dest_coeffsB[kk] /= a0;
  }
  dest_coeffsA[0] = 1.f;
}
