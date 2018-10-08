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

  for (int n = 0; n < config.nNodes; n++) {
    for (size_t d = 0; d < 4; ++d) {
      // TODO(lucasw) make a function to eliminate redundant code-
      // make it a simData method.
      // delete A filter coefficient arrays
      if (data.nodes_[n].filter_[d].coeffsA_ != NULL) {
        delete[] data.nodes_[n].filter_[d].coeffsA_;
        data.nodes_[n].filter_[d].coeffsA_ = NULL;
      }
      if (data.nodes_[n].filter_[d].coeffsB_ != NULL) {
        delete[] data.nodes_[n].filter_[d].coeffsB_;
        data.nodes_[n].filter_[d].coeffsB_ = NULL;
      }
    }  // loop through directions
  }  // loop through nodes
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
  }

  // Delete bottom filter non-recursive memory
  for (int n = 0; n < config.nNodes; n++) {
    for (size_t d = 0; d < 4; ++d) {
      data.nodes_[n].filter_[d].oldx_.reset(nullptr);
      data.nodes_[n].filter_[d].oldy_.reset(nullptr);
      data.nodes_[n].filter_[d].velo_ = 0.0;
    }
  }
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
      data.nodes_[srcxy].filter_[LEFT].filt_ = true;
      data.nodes_[srcxy].filter_[TOP].filt_ = true;
      adaptreflexionfactor(
          data.nodes_[srcxy].filter_[LEFT].numcoeffs_,
          data.nodes_[srcxy].filter_[LEFT].coeffsA_,
          data.nodes_[srcxy].filter_[LEFT].coeffsB_, 1.f, 180.f, kHorizontal);
      adaptreflexionfactor(data.nodes_[srcxy].filter_[TOP].numcoeffs_,
                           data.nodes_[srcxy].filter_[TOP].coeffsA_,
                           data.nodes_[srcxy].filter_[TOP].coeffsB_, 1.f, 270.f, kVertical);
    } else if ((alpha >= 90.f) && (alpha < 180.f)) {
      // set up a top- and right- filter if angle of incidence is
      // between 90 and 180 degrees
      data.boundary[srcxy] = true;
      data.nodes_[srcxy].filter_[TOP].filt_ = true;
      data.nodes_[srcxy].filter_[RIGHT].filt_ = true;
      adaptreflexionfactor(data.nodes_[srcxy].filter_[TOP].numcoeffs_,
                           data.nodes_[srcxy].filter_[TOP].coeffsA_,
                           data.nodes_[srcxy].filter_[TOP].coeffsB_, 1.f, 270.f, kVertical);
      adaptreflexionfactor(
          data.nodes_[srcxy].filter_[RIGHT].numcoeffs_,
          data.nodes_[srcxy].filter_[RIGHT].coeffsA_,
          data.nodes_[srcxy].filter_[RIGHT].coeffsB_, 1.f, 0.f, kHorizontal);
    } else if ((alpha >= 180.f) && (alpha < 270.f)) {
      // set up a right- and bottom- filter if angle of incidence is
      // between 180 and 270 degrees
      data.boundary[srcxy] = true;
      data.nodes_[srcxy].filter_[RIGHT].filt_ = true;
      data.nodes_[srcxy].filter_[BOTTOM].filt_ = true;
      adaptreflexionfactor(
          data.nodes_[srcxy].filter_[RIGHT].numcoeffs_,
          data.nodes_[srcxy].filter_[RIGHT].coeffsA_,
          data.nodes_[srcxy].filter_[RIGHT].coeffsB_, 1.f, 0.f, kHorizontal);
      adaptreflexionfactor(
          data.nodes_[srcxy].filter_[BOTTOM].numcoeffs_,
          data.nodes_[srcxy].filter_[BOTTOM].coeffsA_,
          data.nodes_[srcxy].filter_[BOTTOM].coeffsB_, 1.f, 90.f, kVertical);
    } else if ((alpha >= 270.f) && (alpha < 360.f)) {
      // set up a bottom- and left- filter if angle of incidence is
      // between 270 and 360 degrees
      data.boundary[srcxy] = true;
      data.nodes_[srcxy].filter_[BOTTOM].filt_ = true;
      data.nodes_[srcxy].filter_[LEFT].filt_ = true;
      adaptreflexionfactor(
          data.nodes_[srcxy].filter_[BOTTOM].numcoeffs_,
          data.nodes_[srcxy].filter_[BOTTOM].coeffsA_,
          data.nodes_[srcxy].filter_[BOTTOM].coeffsB_, 1.f, 90.f, kVertical);
      adaptreflexionfactor(
          data.nodes_[srcxy].filter_[LEFT].numcoeffs_,
          data.nodes_[srcxy].filter_[LEFT].coeffsA_,
          data.nodes_[srcxy].filter_[LEFT].coeffsB_, 1.f, 180.f, kHorizontal);
    }
  }
  return NONE;
}

//  ----- PREPROCESSING OF THE ENVIRONMENT -----
void Lambda::initEnvironment() {
  data.boundary = new bool[config.nNodes]; // mem for boundary indicator
  for (int pos = 0; pos < config.nNodes; pos++) {
    data.boundary[pos] = false;
  }
  // data.deadnode=new bool[config.nNodes];              // mem for deadnode
  // indicator
  data.nodes_.reset(new Node[config.nNodes]);

  int actrec = 0;

  // work through all nodes in the environment
  for (int y = 0; y < config.nY; y++) {
    for (int x = 0; x < config.nX; x++) {
      const int pos = y * config.nX + x;
      if ((y == 0) || (x == 0) || (y == config.nY - 1) ||
          (x == config.nX - 1)) // is this a simfield border node?
      {
        data.boundary[pos] = true;
        if (x == 0) // left simfield border
        {
          data.nodes_[pos].filter_[LEFT].filt_ = true;
          // apply a zero-reflection-filter to left border node
          adaptreflexionfactor(
              data.nodes_[pos].filter_[LEFT].numcoeffs_,
              data.nodes_[pos].filter_[LEFT].coeffsA_,
              data.nodes_[pos].filter_[LEFT].coeffsB_, 0.f, 180.f, kHorizontal);
        }
        if (y == 0) // top simfield border
        {
          data.nodes_[pos].filter_[TOP].filt_ = true;
          // apply a zero-reflection-filter to top border node
          adaptreflexionfactor(
              data.nodes_[pos].filter_[TOP].numcoeffs_,
              data.nodes_[pos].filter_[TOP].coeffsA_,
              data.nodes_[pos].filter_[TOP].coeffsB_, 0.f, 270.f, kVertical);
        }
        if (x == config.nX - 1) // right simfield border
        {
          data.nodes_[pos].filter_[RIGHT].filt_ = true;
          // apply a zero-reflection-filter to right border node
          adaptreflexionfactor(
              data.nodes_[pos].filter_[RIGHT].numcoeffs_,
              data.nodes_[pos].filter_[RIGHT].coeffsA_,
              data.nodes_[pos].filter_[RIGHT].coeffsB_, 0.f, 0.f, kHorizontal);
        }
        if (y == config.nY - 1) // bottom simfield border
        {
          data.nodes_[pos].filter_[BOTTOM].filt_ = true;
          // apply a zero-reflection-filter to bottom border node
          adaptreflexionfactor(
              data.nodes_[pos].filter_[BOTTOM].numcoeffs_,
              data.nodes_[pos].filter_[BOTTOM].coeffsA_,
              data.nodes_[pos].filter_[BOTTOM].coeffsB_, 0.f, 90.f, kVertical);
        }
      }
      // is the actual node a real-valued-reflecting node?
      processWall(x, y);
    } // x-loop
  }   // y-loop
}

void Lambda::processWall(const int x, const int y) {
  const int pos = y * config.nX + x;
  const float envi = data.envi.ptr<float>(0)[pos];
  const float angle = data.angle.ptr<float>(0)[pos];
  // TODO(lucasw) is there any difference with the > 1.0 code below?
  if ((envi >= -1.0) && (envi != 0.0) && (envi <= 1.0)) {
    data.boundary[pos] = true;
    data.nodes_[pos].filter_[LEFT].filt_ = true;
    data.nodes_[pos].filter_[TOP].filt_ = true;
    data.nodes_[pos].filter_[RIGHT].filt_ = true;
    data.nodes_[pos].filter_[BOTTOM].filt_ = true;
    // apply a left filter with correspondig reflection factor to it
    adaptreflexionfactor(data.nodes_[pos].filter_[LEFT].numcoeffs_,
                         data.nodes_[pos].filter_[LEFT].coeffsA_,
                         data.nodes_[pos].filter_[LEFT].coeffsB_, envi,
                         angle, kHorizontal);
    // apply a top filter with correspondig reflection factor to it
    adaptreflexionfactor(data.nodes_[pos].filter_[TOP].numcoeffs_,
                         data.nodes_[pos].filter_[TOP].coeffsA_,
                         data.nodes_[pos].filter_[TOP].coeffsB_, envi,
                         angle, kVertical);
    // apply a right filter with correspondig reflection factor to it
    adaptreflexionfactor(data.nodes_[pos].filter_[RIGHT].numcoeffs_,
                         data.nodes_[pos].filter_[RIGHT].coeffsA_,
                         data.nodes_[pos].filter_[RIGHT].coeffsB_, envi,
                         angle, kHorizontal);
    // apply a bottom filter with correspondig reflection factor to it
    adaptreflexionfactor(data.nodes_[pos].filter_[BOTTOM].numcoeffs_,
                         data.nodes_[pos].filter_[BOTTOM].coeffsA_,
                         data.nodes_[pos].filter_[BOTTOM].coeffsB_, envi,
                         angle, kVertical);

    for (size_t d = 0; d < 4; ++d)
      setupOldXY(d, pos);

    if (x <
        config.nX - 1) // apply a left filter to its right neighbour, if it
    {                  // isn't outside the simfield
      const int new_pos = pos + 1;
      const size_t d = LEFT;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptreflexionfactor(data.nodes_[new_pos].filter_[d].numcoeffs_,
                           data.nodes_[new_pos].filter_[d].coeffsA_,
                           data.nodes_[new_pos].filter_[d].coeffsB_, envi,
                           angle, kHorizontal);
      setupOldXY(d, new_pos);
    }
    if (y <
        config.nY - 1) // apply a top filter to its bottom neighbour, if it
    {                  // isn't outside the simfield
      const int new_pos = pos + config.nX;
      const size_t d = TOP;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptreflexionfactor(data.nodes_[new_pos].filter_[d].numcoeffs_,
                           data.nodes_[new_pos].filter_[d].coeffsA_,
                           data.nodes_[new_pos].filter_[d].coeffsB_,
                           envi, angle, kVertical);
      setupOldXY(d, new_pos);
    }
    if (x > 0) // apply a right filter to its left neighbour, if it
    {          // isn't outside the simfield
      const int new_pos = pos - 1;
      const size_t d = RIGHT;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptreflexionfactor(data.nodes_[new_pos].filter_[d].numcoeffs_,
                           data.nodes_[new_pos].filter_[d].coeffsA_,
                           data.nodes_[new_pos].filter_[d].coeffsB_, envi,
                           angle, kHorizontal);
      setupOldXY(d, new_pos);
    }
    if (y > 0) // apply a bottom filter to its top neighbour, if it
    {          // isn't outside the simfield
      const int new_pos = pos - config.nX;
      const size_t d = BOTTOM;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptreflexionfactor(data.nodes_[new_pos].filter_[d].numcoeffs_,
                           data.nodes_[new_pos].filter_[d].coeffsA_,
                           data.nodes_[new_pos].filter_[d].coeffsB_,
                           envi, angle, kVertical);
      setupOldXY(d, new_pos);
    }
  // is the actual node a filter-node?
  } else if ((envi > 1.0) && (envi <= 1000.0)) {
    data.boundary[pos] = true;
    data.nodes_[pos].filter_[LEFT].filt_ = true;
    data.nodes_[pos].filter_[TOP].filt_ = true;
    data.nodes_[pos].filter_[RIGHT].filt_ = true;
    data.nodes_[pos].filter_[BOTTOM].filt_ = true;
    // apply the left filter with the correspondig ID to it
    adaptfilter(data.nodes_[pos].filter_[LEFT].numcoeffs_,
                data.nodes_[pos].filter_[LEFT].coeffsA_,
                data.nodes_[pos].filter_[LEFT].coeffsB_, tmp_filtid, tmp_filtnumcoeffs,
                tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                (int)envi, angle, kHorizontal);
    // apply the top filter with the correspondig ID to it
    adaptfilter(data.nodes_[pos].filter_[TOP].numcoeffs_,
                data.nodes_[pos].filter_[TOP].coeffsA_,
                data.nodes_[pos].filter_[TOP].coeffsB_, tmp_filtid, tmp_filtnumcoeffs,
                tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                (int)envi, angle, kVertical);
    // apply the right filter with the correspondig ID to it
    adaptfilter(data.nodes_[pos].filter_[RIGHT].numcoeffs_,
                data.nodes_[pos].filter_[RIGHT].coeffsA_,
                data.nodes_[pos].filter_[RIGHT].coeffsB_, tmp_filtid, tmp_filtnumcoeffs,
                tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                (int)envi, angle, kHorizontal);
    // apply the bottom filter with the correspondig ID to it
    adaptfilter(data.nodes_[pos].filter_[BOTTOM].numcoeffs_,
                data.nodes_[pos].filter_[BOTTOM].coeffsA_,
                data.nodes_[pos].filter_[BOTTOM].coeffsB_,
                tmp_filtid, tmp_filtnumcoeffs, tmp_filtcoeffsA,
                tmp_filtcoeffsB, tmp_numfilters, (int)envi,
                angle, kVertical);

    for (size_t d = 0; d < 4; ++d)
      setupOldXY(d, pos);

    if (x <
        config.nX - 1) // apply a left filter to its right neighbour, if it
    {                  // isn't outside the simfield
      const int new_pos = pos + 1;
      const size_t d = LEFT;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptfilter(
          data.nodes_[new_pos].filter_[d].numcoeffs_,
          data.nodes_[new_pos].filter_[d].coeffsA_,
          data.nodes_[new_pos].filter_[d].coeffsB_, tmp_filtid, tmp_filtnumcoeffs,
          tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
          (int)envi, angle, kHorizontal);
      setupOldXY(d, new_pos);
    }
    if (y <
        config.nY - 1) // apply a top filter to its bottom neighbour, if it
    {                  // isn't outside the simfield
      const int new_pos = pos + config.nX;
      const size_t d = TOP;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptfilter(data.nodes_[new_pos].filter_[d].numcoeffs_,
                  data.nodes_[new_pos].filter_[d].coeffsA_,
                  data.nodes_[new_pos].filter_[d].coeffsB_, tmp_filtid,
                  tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                  tmp_numfilters, (int)envi, angle,
                  kVertical);
      setupOldXY(d, new_pos);
    }
    if (x > 0) // apply a right filter to its left neighbour, if it
    {          // isn't outside the simfield
      const int new_pos = pos - 1;
      const size_t d = RIGHT;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[d].filt_ = true;
      adaptfilter(data.nodes_[new_pos].filter_[d].numcoeffs_,
                  data.nodes_[new_pos].filter_[d].coeffsA_,
                  data.nodes_[new_pos].filter_[d].coeffsB_, tmp_filtid,
                  tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                  tmp_numfilters, (int)envi, angle,
                  kHorizontal);
      setupOldXY(d, new_pos);
    }
    if (y > 0) // apply a bottom filter to its top neighbour, if it
    {          // isn't outside the simfield
      const int new_pos = pos - config.nX;
      const size_t d = BOTTOM;
      data.boundary[new_pos] = true;
      data.nodes_[new_pos].filter_[BOTTOM].filt_ = true;
      adaptfilter(data.nodes_[new_pos].filter_[BOTTOM].numcoeffs_,
                  data.nodes_[new_pos].filter_[BOTTOM].coeffsA_,
                  data.nodes_[new_pos].filter_[BOTTOM].coeffsB_, tmp_filtid,
                  tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                  tmp_numfilters, (int)envi, angle,
                  kVertical);
      setupOldXY(d, new_pos);
    }
  } else if (envi < -1.0) // is the actual node a receiver-node?
  {
    #if 0
    if (actrec < config.nRec) {
      data.recIdx[actrec] =
          pos;  // yes, add the receiver's position into the
      actrec++; // recIdx-Array
    }
    #endif
  }
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

  initEnvironment();

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
        const int pos_left = y * config.nX + x - 1;
        const int pos_top = (y - 1) * config.nX + x;
        const int pos_right = y * config.nX + x + 1;
        const int pos_bottom = (y + 1) * config.nX + x;
        if (x > 0)
          if (isvelosource[pos_left]) // is left neighbour a velo src?
            data.nodes_[pos_left].filter_[LEFT].filt_ =
                false; // yes, disable left filter
        if (y > 0)
          if (isvelosource[pos_top]) // is top neighbour a velo src?
            data.nodes_[pos_top].filter_[TOP].filt_ = false; // yes, disable top filter
        if (x < config.nX - 1)
          if (isvelosource[pos_right]) // is right neighbour a velo src?
            data.nodes_[pos_right].filter_[RIGHT].filt_ =
                false; // yes, disable right filter
        if (y < config.nY - 1)
          if (isvelosource[pos_bottom]) // is bottom neighbour a velo src?
            data.nodes_[pos_bottom].filter_[BOTTOM].filt_ =
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
    if ((data.nodes_[pos].filter_[LEFT].filt_) &&
        (data.nodes_[pos].filter_[TOP].filt_) &&
        (data.nodes_[pos].filter_[RIGHT].filt_) &&
        (data.nodes_[pos].filter_[BOTTOM].filt_)) {
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

  initEnvironment();
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
  for (int pos = 0; pos < config.nNodes; pos++) {
    for (size_t d = 0; d < 4; ++d) {
      setupOldXY(d, pos);
    }
  }

  return NONE;
}

void Lambda::setupOldXY(const size_t d, const int pos)
{
  if (data.nodes_[pos].filter_[d].numcoeffs_ >= 1) {
    // reserve+initialize recursive and non-recursive memory for left filters
    int memorycnt = data.nodes_[pos].filter_[d].numcoeffs_ - 1;
    if (memorycnt == 0) // to ensure that even 0th order filters have
      memorycnt = 1; // memory; this spares an if-condition in the algorithm
    // data.nodes_[pos].filter_[d].oldx_.resize(memorycnt, 0.0f);
    // data.nodes_[pos].filter_[d].oldy_.resize(memorycnt, 0.0f);
    data.nodes_[pos].filter_[d].oldx_.reset(new float[memorycnt]);
    data.nodes_[pos].filter_[d].oldy_.reset(new float[memorycnt]);
  }
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

void DirectionalFilter::print()
{
  std::cout << " has filter " << filt_ << ", num coeffs "
      << numcoeffs_ << "\n";
  for (size_t i = 0; i < numcoeffs_; ++i)
  {
    std::cout << "        " << coeffsA_[i] << " "
       << coeffsB_[i] << "\n";
    // std::cout << "        " << coeffsA_ << " " << coeffsA_[i] << " "
    //     << coeffsB_ << " " << coeffsB_[i] << "\n";
    // TODO(lucasw) print out oldx and oldy also?
  }
}

float Lambda::getPressure(const size_t x, const size_t y)
{
  const int idx = ((config.n + 1) % 3); // present index
  if (x >= data.pressure_[idx].cols)
    return 0.0;
  if (y >= data.pressure_[idx].rows)
    return 0.0;

  const size_t pos = y * config.nX + x;

  // print out filter information
  std::cout << "x: " << x <<  ", y: " << y << "\n";
  for (size_t d = 0; d < 4; ++d)
  {
    std::cout << "    " << d <<  " ";
    data.nodes_[pos].filter_[d].print();
  }
  return data.pressure_[idx].at<float>(y, x);
}

// this has to be done before initSimulation
void Lambda::setWall(const size_t x, const size_t y, const float value)
{
  if (x >= config.nX)
    return;
  if (y >= config.nY)
    return;
  data.envi.at<float>(y, x) = value;
  processWall(x, y);
  for (size_t d = 0; d < 4; ++d)
    setupOldXY(d, y * config.nX + x);
}

// alpha is angle
void Lambda::setVel(const int& srcxy, const float& magnitude, const float& alpha) {
  if ((alpha >= 0.f) && (alpha < 90.f)) {
    // alpha between 0 and 90 degrees? -> left and top incidence
    const float rad = alpha * rad_per_deg;
    data.nodes_[srcxy].filter_[LEFT].velo_ = cos(rad) * magnitude;
    data.nodes_[srcxy].filter_[TOP].velo_ = sin(rad) * magnitude;
  } else if ((alpha >= 90.f) && (alpha < 180.f)) {
    // alpha between 90 and 180 degrees? -> top and right incidence
    const float rad = (alpha - 90.f) * rad_per_deg;
    data.nodes_[srcxy].filter_[TOP].velo_ = cos(rad) * magnitude;
    data.nodes_[srcxy].filter_[RIGHT].velo_ = sin(rad) * magnitude;
  } else if ((alpha >= 180.f) && (alpha < 270.f)) {
    // alpha between 180 and 270 degrees? -> right and bottom incidence
    const float rad = (alpha - 180.f) * rad_per_deg;
    data.nodes_[srcxy].filter_[RIGHT].velo_ = cos(rad) * magnitude;
    data.nodes_[srcxy].filter_[BOTTOM].velo_ = sin(rad) * magnitude;
  } else if ((alpha >= 270.f) && (alpha < 360.f)) {
    // alpha between 270 and 360 degrees? -> bottom and left incidence
    const float rad = (alpha - 270.f) * rad_per_deg;
    data.nodes_[srcxy].filter_[BOTTOM].velo_ = cos(rad) * magnitude;
    data.nodes_[srcxy].filter_[LEFT].velo_ = sin(rad) * magnitude;
  }
}

// TODO(lucasw probably going to delete this once confident
// all the nuance of these sources can be handled externally
void Lambda::processSources(float*& presPres) {
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
}

///////////////////////////////////////////////////////////////////////////////
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
    //    << " " << index.presFutu[2] << "\n";
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

    processSources(presPres);

    ////////////////////////////////////////////////////////////////////////
    // it's expensive to do a lot of map lookups in the big loop before,
    // they don't get optimized out, so set up these temp arrays instead.
    std::vector<Inci*> incis;
    for (const std::string& dir : dirs_) {
      incis.push_back(&index.inci_[dir]);
    }
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
          if (data.nodes_[pos].filter_[d].filt_) {
            // for filter processing it would pay to have a single
            // node data structure that holds most of the values below
            // adjacent in memory rather that in parallel arrays.
            // No neighboring nodes are used at all.
            bool debug = false;
            // calculate filter input
            const float scat_futu = presPres[pos] - incis[d]->pres_[pos];
            // calculate the digital filter
            const float cb0 = data.nodes_[pos].filter_[d].coeffsB_[0];
            // filter output - why isn't ca0 used?
            float yn = scat_futu * cb0;
            debug = debug && std::abs(yn) > 0.00001;
            if (debug) {
              const int x = pos % config.nX;
              const int y = (pos - x) / config.nX;
              std::cout << pos << ", " << x << " " << y << " : dir " << d << "\n";
              std::cout << "   scat futu " << scat_futu << " = " << presPres[pos]
                  << " - " << incis[d]->pres_[pos] << "\n";
              std::cout << "   "
                  << "yn " << yn << ", cb0 " << cb0 << "\n";
            }
            // standard walls never have this many coefficients, only cb0 matters
            for (int n = 1; n < data.nodes_[pos].filter_[d].numcoeffs_; n++) {
              const float oldx = data.nodes_[pos].filter_[d].oldx_[n - 1];
              const float cb = data.nodes_[pos].filter_[d].coeffsB_[n];
              const float oldy = data.nodes_[pos].filter_[d].oldy_[n - 1];
              const float ca = data.nodes_[pos].filter_[d].coeffsA_[n];
              if (debug) std::cout << n << ", oldx " << oldx << " cb " << cb
                  << ", oldy " << oldy << " " << ca << "\n";
              yn += oldx * cb - oldy * ca;
            }
            // add magnitude of a possible velocity source
            const float velo = data.nodes_[pos].filter_[d].velo_;
            if (debug) std::cout << "velo " << velo << "\n";
            yn += velo;
            // TODO(lucasw) replace with an array that has a start index
            // that moves in a ring.
            // should be faster than all this copying.
            // But normal filter length 1 walls don't even use this.
            // rotate the filter memories
            for (int n = data.nodes_[pos].filter_[d].numcoeffs_ - 2; n > 0; n--) {
              data.nodes_[pos].filter_[d].oldx_[n] = data.nodes_[pos].filter_[d].oldx_[n - 1];
              data.nodes_[pos].filter_[d].oldy_[n] = data.nodes_[pos].filter_[d].oldy_[n - 1];
            }
            data.nodes_[pos].filter_[d].oldx_[0] = scat_futu;
            data.nodes_[pos].filter_[d].oldy_[0] = yn;
            // and write the filter output into the pressure matrix
            incis[d]->futu_[pos] = yn;
            index_presFutu[pos] += incis[d]->futu_[pos];
          } else {
            // no filter in this direction
            const float scat_pres = index.presPast[pos] - incis[d]->past_[pos];

            float incis_futu = 0.0;
            if (d == LEFT)
              incis_futu = presPres[pos - 1] - scat_pres;
            else if (d == TOP)
              incis_futu = presPres[pos - config_nX] - scat_pres;
            else if (d == RIGHT)
              incis_futu = presPres[pos + 1] - scat_pres;
            else if (d == BOTTOM)
              incis_futu = presPres[pos + config_nX] - scat_pres;

            incis[d]->futu_[pos] = incis_futu;
            index_presFutu[pos] += incis_futu;
          }
        } // dir loop
        index_presFutu[pos] *= 0.5f;
      } else {
        // TODO(lucasw) this is mostly likely getting executed the most,
        // so it pays to have all the pressures future past present and
        // immediate neighbors close together in memory- that argues for preserving
        // the cv::Mats, though later should break them up into smaller sized mats
        // that are processed individually which will lend well to multi-threading.
        // (each thread only needs sub-grid border updates from neighbors)
        // TODO(lucasw) if only keeping three pressure matrices around
        // (and not arbitrarily long ring buffers it may pay to use
        // three channels of a cv::Mat - if each color channel is adjacent in
        // memory the accessing future and past will be very fast, though getting to
        // neighbors becomes a longer distance.
        // TODO(lucasw) do an experiment with 32FC3 to make sure the channels are near
        // in memory.

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
