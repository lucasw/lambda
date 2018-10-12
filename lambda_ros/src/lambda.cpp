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
#include <omp.h>

///////////////////////////////////////////////////////////////////////////////
//   Constructor for the program's main class, initializes program and builds up
Lambda::Lambda(const size_t width, const size_t height) :
    width_(width), height_(height), num_(width * height) {
  envi_ = cv::Mat(cv::Size(width_, height_), CV_32FC1, cv::Scalar::all(0));
  // 400.0f means no preemphasis is done on the nodes)
  angle_ = cv::Mat(cv::Size(width_, height_), CV_32FC1, cv::Scalar::all(400.0f));

  deadnode.reset(new bool[num_]); // reserve mem for deadnode matrix
  boundary.reset(new bool[num_]);
  for (size_t pos = 0; pos < num_; pos++) {
    deadnode[pos] = false;
    boundary[pos] = false;
  }

  nodes_.reset(new Node[num_]);

  init();
  // TODO(lucasw) let something else determine whether or not to randomize like
  // this
  // srand(time(nullptr));
}

void simConfig::reset() {
  n = 0;
}

void DirectionalFilter::reset() {
  // oldx_.reset(nullptr);
  // oldy_.reset(nullptr);
  for (size_t i = 0; i < oldx_.size(); ++i)
    oldx_[i] = 0.0;
  for (size_t i = 0; i < oldy_.size(); ++i)
    oldy_[i] = 0.0;
  velo_ = 0.0;

  for (size_t t = 0; t < inci_.size(); ++t) {
    inci_[t] = 0.0;
  }
}

void Node::reset() {
  for (size_t d = 0; d < filter_.size(); ++d) {
    filter_[d].reset();
  }
}

//   Resets variables and arrays used directly for simulation purposes.
// Should be able to restart the simulation from time zero after this
void Lambda::reset() {
  // Delete bottom filter non-recursive memory
  for (size_t n = 0; n < num_; n++) {
    nodes_[n].reset();
  }

  config.reset();
}

#if 0
  if (what == "cTube") {
    // If cTube is to be set, check if new tube speed is greater than 0.
    if (value <= 0)
      return TUBE_SPEED_BAD;
    config.cTube = value;
    set("fSample", config.cTube / config.lTube);
  }
  if (what == "lTube") {
    // If lTube is to be set, check if new tube length is greater than 0.
    if (value <= 0)
      return TUBE_LENGTH_BAD;
    config.lTube = value;
    set("fSample", config.cTube / config.lTube);
  }
  if (what == "rho") {
    // If rho is to be set, check if new rho is greater than 0.
    if (value <= 0)
      return RHO_BAD;
    config.rho = value;
  }
  if (what == "nSamples") {
    config.nSamples = (int)value;
  }
  if (what == "tSample") {
    config.tSample = value;
  }
  if (what == "fSample") {
    config.fSample = value;
    // Adjust sample duration
    set("tSample", 1 / config.fSample);
  }
#endif

#if 0
  int srcxy;
  srcxy = (int)srcs[idx * 6 + 0] + (int)srcs[idx * 6 + 1];
  float alpha;
  // get the desired incidence angle out of angle matrix
  alpha = angle_.ptr<float>(0)[srcxy];
  // angle out of bounds?
  if ((alpha < 0.f) || (alpha >= 360.f))
    alpha = 0.f;
  if ((srcData->type >= 6) && (srcData->type <= 10)) {
    if ((alpha >= 0.f) && (alpha < 90.f)) {
      // set up a left- and top- filter if angle of incidence is
      // between 0 and 90 degrees
      boundary[srcxy] = true;
      nodes_[srcxy].filter_[LEFT].filt_ = true;
      nodes_[srcxy].filter_[TOP].filt_ = true;
      adaptreflexionfactor(
          nodes_[srcxy].filter_[LEFT].numcoeffs_,
          nodes_[srcxy].filter_[LEFT].coeffsA_,
          nodes_[srcxy].filter_[LEFT].coeffsB_, 1.f, 180.f, kHorizontal);
      adaptreflexionfactor(nodes_[srcxy].filter_[TOP].numcoeffs_,
                           nodes_[srcxy].filter_[TOP].coeffsA_,
                           nodes_[srcxy].filter_[TOP].coeffsB_, 1.f, 270.f, kVertical);
    } else if ((alpha >= 90.f) && (alpha < 180.f)) {
      // set up a top- and right- filter if angle of incidence is
      // between 90 and 180 degrees
      boundary[srcxy] = true;
      nodes_[srcxy].filter_[TOP].filt_ = true;
      nodes_[srcxy].filter_[RIGHT].filt_ = true;
      adaptreflexionfactor(nodes_[srcxy].filter_[TOP].numcoeffs_,
                           nodes_[srcxy].filter_[TOP].coeffsA_,
                           nodes_[srcxy].filter_[TOP].coeffsB_, 1.f, 270.f, kVertical);
      adaptreflexionfactor(
          nodes_[srcxy].filter_[RIGHT].numcoeffs_,
          nodes_[srcxy].filter_[RIGHT].coeffsA_,
          nodes_[srcxy].filter_[RIGHT].coeffsB_, 1.f, 0.f, kHorizontal);
    } else if ((alpha >= 180.f) && (alpha < 270.f)) {
      // set up a right- and bottom- filter if angle of incidence is
      // between 180 and 270 degrees
      boundary[srcxy] = true;
      nodes_[srcxy].filter_[RIGHT].filt_ = true;
      nodes_[srcxy].filter_[BOTTOM].filt_ = true;
      adaptreflexionfactor(
          nodes_[srcxy].filter_[RIGHT].numcoeffs_,
          nodes_[srcxy].filter_[RIGHT].coeffsA_,
          nodes_[srcxy].filter_[RIGHT].coeffsB_, 1.f, 0.f, kHorizontal);
      adaptreflexionfactor(
          nodes_[srcxy].filter_[BOTTOM].numcoeffs_,
          nodes_[srcxy].filter_[BOTTOM].coeffsA_,
          nodes_[srcxy].filter_[BOTTOM].coeffsB_, 1.f, 90.f, kVertical);
    } else if ((alpha >= 270.f) && (alpha < 360.f)) {
      // set up a bottom- and left- filter if angle of incidence is
      // between 270 and 360 degrees
      boundary[srcxy] = true;
      nodes_[srcxy].filter_[BOTTOM].filt_ = true;
      nodes_[srcxy].filter_[LEFT].filt_ = true;
      adaptreflexionfactor(
          nodes_[srcxy].filter_[BOTTOM].numcoeffs_,
          nodes_[srcxy].filter_[BOTTOM].coeffsA_,
          nodes_[srcxy].filter_[BOTTOM].coeffsB_, 1.f, 90.f, kVertical);
      adaptreflexionfactor(
          nodes_[srcxy].filter_[LEFT].numcoeffs_,
          nodes_[srcxy].filter_[LEFT].coeffsA_,
          nodes_[srcxy].filter_[LEFT].coeffsB_, 1.f, 180.f, kHorizontal);
    }
  }
#endif

// TODO(lucasw) merge this with other init methods
bool Lambda::init() {
  // Check one more time
  if (num_ < 1)
    return false;

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

  // set up indices needed for the simulation
  for (int x = 0; x < 3; x++) {
    pressure_[x] = cv::Mat(cv::Size(width_, height_), CV_32FC1, cv::Scalar::all(0));
  }

  // work through all nodes in the environment
  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      processWall(x, y);

      #if USE_WRAP
      const int pos = y * width_ + x;
      if (x > 0)
        nodes_[pos].neighbors_[LEFT] = y * width_ + (x - 1);
      else if ((x == 0) && wrap_)
        nodes_[pos].neighbors_[LEFT] = y * width_ + width_ - 1;  // TODO %?

      if ((y > 0))
        nodes_[pos].neighbors_[TOP] = ((y - 1) % height_) * width_ + x;
      else if ((y == 0) && wrap_)
        nodes_[pos].neighbors_[TOP] = (height_ - 1) * width_ + x;  // TODO %?

      if (x < height_ - 1)
        nodes_[pos].neighbors_[RIGHT] = y * width_ + x + 1;
      else if ((x == height_ - 1) && (wrap_))
        nodes_[pos].neighbors_[RIGHT] = y * width_;  // TODO %?

      if (y < height_ - 1)
        nodes_[pos].neighbors_[BOTTOM] = (y + 1) * width_ + x;
      else if ((y == height_ - 1) && (wrap_))
        nodes_[pos].neighbors_[BOTTOM] = x;  // TODO %?
      #endif
    } // x-loop
  }   // y-loop

  return true;
}

void Lambda::getFilterImage(cv::Mat& image, const int d, const std::string type, const int i)
{
  image = cv::Mat(cv::Size(width_, height_), CV_32FC1, cv::Scalar::all(0));

  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      const int pos = y * width_ + x;
      // if (!nodes_[pos].filter_[d].filt_)
      //   continue;
      if (i >= nodes_[pos].filter_[d].coeffsA_.size())
        continue;
      float value = 0.0;
      if (type == "f")
        value = nodes_[pos].filter_[d].filt_ ? 1.0 : 0.0;
      else if (type == "a")
        value = nodes_[pos].filter_[d].coeffsA_[i];
      else if (type == "b")
        value = nodes_[pos].filter_[d].coeffsB_[i];
      else if (type == "x")
        value = nodes_[pos].filter_[d].oldx_[i];
      else if (type == "y")
        value = nodes_[pos].filter_[d].oldy_[i];
      else if ((type == "i") && (i < nodes_[pos].filter_[d].inci_.size()))
        value = nodes_[pos].filter_[d].inci_[i];
      else if (type == "c")
        value = nodes_[pos].filter_[d].numcoeffs_;
      else
        continue;
      // std::cout << y << " " << x << " " << value << std::endl;
      image.at<float>(y, x) = value;
    }
  }
}

void Lambda::addFilter(const int x, const int y, const int d,
    float envi, float angle) {
  if ((x < 0) || (x >= width_))
    return;
  if ((y < 0) || (y >= height_))
    return;
  const int pos = y * width_ + x;
  addFilter(pos, d, envi, angle);
}

void Lambda::addFilter(const int pos, const int d,
    float envi, float angle) {
  if ((pos < 0) || (pos >= num_))
    return;

  const int x = pos % width_;
  const int y = (pos - x) / height_;
  // is it on simfield border?  In that case need to override
  // thye angle and envi even if this is a wall
  bool border = true;
  #if USE_WRAP
  if (wrap_) {
    border = false;
  } else
  #endif
  if ((d == LEFT) && (x == 0)) {
    angle = 180.0f;
  } else if ((d == TOP) && (y == 0)) {
    angle = 270.0f;
  } else if ((d == RIGHT) && (x == width_ - 1)) {
    angle = 0.0f;
  } else if ((d == BOTTOM) && (y == height_ - 1)) {
    angle = 90.0f;
  } else {
    border = false;
  }
  if (border)
    envi = 0.0;

  if ((envi >= -1.0) && (envi <= 1.0)) {
    if ((envi == 0.0) && (!border)) {
      // TODO(lucasw) is this similar to envi=0.0 to adaptfilter, but more efficient?
      nodes_[pos].filter_[d].numcoeffs_ = 0;
      nodes_[pos].filter_[d].filt_ = false;
    } else {
      boundary[pos] = true;
      nodes_[pos].filter_[d].filt_ = true;
      adaptreflexionfactor(nodes_[pos].filter_[d].numcoeffs_,
                           nodes_[pos].filter_[d].coeffsA_,
                           nodes_[pos].filter_[d].coeffsB_,
                           envi, angle, dirToPreemphasis(d));
    }
  } else if ((envi > 1.0) && (envi <= 1000.0)) {
    boundary[pos] = true;
    nodes_[pos].filter_[d].filt_ = true;
    adaptfilter(
          nodes_[pos].filter_[d].numcoeffs_,
          nodes_[pos].filter_[d].coeffsA_,
          nodes_[pos].filter_[d].coeffsB_, tmp_filtid, tmp_filtnumcoeffs,
          tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
          (int)envi, angle, dirToPreemphasis(d));
  } else {
  }
}

// TODO(lucasw) if walls only reflected internally then there wouldn't
// need to be neighbor walls
void Lambda::addNeighborFilter(const int x, const int y,
    const float envi, const float angle) {
  // TODO(lucasw) if envi == 0.0 and the neighbor was a deadnode
  // this may result in 'leaking' or otherwise be wrong.
  // bounds are checked within addFilter
  const int pos = y * width_ + x;
#if USE_WRAP
  const std::array<int, 4>& neighbors = nodes_[pos].neighbors_;
#else
  const std::array<int, 4> neighbors = {pos - 1, pos - width_, pos + 1, pos + width_};
#endif
  addFilter(neighbors[RIGHT], LEFT, envi, angle);
  addFilter(neighbors[BOTTOM], TOP, envi, angle);
  addFilter(neighbors[LEFT], RIGHT, envi, angle);
  addFilter(neighbors[TOP], BOTTOM, envi, angle);
}

void Lambda::processWall(const int x, const int y) {
  const int pos = y * width_ + x;
  const float envi = envi_.ptr<float>(0)[pos];
  const float angle = angle_.ptr<float>(0)[pos];

  // TODO(lucasw) is there any difference with the > 1.0 code below?
  if ((envi >= -1.0) && (envi <= 1000.0)) {
    if (envi != 0.0) {
      deadnode[pos] = true;
    } else {
      // TODO(lucasw) can't do this if at the true boundary (x == 0, y == 0 etc)
      // but in that case addFilter will override this boundary
      boundary[pos] = false;
      deadnode[pos] = false;
    }
    for (size_t d = 0; d < 4; ++d) {
      addFilter(x, y, d, envi, angle);
      // std::cout << y << " " << x << " ";
      // nodes_[pos].filter_[d].print();
    }
    addNeighborFilter(x, y, envi, angle);
  } else if (envi < -1.0) {
    // envi < -1.0 used to be used for recorders
    // TODO(lucasw) does < -1.0 passed to adaptfilter do anything interesting?
  }
}

#if 0
  //  ----- DEACTIVATE BOUNDARIES BETWEEN ADJACENT VELOCITY SOURCES
  // this is important for the stability of the simulation. all boundaries
  // between adjacent velocity source nodes must be removed again.
  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      if (isvelosource[y * width_ +
                       x]) // is actual node a velocity source node?
      {
        const int pos_left = y * width_ + x - 1;
        const int pos_top = (y - 1) * width_ + x;
        const int pos_right = y * width_ + x + 1;
        const int pos_bottom = (y + 1) * width_ + x;
        if (x > 0)
          if (isvelosource[pos_left]) // is left neighbour a velo src?
            nodes_[pos_left].filter_[LEFT].filt_ =
                false; // yes, disable left filter
        if (y > 0)
          if (isvelosource[pos_top]) // is top neighbour a velo src?
            nodes_[pos_top].filter_[TOP].filt_ = false; // yes, disable top filter
        if (x < width_ - 1)
          if (isvelosource[pos_right]) // is right neighbour a velo src?
            nodes_[pos_right].filter_[RIGHT].filt_ =
                false; // yes, disable right filter
        if (y < height_ - 1)
          if (isvelosource[pos_bottom]) // is bottom neighbour a velo src?
            nodes_[pos_bottom].filter_[BOTTOM].filt_ =
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
  for (int pos = 0; pos < num_; pos++) {
    // four walls around the node?
    if ((nodes_[pos].filter_[LEFT].filt_) &&
        (nodes_[pos].filter_[TOP].filt_) &&
        (nodes_[pos].filter_[RIGHT].filt_) &&
        (nodes_[pos].filter_[BOTTOM].filt_)) {
      deadnode[pos] = true; // ---> yes, it's a deadnode
    }
  }
#endif

void Lambda::setPressure(const size_t x, const size_t y, const float value)
{
  if (x >= width_)
    return;
  if (y >= height_)
    return;
  int idx = ((config.n + 1) % 3); // current index
  pressure_[idx].at<float>(y, x) = value;
}

void Lambda::addPressure(const size_t x, const size_t y, const float value)
{
  if (x >= width_)
    return;
  if (y >= height_)
    return;
  int idx = ((config.n + 1) % 3); // current index
  pressure_[idx].at<float>(y, x) += value;
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
  if (x >= pressure_[idx].cols)
    return 0.0;
  if (y >= pressure_[idx].rows)
    return 0.0;

  const size_t pos = y * width_ + x;
#if 0
  // print out filter information
  std::cout << "x: " << x <<  ", y: " << y << "\n";
  for (size_t d = 0; d < nodes_[pos].filter_.size(); ++d)
  {
    std::cout << "    " << d <<  " ";
    nodes_[pos].filter_[d].print();
  }
#endif
  return pressure_[idx].at<float>(y, x);
}

bool Lambda::setWall(const size_t x, const size_t y, const float value)
{
  if (x >= envi_.cols) {
    return false;
  }
  if (y >= envi_.rows) {
    return false;
  }
  envi_.at<float>(y, x) = value;
  processWall(x, y);
  return true;
}

// alpha is angle
void Lambda::setVel(const int& srcxy, const float& magnitude, const float& alpha) {
  if ((alpha >= 0.f) && (alpha < 90.f)) {
    // alpha between 0 and 90 degrees? -> left and top incidence
    const float rad = alpha * rad_per_deg;
    nodes_[srcxy].filter_[LEFT].velo_ = cos(rad) * magnitude;
    nodes_[srcxy].filter_[TOP].velo_ = sin(rad) * magnitude;
  } else if ((alpha >= 90.f) && (alpha < 180.f)) {
    // alpha between 90 and 180 degrees? -> top and right incidence
    const float rad = (alpha - 90.f) * rad_per_deg;
    nodes_[srcxy].filter_[TOP].velo_ = cos(rad) * magnitude;
    nodes_[srcxy].filter_[RIGHT].velo_ = sin(rad) * magnitude;
  } else if ((alpha >= 180.f) && (alpha < 270.f)) {
    // alpha between 180 and 270 degrees? -> right and bottom incidence
    const float rad = (alpha - 180.f) * rad_per_deg;
    nodes_[srcxy].filter_[RIGHT].velo_ = cos(rad) * magnitude;
    nodes_[srcxy].filter_[BOTTOM].velo_ = sin(rad) * magnitude;
  } else if ((alpha >= 270.f) && (alpha < 360.f)) {
    // alpha between 270 and 360 degrees? -> bottom and left incidence
    const float rad = (alpha - 270.f) * rad_per_deg;
    nodes_[srcxy].filter_[BOTTOM].velo_ = cos(rad) * magnitude;
    nodes_[srcxy].filter_[LEFT].velo_ = sin(rad) * magnitude;
  }
}

#if 0
// TODO(lucasw probably going to delete this once confident
// all the nuance of these sources can be handled externally
void Lambda::processSources(float*& presPres) {
    // TODO(lucasw) is there anything special about these sources vs.
    // feeding in the source externally?
    // Work through all the sources. First init all the sources
    for (int src = 0; src < config.nSrc; src++) {
      int srcpos = src * 6;
      int srcxy = (int)srcs[srcpos] +
                  (int)srcs[srcpos + 1]; // get actual source pos.
      presPres[srcxy] = 0;
    }
    // then calculate press for each.
    // this allows pressure sources to share the same node
    for (int src = 0; src < config.nSrc; src++) {
      register int srcpos = src * 6;
      int srcxy = (int)srcs[srcpos] +
                  (int)srcs[srcpos + 1]; // get actual source pos.
      int type = (int)srcs[srcpos + 2];  // get actual source type
      float amp = srcs[srcpos + 3];      // get actual source amplitude
      float freq = srcs[srcpos + 4];     // get actual source frequency
      float phi = srcs[srcpos + 5];      // get actual source phase offset
      float alpha = angle_.ptr<float>(0)[srcxy]; // get actual source angle of incidence
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
        b0 = mem[mempos];
        b1 = mem[mempos + 1];
        b2 = mem[mempos + 2];
        b0 = 0.99765f * b0 + white * 0.0990460f;
        b1 = 0.96300f * b1 + white * 0.2965164f;
        b2 = 0.57000f * b2 + white * 1.0526913f;
        presPres[srcxy] += b0 + b1 + b2 + white * 0.1848;
        mem[mempos] = b0;
        mem[mempos + 1] = b1;
        mem[mempos + 2] = b2;
        break;
      case 30: // sample
        mempos = src * MEMSRC;
        sample = samples[(int)freq];  // freq contains the sample id
        samplepos = (int)mem[mempos]; // read position
        presPres[srcxy] = sample->data[samplepos] *
                          amp; // read individual sample, scale it to amp
        mem[mempos] =
            (float)((samplepos + 1) %
                    sample->nsamples); // advance read position, loop if at end
      }
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////
//   Processes the next simulation iteration.
void Lambda::processSim() {
  {
    // periodic cycling of time indices
    const size_t idxPast = ((config.n + 0) % 3); // past index
    const size_t idxPres = ((config.n + 1) % 3); // present index
    const size_t idxFutu = ((config.n + 2) % 3); // future index

    float* presPast = pressure_[idxPast].ptr<float>(0);
    float* presPres = pressure_[idxPres].ptr<float>(0);
    float* presFutu = pressure_[idxFutu].ptr<float>(0);

    // Work through all the nodes in the environment
    // TODO(lucasw) static will be imbalanced when the distribution
    // of boundaries is imbalanced
    // # pragma omp parallel for schedule(dynamic, 400)
    for (int pos = 0; pos < num_; pos++) {
      presFutu[pos] = 0.f;
      if (deadnode[pos]) // deadnode? --> no calculation needed!
        continue;

      // neighbor indices that can be anything really slow down the simulation-
      // by about 4x- is it because the compiler can't predict where in memory
      // the indices might go?
      #if USE_WRAP
      const std::array<int, 4>& neighbors = nodes_[pos].neighbors_;
      #else
      const std::array<int, 4> neighbors = {pos - 1, pos - width_, pos + 1, pos + width_};
      #endif

      // boundary? --> no standard propagation!
      // boundary[pos] ought to be true if any of the filt_ are true.
      // it would be easy to check here and update boundary if they aren't,
      // going the other way to set boundary true is the responsibility
      // of methods that alter filters.
      if (boundary[pos]) {
      // if (nodes_[pos].boundary_) {
        for (size_t d = 0; d < 4; ++d) {
          DirectionalFilter& filter = nodes_[pos].filter_[d];
          // TODO(lucasw) clean this up by make this a method of DirData?
          //  filter
          if (filter.filt_) {
            // for filter processing it would pay to have a single
            // node data structure that holds most of the values below
            // adjacent in memory rather that in parallel arrays.
            // No neighboring nodes are used at all.
            bool debug = false;
            // calculate filter input
            const float scat_futu = presPres[pos] - filter.inci_[idxPres];
            // calculate the digital filter
            const float cb0 = filter.coeffsB_[0];
            // filter output - why isn't ca0 used?
            float yn = scat_futu * cb0;
            #if 0
            debug = debug && std::abs(yn) > 0.00001;
            if (debug) {
              const int x = pos % width_;
              const int y = (pos - x) / width_;
              std::cout << pos << ", " << x << " " << y << " : dir " << d << "\n";
              std::cout << "   scat futu " << scat_futu << " = " << presPres[pos]
                  << " - " << nodes_[pos].inci_[d][idxPres] << "\n";
              std::cout << "   "
                  << "yn " << yn << ", cb0 " << cb0 << "\n";
            }
            #endif
            // standard walls never have this many coefficients, only cb0 matters
            for (int n = 1; n < filter.numcoeffs_; n++) {
              const float oldx = filter.oldx_[n - 1];
              const float cb = filter.coeffsB_[n];
              const float oldy = filter.oldy_[n - 1];
              const float ca = filter.coeffsA_[n];
              if (debug) std::cout << n << ", oldx " << oldx << " cb " << cb
                  << ", oldy " << oldy << " " << ca << "\n";
              yn += oldx * cb - oldy * ca;
            }
            // add magnitude of a possible velocity source
            {
              const float velo = filter.velo_;
              #if 0
              if (debug) std::cout << "velo " << velo << "\n";
              #endif
              yn += velo;
            }
            // TODO(lucasw) replace with an array that has a start index
            // that moves in a ring.
            // should be faster than all this copying.
            // But normal filter length 1 walls don't even use this.
            // rotate the filter memories
            for (int n = filter.numcoeffs_ - 2; n > 0; n--) {
              filter.oldx_[n] = filter.oldx_[n - 1];
              filter.oldy_[n] = filter.oldy_[n - 1];
            }
            filter.oldx_[0] = scat_futu;
            filter.oldy_[0] = yn;
            // and write the filter output into the pressure matrix
            filter.inci_[idxFutu] = yn;
            presFutu[pos] += yn;
          } else {
            // TODO(lucasw) a proper setting of boundaries on the edges
            // of the grid means that neighbors[d] will never be out of bounds.
            // There is no filter in this direction
            const float scat_pres = presPast[pos] - filter.inci_[idxPast];
            const float incis_futu = presPres[neighbors[d]] - scat_pres;
            filter.inci_[idxFutu] = incis_futu;
            presFutu[pos] += incis_futu;
          }
        } // dir loop
        presFutu[pos] *= 0.5f;
      } else {  // not a boundary
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
        presFutu[pos] =
            (presPres[neighbors[LEFT]] +
             presPres[neighbors[TOP]] +
             presPres[neighbors[RIGHT]] +
             presPres[neighbors[BOTTOM]]) *
                0.5f - presPast[pos];
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
void Lambda::adaptreflexionfactor(int &dest_numcoeffs,
                                  std::array<float, 4>& dest_coeffsA,
                                  std::array<float, 4>& dest_coeffsB,
                                  float r, float alpha,
                                  simAngularType direction) {
  // new filter has got only one a- and one b-coefficient
  dest_numcoeffs = 1;
  // if a destination filter is already existing, then delete it
  #if 0
  if (dest_coeffsA != nullptr)
    delete[] dest_coeffsA;
  if (dest_coeffsB != nullptr)
    delete[] dest_coeffsB;
  // reserve memory for the a- and b-coefficient
  dest_coeffsA = new float[1];
  dest_coeffsB = new float[1];
  #endif
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
void Lambda::adaptfilter(int &dest_numcoeffs,
                         std::array<float, 4>& dest_coeffsA,
                         std::array<float, 4>& dest_coeffsB,
                         int *src_id, int *src_numcoeffs,
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
  if ((src_numcoeffs[actnum]) > dest_coeffsA.size())
    dest_numcoeffs = 4;

  // if a destination filter is already existing, then delete it
  #if 0
  if (dest_coeffsA != nullptr)
    delete[] dest_coeffsA;
  if (dest_coeffsB != nullptr)
    delete[] dest_coeffsB;
  // reserve memory for the a- and b-coefficients
  dest_coeffsA = new float[src_numcoeffs[actnum]];
  dest_coeffsB = new float[src_numcoeffs[actnum]];
  #endif
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
