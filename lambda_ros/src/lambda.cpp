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

inline void interpolcol(float dx, int r0, int g0, int b0, int r1, int g1,
                        int b1, int &r, int &g, int &b) {
  r = r0 + dx * (r1 - r0);
  g = g0 + dx * (g1 - g0);
  b = b0 + dx * (b1 - b0);
}

inline void colormap_paint(float v, int cmap, int &r, int &g, int &b) {
  switch (cmap) {
  case 0:
    // greyscale
    r = v * 255;
    g = v * 255, b = v * 255;
    break;
  case 1:
    // heat
    switch (int(v * 4)) {
    case 0:
      interpolcol(v / 0.25, 0, 0, 0, 69, 25, 94, r, g, b);
      break;
    case 1:
      interpolcol((v - 0.25) / 0.25, 60, 25, 94, 216, 82, 50, r, g, b);
      break;
    case 2:
      interpolcol((v - 0.5) / 0.25, 216, 82, 50, 253, 162, 28, r, g, b);
      break;
    case 3:
      interpolcol((v - 0.75) / 0.25, 253, 162, 28, 255, 255, 255, r, g, b);
      break;
    default:
      r = 255;
      g = 255;
      b = 255;
      break;
    }
    break;
  case 2:
    // temp
    switch (int(v * 4)) {
    case 0:
      interpolcol(v / 0.25, 0, 0, 102, 87, 113, 234, r, g, b);
      break;
    case 1:
      interpolcol((v - 0.25) / 0.25, 87, 113, 234, 228, 234, 252, r, g, b);
      break;
    case 2:
      interpolcol((v - 0.5) / 0.25, 228, 234, 252, 253, 251, 130, r, g, b);
      break;
    case 3:
      interpolcol((v - 0.75) / 0.25, 253, 251, 130, 208, 34, 41, r, g, b);
      break;
    default:
      r = 208;
      g = 34;
      b = 41;
      break;
    }
  }
}

void colormap_heat(float v, int &r, int &g, int &b) {
  // v should be 0-1
  switch (int(v * 4)) {
  case 0:
    // interpolcol(v/0.25, 0, 0, 0, 69, 25, 94, r, g, b);
    v *= 4;
    r = v * 69;
    g = v * 25;
    b = v * 94;
    return;
  case 1:
    // interpolcol((v-0.25)/0.25, 60, 25, 94, 216, 82, 50, r, g, b);
    v = (v - 0.25) * 4;
    r = 60 + v * (216 - 60);
    g = 25 + v * (82 - 25);
    b = 94 + v * (50 - 94);
    return;
  case 2:
    // interpolcol((v-0.5)*4, 216, 82, 50, 253, 162, 28, r, g, b);
    v = (v - 0.5) * 4;
    r = 216 + v * (253 - 216);
    g = 82 + v * (162 - 82);
    b = 50 + v * (28 - 50);
    return;
  case 3:
    // interpolcol((v-0.75)*4, 253, 162, 28, 255, 255, 255, r, g, b);
    v = (v - 0.75) * 4;
    r = 253 + v * (255 - 253);
    g = 162 + v * (255 - 162);
    b = 28 + v * (255 - 28);
    break;
  case 4:
    r = 255;
    g = 255;
    b = 255;
    return;
  }
}

void colormap_temp(float v, int &r, int &g, int &b) {
  switch (int(v * 4)) {
  case 0:
    // interpolcol(v/0.25, 0, 0, 102, 87, 113, 234, r, g, b);
    v *= 4;
    r = v * 87;
    g = v * 113;
    b = 102 + v * (234 - 102);
    return;
  case 1:
    // interpolcol((v-0.25)/0.25, 87, 113, 234, 228, 234, 252, r, g, b);
    v = (v - 0.25) * 4;
    r = 87 + v * (228 - 87);
    g = 113 + v * (234 - 113);
    b = 234 + v * (252 - 234);
    return;
  case 2:
    // interpolcol((v-0.5)/0.25, 228, 234, 252, 253, 251, 130, r, g, b);
    v = (v - 0.5) * 4;
    r = 228 + v * (253 - 228);
    g = 234 + v * (251 - 234);
    b = 252 + v * (130 - 252);
    return;
  case 3:
    // interpolcol((v-0.75)/0.25, 253, 251, 130, 208, 34, 41, r, g, b);
    v = (v - 0.75) * 4;
    r = 253 + v * (208 - 253);
    g = 251 + v * (34 - 251);
    b = 130 + v * (41 - 130);
    return;
  case 4:
    r = 208;
    g = 34;
    b = 41;
    return;
  }
}

void colormap_grey(float v, int &r, int &g, int &b) {
  r = v * 255;
  g = v * 255, b = v * 255;
}

typedef void (*colormap_t)(float, int &, int &, int &);

colormap_t get_colormap(int colormap_index) {
  switch (colormap_index) {
  case 0:
    return colormap_grey;
  case 1:
    return colormap_heat;
  case 2:
    return colormap_temp;
  default:
    return colormap_grey;
  }
}

#define DEBUG_ENCODING
#define CLAMP(x, low, high)                                                    \
  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::lambda()
//
// PURPOSE
//   Constructor for the program's main class, initializes program and builds up
//
// INPUT
//   int argc         : Number of elements in input vector argv
//   char *argv[]     : Array of variable input parameters
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland 	no changes
//05/09	2.0
//
lambda::lambda(const char *name, int argc, char *argv[]) :
  GFX_MAXCONTRAST(100),
  GFX_MINCONTRAST(0),
  GFX_STDCONTRAST(50),
  GFX_MAXSAMPLES(9999999),
  GFX_MINSAMPLES(0),
  GFX_STDSAMPLES(0),
  GFX_MAXSKIP(999),
  GFX_MINSKIP(0),
  GFX_STDSKIP(0),
  GFX_MAXZOOM(999),
  GFX_MINZOOM(1),
  GFX_STDZOOM(1),
  AUTOEXIT(false),
  MEMSRC(20),
  COLORMAP(1)
{
  initVariables();
  // Process input parameters that might be provided in argv
  handleParameters(argc, argv);
  srand(time(NULL));
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::initVariables()
//
// PURPOSE
//   This function intializes all the important variables, arrays and matrices.
//   Sets pointers to NULL. Called only one single time at startup.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens	First build
//05/06	1.0 	M. Ruhland	partly rewritten; added pointers for		05/09
//2.0
//              filter handling, velocity sources
//              and angular matrix
//
void lambda::initVariables() {
  // Initialize receiver/recorder pointers
  data.recs = NULL;
  data.record = NULL;
  data.recIdx = NULL;
  // Initialize graphics properties & colours
  set("contrast", GFX_STDCONTRAST);
  graphics.zoom = GFX_STDZOOM;
  graphics.skip = GFX_STDSKIP + 1;
  graphics.dispSizeX = 0;
  graphics.dispSizeY = 0;
  graphics.colors.white[0] = 255.f;
  graphics.colors.grey[0] = 127.f;
  graphics.colors.black[0] = 0.f;
  graphics.colors.white[1] = 255.f;
  graphics.colors.grey[1] = 127.f;
  graphics.colors.black[1] = 0.f;
  graphics.colors.white[2] = 255.f;
  graphics.colors.grey[2] = 127.f;
  graphics.colors.black[2] = 0.f;
  graphics.screen = NULL;
  graphics.frame = NULL;
  // Initialize simulation environment data pointers
  data.envi = NULL;
  data.angle = NULL;
  data.srcs = NULL;
  data.boundary = NULL;
  data.deadnode = NULL;
  data.filt_left = NULL;
  data.filt_top = NULL;
  data.filt_right = NULL;
  data.filt_bottom = NULL;
  data.pres = NULL;
  data.inci = NULL;
  // Initialize filter memory pointers
  data.oldx_left = NULL;
  data.oldx_top = NULL;
  data.oldx_right = NULL;
  data.oldx_bottom = NULL;
  data.oldy_left = NULL;
  data.oldy_top = NULL;
  data.oldy_right = NULL;
  data.oldy_bottom = NULL;
  // Initialize filter coefficient pointers
  data.filtnumcoeffs_left = NULL;
  data.filtnumcoeffs_top = NULL;
  data.filtnumcoeffs_right = NULL;
  data.filtnumcoeffs_bottom = NULL;
  data.filtcoeffsA_left = NULL;
  data.filtcoeffsB_left = NULL;
  data.filtcoeffsA_top = NULL;
  data.filtcoeffsB_top = NULL;
  data.filtcoeffsA_right = NULL;
  data.filtcoeffsB_right = NULL;
  data.filtcoeffsA_bottom = NULL;
  data.filtcoeffsB_bottom = NULL;
  // Initialize velocity source pointers
  data.velo_left = NULL;
  data.velo_top = NULL;
  data.velo_right = NULL;
  data.velo_bottom = NULL;
  data.mem = NULL;
  data.samples = NULL;

  resetAll();

  status = MISMATCH;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::resetAll()
//
// PURPOSE
//   Resets important variables, arrays and matrices to zero, e.g. before
//   starting new simulation runs.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens	First build
//05/06	1.0 	M. Ruhland	mainly rewritten; added code for resetting	05/09
//2.0
//              the filter data and the angular matrix
//
void lambda::resetAll() {
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
  config.tSample = 0;
  config.fSample = 0;
  config.t0 = 0;
  config.colormap = COLORMAP;

  // delete simulation environment data pointers
  if (data.envi != NULL) {
    delete[] data.envi;
    data.envi = NULL;
  }
  if (data.angle != NULL) {
    delete[] data.angle;
    data.angle = NULL;
  }
  if (data.srcs != NULL) {
    delete[] data.srcs;
    data.srcs = NULL;
  }
  if (data.mem != NULL) {
    delete[] data.mem;
    data.mem = NULL;
  }
  if (data.filt_left != NULL) {
    delete[] data.filt_left;
    data.filt_left = NULL;
  }
  if (data.filt_top != NULL) {
    delete[] data.filt_top;
    data.filt_top = NULL;
  }
  if (data.filt_right != NULL) {
    delete[] data.filt_right;
    data.filt_right = NULL;
  }
  if (data.filt_bottom != NULL) {
    delete[] data.filt_bottom;
    data.filt_bottom = NULL;
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

  // delete left filter coefficient arrays
  if (data.filtcoeffsA_left != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsA_left[n] != NULL)
        delete[] data.filtcoeffsA_left[n];
      data.filtcoeffsA_left[n] = NULL;
    }
    delete[] data.filtcoeffsA_left;
    data.filtcoeffsA_left = NULL;
  }
  // delete left filter coefficient arrays
  if (data.filtcoeffsB_left != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsB_left[n] != NULL)
        delete[] data.filtcoeffsB_left[n];
      data.filtcoeffsB_left[n] = NULL;
    }
    delete[] data.filtcoeffsB_left;
    data.filtcoeffsB_left = NULL;
  }
  // delete top filter coefficient arrays
  if (data.filtcoeffsA_top != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsA_top[n] != NULL)
        delete[] data.filtcoeffsA_top[n];
      data.filtcoeffsA_top[n] = NULL;
    }
    delete[] data.filtcoeffsA_top;
    data.filtcoeffsA_top = NULL;
  }
  // delete top filter coefficient arrays
  if (data.filtcoeffsB_top != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsB_top[n] != NULL)
        delete[] data.filtcoeffsB_top[n];
      data.filtcoeffsB_top[n] = NULL;
    }
    delete[] data.filtcoeffsB_top;
    data.filtcoeffsB_top = NULL;
  }
  // delete right filter coefficient arrays
  if (data.filtcoeffsA_right != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsA_right[n] != NULL)
        delete[] data.filtcoeffsA_right[n];
      data.filtcoeffsA_right[n] = NULL;
    }
    delete[] data.filtcoeffsA_right;
    data.filtcoeffsA_right = NULL;
  }
  // delete right filter coefficient arrays
  if (data.filtcoeffsB_right != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsB_right[n] != NULL)
        delete[] data.filtcoeffsB_right[n];
      data.filtcoeffsB_right[n] = NULL;
    }
    delete[] data.filtcoeffsB_right;
    data.filtcoeffsB_right = NULL;
  }
  // delete bottom filter coefficient arrays
  if (data.filtcoeffsA_bottom != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsA_bottom[n] != NULL)
        delete[] data.filtcoeffsA_bottom[n];
      data.filtcoeffsA_bottom[n] = NULL;
    }
    delete[] data.filtcoeffsA_bottom;
    data.filtcoeffsA_bottom = NULL;
  }
  // delete bottom filter coefficient arrays
  if (data.filtcoeffsB_bottom != NULL) {
    for (int n = 0; n < config.nNodes; n++) {
      if (data.filtcoeffsB_bottom[n] != NULL)
        delete[] data.filtcoeffsB_bottom[n];
      data.filtcoeffsB_bottom[n] = NULL;
    }
    delete[] data.filtcoeffsB_bottom;
    data.filtcoeffsB_bottom = NULL;
  }

  // delete filter number-of-coefficients arrays
  if (data.filtnumcoeffs_bottom != NULL) {
    delete[] data.filtnumcoeffs_bottom;
    data.filtnumcoeffs_bottom = NULL;
  }
  if (data.filtnumcoeffs_right != NULL) {
    delete[] data.filtnumcoeffs_right;
    data.filtnumcoeffs_right = NULL;
  }
  if (data.filtnumcoeffs_top != NULL) {
    delete[] data.filtnumcoeffs_top;
    data.filtnumcoeffs_top = NULL;
  }
  if (data.filtnumcoeffs_left != NULL) {
    delete[] data.filtnumcoeffs_left;
    data.filtnumcoeffs_left = NULL;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::resetSimulation()
//
// PURPOSE
//   Resets variables and arrays used directly for simulation purposes.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens	First build
//05/06	1.0 	M. Ruhland	mainly rewritten; added resetting of the 	05/09
//2.0
//              filter's memories and the velocity sources;
//				removed some old optimization pointers
//              that are not needed any more
//
void lambda::resetSimulation() {
  // Delete simulation environment data memory
  if (data.pres != NULL) {
    delete[] data.pres;
    data.pres = NULL;
  }
  if (data.inci != NULL) {
    delete[] data.inci;
    data.inci = NULL;
  }
  // Reset pressure index pointers
  index.presPast = NULL;
  index.presPres = NULL;
  index.presFutu = NULL;
  // Reset incident pressure index pointers (past)
  index.inciPastTop = NULL;
  index.inciPastRight = NULL;
  index.inciPastBottom = NULL;
  index.inciPastLeft = NULL;
  // Reset incident pressure index pointers (present)
  index.inciPresTop = NULL;
  index.inciPresRight = NULL;
  index.inciPresBottom = NULL;
  index.inciPresLeft = NULL;
  // Reset incident pressure index pointers (future)
  index.inciFutuTop = NULL;
  index.inciFutuRight = NULL;
  index.inciFutuBottom = NULL;
  index.inciFutuLeft = NULL;
  // Reset indices used during calculation
  for (int x = 0; x < 3; x++) {
    index.idxP[x] = 0;
    index.idxITop[x] = 0;
    index.idxILeft[x] = 0;
    index.idxIRight[x] = 0;
    index.idxIBottom[x] = 0;
  }
  if (1) {
    // Delete bottom filter non-recursive memory
    if (data.oldx_bottom != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldx_bottom[n] != NULL)
          delete[] data.oldx_bottom[n];
        data.oldx_bottom[n] = NULL;
      }
      delete[] data.oldx_bottom;
      data.oldx_bottom = NULL;
    }
    // Delete right filter non-recursive memory
    if (data.oldx_right != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldx_right[n] != NULL)
          delete[] data.oldx_right[n];
        data.oldx_right[n] = NULL;
      }
      delete[] data.oldx_right;
      data.oldx_right = NULL;
    }
    // Delete top filter non-recursive memory
    if (data.oldx_top != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldx_top[n] != NULL)
          delete[] data.oldx_top[n];
        data.oldx_top[n] = NULL;
      }
      delete[] data.oldx_top;
      data.oldx_top = NULL;
    }
    // Delete left filter non-recursive memory
    if (data.oldx_left != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldx_left[n] != NULL)
          delete[] data.oldx_left[n];
        data.oldx_left[n] = NULL;
      }
      delete[] data.oldx_left;
      data.oldx_left = NULL;
    }
    // Delete bottom filter recursive memory
    if (data.oldy_bottom != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldy_bottom[n] != NULL)
          delete[] data.oldy_bottom[n];
        data.oldy_bottom[n] = NULL;
      }
      delete[] data.oldy_bottom;
      data.oldy_bottom = NULL;
    }
    // Delete right filter recursive memory
    if (data.oldy_right != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldy_right[n] != NULL)
          delete[] data.oldy_right[n];
        data.oldy_right[n] = NULL;
      }
      delete[] data.oldy_right;
      data.oldy_right = NULL;
    }
    // Delete top filter recursive memory
    if (data.oldy_top != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldy_top[n] != NULL)
          delete[] data.oldy_top[n];
        data.oldy_top[n] = NULL;
      }
      delete[] data.oldy_top;
      data.oldy_top = NULL;
    }
    // Delete left filter recursive memory
    if (data.oldy_left != NULL) {
      for (int n = 0; n < config.nNodes; n++) {
        if (data.oldy_left[n] != NULL)
          delete[] data.oldy_left[n];
        data.oldy_left[n] = NULL;
      }
      delete[] data.oldy_left;
      data.oldy_left = NULL;
    }
  }
  // Delete velocity source memory
  if (data.velo_left != NULL) {
    delete[] data.velo_left;
    data.velo_left = NULL;
  }
  if (data.velo_top != NULL) {
    delete[] data.velo_top;
    data.velo_top = NULL;
  }
  if (data.velo_right != NULL) {
    delete[] data.velo_right;
    data.velo_right = NULL;
  }
  if (data.velo_bottom != NULL) {
    delete[] data.velo_bottom;
    data.velo_bottom = NULL;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::handleParameters()
//
// PURPOSE
//   Processes input parameters and sets internal variables accordingly.
//
// INPUT
//   int argc     : Number of elements in input vector argv
//   char *argv[] : Array of variable input parameters
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens	First build
//05/06	1.0
//	M. Ruhland	added "-walls" parameter and
//05/09   2.0
//              "-exit" parameter
//
void lambda::handleParameters(int argc, char *argv[]) {
  string argument;
  int arg;
  // Order of parameters is random, but rce, rco and vis must be
  // activated after loading a file, if requested. If a file is requested,
  // the last step must be starting the simulation.
  // To achieve this order of processing, the parameters are checked and read in
  // a first step, notifying necessary further steps through theses bool
  // variables. Theses steps will then be processed in the right order later.
  bool startSim = false, clickAvi = false, clickRce = false, clickRco = false,
       clickVis = false, clickWalls = false;

  // Parse array of input arguments
  for (arg = 0; arg < argc; arg++) {
    argument = argv[arg];

    if ((argument == "-file") || (argument == "-File") ||
        (argument == "-FILE") || (argument == "/file") ||
        (argument == "/File") || (argument == "/FILE")) {
      // if this argument is "-file" and there is another argument following
      // this one, try to load the file named like that following parameter.
      if (arg < argc - 1) {
        startSim = true;
        string fileName = argv[arg + 1];
        if (loadFile(fileName) != NONE)
          startSim = false;
      }
    } else if ((argument == "-zoom") || (argument == "-Zoom") ||
               (argument == "-ZOOM") || (argument == "/zoom") ||
               (argument == "/Zoom") || (argument == "/ZOOM")) {
      if (arg < argc - 1)
        set("zoom", atoi(argv[arg + 1])); // set the zoom value
    } else if ((argument == "-contrast") || (argument == "-Contrast") ||
               (argument == "-CONTRAST") || (argument == "/contrast") ||
               (argument == "/Contrast") || (argument == "/CONTRAST")) {
      if (arg < argc - 1)
        set("contrast", atoi(argv[arg + 1])); // set the contrast value
    } else if ((argument == "-skip") || (argument == "-Skip") ||
               (argument == "-SKIP") || (argument == "/skip") ||
               (argument == "/Skip") || (argument == "/SKIP")) {
      if (arg < argc - 1)
        set("skip", atoi(argv[arg + 1])); // set the skip value
    } else if ((argument == "-iterations") || (argument == "-Iterations") ||
               (argument == "-ITERATIONS") || (argument == "/iterations") ||
               (argument == "/Iterations") || (argument == "/ITERATIONS")) {
      if (arg < argc - 1)
        set("nN", atoi(argv[arg + 1])); // set total number of iterations
    } else if ((argument == "-colormap") || (argument == "-colormap") ||
               (argument == "-colormap") || (argument == "/colormap") ||
               (argument == "/colormap") || (argument == "/colormap")) {
      if (arg < argc - 1)
        set("colormap", atoi(argv[arg + 1]));
    } else if ((argument == "-rce") || (argument == "-Rce") ||
               (argument == "-RCE") || (argument == "/rce") ||
               (argument == "/Rce") || (argument == "/RCE")) {
      clickRce = true; // check rce checkbox
    } else if ((argument == "-rco") || (argument == "-Rco") ||
               (argument == "-RCO") || (argument == "/rco") ||
               (argument == "/Rco") || (argument == "/RCO")) {
      clickRco = true; // check rco checkbox
    } else if ((argument == "-vis") || (argument == "-Vis") ||
               (argument == "-VIS") || (argument == "/vis") ||
               (argument == "/Vis") || (argument == "/VIS")) {
      clickVis = true; // check vis checkbox
    } else if ((argument == "-walls") || (argument == "-Walls") ||
               (argument == "-WALLS") || (argument == "/walls") ||
               (argument == "/Walls") || (argument == "/WALLS")) {
      clickWalls = true; // check walls checkbox
    } else if ((argument == "-exit") || (argument == "-Exit") ||
               (argument == "-EXIT") || (argument == "/exit") ||
               (argument == "/Exit") || (argument == "/EXIT")) {
      AUTOEXIT = true; // enable automatic exit at the end of simulation
    } else if ((argument == "-help") || (argument == "--help") ||
               (argument == "/help")) {
      cout << "lambda [options]\n"
              "\n"
              "-file simfile   : open the .sim file\n"
              "-vis            : activate visualization\n"
              "-rce            : activate recording at receivers, if defined\n"
              "-walls          : show walls, if defined\n"
              "(default 100)\n"
              "-contrast N (0-100)  : adjust the contrast of the visualization "
              "(default 50)\n"
              "-colormap N (0-2)    : set the color map (0=gray, 1=hot, "
              "2=temp) (def.=0)\n"
              "-rco            : activate the recording of the visualization "
              "(as a .rco file)\n"
              "-zoom           : set the zoom level of a visualization (an "
              "integer number)\n"
              "-skip           : set number of frames to skip (default 0)\n"
              "-exit           : exit the program after finishing (good for "
              "batch processes)\n"
              "\n"

          ;
      exit(0);
    }
  }
}

#if 0
void lambda::stop() {
  resetSimulation();
  // Reset display, if vis was on and simulation was reset manually
  // (don't change picture if simulation ended automatically)
  //  drawLambda();
  config.n = 0;
  // And set new status. If the stopped process was a simulation, data.envi must
  // have been initialized before and set to an adress != NULL.
  if (data.envi != NULL)
    // Was simulating, set status ready to simulate.
    set("status", SIMULATOR);
  else
    // Was replaying, set status ready for replay.
    set("status", PLAYER);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::vis()
//
// PURPOSE
//   QT Slot connected to the Vis checkbox, starts or quits visualization
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland	Modified to show simfield immediatedly if   05/09   2.0
//				visbox is checked during paused simulation;
//              needed for walls-visualization
//
void lambda::vis() {
  {
    // If visualization just got switched ON, prepare everything for
    // visualization First of all, make sure that the visualization has been
    // closed and reset or initialized properly before. Screen and frame ought
    // to be set to NULL.
    if ((graphics.screen == NULL) && (graphics.frame == NULL)) {
      // If so, initialize frame and screen. If not, the old variables will be
      // used graphics.frame = new CImg<float>(config.nX,config.nY);
      graphics.frame = new CImg<float>(config.nX, config.nY, 1, 3);

      // graphics.screen=new
      // CImgDisplay(graphics.dispSizeX,graphics.dispSizeY,"Lambda
      // visualization",0,2,0,0);

      // x, y, title, normalization, is_fullscreen, is_closed
      // normalization: 0=none, 1=always, 2=once
      printf("opening visualization\n");
      graphics.screen = new CImgDisplay(graphics.dispSizeX, graphics.dispSizeY,
                                        "Visualization", 0, 0, 0);
      // drawLambda();
    }
    // Now that frames are being calculated, enable screenshot button
    // if status simulation is running or paused (in other cases, no frame has
    // been calculated yet).
    if ((status == RUNNING) || (status == PAUSED))
      processVis();
  }
  #if 0
  else {
    // If visualization got switched OFF, tidy up the leftovers
    // Screenshots are not available anymore, since no frames will be calculated
    if ((graphics.screen != NULL) && (graphics.frame != NULL)) {
      // delete frame and screen, reset to NULL
      delete graphics.frame;
      delete graphics.screen;
      graphics.frame = NULL;
      graphics.screen = NULL;
    }
  }
  #endif
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::showbounds()
//
// PURPOSE
//   QT Slot connected to the Walls/Showbounds checkbox. Updates the
//   visualization window when checkbox is clicked.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	M. Ruhland 	new function
//05/09	2.0
//
void lambda::showbounds() {
  // make sure to call processVis() if checkbox walls is checked/unchecked
  // so that the change has an immediate effect on the vis screen

  if (status != RUNNING)
    drawLambda();
  if ((status == RUNNING) || (status == PAUSED))
    processVis();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::processVis()
//
// PURPOSE
//   Updates the visualization screen after each simulation iteration, if vis is
//   on.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0
//  M. Ruhland  changed copyright line                      04/08
//	M. Ruhland 	Added drawing of the walls and receivers
//05/09	2.0
//              if the "walls"-checkbox is checked
//
void lambda::processVis() {
  if (graphics.screen != NULL) {
    processFrame(graphics.frame, index.presPres);
  }
}

void lambda::processFrame(CImg<float> *frame, float *pressure,
    const bool showbounds_box) {
  int r, g, b, r0, g0, b0;
  float v;
  char textbuf[16];
  float contrast = (float)graphics.contrast / 255.f;
  colormap_t cm = get_colormap(config.colormap);
  // int cmap = config.colormap;
  unsigned int nNodes = config.nNodes;
  unsigned int nNodes2 = nNodes * 2;
  float *framedata = frame->data();
  bool showbounds = showbounds_box;
  // float *pressure = index.presPres;
  float *fr = framedata;
  float *fg = framedata + nNodes;
  float *fb = framedata + nNodes2;

  bool *deadnodes = data.deadnode;
  float *envi = data.envi;
  cm(0.5, r0, g0, b0);

  for (register unsigned int n = 0; n < nNodes; n++) {
    if (deadnodes[n]) {
      /*
      fr[n] = showbounds ? 40  : r0;
      fg[n] = showbounds ? 200 : g0;
      fb[n] = showbounds ? 40  : b0;
      */

      if (showbounds) {
        fr[n] = 40;
        fg[n] = 200;
        fb[n] = 40;
      } else {
        fr[n] = r0;
        fg[n] = g0;
        fb[n] = b0;
      }
    } else {
      if (showbounds && envi[n] != 0.f) {
        // is it a wall?
        fr[n] = 50;
        fg[n] = 50;
        fb[n] = 255;
      } else {
        v = pressure[n] * contrast + 0.5;
        if (v > 1)
          v = 1;
        else if (v < 0)
          v = 0;
        cm(v, r, g, b);
        // colormap_paint(v, cmap, r, g, b);
        fr[n] = r;
        fg[n] = g;
        fb[n] = b;
      }
    }
  };
  /*
      // draw walls and receivers, if walls-checkbox is checked
  float *fr = framedata;
  float *fg = framedata + nNodes;
  float *fb = framedata + nNodes2;
      if (gui.showboundsBox->isChecked())
      {
              for (register unsigned int n=0;n<nNodes;n++) {
          if (deadnodes[n]) {
              fr[n] = 40;
              fg[n] = 200;
              fb[n] = 40;
          }
          else if( envi[n]!=0.f ) {
              // is it a wall?
                              fr[n] = 50;
                              fg[n] = 50;
                              fb[n] = 255;
                      }
          
              }
      }
  */

  float bg[3] = {static_cast<float>(r0), static_cast<float>(g0), static_cast<float>(b0)};
  sprintf(textbuf, "C%1.1f F%05i", contrast, config.n + 1);
  frame->draw_text(0, -2, textbuf, graphics.colors.black, bg, 0.5);
  sprintf(textbuf, "ms%.1f", config.n * config.tSample * 1E3);
  frame->draw_text(0, 11, textbuf, graphics.colors.black, bg, 0.5);
  graphics.frame_ready = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::processRce()
//
// PURPOSE
//   Processes the receiver output after each calculated sim iteration if Rce is
//   switched on.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0
//  M. Ruhland  no changes                                  05/09   2.0
//
void lambda::processRce() {
  double *dummy = new double; // double pointer needed for float2double cast
  for (int rec = 0; rec < config.nRec; rec++) {
    // For each receiver in the simulation environment, cast
    // its sound pressure value to double and append it to the rce file.
    *dummy = (double)*(index.presPres + *(data.recIdx + rec));
    files.rceFile.write((char *)dummy, sizeof(double));
  }
  delete dummy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::processRco()
//
// PURPOSE
//   Processes the recorder output after each calculated sim iteration if Rco is
//   switched on.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0
//  M. Ruhland  no changes                                  05/09   2.0
//
void lambda::processRco() {
  // just write all the future pressure data into the rco file...
  files.rcoFile.write((char *)index.presPres, sizeof(float) * config.nNodes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::set()
//
// PURPOSE
//   Function template. This function should be used whenever one of the key
//   variables of the lambda class is changed. It performs the necessary checks
//   before changing one of those variables and makes necessary additional
//   changes after changing. For example, the number of elements in X-direction
//   should not just be changed. Instead, you should use set("nX",41);. This
//   will check wether the second argument is a valid value for nX
//   (>0) and will update nNodes (which should always be nX*nY at any time) and
//   dispSizeX automatically.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0
//  M. Ruhland  fixed some warnings                         04/08
//  M. Ruhland  added "rho"-parameter for velocity sources	05/09   2.0
//              and removed config.FourFields, which is
//              no longer needed
//
template <class T> simError lambda::set(const string what, const T value) {
  if (what == "nX") {
    // If nX is to be set, check if new X-size is greater than 0.
    if ((int)value < 1)
      return NO_NODES;
    config.nX = (int)value;
    // Adjust nNodes and dispSizeX to fit the new setting
    set("nNodes", config.nX * config.nY);
    set("dispSizeX", (int)value * graphics.zoom);
    return NONE;
  }
  if (what == "nY") {
    // If nY is to be set, check if new Y-size is greater than 0.
    if ((int)value < 1)
      return NO_NODES;
    config.nY = (int)value;
    // Adjust nNodes and dispSizeY to fit the new setting
    set("nNodes", config.nX * config.nY);
    set("dispSizeY", (int)value * graphics.zoom);
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
  if (what == "contrast") {
    // If contrast is to be set, check if new contrast is between 0 and 999.
    if (((int)value < (int)GFX_MINCONTRAST) ||
        ((int)value > (int)GFX_MAXCONTRAST))
      return CONTRAST_OUT_OF_RANGE;
    if ((int)value == 0)
      graphics.contrast = 0;
    else if ((int)value == 100)
      graphics.contrast = -1;
    else
      graphics.contrast =
          (int)(pow(2, .1 * (float)value) +
                5 * ((float)value -
                     1)); //(unsigned int)exp(.075*(unsigned int)value);
    // Update Gui (contrast might have been set by input parameter)
    // If visualization is on and paused, update the screen with the new
    // contrast setting. If the simulation is not paused, the new contrast will
    // be used for the next frame anyways.
    // if ((status == PAUSED) && (gui.visBox->isChecked()))
    //  processVis();
    return NONE;
  }
  if (what == "zoom") {
    // If zoom is to be set, check if new zoom is between 1 and 999.
    if (((int)value < 1) || ((int)value > 999))
      return ZOOM_OUT_OF_RANGE;
    graphics.zoom = (int)value;
    // Update Gui (zoom change might have been invoked from somewhere else)
    // Adjust vis screen size
    set("dispSizeX", (int)value * config.nX);
    set("dispSizeY", (int)value * config.nY);
    if (graphics.screen != NULL) {
      graphics.screen->resize((int)graphics.dispSizeX, (int)graphics.dispSizeY);
      if (status != RUNNING)
        drawLambda();
    }
    return NONE;
  }
  if (what == "skip") {
    // If skip is to be set, check if new contrast is between 0 and 999.
    if (((int)value < 0) || ((int)value > 999))
      return CONTRAST_OUT_OF_RANGE;
    // The algorithm that checks for skipped frames needs skip to be the
    // selected value, e.g. 1 if 0 frames are to be skipped
    graphics.skip = (int)value + 1;
    // Update Gui
    return NONE;
  }
  if (what == "colormap") {
    COLORMAP = (int)value;
    config.colormap = (int)value;
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
  if (what == "dispSizeX") {
    graphics.dispSizeX = (int)value;
    // Resize screen if open
    if (graphics.screen != NULL)
      graphics.screen->resize((int)graphics.dispSizeX, (int)graphics.dispSizeY);
    return NONE;
  }
  if (what == "dispSizeY") {
    graphics.dispSizeY = (int)value;
    // Resize screen if open
    if (graphics.screen != NULL)
      graphics.screen->resize((int)graphics.dispSizeX, (int)graphics.dispSizeY);
    return NONE;
  }
  if (what == "status") {
    // Procedures for changing the program's status are more complex.
    // First of all, update the status variable
    status = (simStatus)value;
    switch ((simStatus)value) {
    // Take necessary actions for the new status
    case PLAYER:
      // statusLine->setText("<font color=red>Ready for replay</font>");
      // During replays, repTimer is resonsible for correct timing. Make
      // Replays always have the visualization feature switched on
      // But all the other features are disabled
      // Show empty frame in vis window
      drawLambda();
      break;

    case PAUSED:
      // statusLine->setText("<font color=red>Paused</font>");
      break;
    case MISMATCH:
      // statusLine->setText("<font color=red>Bad data</font>");
      // If no data is loaded, disable everything. No action can be performed.

      break;
    }
  }
  return NONE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::defineSource()
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
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland 	added velocity sources handling
//05/09	2.0
//
simError lambda::defineSource(const int idx, const simSource *srcData) {
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
  alpha = data.angle[srcxy];
  // angle out of bounds?
  if ((alpha < 0.f) || (alpha >= 360.f))
    alpha = 0.f;
  if ((srcData->type >= 6) && (srcData->type <= 10)) {
    if ((alpha >= 0.f) && (alpha < 90.f)) {
      // set up a left- and top- filter if angle of incidence is
      // between 0 and 90 degrees
      data.boundary[srcxy] = true;
      data.filt_left[srcxy] = true;
      data.filt_top[srcxy] = true;
      adaptreflexionfactor(
          data.filtnumcoeffs_left[srcxy], data.filtcoeffsA_left[srcxy],
          data.filtcoeffsB_left[srcxy], 1.f, 180.f, kHorizontal);
      adaptreflexionfactor(data.filtnumcoeffs_top[srcxy],
                           data.filtcoeffsA_top[srcxy],
                           data.filtcoeffsB_top[srcxy], 1.f, 270.f, kVertical);
    } else if ((alpha >= 90.f) && (alpha < 180.f)) {
      // set up a top- and right- filter if angle of incidence is
      // between 90 and 180 degrees
      data.boundary[srcxy] = true;
      data.filt_top[srcxy] = true;
      data.filt_right[srcxy] = true;
      adaptreflexionfactor(data.filtnumcoeffs_top[srcxy],
                           data.filtcoeffsA_top[srcxy],
                           data.filtcoeffsB_top[srcxy], 1.f, 270.f, kVertical);
      adaptreflexionfactor(
          data.filtnumcoeffs_right[srcxy], data.filtcoeffsA_right[srcxy],
          data.filtcoeffsB_right[srcxy], 1.f, 0.f, kHorizontal);
    } else if ((alpha >= 180.f) && (alpha < 270.f)) {
      // set up a right- and bottom- filter if angle of incidence is
      // between 180 and 270 degrees
      data.boundary[srcxy] = true;
      data.filt_right[srcxy] = true;
      data.filt_bottom[srcxy] = true;
      adaptreflexionfactor(
          data.filtnumcoeffs_right[srcxy], data.filtcoeffsA_right[srcxy],
          data.filtcoeffsB_right[srcxy], 1.f, 0.f, kHorizontal);
      adaptreflexionfactor(
          data.filtnumcoeffs_bottom[srcxy], data.filtcoeffsA_bottom[srcxy],
          data.filtcoeffsB_bottom[srcxy], 1.f, 90.f, kVertical);
    } else if ((alpha >= 270.f) && (alpha < 360.f)) {
      // set up a bottom- and left- filter if angle of incidence is
      // between 270 and 360 degrees
      data.boundary[srcxy] = true;
      data.filt_bottom[srcxy] = true;
      data.filt_left[srcxy] = true;
      adaptreflexionfactor(
          data.filtnumcoeffs_bottom[srcxy], data.filtcoeffsA_bottom[srcxy],
          data.filtcoeffsB_bottom[srcxy], 1.f, 90.f, kVertical);
      adaptreflexionfactor(
          data.filtnumcoeffs_left[srcxy], data.filtcoeffsA_left[srcxy],
          data.filtcoeffsB_left[srcxy], 1.f, 180.f, kHorizontal);
    }
  }
  return NONE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::loadSimulation
//
// PURPOSE
//   Tries to load a simulation file.
//
// INPUT
//   const string fileName : name of the file to be opened.
//
// OUTPUT
//   None
//
// RETURN VALUE
//   simError: NONE if file was opened successfully, error identfier otherwise.
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland	implemented sqrt(2) reflection factor
//04/08	1.1 				preemphasis, and extended reflection 				factor value range from -1
//to 1. 				Simulation field borders initialized 				for zero reflection.
//  M. Ruhland  Completely rewritten for new 2.0 file       05/09   2.0
//              structure and computation model.
//
simError lambda::loadSimulation(const string fileName) {
  // some variables needed during the read-in process
  struct stat results;
  char *pblockid;
  double *pdummy;
  int actrec = 0;
  simError error = NONE;
  simSource curSource;
  bool donotreadnextblockid = false;
  float value;

  // check the filename
  if (stat((char *)fileName.c_str(), &results) != 0)
    return FILE_BAD;
  set("status", MISMATCH);
  // reset all variables
  resetAll();
  // open the simfile
  ifstream simFile((char *)fileName.c_str(), ios::in | ios::binary);

  //  ------ HEADER ------
  //  read the simfile header. it consists of the string "LAMBDASIM200".
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
    cout << "parsing DEF chunk\n";
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
          sqrt(2.f) *
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
    cout << "no DEF-Chunk exists, close the file and exit\n";
    simFile.close();
    delete[] pblockid;
    return FILE_DEF_BLOCK_BAD;
  }

  //  ----- ENVIRONMENT -----
  // read in the simulation environment matrix.
  simFile.read(pblockid, sizeof(char) * 3);
  if (strncmp(pblockid, "ENV", 3) == 0) // ENV-Chunk exists?
  {
    cout << "parsing ENV chunk\n";
    pdummy =
        new double[config.nNodes]; // reserve memory for env-data in simfile
    data.envi = new float[config.nNodes];    // reserve memory for envi-matrix
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
      data.envi[pos] = (float)value; // (all nodes)
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
    cout << "parsing ANG chunk\n";
    pdummy =
        new double[config.nNodes]; // reserve memory for ang-data in simfile
    data.angle = new float[config.nNodes]; // reserve memory for angle-matrix
    simFile.read((char *)pdummy,
                 sizeof(double) * config.nNodes); // read angle-matrix
    for (int pos = 0; pos < config.nNodes;
         pos++)                             // and cast it from double to float
      data.angle[pos] = (float)pdummy[pos]; // (all nodes)
    delete[] pdummy;
    donotreadnextblockid = false; // make shure that the next chunk will be read
  } else if ((strncmp(pblockid, "FLT", 3) == 0) ||
             (strncmp(pblockid, "SRC", 3) == 0) ||
             (strncmp(pblockid, "SMP", 3) ==
              0)) { // if angle-matrix does not exist and the next Chunk is FLT
                    // or SMP or SRC
    cout << "angle-matrix does not exist and the next Chunk is FLT or SMP or "
            "SRC\n";
    data.angle = new float[config.nNodes];        // create empty angle matrix
    for (int pos = 0; pos < config.nNodes; pos++) // and fill it with 400.f
      data.angle[pos] =
          400.f; // (this means no preemphasis is done on the nodes)
    donotreadnextblockid =
        true; // do not read the next Chunk ID, because we have it already
  } else      // if angle-matrix does not exist and no valid chunk is following
  {
    cout << "angle matrix does not exist and no valid chunk is following\n";
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
    cout << "parsing FLT chunk\n";
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

      tmp_filtnumcoeffs[n] = max(numcoeffsA, numcoeffsB); // numcoeffs=maximum

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
    cout << "the FLT-chunk is missing, this a valid chunk, initialize filters "
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
    cout << "FLT-chunk does not exist and no valid chunk is following\n";
    simFile.close();
    delete[] pblockid;
    return FILE_FLT_BLOCK_BAD;
  }

  //  ----- count the receivers -----
  set("nRec", 0);
  for (int pos = 0; pos < config.nNodes; pos++)
    if (data.envi[pos] < -1.0)
      set("nRec", config.nRec + 1);
  // and reserve memory for the receivers
  if (config.nRec > 0) {
    if (data.recIdx != NULL)
      delete[] data.recIdx;
    data.recIdx = new int[config.nRec];
  }

  //  ----- PREPROCESSING OF THE ENVIRONMENT -----
  data.boundary = new bool[config.nNodes]; // mem for boundary indicator
  // data.deadnode=new bool[config.nNodes];              // mem for deadnode
  // indicator
  data.filt_left = new bool[config.nNodes];   // mem for filter left indicator
  data.filt_top = new bool[config.nNodes];    // mem for filter top indicator
  data.filt_right = new bool[config.nNodes];  // mem for filter right indicator
  data.filt_bottom = new bool[config.nNodes]; // mem for filter bottom indicator
  data.filtnumcoeffs_left =
      new int[config.nNodes]; // mem for number of filter coeffs left
  data.filtnumcoeffs_top =
      new int[config.nNodes]; // mem for number of filter coeffs top
  data.filtnumcoeffs_right =
      new int[config.nNodes]; // mem for number of filter coeffs right
  data.filtnumcoeffs_bottom =
      new int[config.nNodes]; // mem for number of filter coeffs bottom
  data.filtcoeffsA_left =
      new float *[config.nNodes]; // mem for filter a-coeffs left
  data.filtcoeffsA_top =
      new float *[config.nNodes]; // mem for filter a-coeffs top
  data.filtcoeffsA_right =
      new float *[config.nNodes]; // mem for filter a-coeffs right
  data.filtcoeffsA_bottom =
      new float *[config.nNodes]; // mem for filter a-coeffs bottom
  data.filtcoeffsB_left =
      new float *[config.nNodes]; // mem for filter b-coeffs left
  data.filtcoeffsB_top =
      new float *[config.nNodes]; // mem for filter b-coeffs top
  data.filtcoeffsB_right =
      new float *[config.nNodes]; // mem for filter b-coeffs right
  data.filtcoeffsB_bottom =
      new float *[config.nNodes]; // mem for filter b-coeffs bottom
  // Initialize all these memories
  for (int pos = 0; pos < config.nNodes; pos++) {
    data.boundary[pos] = false;
    // data.deadnode[pos]=false;
    data.filt_left[pos] = false;
    data.filt_top[pos] = false;
    data.filt_right[pos] = false;
    data.filt_bottom[pos] = false;
    data.filtnumcoeffs_left[pos] = 0;
    data.filtnumcoeffs_top[pos] = 0;
    data.filtnumcoeffs_right[pos] = 0;
    data.filtnumcoeffs_bottom[pos] = 0;
    data.filtcoeffsA_left[pos] = NULL;
    data.filtcoeffsA_top[pos] = NULL;
    data.filtcoeffsA_right[pos] = NULL;
    data.filtcoeffsA_bottom[pos] = NULL;
    data.filtcoeffsB_left[pos] = NULL;
    data.filtcoeffsB_top[pos] = NULL;
    data.filtcoeffsB_right[pos] = NULL;
    data.filtcoeffsB_bottom[pos] = NULL;
  }

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
          data.filt_left[pos] = true;
          // apply a zero-reflection-filter to left border node
          adaptreflexionfactor(
              data.filtnumcoeffs_left[pos], data.filtcoeffsA_left[pos],
              data.filtcoeffsB_left[pos], 0.f, 180.f, kHorizontal);
        }
        if (y == 0) // top simfield border
        {
          data.filt_top[pos] = true;
          // apply a zero-reflection-filter to top border node
          adaptreflexionfactor(
              data.filtnumcoeffs_top[pos], data.filtcoeffsA_top[pos],
              data.filtcoeffsB_top[pos], 0.f, 270.f, kVertical);
        }
        if (x == config.nX - 1) // right simfield border
        {
          data.filt_right[pos] = true;
          // apply a zero-reflection-filter to right border node
          adaptreflexionfactor(
              data.filtnumcoeffs_right[pos], data.filtcoeffsA_right[pos],
              data.filtcoeffsB_right[pos], 0.f, 0.f, kHorizontal);
        }
        if (y == config.nY - 1) // bottom simfield border
        {
          data.filt_bottom[pos] = true;
          // apply a zero-reflection-filter to bottom border node
          adaptreflexionfactor(
              data.filtnumcoeffs_bottom[pos], data.filtcoeffsA_bottom[pos],
              data.filtcoeffsB_bottom[pos], 0.f, 90.f, kVertical);
        }
      }
      if ((data.envi[pos] >= -1.0) && (data.envi[pos] != 0.0) &&
          (data.envi[pos] <=
           1.0)) { // is the actual node a real-valued-reflecting node?
        data.boundary[pos] = true;
        data.filt_left[pos] = true;
        data.filt_top[pos] = true;
        data.filt_right[pos] = true;
        data.filt_bottom[pos] = true;
        // apply a left filter with correspondig reflection factor to it
        adaptreflexionfactor(data.filtnumcoeffs_left[pos],
                             data.filtcoeffsA_left[pos],
                             data.filtcoeffsB_left[pos], data.envi[pos],
                             data.angle[pos], kHorizontal);
        // apply a top filter with correspondig reflection factor to it
        adaptreflexionfactor(data.filtnumcoeffs_top[pos],
                             data.filtcoeffsA_top[pos],
                             data.filtcoeffsB_top[pos], data.envi[pos],
                             data.angle[pos], kVertical);
        // apply a right filter with correspondig reflection factor to it
        adaptreflexionfactor(data.filtnumcoeffs_right[pos],
                             data.filtcoeffsA_right[pos],
                             data.filtcoeffsB_right[pos], data.envi[pos],
                             data.angle[pos], kHorizontal);
        // apply a bottom filter with correspondig reflection factor to it
        adaptreflexionfactor(data.filtnumcoeffs_bottom[pos],
                             data.filtcoeffsA_bottom[pos],
                             data.filtcoeffsB_bottom[pos], data.envi[pos],
                             data.angle[pos], kVertical);
        if (x <
            config.nX - 1) // apply a left filter to its right neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + 1] = true;
          data.filt_left[pos + 1] = true;
          adaptreflexionfactor(data.filtnumcoeffs_left[pos + 1],
                               data.filtcoeffsA_left[pos + 1],
                               data.filtcoeffsB_left[pos + 1], data.envi[pos],
                               data.angle[pos], kHorizontal);
        }
        if (y <
            config.nY - 1) // apply a top filter to its bottom neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + config.nX] = true;
          data.filt_top[pos + config.nX] = true;
          adaptreflexionfactor(data.filtnumcoeffs_top[pos + config.nX],
                               data.filtcoeffsA_top[pos + config.nX],
                               data.filtcoeffsB_top[pos + config.nX],
                               data.envi[pos], data.angle[pos], kVertical);
        }
        if (x > 0) // apply a right filter to its left neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - 1] = true;
          data.filt_right[pos - 1] = true;
          adaptreflexionfactor(data.filtnumcoeffs_right[pos - 1],
                               data.filtcoeffsA_right[pos - 1],
                               data.filtcoeffsB_right[pos - 1], data.envi[pos],
                               data.angle[pos], kHorizontal);
        }
        if (y > 0) // apply a bottom filter to its top neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - config.nX] = true;
          data.filt_bottom[pos - config.nX] = true;
          adaptreflexionfactor(data.filtnumcoeffs_bottom[pos - config.nX],
                               data.filtcoeffsA_bottom[pos - config.nX],
                               data.filtcoeffsB_bottom[pos - config.nX],
                               data.envi[pos], data.angle[pos], kVertical);
        }
      } else if ((data.envi[pos] > 1.0) &&
                 (data.envi[pos] <=
                  1000.0)) { // is the actual node a filter-node?
        data.boundary[pos] = true;
        data.filt_left[pos] = true;
        data.filt_top[pos] = true;
        data.filt_right[pos] = true;
        data.filt_bottom[pos] = true;
        // apply the left filter with the correspondig ID to it
        adaptfilter(data.filtnumcoeffs_left[pos], data.filtcoeffsA_left[pos],
                    data.filtcoeffsB_left[pos], tmp_filtid, tmp_filtnumcoeffs,
                    tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                    (int)data.envi[pos], data.angle[pos], kHorizontal);
        // apply the top filter with the correspondig ID to it
        adaptfilter(data.filtnumcoeffs_top[pos], data.filtcoeffsA_top[pos],
                    data.filtcoeffsB_top[pos], tmp_filtid, tmp_filtnumcoeffs,
                    tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                    (int)data.envi[pos], data.angle[pos], kVertical);
        // apply the right filter with the correspondig ID to it
        adaptfilter(data.filtnumcoeffs_right[pos], data.filtcoeffsA_right[pos],
                    data.filtcoeffsB_right[pos], tmp_filtid, tmp_filtnumcoeffs,
                    tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
                    (int)data.envi[pos], data.angle[pos], kHorizontal);
        // apply the bottom filter with the correspondig ID to it
        adaptfilter(data.filtnumcoeffs_bottom[pos],
                    data.filtcoeffsA_bottom[pos], data.filtcoeffsB_bottom[pos],
                    tmp_filtid, tmp_filtnumcoeffs, tmp_filtcoeffsA,
                    tmp_filtcoeffsB, tmp_numfilters, (int)data.envi[pos],
                    data.angle[pos], kVertical);
        if (x <
            config.nX - 1) // apply a left filter to its right neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + 1] = true;
          data.filt_left[pos + 1] = true;
          adaptfilter(
              data.filtnumcoeffs_left[pos + 1], data.filtcoeffsA_left[pos + 1],
              data.filtcoeffsB_left[pos + 1], tmp_filtid, tmp_filtnumcoeffs,
              tmp_filtcoeffsA, tmp_filtcoeffsB, tmp_numfilters,
              (int)data.envi[pos], data.angle[pos], kHorizontal);
        }
        if (y <
            config.nY - 1) // apply a top filter to its bottom neighbour, if it
        {                  // isn't outside the simfield
          data.boundary[pos + config.nX] = true;
          data.filt_top[pos + config.nX] = true;
          adaptfilter(data.filtnumcoeffs_top[pos + config.nX],
                      data.filtcoeffsA_top[pos + config.nX],
                      data.filtcoeffsB_top[pos + config.nX], tmp_filtid,
                      tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                      tmp_numfilters, (int)data.envi[pos], data.angle[pos],
                      kVertical);
        }
        if (x > 0) // apply a right filter to its left neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - 1] = true;
          data.filt_right[pos - 1] = true;
          adaptfilter(data.filtnumcoeffs_right[pos - 1],
                      data.filtcoeffsA_right[pos - 1],
                      data.filtcoeffsB_right[pos - 1], tmp_filtid,
                      tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                      tmp_numfilters, (int)data.envi[pos], data.angle[pos],
                      kHorizontal);
        }
        if (y > 0) // apply a bottom filter to its top neighbour, if it
        {          // isn't outside the simfield
          data.boundary[pos - config.nX] = true;
          data.filt_bottom[pos - config.nX] = true;
          adaptfilter(data.filtnumcoeffs_bottom[pos - config.nX],
                      data.filtcoeffsA_bottom[pos - config.nX],
                      data.filtcoeffsB_bottom[pos - config.nX], tmp_filtid,
                      tmp_filtnumcoeffs, tmp_filtcoeffsA, tmp_filtcoeffsB,
                      tmp_numfilters, (int)data.envi[pos], data.angle[pos],
                      kVertical);
        }
      } else if (data.envi[pos] < -1.0) // is the actual node a receiver-node?
      {
        if (actrec < config.nRec) {
          data.recIdx[actrec] =
              pos;  // yes, add the receiver's position into the
          actrec++; // recIdx-Array
        }
      }
    } // x-loop
  }   // y-loop

  // read samples
  if (!donotreadnextblockid) // read the chunk header if it is required (see
                             // above)
    simFile.read(pblockid, sizeof(char) * 3);
  set("nSamples", 0);
  simSample *sample = NULL;
  if (strncmp(pblockid, "SMP", 3) == 0) // is it a SMP-chuck
  {
    cout << "parsing SMP\n";
    pdummy = new double;
    simFile.read((char *)pdummy, sizeof(double));
    set("nSamples", (int)*pdummy);
    delete[] pdummy;
    cout << "found " << config.nSamples << " sources\n";
    data.samples = new_simSample_array(config.nSamples);
    for (int n = 0; n < config.nSamples; n++) // read all the samples
    {
      cout << "reading source " << n << "\n";
      pdummy = new double;
      sample = data.samples[n];
      simFile.read((char *)pdummy, sizeof(double)); // read sample ID
      sample->id = (int)*pdummy;
      cout << "IDX: " << sample->id << "\n";
      simFile.read((char *)pdummy, sizeof(double)); // read sample SR
      sample->sr = (int)*pdummy;
      cout << "SR: " << sample->sr << "\n";
      simFile.read((char *)pdummy, sizeof(double)); // read numsamples
      sample->nsamples = (int)*pdummy;
      cout << "nsamples: " << sample->nsamples << "\n";
      sample->data = new float[sample->nsamples]; // allocate memory
      delete[] pdummy;

      pdummy = new double[sample->nsamples];
      cout << "reading sample data\n";
      simFile.read((char *)pdummy, sizeof(double) * sample->nsamples);
      cout << "finished reading sample data\n";
      for (int pos = 0; pos < sample->nsamples; pos++) // convert it to float
      {
        sample->data[pos] = (float)pdummy[pos];
      }
      delete[] pdummy;
      cout << "\nfinished reading samples\n";
    }
    cout << "finished reading all samples\n";
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
    cout << "reading block ID\n";
    simFile.read(pblockid, sizeof(char) * 3);
    cout << "found: " << pblockid << "\n";
  } else
    cout << "skipping reading blockid for SRC chunk\n";
  if (strncmp(pblockid, "SRC", 3) == 0) // is it a SRC-chunk?
  {
    cout << "parsing SRC chunk\n";
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
        cout << "sample source: IDX=" << sample->id << " SR=" << sample->sr
             << " NSAMPLES=" << sample->nsamples
             << " DURATION(ms)=" << (sample->nsamples * 1000) / sample->sr
             << "\n";
      }
      defineSource(n, &curSource); // and add the source to the simulation
      cout << "finished defining source of type " << curSource.type << "\n";
    }
    delete pdummy;
  } else // no SRC-chunk found --> delete all our work we've done so far
  {
    cout << "no SRC-chunk found, found instead " << pblockid << "\n";
    cout << "no SRC-chunk found --> delete all our work we've done so far\n";
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
            data.filt_left[y * config.nX + x] =
                false; // yes, disable left filter
        if (y > 0)
          if (isvelosource[(y - 1) * config.nX +
                           x]) // is top neighbour a velo src?
            data.filt_top[y * config.nX + x] = false; // yes, disable top filter
        if (x < config.nX - 1)
          if (isvelosource[y * config.nX + x +
                           1]) // is right neighbour a velo src?
            data.filt_right[y * config.nX + x] =
                false; // yes, disable right filter
        if (y < config.nY - 1)
          if (isvelosource[(y + 1) * config.nX +
                           x]) // is bottom neighbour a velo src?
            data.filt_bottom[y * config.nX + x] =
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
    if ((data.filt_left[pos]) && (data.filt_top[pos]) &&
        (data.filt_right[pos]) && (data.filt_bottom[pos])) {
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
  set("status", SIMULATOR);      // and now we're ready to go
  return NONE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::loadRecord
//
// PURPOSE
//   Tries to load a recorded playback file.
//
// INPUT
//   const string fileName : name of the file to be opened.
//
// OUTPUT
//   None
//
// RETURN VALUE
//   simError: NONE if file was opened successfully, error identfier otherwise.
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland 	no changes
//05/09	2.0
//
simError lambda::loadRecord(const string fileName) {
  struct stat results; // File info
  // Try to get file info, return error if file could not be opened
  if (stat((char *)fileName.c_str(), &results) != 0)
    return FILE_BAD;
  // Each record file should at least contain 2 doubles: nY and nX.
  if ((int)results.st_size <= 2 * (int)sizeof(double))
    return FILE_SIZE_BAD;
  // Set status to mismatch before changing anything
  set("status", MISMATCH);
  resetAll();
  // open recfile and create pointer
  ifstream recFile((char *)fileName.c_str(), ios::in | ios::binary);
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
  set("status", PLAYER);
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

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::initSimulation
//
// PURPOSE
//   Prepares variables needed for simulation.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   simError: NONE if no error occured, error identfier otherwise.
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland 	mainly rewritten; added code for
//05/09	2.0
//              allocating and resetting the filter's
//              memories and the velocity source arrays
//
simError lambda::initSimulation() {
  // Check one more time
  if (config.nNodes < 1)
    return NO_NODES;

  config.n = 0;
  // reserve memory for node pressure and incident pressure pulses
  data.pres = new float[3 * config.nNodes];
  for (int pos = 0; pos < 3 * config.nNodes; pos++) {
    data.pres[pos] = 0;
  }
  data.inci = new float[12 * config.nNodes];
  for (int pos = 0; pos < 12 * config.nNodes; pos++) {
    data.inci[pos] = 0;
  }
  // set up indices needed for the simulation
  for (int x = 0; x < 3; x++) {
    index.idxP[x] = data.pres + x * config.nNodes;
    index.idxILeft[x] = data.inci + (x * 4 + 0) * config.nNodes;
    index.idxITop[x] = data.inci + (x * 4 + 1) * config.nNodes;
    index.idxIRight[x] = data.inci + (x * 4 + 2) * config.nNodes;
    index.idxIBottom[x] = data.inci + (x * 4 + 3) * config.nNodes;
  }
  // reserve memory for the filter memories
  // data.oldx_left=new float*[config.nNodes];
  data.oldx_left = new_array2(config.nNodes);
  // data.oldx_top=new float*[config.nNodes];
  data.oldx_top = new_array2(config.nNodes);
  // data.oldx_right=new float*[config.nNodes];
  data.oldx_right = new_array2(config.nNodes);
  // data.oldx_bottom=new float*[config.nNodes];
  data.oldx_bottom = new_array2(config.nNodes);
  // data.oldy_left=new float*[config.nNodes];
  data.oldy_left = new_array2(config.nNodes);
  // data.oldy_top=new float*[config.nNodes];
  data.oldy_top = new_array2(config.nNodes);
  // data.oldy_right=new float*[config.nNodes];
  data.oldy_right = new_array2(config.nNodes);
  // data.oldy_bottom=new float*[config.nNodes];
  data.oldy_bottom = new_array2(config.nNodes);
  for (int pos = 0; pos < config.nNodes; pos++) {
    if (data.filtnumcoeffs_left[pos] >= 1) {
      // reserve+initialize recursive and non-recursive memory for left filters
      int memorycnt = data.filtnumcoeffs_left[pos] - 1;
      if (memorycnt == 0) // to ensure that even 0th order filters have
        memorycnt = 1; // memory; this spares an if-condition in the algorithm
      data.oldx_left[pos] = new float[memorycnt];
      data.oldy_left[pos] = new float[memorycnt];
      for (int k = 0; k < memorycnt; k++) {
        data.oldx_left[pos][k] = 0.f;
        data.oldy_left[pos][k] = 0.f;
      }
    }
    if (data.filtnumcoeffs_top[pos] >= 1) {
      // reserve+initialize recursive and non-recursive memory for top filters
      int memorycnt = data.filtnumcoeffs_top[pos] - 1;
      if (memorycnt == 0) // to ensure that even 0th order filters have
        memorycnt = 1; // memory; this spares an if-condition in the algorithm
      data.oldx_top[pos] = new float[memorycnt];
      data.oldy_top[pos] = new float[memorycnt];
      for (int k = 0; k < memorycnt; k++) {
        data.oldx_top[pos][k] = 0.f;
        data.oldy_top[pos][k] = 0.f;
      }
    }
    if (data.filtnumcoeffs_right[pos] >= 1) {
      // reserve+initialize recursive and non-recursive memory for right filters
      int memorycnt = data.filtnumcoeffs_right[pos] - 1;
      if (memorycnt == 0) // to ensure that even 0th order filters have
        memorycnt = 1; // memory; this spares an if-condition in the algorithm
      data.oldx_right[pos] = new float[memorycnt];
      data.oldy_right[pos] = new float[memorycnt];
      for (int k = 0; k < memorycnt; k++) {
        data.oldx_right[pos][k] = 0.f;
        data.oldy_right[pos][k] = 0.f;
      }
    }
    if (data.filtnumcoeffs_bottom[pos] >= 1) {
      // reserve+initialize recursive and non-recursive memory for bottom
      // filters
      int memorycnt = data.filtnumcoeffs_bottom[pos] - 1;
      if (memorycnt == 0) // to ensure that even 0th order filters have
        memorycnt = 1; // memory; this spares an if-condition in the algorithm
      data.oldx_bottom[pos] = new float[memorycnt];
      data.oldy_bottom[pos] = new float[memorycnt];
      for (int k = 0; k < memorycnt; k++) {
        data.oldx_bottom[pos][k] = 0.f;
        data.oldy_bottom[pos][k] = 0.f;
      }
    }
  }

  // reserve+initialize memory for velocity sources
  data.velo_left = new float[config.nNodes];
  data.velo_top = new float[config.nNodes];
  data.velo_right = new float[config.nNodes];
  data.velo_bottom = new float[config.nNodes];

  for (int pos = 0; pos < config.nNodes; pos++) {
    data.velo_left[pos] = 0.f;
    data.velo_top[pos] = 0.f;
    data.velo_right[pos] = 0.f;
    data.velo_bottom[pos] = 0.f;
  }

  return NONE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::processRep
//
// PURPOSE
//   Processes the next replay frame.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	irst build
//05/06	1.0
//  M. Ruhland  changed copyright line                      04/08
//	M. Ruhland 	no changes
//05/09	2.0
//
void lambda::processRep() {
  // Skip this frame if demanded
  if (config.n % graphics.skip == 0) {
    processFrame(graphics.frame, data.record + config.n * config.nNodes);
    // Display frame on screen
    graphics.screen->display(*graphics.frame);
  }
  config.n++;
  // if (graphics.screen->is_closed() || config.n >= config.nN)
  //   stopButton->click();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::processSim
//
// PURPOSE
//   Processes the next simulation iteration.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//05/06	1.0 	M. Ruhland 	mainly rewritten. Completely new
//05/09	2.0
//              simulation code for calculating digital
//              filters and velocity sources at a time.
//              Former pointer optimization not needed
//              any longer, GDB does this for us now.
//              Added new source types. Added progress
//              indicator. Former walls removed. All
//              walls are now treated as filters. Added
//              AUTOEXIT-functionality.
//

void lambda::processSim() {
  if (data.pres != NULL) {
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
    float *index_presFutu = index.presFutu;
    float *presPres = index.presPres;

    index.inciPastLeft =
        index.idxILeft[idxPast]; // past left incident pressure index
    index.inciPastTop =
        index.idxITop[idxPast]; // past top incident pressure index
    index.inciPastRight =
        index.idxIRight[idxPast]; // past right incident pressure index
    index.inciPastBottom =
        index.idxIBottom[idxPast]; // past bottom incident pressure index

    index.inciPresLeft =
        index.idxILeft[idxPres]; // present left incident pressure index
    index.inciPresTop =
        index.idxITop[idxPres]; // present top incident pressure index
    index.inciPresRight =
        index.idxIRight[idxPres]; // present right incident pressure index
    index.inciPresBottom =
        index.idxIBottom[idxPres]; // present bottom incident pressure index

    index.inciFutuLeft =
        index.idxILeft[idxFutu]; // future left incident pressure index
    index.inciFutuTop =
        index.idxITop[idxFutu]; // future top incident pressure index
    index.inciFutuRight =
        index.idxIRight[idxFutu]; // future right incident pressure index
    index.inciFutuBottom =
        index.idxIBottom[idxFutu]; // future bottom incidient pressure index

    float scatFutuLeft, scatPresLeft; // future+present left scattered pressure
    float scatFutuTop, scatPresTop;   // future+present top scattered pressure
    float scatFutuRight,
        scatPresRight; // future+present right scattered pressure
    float scatFutuBottom,
        scatPresBottom; // future+present bottom scattered pressure

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
      float alpha = data.angle[srcxy]; // get actual source angle of incidence
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
              (float)cos(3.1415926f * (t + T / 2.f) / T); // compute hann window
          hann *= hann;
        }
        presPres[srcxy] +=
            hann * amp * (float)sin(twopi_freq * t + onepi_phi_180);
        break;
      case 6: // sinusoidal velocity source
        magnitude = config.rho * config.cTube * amp *
                    sin(twopi_freq * t + onepi_phi_180);
        if ((alpha >= 0.f) && (alpha < 90.f)) {
          // alpha between 0 and 90 degrees? -> left and top incidence
          tmp0 = alpha * (3.1415926f / 180.f);
          data.velo_left[srcxy] = cos(tmp0) * magnitude;
          data.velo_top[srcxy] = sin(tmp0) * magnitude;
        } else if ((alpha >= 90.f) && (alpha < 180.f)) {
          // alpha between 90 and 180 degrees? -> top and right incidence
          tmp0 = (alpha - 90.f) * (3.1415926f / 180.f);
          data.velo_top[srcxy] = cos(tmp0) * magnitude;
          data.velo_right[srcxy] = sin(tmp0) * magnitude;
        } else if ((alpha >= 180.f) && (alpha < 270.f)) {
          // alpha between 180 and 270 degrees? -> right and bottom incidence
          tmp0 = (alpha - 180.f) * (3.1415926f / 180.f);
          data.velo_right[srcxy] = cos(tmp0) * magnitude;
          data.velo_bottom[srcxy] = sin(tmp0) * magnitude;
        } else if ((alpha >= 270.f) && (alpha < 360.f)) {
          // alpha between 270 and 360 degrees? -> bottom and left incidence
          tmp0 = (alpha - 270.f) * (3.1415926f / 180.f);
          data.velo_bottom[srcxy] = cos(tmp0) * magnitude;
          data.velo_left[srcxy] = sin(tmp0) * magnitude;
        }
        break;
      case 7: // rectangular velocity source
        if ((int)(2.f * (freq * t + phi / 360.f)) % 2 == 0)
          magnitude = config.rho * config.cTube * amp;
        else
          magnitude = -config.rho * config.cTube * amp;
        if ((alpha >= 0.f) && (alpha < 90.f)) {
          // alpha between 0 and 90 degrees? -> left and top incidence
          tmp0 = alpha * (3.1415926f / 180.f);
          data.velo_left[srcxy] = cos(tmp0) * magnitude;
          data.velo_top[srcxy] = sin(tmp0) * magnitude;
        } else if ((alpha >= 90.f) && (alpha < 180.f)) {
          // alpha between 90 and 180 degrees? -> top and right incidence
          tmp0 = (alpha - 90.f) * (3.1415926f / 180.f);
          data.velo_top[srcxy] = cos(tmp0) * magnitude;
          data.velo_right[srcxy] = sin(tmp0) * magnitude;
        } else if ((alpha >= 180.f) && (alpha < 270.f)) {
          // alpha between 180 and 270 degrees? -> right and bottom incidence
          tmp0 = (alpha - 180.f) * (3.1415926f / 180.f);
          data.velo_right[srcxy] = cos(tmp0) * magnitude;
          data.velo_bottom[srcxy] = sin(tmp0) * magnitude;
        } else if ((alpha >= 270.f) && (alpha < 360.f)) {
          // alpha between 270 and 360 degrees? -> bottom and left incidence
          tmp0 = (alpha - 270.f) * (3.1415926f / 180.f);
          data.velo_bottom[srcxy] = cos(tmp0) * magnitude;
          data.velo_left[srcxy] = sin(tmp0) * magnitude;
        }
        break;
      case 8: // delta-pulse velocity source
        if (config.n == 0)
          magnitude = config.rho * config.cTube * amp;
        if ((alpha >= 0.f) && (alpha < 90.f)) {
          // alpha between 0 and 90 degrees? -> left and top incidence
          data.velo_left[srcxy] = cos(alpha * (3.1415926f / 180.f)) * magnitude;
          data.velo_top[srcxy] = sin(alpha * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 90.f) && (alpha < 180.f)) {
          // alpha between 90 and 180 degrees? -> top and right incidence
          data.velo_top[srcxy] =
              cos((alpha - 90.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_right[srcxy] =
              sin((alpha - 90.f) * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 180.f) && (alpha < 270.f)) {
          // alpha between 180 and 270 degrees? -> right and bottom incidence
          data.velo_right[srcxy] =
              cos((alpha - 180.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_bottom[srcxy] =
              sin((alpha - 180.f) * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 270.f) && (alpha < 360.f)) {
          // alpha between 270 and 360 degrees? -> bottom and left incidence
          data.velo_bottom[srcxy] =
              cos((alpha - 270.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_left[srcxy] =
              sin((alpha - 270.f) * (3.1415926f / 180.f)) * magnitude;
        }
        break;
      case 9: // exponential decay velocity source (not working correctly
              // yet!!!)
        magnitude = config.rho * config.cTube * amp * exp(-(float)config.n);
        if ((alpha >= 0.f) && (alpha < 90.f)) {
          // alpha between 0 and 90 degrees? -> left and top incidence
          data.velo_left[srcxy] = cos(alpha * (3.1415926f / 180.f)) * magnitude;
          data.velo_top[srcxy] = sin(alpha * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 90.f) && (alpha < 180.f)) {
          // alpha between 90 and 180 degrees? -> top and right incidence
          data.velo_top[srcxy] =
              cos((alpha - 90.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_right[srcxy] =
              sin((alpha - 90.f) * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 180.f) && (alpha < 270.f)) {
          // alpha between 180 and 270 degrees? -> right and bottom incidence
          data.velo_right[srcxy] =
              cos((alpha - 180.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_bottom[srcxy] =
              sin((alpha - 180.f) * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 270.f) && (alpha < 360.f)) {
          // alpha between 270 and 360 degrees? -> bottom and left incidence
          data.velo_bottom[srcxy] =
              cos((alpha - 270.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_left[srcxy] =
              sin((alpha - 270.f) * (3.1415926f / 180.f)) * magnitude;
        }
        break;
      case 10: // hann-windowed sinusoidal velocity source
        if (t < 0.f)
          hann = 0.f;
        else if (t > (T / 2.f))
          hann = 1.f;
        else {
          hann =
              (float)cos(3.1415926f * (t + T / 2.f) / T); // compute hann window
          hann *= hann;
        }
        magnitude = config.rho * config.cTube * hann * amp *
                    (float)sin(twopi * freq * t + 3.1415926f * phi / 180.f);
        if ((alpha >= 0.f) && (alpha < 90.f)) {
          // alpha between 0 and 90 degrees? -> left and top incidence
          data.velo_left[srcxy] = cos(alpha * (3.1415926f / 180.f)) * magnitude;
          data.velo_top[srcxy] = sin(alpha * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 90.f) && (alpha < 180.f)) {
          // alpha between 90 and 180 degrees? -> top and right incidence
          data.velo_top[srcxy] =
              cos((alpha - 90.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_right[srcxy] =
              sin((alpha - 90.f) * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 180.f) && (alpha < 270.f)) {
          // alpha between 180 and 270 degrees? -> right and bottom incidence
          data.velo_right[srcxy] =
              cos((alpha - 180.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_bottom[srcxy] =
              sin((alpha - 180.f) * (3.1415926f / 180.f)) * magnitude;
        } else if ((alpha >= 270.f) && (alpha < 360.f)) {
          // alpha between 270 and 360 degrees? -> bottom and left incidence
          data.velo_bottom[srcxy] =
              cos((alpha - 270.f) * (3.1415926f / 180.f)) * magnitude;
          data.velo_left[srcxy] =
              sin((alpha - 270.f) * (3.1415926f / 180.f)) * magnitude;
        }
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

    int n;    // counter variable
    float yn; // filter output
    int config_nX = config.nX;
    // Work through all the nodes in the environment
    for (int pos = 0; pos < config.nNodes; pos++) {
      index_presFutu[pos] = 0.f;
      if (!data.deadnode[pos]) // deadnode? --> no calculation needed!
      {
        if (data.boundary[pos]) // boundary? --> no standard propagation!
        {
          if (data.filt_left[pos]) // left filter
          {
            // calculate filter input
            scatFutuLeft = presPres[pos] - index.inciPresLeft[pos];
            // calculate the digital filter
            yn = scatFutuLeft * data.filtcoeffsB_left[pos][0];
            for (n = 1; n < data.filtnumcoeffs_left[pos]; n++) {
              yn += data.oldx_left[pos][n - 1] * data.filtcoeffsB_left[pos][n];
              yn -= data.oldy_left[pos][n - 1] * data.filtcoeffsA_left[pos][n];
            }
            // add magnitude of a possible velocity source
            yn += data.velo_left[pos];
            // rotate the filter memories
            for (n = data.filtnumcoeffs_left[pos] - 2; n > 0; n--) {
              data.oldx_left[pos][n] = data.oldx_left[pos][n - 1];
              data.oldy_left[pos][n] = data.oldy_left[pos][n - 1];
            }
            data.oldx_left[pos][0] = scatFutuLeft;
            data.oldy_left[pos][0] = yn;
            // and write the filter output into the pressure matrix
            index.inciFutuLeft[pos] = yn;
            index_presFutu[pos] += index.inciFutuLeft[pos];
          } else // no left filter
          {
            scatPresLeft = index.presPast[pos] - index.inciPastLeft[pos];
            index.inciFutuLeft[pos] = presPres[pos - 1] - scatPresLeft;
            index_presFutu[pos] += index.inciFutuLeft[pos];
          }
          if (data.filt_top[pos]) // top filter
          {
            // calculate filter input
            scatFutuTop = presPres[pos] - index.inciPresTop[pos];
            // calculate the digital filter
            yn = scatFutuTop * data.filtcoeffsB_top[pos][0];
            for (n = 1; n < data.filtnumcoeffs_top[pos]; n++) {
              yn += data.oldx_top[pos][n - 1] * data.filtcoeffsB_top[pos][n];
              yn -= data.oldy_top[pos][n - 1] * data.filtcoeffsA_top[pos][n];
            }
            // add magnitude of a possible velocity source
            yn += data.velo_top[pos];
            // rotate the filter memories
            for (n = data.filtnumcoeffs_top[pos] - 2; n > 0; n--) {
              data.oldx_top[pos][n] = data.oldx_top[pos][n - 1];
              data.oldy_top[pos][n] = data.oldy_top[pos][n - 1];
            }
            data.oldx_top[pos][0] = scatFutuTop;
            data.oldy_top[pos][0] = yn;
            // and write the filter output into the pressure matrix
            index.inciFutuTop[pos] = yn;
            index_presFutu[pos] += index.inciFutuTop[pos];
          } else // no top filter
          {
            scatPresTop = index.presPast[pos] - index.inciPastTop[pos];
            index.inciFutuTop[pos] = presPres[pos - config_nX] - scatPresTop;
            index_presFutu[pos] += index.inciFutuTop[pos];
          }
          if (data.filt_right[pos]) // right filter
          {
            // calculate filter input
            scatFutuRight = presPres[pos] - index.inciPresRight[pos];
            // calculate the digital filter
            yn = scatFutuRight * data.filtcoeffsB_right[pos][0];
            for (n = 1; n < data.filtnumcoeffs_right[pos]; n++) {
              yn +=
                  data.oldx_right[pos][n - 1] * data.filtcoeffsB_right[pos][n];
              yn -=
                  data.oldy_right[pos][n - 1] * data.filtcoeffsA_right[pos][n];
            }
            // add magnitude of a possible velocity source
            yn += data.velo_right[pos];
            // rotate the filter memories
            for (n = data.filtnumcoeffs_right[pos] - 2; n > 0; n--) {
              data.oldx_right[pos][n] = data.oldx_right[pos][n - 1];
              data.oldy_right[pos][n] = data.oldy_right[pos][n - 1];
            }
            data.oldx_right[pos][0] = scatFutuRight;
            data.oldy_right[pos][0] = yn;
            // and write the filter output into the pressure matrix
            index.inciFutuRight[pos] = yn;
            index_presFutu[pos] += index.inciFutuRight[pos];
          } else // no right filter
          {
            scatPresRight = index.presPast[pos] - index.inciPastRight[pos];
            index.inciFutuRight[pos] = presPres[pos + 1] - scatPresRight;
            index_presFutu[pos] += index.inciFutuRight[pos];
          }
          if (data.filt_bottom[pos]) // bottom filter
          {
            // calculate filter input
            scatFutuBottom = presPres[pos] - index.inciPresBottom[pos];
            // calculate the digital filter
            yn = scatFutuBottom * data.filtcoeffsB_bottom[pos][0];
            for (n = 1; n < data.filtnumcoeffs_bottom[pos]; n++) {
              yn += data.oldx_bottom[pos][n - 1] *
                    data.filtcoeffsB_bottom[pos][n];
              yn -= data.oldy_bottom[pos][n - 1] *
                    data.filtcoeffsA_bottom[pos][n];
            }
            // add magnitude of a possible velocity source
            yn += data.velo_bottom[pos];
            // rotate the filter memories
            for (n = data.filtnumcoeffs_bottom[pos] - 2; n > 0; n--) {
              data.oldx_bottom[pos][n] = data.oldx_bottom[pos][n - 1];
              data.oldy_bottom[pos][n] = data.oldy_bottom[pos][n - 1];
            }
            data.oldx_bottom[pos][0] = scatFutuBottom;
            data.oldy_bottom[pos][0] = yn;
            // and write the filter output into the pressure matrix
            index.inciFutuBottom[pos] = yn;
            index_presFutu[pos] += index.inciFutuBottom[pos];
          } else // no bottom filter
          {
            scatPresBottom = index.presPast[pos] - index.inciPastBottom[pos];
            index.inciFutuBottom[pos] =
                presPres[pos + config_nX] - scatPresBottom;
            index_presFutu[pos] += index.inciFutuBottom[pos];
          }
          index_presFutu[pos] *= 0.5f;
        } else // no boundary node: do the fast standard propagation
        {
          index_presFutu[pos] =
              (presPres[pos - 1] + presPres[pos - config_nX] +
               presPres[pos + 1] + presPres[pos + config_nX]) *
                  0.5f -
              index.presPast[pos];
        }
      }
    }
    graphics.frame_ready = false;
    // Process actions like rce, rco or vis if required
    #if 0
    if (rceBox->isChecked())
      processRce();
    if (rcoBox->isChecked())
      if (config.n % graphics.skip == 0)
        processRco();
    if (visBox->isChecked())
      if (config.n % graphics.skip == 0)
        processVis();
    #endif
    // update the progress indicator
    time_t t1;
    static time_t t0;
    static size_t lastn;
    char buf[50];
    time(&t1);
    double dif = difftime(t1, t0);
    if (dif > 0.25) {
      t0 = t1;
      int ms = config.n * config.tSample * 1E3;
      sprintf(buf, "ms:%d step:%d fps:%lu", ms, config.n, (config.n - lastn));
      lastn = config.n;
      // gui.statusLine->setText(buf);
    }

    // update counter. Stop simulation if desired nr. of iterations is reached.
    config.n++;
    if ((config.n >= config.nN) && (config.nN != 0)) {
      // if AUTOEXIT is set, terminate lambda by self-clicking the quit-button
      // TODO(lucasw)
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::drawLambda
//
// PURPOSE
//   Draws the lambda logo on the visualization screen.
//
// INPUT
//   None
//
// OUTPUT
//   None
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	S. Ahrens 	First build
//08/06	1.0
//  M. Ruhland  fixed some warnings                         04/08   1.1
//  M. Ruhland  changed copyright line                      04/08
//	M. Ruhland 	no changes
//05/09	2.0
//
void lambda::drawLambda() {
  int size, offsetX, offsetY;
  float radius;
  float value1 = 140.f;
  float value2 = 127.f;
  graphics.frame->fill(127.f);

  if (config.nX < config.nY) {
    size = config.nX;
    offsetX = 0;
    offsetY = (int)(config.nY / 2) - (int)(size / 2);
  } else {
    size = config.nY;
    offsetY = 0;
    offsetX = (int)(config.nX / 2) - (int)(size / 2);
  }
  radius = (int)(size / 30);

  graphics.frame->draw_circle((int)(size / 2) + offsetX,
                              (int)(size / 2) + offsetY,
                              (int)(size / 2) - radius, &value1, 0, 1.f);
  graphics.frame->draw_circle((int)(size / 2) + offsetX,
                              (int)(size / 2) + offsetY,
                              (int)(size / 2) - 2 * radius, &value2, 0, 1.f);

  int m = 4 * (int)radius;
  for (int n = 8 * (int)radius; m < size - 4 * radius; n++) {
    if (n > 10 * radius) {
      graphics.frame->draw_circle(n + 2 * (int)radius + offsetX,
                                  m - (int)(3 * radius) + offsetY, radius,
                                  &value1, 0, 1.f);
      graphics.frame->draw_circle(n + 2 * (int)radius + offsetX,
                                  m + 1 - (int)(3 * radius) + offsetY, radius,
                                  &value1, 0, 1.f);
    }
    if (m < (int)(size / 2)) {
      graphics.frame->draw_circle(n + 2 * (int)radius + offsetX,
                                  size - m - (int)(3 * radius) + offsetY,
                                  radius, &value1, 0, 1.f);
      graphics.frame->draw_circle(n + 2 * (int)radius + offsetX,
                                  size - m - 1 - (int)(3 * radius) + offsetY,
                                  radius, &value1, 0, 1.f);
    }
    m += 2;
  }
  // graphics.frame->draw_text(config.nX-242,config.nY-12,graphics.colors.white,graphics.colors.grey,0.5,"M.Ruhland,
  // M.Blau, and others; IHA Oldenburg");
  float *pixbuf;
  // if (gui.showboundsBox->isChecked())
  {
    pixbuf = graphics.frame->data();
    for (int n = 0; n < config.nNodes; n++)
      // if (abs(data.envi[n])!=0.f) graphics.frame->data[n]=255;
      if (abs(data.envi[n]) != 0.f)
        pixbuf[n] = 255;
  }

  // TODO(lucasw) return graphics to caller
  // graphics.screen->display(*graphics.frame);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::adaptreflexionfactor
//
// PURPOSE
//   Creates a new digital filter for a given real-valued reflexion factor and
//   preemphases the filter coefficients due to a given sonic incidence angle
//   and horizontal/vertical alignment of the filter.
//
// INPUT
//   float r    	: desired real-valued reflexion factor
//   float alpha	: angle of incidence for the desired filter, needed for
//   preemphasis simAngularType direction : sets whether the filter is used in
//   horizontal
//                              or vertical tubes, needed for preemphasis
//
// OUTPUT
//   int& dest_numcoeffs    : number of filter coefficients for the calculated
//   filter float*& dest_coeffsA   : array containig the new computed
//   a-Filter-coefficients float*& dest_coeffsB   : array containig the new
//   computed b-Filter-coefficients
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	M. Ruhland 	new function
//05/09	2.0
//
void lambda::adaptreflexionfactor(int &dest_numcoeffs, float *&dest_coeffsA,
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
        (a + b) + sqrt(2.f) * abs(cos(alpha * (3.1415926f / 180.f))) * (a - b);
    bnew =
        (a + b) - sqrt(2.f) * abs(cos(alpha * (3.1415926f / 180.f))) * (a - b);
  } else if (direction == kVertical) {
    // do vertical preemphasis
    anew =
        (a + b) + sqrt(2.f) * abs(sin(alpha * (3.1415926f / 180.f))) * (a - b);
    bnew =
        (a + b) - sqrt(2.f) * abs(sin(alpha * (3.1415926f / 180.f))) * (a - b);
  } else {
    // no preemphasis
    anew = a;
    bnew = b;
  }
  // normalize the filter coefficients
  dest_coeffsA[0] = 1.f;
  dest_coeffsB[0] = bnew / anew;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// lambda::adaptfilter
//
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
//   horizontal
//                              or vertical tubes, needed for preemphasis
//
// OUTPUT
//   int& dest_numcoeffs    : number of filter coefficients for the calculated
//   filter float*& dest_coeffsA   : array containig the new computed
//   a-Filter-coefficients float*& dest_coeffsB   : array containig the new
//   computed b-Filter-coefficients
//
// RETURN VALUE
//   None
//
//	AUTHOR		CHANGES
//DATE	VERSION 	M. Ruhland 	new function
//05/09	2.0
//
void lambda::adaptfilter(int &dest_numcoeffs, float *&dest_coeffsA,
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
      anew = (a + b) +
             sqrt(2.f) * abs(cos(alpha * (3.1415926f / 180.f))) * (a - b);
      bnew = (a + b) -
             sqrt(2.f) * abs(cos(alpha * (3.1415926f / 180.f))) * (a - b);
    } else if (direction == kVertical) {
      // do vertical preemphasis
      anew = (a + b) +
             sqrt(2.f) * abs(sin(alpha * (3.1415926f / 180.f))) * (a - b);
      bnew = (a + b) -
             sqrt(2.f) * abs(sin(alpha * (3.1415926f / 180.f))) * (a - b);
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
