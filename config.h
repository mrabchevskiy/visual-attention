                                                                                                                              /*
  Configuration parameters for whole CoreAGI project.
  Parameters includes constexpr constants, type aliases and
  some related things tightly coupled logically with type
  aliases.

  Configuration should not refer (directly or indirectly) logger
  and headers/classes except ones ( like `Set` in set.h ) that
  defines unification interface (wrappers) for different underlying
  containers.

  Each configurable class has own configuration namespace enclosed into
  `CoreAGI::Config` namespace. Configuration parameters that related to
  whole project located in `CoreAGI` namespace.

  ____________________

  2020.04.30  Initial version

  2020.05.21  Config::terminal added

  2021.02.13  Key definition moved from `data.h`

  2021/02.20  Definitions of Identity, Key etc moved to `def.h`

________________________________________________________________________________________________________________________________
                                                                                                                              */
#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "def.h"

namespace CoreAGI::Config {

  namespace camera {
  
    constexpr int FRAME_MAX_WIDTH   { 1920 };
    constexpr int FRAME_MAX_HEIGHT  { 1080 };
    
  };

  namespace visualattention {

    constexpr unsigned CAMERA_BUFFERS {    2 }; // :number of buffer of video driver
    constexpr unsigned NUMBER_OF_FAGS {    4 }; // :number of cell processing working threads

  };

};// namespace

#endif // CONFIG_H_INCLUDED
