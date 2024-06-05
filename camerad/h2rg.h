/**
 * @file    h2rg.h
 * @brief   
 * @details 
 * @author  David Hale <dhale@astro.caltech.edu>
 *
 */
#ifndef H2RG_H
#define H2RG_H

#include <CCfits/CCfits>           //!< needed here for types in set_axes()
#include <atomic>
#include <chrono>
#include <numeric>
#include <fenv.h>

#include "utilities.h"
#include "common.h"
#include "camera.h"
#include "config.h"
#include "logentry.h"
#include "network.h"
#include "fits.h"

namespace Archon {

  class Interface {

    public:

      bool is_window;                        //!< true if in window mode for h2rg, false if not

      int win_hstart;
      int win_hstop;
      int win_vstart;
      int win_vstop;

      int taplines_store;                   //!< int number of original taplines
      std::string tapline0_store;           //!< store tapline0 for window mode so can restore later

      // Functions
      //
      long hread_frame();
      long hexpose(std::string nseq_in);
      long hsetup();
      long hroi(std::string geom_in, std::string &retstring);
      long hwindow(std::string state_in, std::string &state_out);
      long video();
      long hwait_for_readout();
      long nlines(std::string count, std::string &retstring);

  };

}

#endif
