/**
 * @file    archon.cpp
 * @brief   camera interface functions
 * @details 
 * @author  David Hale <dhale@astro.caltech.edu>
 *
 */
#include "h2rg.h"

#include <sstream>   // for std::stringstream
#include <iomanip>   // for setfil, setw, etc.
#include <iostream>  // for hex, uppercase, etc.
#include <algorithm> 
#include <cctype>
#include <string>
#include <fstream>
#include <array>
#include <utility>

namespace Archon {

  // Archon::Interface constructor
  //
  Interface::Interface() {
    this->is_window = false;
    this->win_hstart = 0;
    this->win_hstop = 2047;
    this->win_vstart = 0;
    this->win_vstop = 2047;
    this->tapline0_store = "";
    this->taplines_store = 0;

  }

    /**************** Archon::Interface::nlines ********************************/
    /**
     * @fn     nlines
     * @brief  set/get the number of lines
     * @param  string
     * @return ERROR or NO_ERROR
     *
     * This function calls "set_parameter()" and "get_parameter()" using
     * the "exptime" parameter (which must already be defined in the ACF file).
     *
     */
    long Interface::nlines(std::string nlines_in, std::string &retstring) {
        std::string function = "Archon::Interface::nlines";
        std::stringstream message;
        long error=NO_ERROR;
        int count = 0;

        // Firmware must be loaded and a mode must have been selected
        //
        if ( ! this->firmwareloaded ) {
            this->camera.log_error( function, "no firmware loaded" );
            return ERROR;
        }

        if ( ! this->modeselected ) {
            this->camera.log_error( function, "no mode selected" );
            return ERROR;
        }

        if ( !nlines_in.empty() ) {
            // Convert to integer to check the value
            //
            try {
                count = std::stoi( nlines_in );

            } catch (std::invalid_argument &) {
                message.str(""); message << "converting nlines: " << nlines_in << " to integer";
                this->camera.log_error( function, message.str() );
                return(ERROR);

            } catch (std::out_of_range &) {
                message.str(""); message << "requested nlines: " << nlines_in << " outside integer range";
                this->camera.log_error( function, message.str() );
                return(ERROR);
            }

            // Archon allows only 20 bit parameters
            //
            if ( count < 0 || count > 0xFFFFF ) {
                message.str(""); message << "requested nlines: " << nlines_in << " out of range {0:1048575}";
                this->camera.log_error( function, message.str() );
                return( ERROR );
            }

            // Now that the value is OK set the parameter on the Archon
            //
            std::stringstream cmd;
            cmd << "H2RG_rows " << nlines_in;
            error = this->set_parameter( cmd.str() );

            if ( error != NO_ERROR ) {
                this->camera.log_error(function, "writing H2RG_rows");
                return error;
            }

            std::string dontcare;
            cmd.str("");
            cmd << "LINECOUNT " << nlines_in;
            error = this->cds( cmd.str(), dontcare );

            // get out now if any errors
            //
            if (error != NO_ERROR) {
                this->camera.log_error(function, "writing LINECOUNT");
                return error;
            }

            // update modemap, in case someone asks again
            //
            std::string mode = this->camera_info.current_observing_mode;

            this->modemap[mode].geometry.linecount = count;
            this->camera_info.region_of_interest[3] = count;
            this->camera_info.detector_pixels[1] = count;
            this->camera_info.set_axes();

        }

        count = this->camera_info.detector_pixels[1];

        // prepare the return value
        //
        message.str(""); message << count;
        retstring = message.str();

        message.str(""); message << "nlines is " << retstring;
        logwrite(function, message.str());

        return(error);
    }
    /**************** Archon::Interface::nlines *********************************/



  /**************** Archon::Interface::hread_frame *****************************/
  /**
   * @fn     hread_frame
   * @brief  read latest Archon frame buffer
   * @param  frame_type
   * @return ERROR or NO_ERROR
   *
   * This is the read_frame function which performs the actual read of the
   * selected frame type.
   *
   * No write takes place here!
   *
   */
    long Interface::hread_frame() {
        std::string function = "Archon::Interface::hread_frame";
        std::stringstream message;
        int retval;
        int bufready;
        char check[5], header[5];
        char *ptr_image;
        int bytesread, totalbytesread, toread;
        uint64_t bufaddr;
        unsigned int block, bufblocks=0;
        long error = ERROR;
        int num_detect = this->modemap[this->camera_info.current_observing_mode].geometry.num_detect;

        // Archon buffer number of the last frame read into memory
        // Archon frame index is 1 biased so add 1 here
        bufready = this->frame.index + 1;

        if (bufready < 1 || bufready > this->camera_info.activebufs) {
            message.str(""); message << "invalid Archon buffer " << bufready << " requested. Expected {1:" << this->camera_info.activebufs << "}";
            this->camera.log_error( function, message.str() );
            return(ERROR);
        }

        message.str(""); message << "will read image data from Archon controller buffer " << bufready << " frame " << this->frame.frame;
        logwrite(function, message.str());

        // Lock the frame buffer before reading it
        //
        // if ( this->lock_buffer(bufready) == ERROR) {
        //    logwrite( function, "ERROR locking frame buffer" );
        //    return (ERROR);
        // }

        // Send the FETCH command to read the memory buffer from the Archon backplane.
        // Archon replies with one binary response per requested block. Each response
        // has a message header.

        // Archon buffer base address
        bufaddr   = this->frame.bufbase[this->frame.index];

        // Calculate the number of blocks expected. image_memory is bytes per detector
        bufblocks =
                (unsigned int) floor( ((this->camera_info.image_memory * num_detect) + BLOCK_LEN - 1 ) / BLOCK_LEN );

        message.str(""); message << "will read " << std::dec << this->camera_info.image_memory << " bytes "
                                 << "0x" << std::uppercase << std::hex << bufblocks << " blocks from bufaddr=0x" << bufaddr;
        logwrite(function, message.str());

        // send the FETCH command.
        // This will take the archon_busy semaphore, but not release it -- must release in this function!
        //
        error = this->fetch(bufaddr, bufblocks);
        if ( error != NO_ERROR ) {
            logwrite( function, "ERROR: fetching Archon buffer" );
            return error;
        }

        // Read the data from the connected socket into memory, one block at a time
        //
        ptr_image = this->image_data;
        totalbytesread = 0;
        std::cerr << "reading bytes: ";
        for (block=0; block<bufblocks; block++) {

            // Are there data to read?
            if ( (retval=this->archon.Poll()) <= 0) {
                if (retval==0) {
                    message.str("");
                    message << "Poll timeout waiting for Archon frame data";
                    error = ERROR;
                }  // TODO should error=TIMEOUT?

                if (retval<0)  {
                    message.str("");
                    message << "Poll error waiting for Archon frame data";
                    error = ERROR;
                }

                if ( error != NO_ERROR ) this->camera.log_error( function, message.str() );
                break;                         // breaks out of for loop
            }

            // Wait for a block+header Bytes to be available
            // (but don't wait more than 1 second -- this should be tens of microseconds or less)
            //
            auto start = std::chrono::steady_clock::now();             // start a timer now

            while ( this->archon.Bytes_ready() < (BLOCK_LEN+4) ) {
                auto now = std::chrono::steady_clock::now();             // check the time again
                std::chrono::duration<double> diff = now-start;          // calculate the duration
                if (diff.count() > 1) {                                  // break while loop if duration > 1 second
                    std::cerr << "\n";
                    this->camera.log_error( function, "timeout waiting for data from Archon" );
                    error = ERROR;
                    break;                       // breaks out of while loop
                }
            }
            if ( error != NO_ERROR ) break;  // needed to also break out of for loop on error

            // Check message header
            //
            SNPRINTF(check, "<%02X:", this->msgref)
            if ( (retval=this->archon.Read(header, 4)) != 4 ) {
                message.str(""); message << "code " << retval << " reading Archon frame header";
                this->camera.log_error( function, message.str() );
                error = ERROR;
                break;                         // break out of for loop
            }
            if (header[0] == '?') {  // Archon retured an error
                message.str(""); message << "Archon returned \'?\' reading image data";
                this->camera.log_error( function, message.str() );
                this->fetchlog();      // check the Archon log for error messages
                error = ERROR;
                break;                         // break out of for loop

            } else if (strncmp(header, check, 4) != 0) {
                message.str(""); message << "Archon command-reply mismatch reading image data. header=" << header << " check=" << check;
                this->camera.log_error( function, message.str() );
                error = ERROR;
                break;                         // break out of for loop
            }

            // Read the frame contents
            //
            bytesread = 0;
            do {
                toread = BLOCK_LEN - bytesread;
                if ( (retval=this->archon.Read(ptr_image, (size_t)toread)) > 0 ) {
                    bytesread += retval;         // this will get zeroed after each block
                    totalbytesread += retval;    // this won't (used only for info purposes)
                    std::cerr << std::setw(10) << totalbytesread << "\b\b\b\b\b\b\b\b\b\b";
                    ptr_image += retval;         // advance pointer
                }
            } while (bytesread < BLOCK_LEN);

        } // end of loop: for (block=0; block<bufblocks; block++)

        // give back the archon_busy semaphore to allow other threads to access the Archon now
        //
        const std::unique_lock<std::mutex> lock(this->archon_mutex);
        this->archon_busy = false;
        this->archon_mutex.unlock();

        std::cerr << std::setw(10) << totalbytesread << " complete\n";   // display progress on same line of std err

        // If we broke out of the for loop for an error then report incomplete read
        //
        if ( error==ERROR || block < bufblocks) {
            message.str(""); message << "incomplete frame read " << std::dec
                                     << totalbytesread << " bytes: " << block << " of " << bufblocks << " 1024-byte blocks";
            logwrite( function, message.str() );
        }

        // Unlock the frame buffer
        //
        // if (error == NO_ERROR) error = this->archon_cmd(UNLOCK);

        // On success, write the value to the log and return
        //
        if (error == NO_ERROR) {
            message.str(""); message << "successfully read " << std::dec << totalbytesread
                    << " image bytes (0x" << std::uppercase << std::hex << bufblocks << " blocks) from Archon controller";
            logwrite(function, message.str());

        } else {
            // Throw an error for any other errors
            logwrite( function, "ERROR: reading Archon camera data to memory!" );
        }
        return(error);
    }
    /**************** Archon::Interface::hread_frame *****************************/



  /**************** Archon::Interface::hsetup ********************************/
  /**
    * @fn     hsetup
    * @brief  setup archon for h2rg
    * @param  NONE
    * @return ERROR or NO_ERROR
    *
    * NOTE: this assumes LVDS is module 10
    * This function does the following:
    *  1) Pulse low on MainResetB
    *  2) sets output to Pad B and HIGHOHM
    *
    */
    long Interface::hsetup() {
        std::string function = "Archon::Interface::hsetup";
        std::stringstream message;
        std::string reg;
        long error = NO_ERROR;

        // H2RG manual says to pull this value low for 100 ns
        // however, currently it is pulled low for ~1000 usec
        this->set_parameter("H2RGMainReset", 1);
        usleep(1500);  // to be sure we are done with the reset
        this->set_parameter("H2RGMainReset", 0);
        usleep(1000);  // to be sure we are done with the reset

        // Enable output to Pad B and HIGHOHM
        error = this->inreg("10 1 16402");      // 0100 000000010010
        if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
        if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0

        if (error != NO_ERROR) {
            message.str(""); message << "enabling output to Pad B and HIGHOHM";
            this->camera.log_error( function, message.str() );
            return(ERROR);
        }

        return (error);
    }
    /**************** Archon::Interface::hsetup *******************************/

    /**************** Archon::Interface::hroi ******************************/
    /**
      * @fn     hroi
      * @brief  set window limits for h2rg
      * @param  geom_in  string, with vstart vstop hstart hstop in pixels
      * @return ERROR or NO_ERROR
      *
      * NOTE: this assumes LVDS is module 10
      * This function does the following:
      *  1) sets limits of sub window using input params
      *
      */
    long Interface::hroi(std::string geom_in, std::string &retstring) {
        std::string function = "Archon::Interface::hroi";
        std::stringstream message;
        std::stringstream cmd;
        std::string dontcare;
        int hstart, hstop, vstart, vstop;
        long error = NO_ERROR;
        std::vector<std::string> tokens;

        // If geom_in is not supplied then set geometry to full frame.
        //
        if ( !geom_in.empty() ) {         // geometry arguments passed in
            Tokenize(geom_in, tokens, " ");

            if (tokens.size() != 4) {
                message.str(""); message << "param expected 4 arguments (vstart, vstop, hstart, hstop) but got " << tokens.size();
                this->camera.log_error( function, message.str() );
                return(ERROR);
            }
            try {
                vstart = std::stoi( tokens[0] ); // test that inputs are integers
                vstop = std::stoi( tokens[1] );
                hstart = std::stoi( tokens[2] );
                hstop = std::stoi( tokens[3]);

            } catch (std::invalid_argument &) {
                message.str(""); message << "unable to convert geometry values: " << geom_in << " to integer";
                this->camera.log_error( function, message.str() );
                return(ERROR);

            } catch (std::out_of_range &) {
                message.str(""); message << "geometry values " << geom_in << " outside integer range";
                this->camera.log_error( function, message.str() );
                return(ERROR);
            }

            // Validate values are within detector
            if ( vstart < 0 || vstop > 2047 || hstart < 0 || hstop > 2047) {
                message.str(""); message << "geometry values " << geom_in << " outside pixel range";
                this->camera.log_error( function, message.str());
                return(ERROR);
            }
            // Validate values have proper ordering
            if (vstart >= vstop || hstart >= hstop) {
                message.str(""); message << "geometry values " << geom_in << " are not correctly ordered";
                this->camera.log_error( function, message.str());
                return(ERROR);
            }

            // Set detector registers and record limits
            // vstart 1000 000000000000 = 32768
            cmd.str("") ; cmd << "10 1 " << (32768 + vstart);
            error = this->inreg(cmd.str());
            if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
            if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0
            if (error == NO_ERROR) this->win_vstart = vstart; // set y lo lim
            // vstop 1001 000000000000 = 36864
            cmd.str("") ; cmd << "10 1 " << (36864 + vstop);
            if (error == NO_ERROR) error = this->inreg(cmd.str());
            if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
            if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0
            if (error == NO_ERROR) this->win_vstop = vstop; // set y hi lim
            // hstart 1010 000000000000 = 40960
            cmd.str("") ; cmd << "10 1 " << (40960 + hstart);
            if (error == NO_ERROR) error = this->inreg(cmd.str());
            if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
            if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0
            if (error == NO_ERROR) this->win_hstart = hstart; // set x lo lim
            // hstop 1011 000000000000 = 45056
            cmd.str("") ; cmd << "10 1 " << (45056 + hstop);
            if (error == NO_ERROR) error = this->inreg(cmd.str());
            if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
            if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0
            if (error == NO_ERROR) this->win_hstop = hstop; // set roi x hi lim

            // If we are in window mode, make adjustments to geometries
            if (this->is_window) {
                // Now set params
                int rows = (this->win_vstop - this->win_vstart) + 1;
                int cols = (this->win_hstop - this->win_hstart) + 1;
                cmd.str("");
                if (error == NO_ERROR) {
                    cmd << "H2RG_win_columns " << cols;
                    error = this->set_parameter(cmd.str());
                }
                cmd.str("");
                if (error == NO_ERROR) {
                    cmd << "H2RG_win_rows " << rows;
                    error = this->set_parameter(cmd.str());
                }

                // Now set CDS
                cmd.str("");
                cmd << "PIXELCOUNT " << cols;
                error = this->cds(cmd.str(), dontcare);
                cmd.str("");
                cmd << "LINECOUNT " << rows;
                error = this->cds(cmd.str(), dontcare);

                // update modemap, in case someone asks again
                std::string mode = this->camera_info.current_observing_mode;

                this->modemap[mode].geometry.linecount = rows;
                this->modemap[mode].geometry.pixelcount = cols;
                this->camera_info.region_of_interest[0] = this->win_hstart;
                this->camera_info.region_of_interest[1] = this->win_hstop;
                this->camera_info.region_of_interest[2] = this->win_vstart;
                this->camera_info.region_of_interest[3] = this->win_vstop;
                this->camera_info.detector_pixels[0] = cols;
                this->camera_info.detector_pixels[1] = rows;

                this->camera_info.set_axes();
            }

        }   // end if geom passed in

        // prepare the return value
        //
        message.str(""); message << this->win_vstart << " " << this->win_vstop << " " << this->win_hstart << " " << this->win_hstop;
        retstring = message.str();

        if (error != NO_ERROR) {
            message.str(""); message << "setting window geometry to " << retstring;
            this->camera.log_error( function, message.str() );
            return(ERROR);
        }

        return (error);
    }
    /**************** Archon::Interface::hroi *********************************/

    /**************** Archon::Interface::hwindow ******************************/
    /**
      * @fn     hwindow
      * @brief  set into/out of window mode for h2rg
      * @param  state_in, string "TRUE, FALSE, 0, or 1"
      * @return ERROR or NO_ERROR
      *
      * NOTE: this assumes LVDS is module 10
      * This function does the following:
      *  1) puts h2rg into or out of window mode
      *
      */
    long Interface::hwindow(std::string state_in, std::string &state_out) {
        std::string function = "Archon::Interface::hwindow";
        std::stringstream message;
        std::string reg;
        std::string nowin_mode = "DEFAULT";
        std::string win_mode = "GUIDING";
        std::string dontcare;
        std::stringstream cmd;
        long error = NO_ERROR;

        // If something is passed then try to use it to set the window state
        //
        if ( !state_in.empty() ) {
            try {
                std::transform( state_in.begin(), state_in.end(), state_in.begin(), ::toupper );  // make uppercase

                if ( state_in == "FALSE" || state_in == "0" ) { // leave window mode
                    this->is_window = false;
                    // Set detector out of window mode
                    error = this->inreg("10 1 28684"); // 0111 000000001100
                    if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
                    if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0

                    // reset taplines
                    cmd.str("");
                    cmd << "TAPLINES " << this->taplines_store;
                    this->cds(cmd.str(), dontcare);
                    cmd.str("");
                    cmd << "TAPLINE0 " << this->tapline0_store;
                    this->cds(cmd.str(), dontcare);

                    // Set camera mode
                    // This resets all internal buffer geometries
                    this->set_camera_mode(nowin_mode);

                    // Now set CDS
                    cmd.str("");
                    cmd << "PIXELCOUNT " << this->modemap[nowin_mode].geometry.pixelcount;
                    error = this->cds( cmd.str(), dontcare );
                    cmd.str("");
                    cmd << "LINECOUNT " << this->modemap[nowin_mode].geometry.linecount;
                    error = this->cds( cmd.str(), dontcare );

                    // Issue Abort to complete window mode exit
                    cmd.str("");
                    if (error == NO_ERROR) {
                        cmd << "Abort 1 ";
                        error = this->set_parameter( cmd.str() );
                    }

                } else if ( state_in == "TRUE" || state_in == "1" ) {  // enter window mode
                    this->is_window = true;
                    // Set detector into window mode
                    error = this->inreg("10 1 28687"); // 0111 000000001111
                    if (error == NO_ERROR) error = this->inreg("10 0 1"); // send to detector
                    if (error == NO_ERROR) error = this->inreg("10 0 0"); // reset to 0

                    // Adjust taplines
                    std::string taplines_str;
                    this->cds("TAPLINES", taplines_str);
                    this->taplines_store = std::stoi(taplines_str);
                    this->cds("TAPLINES 1", dontcare);
                    this->taplines = 1;

                    std::string tapline0;
                    this->cds("TAPLINE0", tapline0);
                    this->tapline0_store = tapline0;
                    this->cds("TAPLINE0 AM33L,1,0", dontcare);

                    // Set camera mode to win_mode
                    this->set_camera_mode(win_mode);

                    // Now set params
                    int rows = (this->win_vstop - this->win_vstart) + 1;
                    int cols = (this->win_hstop - this->win_hstart) + 1;
                    if (error == NO_ERROR) {
                        cmd << "H2RG_win_columns " << cols;
                        error = this->set_parameter( cmd.str() );
                    }
                    cmd.str("");
                    if (error == NO_ERROR) {
                        cmd << "H2RG_win_rows " << rows;
                        error = this->set_parameter( cmd.str() );
                    }

                    // Now set CDS
                    cmd.str("");
                    cmd << "PIXELCOUNT " << cols;
                    error = this->cds( cmd.str(), dontcare );
                    cmd.str("");
                    cmd << "LINECOUNT " << rows;
                    error = this->cds( cmd.str(), dontcare );

                    // update modemap, in case someone asks again
                    std::string mode = this->camera_info.current_observing_mode;

                    // Adjust geometry parameters and camera_info
                    this->modemap[mode].geometry.linecount = rows;
                    this->modemap[mode].geometry.pixelcount = cols;
                    this->camera_info.region_of_interest[0] = this->win_hstart;
                    this->camera_info.region_of_interest[1] = this->win_hstop;
                    this->camera_info.region_of_interest[2] = this->win_vstart;
                    this->camera_info.region_of_interest[3] = this->win_vstop;
                    this->camera_info.detector_pixels[0] = cols;
                    this->camera_info.detector_pixels[1] = rows;

                    this->camera_info.set_axes();

                } else {
                    message.str(""); message << "window state " << state_in << " is invalid. Expecting {true,false,0,1}";
                    this->camera.log_error( function, message.str() );
                    return( ERROR );
                }

            } catch (...) {
                message.str(""); message << "unknown exception converting window state " << state_in << " to uppercase";
                this->camera.log_error( function, message.str() );
                return( ERROR );
            }
        }

        state_out = ( this->is_window ? "true" : "false" );

        if (error != NO_ERROR) {
            message.str(""); message << "setting window state to " << state_in;
            this->camera.log_error( function, message.str() );
            return(ERROR);
        }

        return (error);
    }
    /**************** Archon::Interface::hwindow *******************************/

    /**************** Archon::Interface::hexpose ******************************/
    /**
     * @fn     hexpose
     * @brief  initiate an exposure for h2rg
     * @param  nseq_in string, if set becomes the number of sequences
     * @return ERROR or NO_ERROR
     *
     * This function does the following before returning successful completion:
     *  1) trigger an Archon exposure by setting the EXPOSE parameter = nseq_in
     *  2) wait for exposure delay
     *  3) wait for readout into Archon frame buffer
     *  4) read frame buffer from Archon to host
     *  5) Do NOT write frame to disk (eventually to shared memory)
     *
     * Note that this assumes that the Archon ACF has been programmed to automatically
     * read out the detector into the frame buffer after an exposure.
     *
     */
    long Interface::hexpose(std::string nseq_in) {
        std::string function = "Archon::Interface::hexpose";
        std::stringstream message;
        long error = NO_ERROR;
        std::string nseqstr;
        int nseq, finalframe, nread, currentindex;

        std::string mode = this->camera_info.current_observing_mode;            // local copy for convenience

        if ( ! this->modeselected ) {
            this->camera.log_error( function, "no mode selected" );
            return ERROR;
        }

        // exposeparam is set by the configuration file
        // check to make sure it was set, or else expose won't work
        if (this->exposeparam.empty()) {
            message.str(""); message << "EXPOSE_PARAM not defined in configuration file " << this->config.filename;
            this->camera.log_error( function, message.str() );
            return(ERROR);
        }

        // If the exposure time or longexposure mode were never set then read them from the Archon.
        // This ensures that, if the client doesn't set these values then the server will have the
        // same default values that the ACF has, rather than hope that the ACF programmer picks
        // their defaults to match mine.
        if ( this->camera_info.exposure_time   == -1 ) {
            logwrite( function, "NOTICE:exptime has not been set--will read from Archon" );
            this->camera.async.enqueue( "NOTICE:exptime has not been set--will read from Archon" );

            // read the Archon configuration memory
            //
            std::string etime;
            if ( read_parameter( "exptime", etime ) != NO_ERROR ) { logwrite( function, "ERROR: reading \"exptime\" parameter from Archon" ); return ERROR; }

            // Tell the server these values
            //
            std::string retval;
            if ( this->exptime( etime, retval ) != NO_ERROR ) { logwrite( function, "ERROR: setting exptime" ); return ERROR; }
        }
        if ( this->camera_info.exposure_factor == -1 ||
             this->camera_info.exposure_unit.empty() ) {
            logwrite( function, "NOTICE:longexposure has not been set--will read from Archon" );
            this->camera.async.enqueue( "NOTICE:longexposure has not been set--will read from Archon" );

            // read the Archon configuration memory
            //
            std::string lexp;
            if ( read_parameter( "longexposure", lexp ) != NO_ERROR ) { logwrite( function, "ERROR: reading \"longexposure\" parameter from Archon" ); return ERROR; }

            // Tell the server these values
            //
            std::string retval;
            if ( this->longexposure( lexp, retval ) != NO_ERROR ) { logwrite( function, "ERROR: setting longexposure" ); return ERROR; }
        }

        // If nseq_in is not supplied then set nseq to 1.
        if ( nseq_in.empty() ) {
            nseq = 1;
            nseqstr = std::to_string( nseq );

        } else {                    // sequence argument passed in
            try {
                nseq = std::stoi( nseq_in ) + this->camera_info.num_pre_exposures;      // test that nseq_in is an integer
                nseqstr = std::to_string( nseq );                           // before trying to use it

            } catch (std::invalid_argument &) {
                message.str(""); message << "unable to convert sequences: " << nseq_in << " to integer";
                this->camera.log_error( function, message.str() );
                return(ERROR);

            } catch (std::out_of_range &) {
                message.str(""); message << "sequences " << nseq_in << " outside integer range";
                this->camera.log_error( function, message.str() );
                return(ERROR);
            }
        }

        // Always initialize the extension number because someone could
        // set datacube true and then send "expose" without a number.
        this->camera_info.extension = 0;

        // initialize frame parameters (index, etc.)
        error = this->get_frame_status();
        currentindex = this->frame.index;

        if (error != NO_ERROR) {
            logwrite( function, "ERROR: unable to get frame status" );
            return(ERROR);
        }
        // save the last frame number acquired (wait_for_readout will need this)
        this->lastframe = this->frame.bufframen[this->frame.index];

        // calculate the last frame to handle dropped frames correctly
        finalframe = this->lastframe + nseq;

        if (nseq > 1) {
            message.str(""); message << "starting sequence of " << nseq << " frames. lastframe=" << this->lastframe << " last buffer=" << currentindex+1;
            logwrite(function, message.str());
        }

        // Allocate image buffer once
        this->camera_info.frame_type = Camera::FRAME_IMAGE;
        error = this->prepare_image_buffer();
        if (error == ERROR) {
            logwrite( function, "ERROR: unable to allocate an image buffer" );
            return(ERROR);
        }

        // initiate the exposure here
        logwrite(function, "exposure starting");
        error = this->prep_parameter(this->exposeparam, nseqstr);
        if (error == NO_ERROR) error = this->load_parameter(this->exposeparam, nseqstr);
        if ( error != NO_ERROR ) {
            logwrite( function, "ERROR: could not initiate exposure" );
            return( error );
        }

        // get system time and Archon's timer after exposure starts
        // start_timer is used to determine when the exposure has ended, in wait_for_exposure()
        // this->camera_info.start_time = get_timestamp();                 // current system time formatted as YYYY-MM-DDTHH:MM:SS.sss
        // if ( this->get_timer(&this->start_timer) != NO_ERROR ) {        // Archon internal timer (one tick=10 nsec)
        //     logwrite( function, "ERROR: could not get start time" );
        //     return( ERROR );
        // }
        // this->add_filename_key();                                       // add filename to system keys database

        // Wait for Archon frame buffer to be ready,
        // then read the latest ready frame buffer to the host. If this
        // is a sequence, then loop over all expected frames.

        //
        // -- MAIN SEQUENCE LOOP --
        nread = 0;          // Keep track of how many we actually read
        int ns = nseq;      // Iterate with ns, to preserve original request
        while (ns-- > 0 && this->lastframe < finalframe) {

            // if ( !this->camera.datacube() || this->camera.cubeamps() ) {
            //    this->camera_info.start_time = get_timestamp();               // current system time formatted as YYYY-MM-DDTHH:MM:SS.sss
            //    if ( this->get_timer(&this->start_timer) != NO_ERROR ) {      // Archon internal timer (one tick=10 nsec)
            //        logwrite( function, "ERROR: could not get start time" );
            //        return( ERROR );
            //    }
                // this->add_filename_key();                                     // add filename to system keys database
            // }

            // wait for the exposure delay to complete (if there is one)
            if ( this->camera_info.exposure_time != 0 ) {
                error = this->wait_for_exposure();
                if ( error != NO_ERROR ) {
                    logwrite( function, "ERROR: waiting for exposure" );
                    return error;
                }
            }

            // Wait for the readout into frame buffer,
            error = this->hwait_for_readout();
            if ( error != NO_ERROR ) {
                logwrite( function, "ERROR: waiting for readout" );
                return error;
            }

            // then read the frame buffer to host (and write file) when frame ready.
            error = hread_frame();
            if ( error != NO_ERROR ) {
                logwrite( function, "ERROR: reading frame buffer" );
                return error;
            }

            // ASYNC status message on completion of each readout
            nread++;
            message.str(""); message << "READOUT COMPLETE (" << nread << " of " << nseq << " read)";
            this->camera.async.enqueue( message.str() );
            logwrite( function, message.str() );

            if (error != NO_ERROR) break;                               // should be impossible but don't try additional sequences if there were errors

        }  // end of sequence loop, while (ns-- > 0 && this->lastframe < finalframe)

        // ASYNC status message on completion of each sequence
        message.str(""); message << "READOUT SEQUENCE " << ( error==NO_ERROR ? "COMPLETE" : "ERROR" ) << " (" << nread << " of " << nseq << " read)";
        this->camera.async.enqueue( message.str() );
        error == NO_ERROR ? logwrite( function, message.str() ) : this->camera.log_error( function, message.str() );

        error = get_frame_status();
        if ( error != NO_ERROR ) {
            logwrite( function, "ERROR: getting final frame status" );
            return error;
        }

        message.str(""); message << "Last frame read " << this->frame.frame << " from buffer " << this->frame.index + 1;
        logwrite( function, message.str());

        return (error);
    }
    /**************** Archon::Interface::hexpose *********************************/


    /**************** Archon::Interface::video *********************************/
    /**
     * @fn     video
     * @brief  initiate a video exposure
     * @param  nseq_in string, if set becomes the number of sequences
     * @return ERROR or NO_ERROR
     *
     * This function does the following before returning successful completion:
     *  1) trigger an Archon exposure by setting the EXPOSE parameter = nseq_in
     *  2) wait for exposure delay
     *  3) wait for readout into Archon frame buffer
     *  4) read frame buffer from Archon to host
     *  5) do NOT write frame to disk
     *
     * Note that this assumes that the Archon ACF has been programmed to automatically
     * read out the detector into the frame buffer after an exposure.
     *
     */    long Interface::video() {
      std::string function = "Archon::Interface::expose";
      std::stringstream message;
      long error = NO_ERROR;
      std::string nseqstr;
      int nseq;

      std::string mode = this->camera_info.current_observing_mode;            // local copy for convenience

      if ( ! this->modeselected ) {
          this->camera.log_error( function, "no mode selected" );
          return ERROR;
      }

      // When switching from cubeamps=true to cubeamps=false,
      // simply reset the mode to the current mode in order to
      // reset the image size.
      //
      // This will need to be revisited once ROI is implemented. // TODO
      //
      if ( !this->camera.cubeamps() && ( this->lastcubeamps != this->camera.cubeamps() ) ) {
          message.str(""); message << "detected change in cubeamps -- resetting camera mode to " << mode;
          logwrite( function, message.str() );
          this->set_camera_mode( mode );
      }

      // exposeparam is set by the configuration file
      // check to make sure it was set, or else expose won't work
      //
      if (this->exposeparam.empty()) {
          message.str(""); message << "EXPOSE_PARAM not defined in configuration file " << this->config.filename;
          this->camera.log_error( function, message.str() );
          return(ERROR);
      }

      // If the exposure time or longexposure mode were never set then read them from the Archon.
      // This ensures that, if the client doesn't set these values then the server will have the
      // same default values that the ACF has, rather than hope that the ACF programmer picks
      // their defaults to match mine.
      //
      if ( this->camera_info.exposure_time   == -1 ) {
          logwrite( function, "NOTICE:exptime has not been set--will read from Archon" );
          this->camera.async.enqueue( "NOTICE:exptime has not been set--will read from Archon" );

          // read the Archon configuration memory
          //
          std::string etime;
          if ( read_parameter( "exptime", etime ) != NO_ERROR ) { logwrite( function, "ERROR: reading \"exptime\" parameter from Archon" ); return ERROR; }

          // Tell the server these values
          //
          std::string retval;
          if ( this->exptime( etime, retval ) != NO_ERROR ) { logwrite( function, "ERROR: setting exptime" ); return ERROR; }
      }
      if ( this->camera_info.exposure_factor == -1 ||
           this->camera_info.exposure_unit.empty() ) {
          logwrite( function, "NOTICE:longexposure has not been set--will read from Archon" );
          this->camera.async.enqueue( "NOTICE:longexposure has not been set--will read from Archon" );

          // read the Archon configuration memory
          //
          std::string lexp;
          if ( read_parameter( "longexposure", lexp ) != NO_ERROR ) { logwrite( function, "ERROR: reading \"longexposure\" parameter from Archon" ); return ERROR; }

          // Tell the server these values
          //
          std::string retval;
          if ( this->longexposure( lexp, retval ) != NO_ERROR ) { logwrite( function, "ERROR: setting longexposure" ); return ERROR; }
      }

      // If nseq_in is not supplied then set nseq to 1.
      // Add any pre-exposures onto the number of sequences.
      //

      nseq = 1 + this->camera_info.num_pre_exposures;

      // Always initialize the extension number because someone could
      // set datacube true and then send "expose" without a number.
      //
      this->camera_info.extension = 0;

      error = this->get_frame_status();  // TODO is this needed here?

      if (error != NO_ERROR) {
          logwrite( function, "ERROR: unable to get frame status" );
          return(ERROR);
      }
      this->lastframe = this->frame.bufframen[this->frame.index];     // save the last frame number acquired (wait_for_readout will need this)

      // initiate the exposure here
      //
      error = this->prep_parameter(this->exposeparam, nseqstr);
      if (error == NO_ERROR) error = this->load_parameter(this->exposeparam, nseqstr);
      if ( error != NO_ERROR ) {
          logwrite( function, "ERROR: could not initiate exposure" );
          return( error );
      }

      // get system time and Archon's timer after exposure starts
      // start_timer is used to determine when the exposure has ended, in wait_for_exposure()
      //
      this->camera_info.start_time = get_timestamp();                 // current system time formatted as YYYY-MM-DDTHH:MM:SS.sss
      if ( this->get_timer(&this->start_timer) != NO_ERROR ) {        // Archon internal timer (one tick=10 nsec)
          logwrite( function, "ERROR: could not get start time" );
          return( ERROR );
      }
      this->camera.set_fitstime(this->camera_info.start_time);        // sets camera.fitstime (YYYYMMDDHHMMSS) used for filename
      error=this->camera.get_fitsname(this->camera_info.fits_name);   // assemble the FITS filename
      if ( error != NO_ERROR ) {
          logwrite( function, "ERROR: couldn't validate fits filename" );
          return( error );
      }

      this->add_filename_key();                                       // add filename to system keys database

      logwrite(function, "exposure started");

      this->camera_info.systemkeys.keydb = this->systemkeys.keydb;    // copy the systemkeys database object into camera_info

      if (this->camera.writekeys_when=="before") this->copy_keydb();  // copy the ACF and userkeys database into camera_info

      // If mode is not "RAW" but RAWENABLE is set then we're going to require a multi-extension data cube,
      // one extension for the image and a separate extension for raw data.
      //
      if ( (mode != "RAW") && (this->modemap[mode].rawenable) ) {
          if ( !this->camera.datacube() ) {                                   // if datacube not already set then it must be overridden here
              this->camera.async.enqueue( "NOTICE:override datacube true" );  // let everyone know
              logwrite( function, "NOTICE:override datacube true" );
              this->camera.datacube(true);
          }
          this->camera_info.extension = 0;
      }

      // Save the datacube state in camera_info so that the FITS writer can know about it
      //
      this->camera_info.iscube = this->camera.datacube();

      // Open the FITS file now for cubes
      //
      if ( this->camera.datacube() && !this->camera.cubeamps() ) {
          #ifdef LOGLEVEL_DEBUG
          logwrite( function, "[DEBUG] opening fits file for multi-exposure sequence data cube" );
          #endif
          error = this->fits_file.open_file(
                  this->camera.writekeys_when == "before", this->camera_info );
          if ( error != NO_ERROR ) {
              this->camera.log_error( function, "couldn't open fits file" );
              return( error );
          }
      }

      //  //TODO only use camera_info -- don't use fits_info -- is this OK? TO BE CONFIRMED
      //  this->fits_info = this->camera_info;                            // copy the camera_info class, to be given to fits writer  //TODO

      //  this->lastframe = this->frame.bufframen[this->frame.index];     // save the last frame number acquired (wait_for_readout will need this)

      if (nseq > 1) {
          message.str(""); message << "starting sequence of " << nseq << " frames. lastframe=" << this->lastframe;
          logwrite(function, message.str());
      }

      // If not RAW mode then wait for Archon frame buffer to be ready,
      // then read the latest ready frame buffer to the host. If this
      // is a squence, then loop over all expected frames.
      //
      if ( mode != "RAW" ) {                                          // If not raw mode then
          int expcount = 0;                                             // counter used only for tracking pre-exposures

          //
          // -- MAIN SEQUENCE LOOP --
          //
          while (nseq-- > 0) {

              // Wait for any pre-exposures, first the exposure delay then the readout,
              // but then continue to the next because pre-exposures are not read from
              // the Archon's buffer.
              //
              if ( ++expcount <= this->camera_info.num_pre_exposures ) {

                  message.str(""); message << "pre-exposure " << expcount << " of " << this->camera_info.num_pre_exposures;
                  logwrite( function, message.str() );

                  if ( this->camera_info.exposure_time != 0 ) {                 // wait for the exposure delay to complete (if there is one)
                      error = this->wait_for_exposure();
                      if ( error != NO_ERROR ) {
                          logwrite( function, "ERROR: waiting for pre-exposure" );
                          return error;
                      }
                  }

                  error = this->wait_for_readout();                             // Wait for the readout into frame buffer,
                  if ( error != NO_ERROR ) {
                      logwrite( function, "ERROR: waiting for pre-exposure readout" );
                      return error;
                  }

                  continue;
              }

              // Open a new FITS file for each frame when not using datacubes
              //
              #ifdef LOGLEVEL_DEBUG
              message.str(""); message << "[DEBUG] datacube=" << this->camera.datacube() << " cubeamps=" << this->camera.cubeamps();
                logwrite( function, message.str() );
              #endif
              if ( !this->camera.datacube() || this->camera.cubeamps() ) {
                  this->camera_info.start_time = get_timestamp();               // current system time formatted as YYYY-MM-DDTHH:MM:SS.sss
                  if ( this->get_timer(&this->start_timer) != NO_ERROR ) {      // Archon internal timer (one tick=10 nsec)
                      logwrite( function, "ERROR: could not get start time" );
                      return( ERROR );
                  }
                  this->camera.set_fitstime(this->camera_info.start_time);      // sets camera.fitstime (YYYYMMDDHHMMSS) used for filename
                  error=this->camera.get_fitsname(this->camera_info.fits_name); // Assemble the FITS filename
                  if ( error != NO_ERROR ) {
                      logwrite( function, "ERROR: couldn't validate fits filename" );
                      return( error );
                  }
                  this->add_filename_key();                                     // add filename to system keys database

                  #ifdef LOGLEVEL_DEBUG
                  logwrite( function, "[DEBUG] reset extension=0 and opening new fits file" );
                  #endif
                  // reset the extension number and open the fits file
                  //
                  this->camera_info.extension = 0;
                  error = this->fits_file.open_file(
                          this->camera.writekeys_when == "before", this->camera_info );
                  if ( error != NO_ERROR ) {
                      this->camera.log_error( function, "couldn't open fits file" );
                      return( error );
                  }
              }

              if ( this->camera_info.exposure_time != 0 ) {                   // wait for the exposure delay to complete (if there is one)
                  error = this->wait_for_exposure();
                  if ( error != NO_ERROR ) {
                      logwrite( function, "ERROR: waiting for exposure" );
                      return error;
                  }
              }

              if (this->camera.writekeys_when=="after") this->copy_keydb();   // copy the ACF and userkeys database into camera_info

              error = this->wait_for_readout();                               // Wait for the readout into frame buffer,

              if ( error != NO_ERROR ) {
                  logwrite( function, "ERROR: waiting for readout" );
                  this->fits_file.close_file(
                          this->camera.writekeys_when == "after", this->camera_info );
                  return error;
              }

              error = read_frame();                                           // then read the frame buffer to host (and write file) when frame ready.
              if ( error != NO_ERROR ) {
                  logwrite( function, "ERROR: reading frame buffer" );
                  this->fits_file.close_file(
                          this->camera.writekeys_when == "after", this->camera_info );
                  return error;
              }

              // For non-sequence multiple exposures, including cubeamps, close the fits file here
              //
              if ( !this->camera.datacube() || this->camera.cubeamps() ) {    // Error or not, close the file.
                  #ifdef LOGLEVEL_DEBUG
                  logwrite( function, "[DEBUG] closing fits file (1)" );
                  #endif
                  this->fits_file.close_file(
                          this->camera.writekeys_when == "after", this->camera_info ); // close the file when not using datacubes
                  this->camera.increment_imnum();                           // increment image_num when fitsnaming == "number"

                  // ASYNC status message on completion of each file
                  //
                  message.str(""); message << "FILE:" << this->camera_info.fits_name << " COMPLETE";
                  this->camera.async.enqueue( message.str() );
                  logwrite( function, message.str() );
              }

              if (error != NO_ERROR) break;                               // should be impossible but don't try additional sequences if there were errors

          }  // end of sequence loop, while (nseq-- > 0)

      } else if ( mode == "RAW") {
          error = this->get_frame_status();                             // Get the current frame buffer status
          if (error != NO_ERROR) {
              logwrite( function, "ERROR: unable to get frame status" );
              return(ERROR);
          }
          error = this->camera.get_fitsname( this->camera_info.fits_name ); // Assemble the FITS filename
          if ( error != NO_ERROR ) {
              logwrite( function, "ERROR: couldn't validate fits filename" );
              return( error );
          }
          this->add_filename_key();                                     // add filename to system keys database

          this->camera_info.systemkeys.keydb = this->systemkeys.keydb;  // copy the systemkeys database into camera_info

          this->copy_keydb();                                           // copy the ACF and userkeys databases into camera_info

          error = this->fits_file.open_file(
                  this->camera.writekeys_when == "before", this->camera_info );
          if ( error != NO_ERROR ) {
              this->camera.log_error( function, "couldn't open fits file" );
              return( error );
          }
          error = read_frame();                    // For raw mode just read immediately
          this->fits_file.close_file(this->camera.writekeys_when == "after", this->camera_info );
          this->camera.increment_imnum();          // increment image_num when fitsnaming == "number"
      }

      // for multi-exposure (non-cubeamp) cubes, close the FITS file now that they've all been written
      //
      if ( this->camera.datacube() && !this->camera.cubeamps() ) {
          #ifdef LOGLEVEL_DEBUG
          logwrite( function, "[DEBUG] closing fits file (2)" );
          #endif
          this->fits_file.close_file(this->camera.writekeys_when == "after", this->camera_info );
          this->camera.increment_imnum();          // increment image_num when fitsnaming == "number"

          // ASYNC status message on completion of each file
          //
          message.str(""); message << "FILE:" << this->camera_info.fits_name << " " << ( error==NO_ERROR ? "COMPLETE" : "ERROR" );
          this->camera.async.enqueue( message.str() );
          error == NO_ERROR ? logwrite( function, message.str() ) : this->camera.log_error( function, message.str() );
      }

      // remember the cubeamps setting used for the last completed exposure
      // TODO revisit once region-of-interest is implemented
      //
      this->lastcubeamps = this->camera.cubeamps();

      return (error);
  }
    /**************** Archon::Interface::video *********************************/

    /**************** Archon::Interface::hwait_for_readout ***********************/
    /**
     * @fn     hwait_for_readout
     * @brief  creates a wait until the next frame buffer is ready
     * @param  none
     * @return ERROR or NO_ERROR
     *
     * This function polls the Archon frame status until a new frame is ready.
     *
     */
    long Interface::hwait_for_readout() {
        std::string function = "Archon::Interface::hwait_for_readout";
        std::stringstream message;
        long error = NO_ERROR;
        int currentframe=this->lastframe + 1;

        message.str("");
        message << "waiting for new frame: current frame=" << this->lastframe << " current buffer=" << this->frame.index+1;
        logwrite(function, message.str());

        usleep( 700 );  // tune for size of window

        this->frame.index += 1;

        // Wrap frame.index
        if (this->frame.index >= (int)this->frame.bufframen.size()) {
            this->frame.index = 0;
        }

        this->frame.bufframen[ this->frame.index ] = currentframe;
        this->frame.frame = currentframe;

#ifdef LOGLEVEL_DEBUG
        message.str("");
    message << "[DEBUG] lastframe=" << this->lastframe
            << " currentframe=" << currentframe
            << " bufcomplete=" << this->frame.bufcomplete[this->frame.index];
    logwrite(function, message.str());
#endif
        this->lastframe = currentframe;

        // On success, write the value to the log and return
        //
        if (!this->abort) {
            message.str("");
            message << "received currentframe: " << currentframe << " from buffer " << this->frame.index+1;
            logwrite(function, message.str());
            return(NO_ERROR);

        } else if (this->abort) {
            // If the wait was stopped, log a message and return NO_ERROR
            logwrite(function, "wait for readout stopped by external signal");
            return(NO_ERROR);

        } else {
            // Throw an error for any other errors (should be impossible)
            this->camera.log_error( function, "waiting for readout" );
            return(error);
        }
    }
    /**************** Archon::Interface::hwait_for_readout ***********************/
}
