/** ---------------------------------------------------------------------------
 * @file     server.h
 * @brief    
 * @author   David Hale <dhale@astro.caltech.edu>
 * @date     
 * @modified 
 *
 */

#ifndef SERVER_H
#define SERVER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <csignal>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#ifdef ASTROCAM
#include "astrocam.h"       //!< for any Leach interface, ARC-64 or ARC-66
#elif STA_ARCHON
#include "archon.h"         //!< for STA-Archon
#endif

#include "logentry.h"
#include "config.h"
#include "network.h"

#define  N_THREADS    10    //!< total number of threads spawned by server, one for blocking and the remainder for non-blocking
#define  BUFSIZE      1024  //!< size of the input command buffer
#define  CONN_TIMEOUT 3000  //<! incoming (non-blocking) connection timeout in milliseconds

namespace Camera {

// Camera::Server class must inherit appropriate interface class
//
#ifdef ASTROCAM
  class Server : public AstroCam::Interface {
#elif STA_ARCHON
  class Server : public Archon::Interface {
#endif
    private:
    public:
      Server() {
        this->nbport=-1;
        this->blkport=-1;
        this->asyncport=-1;
      }

      /** Camera::~Server **********************************************************/
      /**
       * @fn     ~Server
       * @brief  class deconstructor cleans up on exit
       */
      ~Server() {
        close(this->nonblocking_socket);
        close(this->blocking_socket);
        closelog();  // close the logfile, if open
      }
      /** Camera::~Server **********************************************************/

      int nbport;                        //!< non-blocking port
      int blkport;                       //!< blocking port
      int asyncport;                     //!< asynchronous message port
      std::string asyncgroup;            //!< asynchronous multicast group

      int nonblocking_socket;
      int blocking_socket;

      Network::TcpSocket nonblocking;

      std::mutex conn_mutex;             //!< mutex to protect against simultaneous access to Accept()

      /** Camera::Server::exit_cleanly *********************************************/
      /**
       * @fn     signal_handler
       * @brief  handles ctrl-C and exits
       * @param  int signo
       * @return nothing
       *
       */
      void exit_cleanly(void) {
        std::string function = "Camera::Server::exit_cleanly";
        this->disconnect_controller();
        logwrite(function, "server exiting");
        exit(EXIT_SUCCESS);
      }
      /** Camera::Server::exit_cleanly *********************************************/

      /** Camera::Server::configure_server *****************************************/
      /**
       * @fn     configure_server
       * @brief  
       * @param  none
       * @return ERROR or NO_ERROR
       *
       */
      long configure_server() {
        std::string function = "Camera::Server::configure_server";
        std::stringstream message;
        int applied=0;
        long error;

        // loop through the entries in the configuration file, stored in config class
        //
        for (int entry=0; entry < this->config.n_entries; entry++) {

          // NBPORT
          if (config.param[entry].compare(0, 6, "NBPORT")==0) {
            int port;
            try {
              port = std::stoi( config.arg[entry] );
            }
            catch (std::invalid_argument &) {
              logwrite(function, "ERROR: bad NBPORT: unable to convert to integer");
              return(ERROR);
            }
            catch (std::out_of_range &) {
              logwrite(function, "NBPORT number out of integer range");
              return(ERROR);
            }
            this->nbport = port;
            applied++;
          }

          // BLKPORT
          if (config.param[entry].compare(0, 7, "BLKPORT")==0) {
            int port;
            try {
              port = std::stoi( config.arg[entry] );
            }
            catch (std::invalid_argument &) {
              logwrite(function, "ERROR: bad BLKPORT: unable to convert to integer");
              return(ERROR);
            }
            catch (std::out_of_range &) {
              logwrite(function, "BLKPORT number out of integer range");
              return(ERROR);
            }
            this->blkport = port;
            applied++;
          }

          // ASYNCPORT
          if (config.param[entry].compare(0, 9, "ASYNCPORT")==0) {
            int port;
            try {
              port = std::stoi( config.arg[entry] );
            }
            catch (std::invalid_argument &) {
              logwrite(function, "ERROR: bad ASYNCPORT: unable to convert to integer");
              return(ERROR);
            }
            catch (std::out_of_range &) {
              logwrite(function, "ASYNCPORT number out of integer range");
              return(ERROR);
            }
            this->asyncport = port;
            applied++;
          }

          // ASYNCGROUP
          if (config.param[entry].compare(0, 10, "ASYNCGROUP")==0) {
            this->asyncgroup = config.arg[entry];
            applied++;
          }

        } // end loop through the entries in the configuration file

        message.str("");
        if (applied==0) {
          message << "ERROR: ";
          error = ERROR;
        } 
        else {
          error = NO_ERROR;
        } 
        message << "applied " << applied << " configuration lines to server";
        logwrite(function, message.str());
        return error;
      }
      /** Camera::Server::configure_server *****************************************/

  };  // end class Server

} // end namespace Camera
#endif
