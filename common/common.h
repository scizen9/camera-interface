/**
 * @file     common.h
 * @brief    Common namespace header file
 * @author   David Hale <dhale@astro.caltech.edu>
 *
 */

#pragma once

#include <sstream>
#include <queue>
#include <string>
#include <mutex>
#include <map>
#include <condition_variable>

#include "logentry.h"

const long NOTHING = -1;
const long NO_ERROR = 0;
const long ERROR = 1;
const long BUSY = 2;
const long TIMEOUT = 3;

namespace Common {

  /**************** Common::FitsKeys ******************************************/
  /*
   * @class  FitsKeys
   * @brief  provides a user-defined keyword database
   *
   */
  class FitsKeys {
    private:
    public:
      FitsKeys() {}
      ~FitsKeys() {}

      std::string get_keytype(std::string keyvalue);         /// return type of keyword based on value
      long listkeys();                                       /// list FITS keys in the internal database
      long addkey(std::string arg);                          /// add FITS key to the internal database

      typedef struct {                                       /// structure of FITS keyword internal database
        std::string keyword;
        std::string keytype;
        std::string keyvalue;
        std::string keycomment;
      } user_key_t;

      typedef std::map<std::string, user_key_t> fits_key_t;  /// STL map for the actual keyword database

      fits_key_t keydb;                                      /// keyword database

      // Find all entries in the keyword database which start with the search_for string,
      // return a vector of iterators.
      //
      std::vector< fits_key_t::const_iterator > FindKeys( std::string search_for ) {
        std::vector< fits_key_t::const_iterator > vec;
        for ( auto it  = this->keydb.lower_bound( search_for );
                   it != std::end( this->keydb ) && it->first.compare( 0, search_for.size(), search_for ) == 0;
                 ++it ) {
          vec.push_back( it );
        }
        return( vec );
      }

      // Find and remove all entries in the keyword database which match the search_for string.
      //
      void EraseKeys( std::string search_for ) {
        std::string function = "FitsKeys::EraseKeys";
        std::stringstream message;
        std::vector< fits_key_t::const_iterator > erasevec;

        // get a vector of iterators for all the keys matching the search string
        //
        erasevec = this->FindKeys( search_for );
#ifdef LOGLEVEL_DEBUG
        message.str(""); message << "[DEBUG] found " << erasevec.size() << " entries matching \"" << search_for << "*\"";
        logwrite( function, message.str() );
#endif

        // loop through that vector and erase the selected iterators from the database
        //
        for ( auto vec : erasevec ) {
#ifdef LOGLEVEL_DEBUG
          message.str(""); message << "[DEBUG] erasing " << vec->first; logwrite( function, message.str() );
#endif
          this->keydb.erase( vec );
        }
        return;
      }
  };
  /**************** Common::FitsKeys ******************************************/


  /**************** Common::Queue *********************************************/
  /**
   * @class  Queue
   * @brief  provides a thread-safe messaging queue
   *
   */
  class Queue {
    private:
      std::queue<std::string> message_queue;
      mutable std::mutex queue_mutex;
      std::condition_variable notifier;
      bool is_running;
    public:
      Queue(void) : message_queue() , queue_mutex() , notifier() { this->is_running = false; };
      ~Queue(void) {}

      void service_running(bool state) { this->is_running = state; };  /// set service running
      bool service_running() { return this->is_running; };             /// is the service running?

      void enqueue(std::string message);                               /// push an element into the queue.
      std::string dequeue(void);                                       /// pop an element from the queue
  };
  /**************** Common::Queue *********************************************/

}
