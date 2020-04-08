/**
 * @file    archon.h
 * @brief   
 * @details 
 * @author  David Hale <dhale@astro.caltech.edu>
 *
 */
#ifndef ARCHON_H
#define ARCHON_H

#include <CCfits/CCfits>                 //!< needed here for types in set_axes()
#include <atomic>

// number of observing modes
//#define NUM_OBS_MODES 1

// poll timeout in msec
#define POLLTIMEOUT 5000

// number of modules per controller
#define NMODS 12

// number of channels per ADC module
#define NADCHAN 4

// max number of AD channels per controller
#define MAXADCHANS 16

// max number of CCDs handled by one controller
#define MAXCCDS 4

// Archon block size
#define BLOCK_LEN   1024

// Reply buffer size (over-estimate)
#define REPLY_LEN   100 * BLOCK_LEN

// Archon commands
#define  SYSTEM        std::string("SYSTEM")
#define  STATUS        std::string("STATUS")
#define  FRAME         std::string("FRAME")
#define  CLEARCONFIG   std::string("CLEARCONFIG")
#define  POLLOFF       std::string("POLLOFF")
#define  POLLON        std::string("POLLON")
#define  APPLYALL      std::string("APPLYALL")
#define  POWERON       std::string("POWERON")
#define  POWEROFF      std::string("POWEROFF")
#define  APPLYCDS      std::string("APPLYCDS")
#define  RESETTIMING   std::string("RESETTIMING")
#define  HOLDTIMING    std::string("HOLDTIMING")
#define  RELEASETIMING std::string("RELEASETIMING")
#define  LOADPARAMS    std::string("LOADPARAMS")
#define  TIMER         std::string("TIMER")
#define  FETCHLOG      std::string("FETCHLOG")
#define  UNLOCK        std::string("LOCK0")

namespace Archon {

  typedef enum {
    FRAME_IMAGE,
    FRAME_RAW,
    NUM_FRAME_TYPES
  } frame_type_t;

  const char * const frame_type_str[NUM_FRAME_TYPES] = {
    "IMAGE",
    "RAW"
  };

  class Information {
    private:
    public:
      std::string   hostname;                //<! Archon controller hostname
      int           port;                    //!< Archon controller TPC/IP port number
      int           nbufs;                   //!< Archon controller number of frame buffers
      int           bitpix;
      std::string   configfilename;          //!< Archon controller configuration file
      frame_type_t  frame_type;              //!< frame_type is IMAGE or RAW
      long          detector_pixels[2];
      long          image_size;              //!< pixels per image sensor
      long          image_memory;            //!< bytes per image sensor
      int           current_observing_mode;
      int           bytes_per_pixel;
      long          naxis;
      long          axes[2];
      int           binning[2];
      long          axis_pixels[2];
      long          region_of_interest[4];
      long          image_center[2];
      bool          data_cube;
      std::string   image_name;
      std::string   fits_start_time;         //!< system time when the exposure started (YYYY-MM-DDTHH:MM:SS.sss)

      Information() {
        this->axes[0] = 1;
        this->axes[1] = 1;
        this->binning[0] = 1;
        this->binning[1] = 1;
        this->region_of_interest[0] = 1;
        this->region_of_interest[1] = 1;
        this->region_of_interest[2] = 1;
        this->region_of_interest[3] = 1;
        this->image_center[0] = 1;
        this->image_center[1] = 1;
        this->data_cube = false;
        this->image_name = "/tmp/test.fits";
      }

      void set_axes(int bitpix_in) {
        this->bitpix = bitpix_in;

        switch(this->bitpix){
          case BYTE_IMG:
          case SBYTE_IMG:
            this->bytes_per_pixel = 1;
            break;
          case SHORT_IMG:
          case USHORT_IMG:
            this->bytes_per_pixel = 2;
            break;
          case LONG_IMG:
          case ULONG_IMG:
          case FLOAT_IMG:
            this->bytes_per_pixel = 4;
            break;
          case LONGLONG_IMG:
          case DOUBLE_IMG:
            this->bytes_per_pixel = 8;
            break;
          default:
            this->bytes_per_pixel = 4;
        }

        this->naxis = 2;

        this->axis_pixels[0] = this->region_of_interest[1] -
                               this->region_of_interest[0] + 1;
        this->axis_pixels[1] = this->region_of_interest[3] -
                               this->region_of_interest[2] + 1;

        this->axes[0] = this->axis_pixels[0] / this->binning[0];
        this->axes[1] = this->axis_pixels[1] / this->binning[1];

        this->image_size   = this->axes[0] * this->axes[1];                          // Pixels per CCD
        this->image_memory = this->axes[0] * this->axes[1] * this->bytes_per_pixel;  // Bytes per CCD
      }
  };

  class Interface {
    private:
      unsigned long int start_time, finish_time;  //!< Archon internal timer, start and end of exposure

    public:
      Interface();
      ~Interface();

      bool connection_open;                  //!< is there a connection open to the controller?
      std::string current_state;             //!< current state of the controller
      int  sockfd;                           //!< socket file descriptor to Archon controller
      int  msgref;                           //!< Archon message reference identifier, matches reply to command
      int  taplines;
      int  gain[MAXADCHANS];                 //!< digital CDS gain (from TAPLINE definition)
      int  offset[MAXADCHANS];               //!< digital CDS offset (from TAPLINE definition)

      char *image_data;                      //!< image data buffer
      uint32_t image_data_bytes;             //!< requested number of bytes allocated for image_data rounded up to block size
      uint32_t image_data_allocated;         //!< allocated number of bytes for image_data

      std::atomic<bool> archon_busy;         //!< indicates a thread is accessing Archon
      std::mutex archon_mutex;               //!< protects Archon from being accessed by multiple threads,
                                             //!< use in conjunction with archon_busy flag

      long prepare_image_buffer();           //!< prepare image_data, allocating memory as needed
      long connect_controller();             //!< open connection to archon controller
      long disconnect_controller();          //!< disconnect from archon controller
      long load_config(std::string cfgfile);
      long set_camera_mode(int mode);
      long load_mode_settings(int mode);
      long archon_native(std::string cmd);
      long archon_cmd(std::string cmd);
      long archon_cmd(std::string cmd, char *reply);
      long read_parameter(std::string paramname, std::string &valstring);
      long prep_parameter(std::string paramname, std::string value);
      long load_parameter(std::string paramname, std::string value);
      long fetchlog();
      long get_frame_status();
      long print_frame_status();
      long lock_buffer(int buffer);
      long get_timer(unsigned long int *timer);
      long fetch(uint64_t bufaddr, uint32_t bufblocks);
      long read_frame();                     //!< read Archon frame buffer into host memory
      long write_frame();                    //!< write (a previously read) Archon frame buffer to disk
      long write_config_key( const char *key, const char *newvalue, bool &changed );
      long write_config_key( const char *key, int newvalue, bool &changed );
      long write_parameter( const char *paramname, const char *newvalue, bool &changed );
      long write_parameter( const char *paramname, int newvalue, bool &changed );
      long write_parameter( const char *paramname, const char *newvalue );
      long write_parameter( const char *paramname, int newvalue );
      long expose();

      Information camera_info;
      Information fits_info;                 //!< copy of camera_info class used to preserve info for FITS writing

      typedef enum {
        MODE_DEFAULT = 0,
        MODE_RAW,
        NUM_OBS_MODES
      } Observing_modes;

      const char * const Observing_mode_str[NUM_OBS_MODES] = {
        "MODE_DEFAULT",
        "MODE_RAW"
      };

      /**
       * @var     struct geometry_t geometry[]
       * @details structure of geometry which is unique to each observing mode
       */
      struct geometry_t {
        int  amps_per_ccd[2];      // number of amplifiers per CCD for each axis, set in set_camera_mode
        int  num_ccds;             // number of CCDs, set in set_camera_mode
        int  linecount;            // number of lines per tap
        int  pixelcount;           // number of pixels per tap
      } geometry[NUM_OBS_MODES];

      /**
       * @var     struct tapinfo_t tapinfo[]
       * @details structure of tapinfo which is unique to each observing mode
       */
      struct tapinfo_t {
        int   num_taps;
        int   tap[16];
        float gain[16];
        float offset[16];
        std::string readoutdir[16];
      } tapinfo[NUM_OBS_MODES];

      /**
       * @var     struct frame_data_t frame
       * @details structure to contain Archon results from "FRAME" command
       */
      struct frame_data_t {
        int      index;                       // index of newest buffer data
        char     timer[17];                   // current hex 64 bit internal timer
        int      rbuf;                        // current buffer locked for reading
        int      wbuf;                        // current buffer locked for writing
        std::vector<int>      bufsample;      // sample mode 0=16 bit, 1=32 bit
        std::vector<int>      bufcomplete;    // buffer complete, 1=ready to read
        std::vector<int>      bufmode;        // buffer mode: 0=top 1=bottom 2=split
        std::vector<uint64_t> bufbase;        // buffer base address for fetching
        std::vector<int>      bufframe;       // buffer frame number
        std::vector<int>      bufwidth;       // buffer width
        std::vector<int>      bufheight;      // buffer height
        std::vector<int>      bufpixels;      // buffer pixel progress
        std::vector<int>      buflines;       // buffer line progress
        std::vector<int>      bufrawblocks;   // buffer raw blocks per line
        std::vector<int>      bufrawlines;    // buffer raw lines
        std::vector<int>      bufrawoffset;   // buffer raw offset
        std::vector<int>      buftimestamp;   // buffer hex 64 bit timestamp
        std::vector<int>      bufretimestamp; // buf trigger rising edge time stamp
        std::vector<int>      buffetimestamp; // buf trigger falling edge time stamp
      } frame;

      /** @var      int lastframe
       *  @details  the last (I.E. previous) frame number acquired
       */
      int lastframe;

      /**
       * rawinfo_t is a struct which contains variables specific to raw data functions
       */
      struct rawinfo_t {
        int adchan;          // selected A/D channels
        int raw_samples;     // number of raw samples per line
        int raw_lines;       // number of raw lines
        int iteration;       // iteration number
        int iterations;      // number of iterations
      } rawinfo;

      /**
       * config_line_t is a struct for the configfile key=value map, used to
       * store the configuration line and its associated line number.
       */
      typedef struct {
        int    line;           // the line number, used for updating Archon
        std::string value;     // used for configmap
      } config_line_t;

      /**
       * param_line_t is a struct for the PARAMETER name key=value map, used to
       * store parameters where the format is PARAMETERn=parametername=value
       */
      typedef struct {
        std::string key;       // the PARAMETERn part
        std::string name;      // the parametername part
        std::string value;     // the value part
        int    line;           // the line number, used for updating Archon
      } param_line_t;

      typedef struct {
        std::string keyword;
        std::string keytype;
        std::string keyvalue;
        std::string keycomment;
      } user_key_t;

      typedef std::map<std::string, config_line_t>  cfg_map_t;
      typedef std::map<std::string, param_line_t>   param_map_t;
      typedef std::map<std::string, user_key_t>     fits_key_t;

      cfg_map_t   configmap;
      param_map_t parammap;

      /** 
       * \var     modeinfo_t modeinfo
       * \details structure contains a configmap and parammap unique to each mode,
       *          specified in the [MODE_*] sections at the end of the .acf file.
       */
      struct modeinfo_t {
        bool        defined;     //!< clear by default, set if mode section encountered in .acf file
        int         rawenable;   //!< initialized to -1, then set according to RAWENABLE in .acf file
        cfg_map_t   configmap;   //!< key=value map for configuration lines set in mode sections
        param_map_t parammap;    //!< PARAMETERn=parametername=value map for mode sections
        fits_key_t  userkeys;    //!< user supplied FITS header keys
      } modeinfo[NUM_OBS_MODES];

      /**
       * generic key=value STL map for Archon commands
       */
      typedef std::map<std::string, std::string> map_t;

      /**
       * \var     map_t systemmap
       * \details key=value map for Archon SYSTEM command
       */
      map_t systemmap;

      /**
       * \var     map_t statusmap
       * \details key=value map for Archon STATUS command
       */
      map_t statusmap;

  };

}

#endif
