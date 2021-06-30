                                                                                                                              /*
 Copyright Mykola Rabchevskiy 2021.
 Distributed under the Boost Software License, Version 1.0.
 (See http://www.boost.org/LICENSE_1_0.txt)
 ______________________________________________________________________________


 Wrapper for linux/videodev2

 2021.06.26 Initial version

________________________________________________________________________________________________________________________________
                                                                                                                              */
#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

#include <fcntl.h>       // : O_RDWR, O_NONBLOCK, ...
#include <sys/ioctl.h>
#include <unistd.h>

#include <cassert>
#include <clocale>
#include <cstdio>
#include <cstring>

#include "linux/videodev2.h"

#include <algorithm>
#include <filesystem>
#include <functional>
#include <ranges>
#include <string>
#include <tuple>
#include <vector>

#include "def.h"
#include "planar.h"

namespace CoreAGI::Visual {
                                                                                                                              /*
  Get info about available camera`s formats:

  v4l2-ctl -dX --list-formats-ext where X is the camera index as in /dev/videoX
                                                                                                                              */

  std::vector< std::string > videoDevices( const char* folder ){
                                                                                                                              /*
    List of Linux video devices:
                                                                                                                              */
    std::vector< std::string > devices;
    for( auto& p: std::filesystem::directory_iterator( folder ) ){
      std::string item{ p.path().filename().string() };
      if( item.starts_with( "video" ) ) devices.push_back( p.path().string() );
    }
    std::ranges::sort( devices );
    return devices;
  }


  enum PixelFormat: uint32_t { YUYV, UYVY };

  const char* LEX( PixelFormat f ){
    switch( f ){
      case YUYV: return "YUYV";
      case UYVY: return "UYVY";
      default  : break;
    }
    return "????";
  }

  using PixelCoord = CoreAGI::Planar::PixelCoord;

  struct FrameDef: public PixelCoord {
    PixelFormat pixelFormat;
    FrameDef(): PixelCoord(), pixelFormat{ YUYV }{}
    FrameDef( int16_t w, int16_t h, PixelFormat f ): PixelCoord( w, h ), pixelFormat{ f }{
      // printf( "\n [FrameDef] w: %i; h: %i; f: %s", x, y, LEX( pixelFormat ) );                                       // DEBUG
      // fflush( stdout );                                                                                              // DEBUG
    }
  };


  template< unsigned NUMBER_OF_BUFFERS > class Camera {
                                                                                                                              /*
    Camera must provide frames in YUYV format
                                                                                                                              */
    static_assert( NUMBER_OF_BUFFERS >= 2 );

    int             descriptor;
    const char*     err;
    v4l2_capability cap;
    v4l2_format     f;

    std::vector< unsigned long > buffer[ NUMBER_OF_BUFFERS ];

    int xioctl( int request, void* arg ){
      int r;
      do r = ioctl( descriptor, request, arg ); while( -1 == r && EINTR == errno );
      return r;
    }

  public:

    std::vector< FrameDef > acceptableFrameDef;

    Camera& operator = ( const Camera& ) = delete;
    Camera             ( const Camera& ) = delete;

    const char* error       () const { return err;                                                       }
    unsigned    width       () const { return f.fmt.pix.width;                                           }
    unsigned    height      () const { return f.fmt.pix.height;                                          }
    unsigned    bytesPerLine() const { return f.fmt.pix.bytesperline;                                    }
    unsigned    size        () const { return f.fmt.pix.sizeimage;                                       }
    PixelFormat pixelFormat () const { return  f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV ? YUYV : UYVY; }

    void clearErr(){ err = nullptr; }

    Camera(): descriptor{ -1 }, err{ nullptr }, cap{}, f{}, acceptableFrameDef{}{}

    Camera( const char* path ):
                                                                                                                              /*
      Function `acceptable` returns `true` if argument-frame-size is acceptable
                                                                                                                              */
      descriptor        { -1      },
      err               { nullptr },
      cap               {         },
      f                 {         },
      acceptableFrameDef{         }
    {

      printf( "\n [Camera] constructor started" ); fflush( stdout ); // DEBUG
                                                                                                                              /*
      Check path:
                                                                                                                              */
      if( not path ){ err = "NULL path"; return; }
                                                                                                                              /*
      Open:
                                                                                                                              */
      printf( "\n [Camera] Open %s..", path ); fflush( stdout );
      descriptor = open( path, O_RDWR /* required */ | O_NONBLOCK, 0 );
      if( descriptor < 0 ){
        printf( "\n [Camera] Can`t open %s", path ); fflush( stdout );
        err = std::strerror( errno );
        return;
      }
                                                                                                                              /*
      Get capabilities:
                                                                                                                              */
      printf( "\n [Camera] Get capabilities.." );
      if( -1 == xioctl( VIDIOC_QUERYCAP, &cap ) ){
        printf( "\n [Camera] Can`t get capabilities" ); fflush( stdout );
        err = std::strerror( errno );
        return;
      }
                                                                                                                              /*
      Check capabilities:
                                                                                                                              */
      if( not( cap.capabilities & V4L2_CAP_VIDEO_CAPTURE ) ){
        printf( "\n [Camera] Device %s is not a video capture device", path );
        err = "Device is not a video capture device";
        return;
      }

      if( not( cap.capabilities & V4L2_CAP_STREAMING ) ){
        printf( "\n [Camera] Device %s does not support streaming i/o", path );
        err = "Device does not support streaming i/o"; return;
      }
                                                                                                                              /*
      Set video format:
                                                                                                                              */
      CLEAR( f );
      f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                                                                                                                              /*
      Get available frame sizes:
                                                                                                                              */
      printf( "\n [Camera] Get available frame formats..\n" );
      v4l2_frmsizeenum s;
      memset( &s, 0, sizeof( s ) );  // NB [+] 2021.06.22
      for( uint32_t i = 0; ; i++ ){
        s.index        = i;
        s.pixel_format = V4L2_PIX_FMT_YUYV;
        if( xioctl( VIDIOC_ENUM_FRAMESIZES, &s ) < 0 ) break;
        if( s.pixel_format == V4L2_PIX_FMT_YUYV ){
          if( s.discrete.width  > CoreAGI::Config::camera::FRAME_MAX_WIDTH  ) continue;
          if( s.discrete.height > CoreAGI::Config::camera::FRAME_MAX_HEIGHT ) continue;
          FrameDef frameDef = FrameDef( s.discrete.width, s.discrete.height, YUYV );
          acceptableFrameDef.emplace_back( frameDef );
        }
      }
      memset( &s, 0, sizeof( s ) );  // NB [+] 2021.06.22
      for( uint32_t i = 0; ; i++ ){
        s.index        = i;
        s.pixel_format = V4L2_PIX_FMT_UYVY;
        if( xioctl( VIDIOC_ENUM_FRAMESIZES, &s ) < 0 ) break;
        if( s.pixel_format == V4L2_PIX_FMT_UYVY ){
          FrameDef frameDef = FrameDef( s.discrete.width, s.discrete.height, UYVY );
          //if( acceptable( frameDef ) ) acceptableFrameDef.emplace_back( frameDef );
          acceptableFrameDef.emplace_back( frameDef );
        }
      }
                                                                                                                              /*
      Check if acceptable frame size presented:
                                                                                                                              */
      if( acceptableFrameDef.empty() ){
        printf( "\n [Camera] There are no acceptable frame sizes for YUYU ot UYUV mode" ); fflush( stdout );
        err = "There are no acceptable frame sizes for YUYU ot UYUV mode"; return;
      }
                                                                                                                              /*
      Sort acceptable frame sizes by ascending and use last one (biggest):
                                                                                                                              */
      std::ranges::sort(
        acceptableFrameDef,
        []( const FrameDef& L, FrameDef& R )->bool{ return L.x*L.y < R.x*R.y; }
      );
                                                                                                                              /*
      Select frame format:
                                                                                                                              */
      std::vector< std::string > choices;
      for( const auto& f: acceptableFrameDef ){
        char row[ 256 ];
        sprintf( row, "%s%s %s%u x %u%s", YELLOW, f.pixelFormat == YUYV ? "YUYV" : "UYVY", CYAN, f.x, f.y, RESET );
        choices.push_back( row );
      }
      constexpr const char* TITLE{ "\n Select frame format:"  };
      int ordinal{ CoreAGI::choice( TITLE, choices ) };
      auto frameDef = acceptableFrameDef[ ordinal ];
      printf( "\n Selected: %s\n", choices[ ordinal ].c_str() );
                                                                                                                              /*
      Set W x H { YUYV | UYVY } INTERLACED:
                                                                                                                              */
      f.fmt.pix.width  = frameDef.x;
      f.fmt.pix.height = frameDef.y;
      switch( frameDef.pixelFormat ){
        case YUYV: f.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; break;
        case UYVY: f.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY; break;
        default  : assert( false );
      }
      f.fmt.pix.field = V4L2_FIELD_INTERLACED;
      if( xioctl( VIDIOC_S_FMT, &f ) < 0 ){ err = "VIDIOC_S_FMT"; return; }
                                                                                                                              /*
      Note: VIDIOC_S_FMT may change width and height
      Buggy driver paranoia:
                                                                                                                              */
      unsigned min = f.fmt.pix.width * 2; // :two bytes per pixel
      if( f.fmt.pix.bytesperline < min ) f.fmt.pix.bytesperline = min;
      min = f.fmt.pix.bytesperline * f.fmt.pix.height;
      if( f.fmt.pix.sizeimage < min ) f.fmt.pix.sizeimage = min;
                                                                                                                              /*
      Set user pointer mode:
                                                                                                                              */
      struct v4l2_requestbuffers req;
      CLEAR( req );
      req.count  = NUMBER_OF_BUFFERS;
      req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_USERPTR;
      if( -1 == xioctl( VIDIOC_REQBUFS, &req ) ){
        err = ( EINVAL == errno ) ? "It does not support user pointer i/o" : "VIDIOC_REQBUFS";
        return;
      }
                                                                                                                              /*
      Allocate buffers:
                                                                                                                              */
      const size_t bufferSize { 1 + ( f.fmt.pix.sizeimage / sizeof( unsigned long ) ) };
      for( auto& b: buffer ) b.reserve( bufferSize );

      printf( "\n [Camera] constructor finished" ); fflush( stdout ); // DEBUG

    }//Camera constructor

    bool start(){
      clearErr();
      for( unsigned i = 0; i < NUMBER_OF_BUFFERS; ++i ){
        v4l2_buffer buf;
        CLEAR( buf );
        buf.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory    = V4L2_MEMORY_USERPTR;
        buf.index     = i;
        buf.m.userptr = (unsigned long)buffer[i].data();
        buf.length    = f.fmt.pix.sizeimage;
        if( xioctl( VIDIOC_QBUF, &buf ) < 0 ){
          // err = "VIDIOC_QBUF";
          err = std::strerror( errno );
          return false;
        }
      }
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if( xioctl( VIDIOC_STREAMON, &type ) < 0 ){ err = "VIDIOC_STREAMON"; return false; }
      return true;
    }

    bool stop(){
      clearErr();
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if( xioctl( VIDIOC_STREAMOFF, &type ) < 0 ){
        // err = "VIDIOC_STREAMOFF";
        err = std::strerror( errno );
        return false;
      }
      return true;
    }

    bool processFrame( std::function< void( void*, unsigned, unsigned, unsigned ) > F ){
      clearErr();
      v4l2_buffer buf;
      CLEAR( buf );
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      if( -1 == xioctl( VIDIOC_DQBUF, &buf ) ){
        switch( errno ){
          case EAGAIN : err = std::strerror( errno ); /* EAGAIN       */ return false;
          case EIO    : // :could ignore EIO, see spec.; fall through.
          default     : err = std::strerror( errno ); /* VIDIOC_DQBUF */ return false;
        }
      }
      if( buf.bytesused != f.fmt.pix.sizeimage ){
        err = "Partial frame";
      } else {
        F( (void*)buf.m.userptr, buf.bytesused, width(), height() );
      }
      if( xioctl( VIDIOC_QBUF, &buf ) < 0 ){
        // err = "VIDIOC_QBUF";
        err = std::strerror( errno );
        return false;
      }
      return true;
    }

   ~Camera(){
      int rc = close( descriptor );
      if( rc < 0 ) printf( "\n\n CAMERA CLOSING FAILED\n" );
      descriptor = -1;
    }

  };//class Camera

}//namespace CoreAGI::Visual

#endif // CAMERA_H_INCLUDED
