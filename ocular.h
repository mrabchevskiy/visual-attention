                                                                                                                              /*
 Copyright Mykola Rabchevskiy 2021.
 Distributed under the Boost Software License, Version 1.0.
 (See http://www.boost.org/LICENSE_1_0.txt)
 ______________________________________________________________________________


 2021.06.26 Initial version

________________________________________________________________________________________________________________________________
                                                                                                                              */
#ifndef OCULAR_H_INCLUDED
#define OCULAR_H_INCLUDED

#include <SFML/Graphics.hpp>

#include <atomic>
#include <cmath>
#include <span>

#include "active.h"
#include "approximation.h"
#include "camera.h"
#include "def.h"
#include "honeycomb.h"
#include "planar.h"
#include "queue.h"
#include "timer.h"


namespace CoreAGI::Visual {

  using namespace CoreAGI;
  using namespace CoreAGI::Planar;


  struct Attentiveness {

    static constexpr unsigned CAPACITY{ Honeycomb::SIZE };

    struct Sample{ int8_t x; int8_t y; uint8_t brightness; };

    Sample  sample[ CAPACITY ]; // :array of samples
    double  D;                  // :dynamism criterion = for average difference between frames
    double  R;                  // :attentiveness criterion  = standard deviation from linearized brigtness
    uint8_t n;                  // :current number of samples

    uint8_t incl( int8_t x, int8_t y, uint8_t brightness, uint8_t prev ){
      assert( n < CAPACITY );
      sample[ n++ ] = Sample{ x, y, brightness };
      D += ( prev > brightness ) ? prev - brightness : brightness - prev;
      return n;
    }

    void process(){
                                                                                                                              /*
      Do nothing if number of actual pixels is too small:
                                                                                                                              */
      if( n < 15 ) return;

      D /= double( n );
                                                                                                                              /*
      Standard deviation of actual brightness from linearized one
                                                                                                                              */
      using Linear = Approximation< 3, 2, double >;
                                                                                                                              /*
      Functional basis and linearization object:
                                                                                                                              */
      Linear::BasisFunction F[3]{
        []( [[maybe_unused]] std::span< double > r )-> double{ return 1.0d; },
        [](                  std::span< double > r )-> double{ return r[0]; },
        [](                  std::span< double > r )-> double{ return r[1]; }
      };
      Linear linear{ F };
                                                                                                                              /*
      Add points:
                                                                                                                              */
      for( unsigned i = 0; i < n; i++ ){
        Sample& Si{ sample[i] };
        double P[2]{ double( Si.x ), double( Si.y ) };
        std::span< double > C( &P[0], 2 );
        linear.point( C, double( Si.brightness ) );
      }
                                                                                                                              /*
      Make linearization:
                                                                                                                              */
      constexpr double COND{ 1.0e4 };       // :condition
      unsigned nc = linear.process( COND ); // :number of eigen vectors
      assert( nc == 3 );
                                                                                                                              /*
      Calculate standard deviation:
                                                                                                                              */
      double Rsq{ 0.0 };
      for( unsigned i = 0; i < n; i++ ){
        Sample& Si{ sample[i] };
        const Cartesian Ci{ double( Si.x ), double( Si.y ) };   // :convert coordinate system
          const double Bi{ linear( Ci ) };                      // :linearized brightness
          const double diff{ Si.brightness - Bi };              // :difference
          Rsq += diff*diff;
      }
      R = sqrt( Rsq / double( n ) );
    }

  };//class Attentiveness


  class Ocular: public CoreAGI::Active {
  protected:

    const char*         device;
    std::atomic< bool > failure;

  public:

    Ocular( const char* device ): device{ device }, failure{ false }{
      start();
      while( not live() ){
        std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
      }
    }

   ~Ocular(){}

  private:

    virtual void run() override {

      constexpr unsigned CAMERA_BUFFERS  { CoreAGI::Config::visualattention::CAMERA_BUFFERS };

      constexpr int      MAX_WIDTH       { CoreAGI::Config::camera::FRAME_MAX_WIDTH         };
      constexpr int      MAX_HEIGHT      { CoreAGI::Config::camera::FRAME_MAX_HEIGHT        };

      printf( "\n [Ocular] run() started" ); fflush( stdout );

      constexpr unsigned BYTES_PER_PIXEL {    4 }; // :in the Texture, not in the video frame

      constexpr unsigned TEXTURE_CAPACITY{ MAX_WIDTH * MAX_HEIGHT * BYTES_PER_PIXEL }; // :required texture capacity

      constexpr const char* TITLE{ "AGI terminal :: use `Ctrl+Del` to finish; `Ctrl+Insert to save frame as *.png file" };

      const unsigned MAX_TEXTURE_SIZE = sf::Texture::getMaximumSize();

      printf( "\n [Ocular] Maximal texture size (depend on videocard): %u pixels", MAX_TEXTURE_SIZE ); fflush( stdout );
      printf( "\n [Ocular] Start camera `%s`..", device ); fflush( stdout );

      CoreAGI::Visual::Camera< CAMERA_BUFFERS > camera( device );
      if( camera.error() ){
        printf( "\n [Ocular] %s; try another camera", camera.error() ); fflush( stdout );
        failure.store( true );
        return;
      }
      assert( camera.width () <= MAX_TEXTURE_SIZE );
      assert( camera.height() <= MAX_TEXTURE_SIZE );
      printf( "\n [Ocular] Camera object created, frame size %u x %u\n", camera.width(), camera.height() ); fflush( stdout );

      if( not camera.start() ){
        printf( "\n [Ocular] Can`t start camera: %s", camera.error() ); fflush( stdout );
        failure.store( true );
        return;
      }
      printf( "\n [Ocular] Camera `%s` started, frame size %u x %u\n", device, camera.width(), camera.height() );
      fflush( stdout );

      class Fag: public Active {
        Attentiveness* attentiveness;
        Queue< unsigned, 8*1024 > Q;
        virtual void run(){
          terminated = false;
          for(;;){
            if( terminate ) break;
            while( not Q.empty() ){
              unsigned cell = Q.pull();
              if( cell == 0 ) break;
              attentiveness[ --cell ].process(); // :cell index in the queue started from 1
            }
            std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
          }
          terminated = true;
        }//run
      public:
        bool idle() const { return Q.empty(); }
        bool push( unsigned i ){ return Q.push( i ); }
        Fag( Attentiveness* A ): attentiveness{ A }, Q{}{
          start();
          while( not live() ) std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
        }
      };//class Fag

      const unsigned NUMPIX{ camera.width() * camera.height() };
      assert( camera.width () <= MAX_TEXTURE_SIZE );
      assert( camera.height() <= MAX_TEXTURE_SIZE );

      using namespace CoreAGI::Visual;
                                                                                                                              /*
      Compose hexagonal mesh:
                                                                                                                              */
      Honeycomb honeycomb{ camera.width(), camera.height() };
                                                                                                                              /*
      Allocate data:
                                                                                                                              */
      struct RGB { uint8_t R; uint8_t G; uint8_t B; };

      RGB*           color         = new RGB          [ honeycomb.size() ]; assert( color         );
      Attentiveness* attentiveness = new Attentiveness[ honeycomb.size() ]; assert( attentiveness );

      constexpr unsigned NUMBER_OF_FAGS{ CoreAGI::Config::visualattention::NUMBER_OF_FAGS };

      Fag* fag[ NUMBER_OF_FAGS ];
      for( auto& Pi: fag ) Pi = new Fag( attentiveness );
                                                                                                                              /*
      Allocate data for moving processing:
                                                                                                                              */
      uint8_t* prev = new uint8_t[ NUMPIX ];         // :previous frame brightness, one byte per pixel
      memset( prev, 0, NUMPIX*sizeof( uint8_t ) );

      const Planar::PixelCoord videoLoc{ 128, 16 };  // :video frame location

      Planar::PixelCoord WINDOW( videoLoc.x + camera.width() + 8, videoLoc.y + camera.height() + 8 ); // :window size
                                                                                                                              /*
      Create the main window
                                                                                                                              */
      sf::ContextSettings windowSettings;
      windowSettings.antialiasingLevel = 4;
                                                                                                                              /*
      Window Style::Titlebar disables window resizing/distorting:
                                                                                                                              */
      sf::RenderWindow window( sf::VideoMode( WINDOW.x, WINDOW.y ), TITLE, sf::Style::Titlebar, windowSettings );
      window.clear( sf::Color( 96, 96, 96 ) );
      window.display();
      std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

      uint8_t frame[ TEXTURE_CAPACITY ]; // :output frame  // NB

      const unsigned W { camera.width () };
      const unsigned H { camera.height() };
      const int      Xo{ int( W / 2 )    };   // :center
      const int      Yo{ int( H / 2 )    };   // :center

      const PixelFormat FMT{ camera.pixelFormat() }; // :flag for `compose` (see below)

      std::atomic< bool > processFrame{ true };
                                                                                                                              /*
      Function that called by video driver to process video frame:
                                                                                                                              */
      auto compose = [&]( void* rawYUYV,
        [[maybe_unused]] unsigned bytes,
        [[maybe_unused]] unsigned width,
        [[maybe_unused]] unsigned height )
      {
        if( not processFrame ) return;
                                                                                                                              /*
        Loop over raw frame data and convert four YUYV bytes (two pixels) into 8 RGBA bytes (two pixes):
                                                                                                                              */
        assert( bytes  == 2*NUMPIX        );
        assert( width  == camera.width()  );
        assert( height == camera.height() );
                                                                                                                              /*
        Clear attentiveness data:
                                                                                                                              */
        memset( attentiveness, 0, sizeof( Attentiveness )*honeycomb.size() );

        uint8_t* inp { (uint8_t*)rawYUYV };
        uint8_t* out { frame             };

        const unsigned* M { honeycomb.mapping() };
                                                                                                                              /*
        Loop over pairs of frame pixels ( 2 bytes per pixel, [YU][YV] or [UY][VY] ):
                                                                                                                              */
        unsigned px{ 0 }; // :pixel index in the frame
        unsigned x { 0 }; // :pixel coordinate
        unsigned y { 0 }; // :pixel coordinate
        unsigned f { 0 }; // :fag index
        for( unsigned i = 0; i < NUMPIX/2; i++ ){
          {                                                                                                                   /*
            First pixel of the current pair:
                                                                                                                              */
            uint8_t Y{ 0 };                                                // :pixe; brightness
            switch( FMT ){
              case YUYV: Y = *inp; inp++; /* U */   inp++; break;          // :select brightness from the first  byte
              case UYVY: /* U */   inp++; Y = *inp; inp++; break;          // :select brightness from the second byte
            }//switch FMT
            const unsigned c    = M[ px ];                                 // :hexagonal cell index
            const HexCell& cell = honeycomb[c];                            // :cell object
            const int      dx   =  int( x ) - Xo - cell.x;                 // :distance of pixel to cell center
            const int      dy   = -int( y ) + Yo - cell.y;                 // :distance of pixel to cell center
            const uint8_t  n    = attentiveness[c].incl( dx, dy, Y, prev[ px ] ); // :add sample to cell, n = number of samples
            prev[ px ] = Y;                                                // :save pixel brightness
            if( n >= cell.N ){                                             // :if cell complete
              assert( fag[f]->push( c + 1 ) );                        // :push index of complete cell into fag queue
              f++; f %= NUMBER_OF_FAGS;                              // :switch to next fag
            }
            assert( px < NUMPIX );                                                                                      // DEBUG
            px++;                                                          // :move to next pixel
            if( ++x >= W ){ x = 0, y++; };                                 // :update pixel coordinates
          }
          {                                                                                                                   /*
            Second pixel of the current pair:
                                                                                                                              */
            uint8_t Y{ 0 };
            switch( FMT ){
              case YUYV: Y = *inp; inp++; /* V */   inp++; break;
              case UYVY: /* V */   inp++; Y = *inp; inp++; break;
            }//switch FMT
            const unsigned c    = M[ px ];
            const HexCell& cell = honeycomb[c];
            const int      dx   =  int( x ) - Xo - cell.x;
            const int      dy   = -int( y ) + Yo - cell.y;
            const uint8_t  n    = attentiveness[c].incl( int8_t( dx ), int8_t( dy ), Y, prev[ px ] );
            prev[ px ] = Y;
            assert( f < NUMBER_OF_FAGS );
            if( n >= cell.N ){
              assert( f < NUMBER_OF_FAGS );
              assert( fag[f]->push( c + 1 ) );
              f++; f %= NUMBER_OF_FAGS;
            }
            assert( px < NUMPIX );
            px++;
            if( ++x >= W ){ x = 0, y++; };
          }
        }//for i
                                                                                                                              /*
        Wait for all processors in the idle state:
                                                                                                                              */
        for(;;){
          bool finished{ true };
          for( const auto& Pi: fag ){
            if( not Pi->idle() ){ finished = false; break; }
          }
          if( finished ) break;
          std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        }
                                                                                                                              /*
        VISUALIZATION __________________________________________________________________________________________________________
                                                                                                                              */
        {
                                                                                                                              /*
          Calculate max values of criteria; DMAX > 0 used to eliminate dynamic noise:
                                                                                                                              */
          constexpr double DMAX{ 128.0 };
          double Dmax{ DMAX };
          double Rmax{ 0    };
          for( unsigned i = 0; i < honeycomb.size(); i++ ){
            const auto& Ai{ attentiveness[i] };
            if( Ai.D > Dmax ) Dmax = Ai.D;
            if( Ai.R > Rmax ) Rmax = Ai.R;
          }//for i
                                                                                                                              /*
          Compose cell colors; scaling factors convert original value range to [ 0..MAX_COLOR ]:
                                                                                                                              */
          constexpr double MAX_COLOR{ 250.0 };
          const double Dfactor = MAX_COLOR/double( Dmax );
          const double Rfactor = MAX_COLOR/double( Rmax );
          for( unsigned i = 0; i < honeycomb.size(); i++ ){
            const auto& Ai{ attentiveness[i] };
            RGB&        Ci{ color        [i] };
            Ci.B = uint8_t( Rfactor*Ai.R ); // :edge   attentiveness
            Ci.R = uint8_t( Dfactor*Ai.D ); // :dynamic attentiveness
            Ci.G = Ci.B/2 + Ci.R/2;
          }//for i
        }
                                                                                                                              /*
        Compose image: loop over frame pixels:
                                                                                                                              */
        for( unsigned px = 0; px < NUMPIX; px++ ){
          const unsigned i{ M[ px ] }; // :cell index
          const RGB& Ci{ color[i] };
          *out = Ci.R; out++;
          *out = Ci.G; out++;
          *out = Ci.B; out++;
          *out = 255;  out++;
        }

      };//compose

      sf::Texture texture;
      sf::Image image;

      unsigned imgNum{ 1     };  // :saved image ordinal
      bool     save  { false };  // :saving flag activated by keyboard
                                                                                                                              /*
	    Main loop:
                                                                                                                              */
      terminated = false;     // :become live
      long unsigned N{ 1 };   // :frame counter
      Timer timer;
      while( window.isOpen() ){

        if( terminate ){ window.close(); continue; }
                                                                                                                              /*
        Refresh video frame:
                                                                                                                              */
        if( camera.processFrame( compose ) ){

          window.clear();
                                                                                                                              /*
          Prepare camera frame and draw as sprite:
                                                                                                                              */
          image.create( camera.width(), camera.height(), (uint8_t*)frame );
          if( save ){
            char path[ 64 ];
            sprintf( path, "ocular-frame-%03u.png", imgNum );
            if( image.saveToFile( std::string( path ) ) ){
              imgNum++;
              printf( "\n [Ocular] Frame saved as %s", path ); fflush( stdout );
            } else {
              printf( "\n [Ocular] Frame failed to save as %s", path );
            }
            save = false;
          }
          texture.loadFromImage( image );
          sf::Sprite sprite( texture );
          sprite.setPosition( float( videoLoc.x ), float( videoLoc.y ) );
          window.draw( sprite );
                                                                                                                              /*
          Update screen:
                                                                                                                              */
          window.display();
          N++;
          if( N >= 256 ){
            auto T = timer.elapsed()/double( N ); // :msec per frame
            printf( "\n [Ocular] Performance: %8.1f msec/frame", T ); fflush( stdout );                                                     // DEBUG
            N = 0;
            timer.start();
          }
        }
                                                                                                                              /*
        Process events:
                                                                                                                              */
        sf::Event event;
        while( window.pollEvent( event ) ){
          switch( event.type ){
            case sf::Event::Closed:
              window.close();
              break;
            case sf::Event::KeyPressed:
              switch( event.key.code ){
                case sf::Keyboard::Key::Insert: save = true;                            break;
                case sf::Keyboard::Key::Delete: if( event.key.control ) window.close(); break;
                default                       :                                         break;
              }
              break;
            default: break;
          }
        }//while event

     }//while window open

     processFrame.store( false ); // :stop frames processing

     printf( "\n [Ocular] Stop capturing.." );
     if( not camera.stop() ) printf( "\n [Ocular] WARNING: Can`t stop camera correctly" );
                                                                                                                              /*
     Stop child threads:
                                                                                                                              */
     printf( "\n [Ocular] Stopping Fag threads.." ); fflush( stdout );
     for( int i = 0; auto& Fi: fag ){
       printf( "\n [Ocular]   Stop Fag thread %2u..", ++i ); fflush( stdout );
       Fi->stop();
       while( Fi->live() ) std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
       printf( "  [ok]" ); fflush( stdout );
     }
     for( auto Fi: fag ) delete Fi;
     delete[] attentiveness;
     delete[] color;
     delete[] prev;
     std::this_thread::sleep_for( std::chrono::seconds( 1 ) ); // :pause just in case
     printf( "\n [Ocular] Finished\n" ); fflush( stdout );
     terminated = true;

   }//run

  };//class Ocular;

}// namespace CoreAGI::Vision;

#endif // OCULAR_H_INCLUDED
