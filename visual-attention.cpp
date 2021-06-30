                                                                                                                              /*
 Copyright Mykola Rabchevskiy 2021.
 Distributed under the Boost Software License, Version 1.0.
 (See http://www.boost.org/LICENSE_1_0.txt)
 ______________________________________________________________________________


  2021.06.26  Initial version


  NOTE: Some USB3 cameras exposes specific behavior: re-launched program does not
        run as expected (no image). It can be fixed by running system command that
        restarted `uvcvideo` (camera driver):

          sudo rmmod uvcvideo && sudo modprobe uvcvideo



________________________________________________________________________________________________________________________________
                                                                                                                              */
#include <cassert>

#include <iostream>
#include <vector>
#include <string>

#include "config.h"
#include "camera.h"
#include "ocular.h"


int main( int argc, char* argv[] ){

  using namespace CoreAGI;
  using namespace CoreAGI::Visual;

  constexpr const char* DEV{ "/dev" };

  printf( "\n %sCore AGI :: Visual Attention :: vers 2021.06%s\n", YELLOW, RESET );    

  printf( "\n %sNOTE:%s Some USB3 cameras exposes specific behavior: re-launched program does not", CYAN, YELLOW );
  printf( "\n       run as expected (no image). It can be fixed by running system command that" );
  printf( "\n       restarted `uvcvideo` (camera driver) using shell comand:" );
  printf( "\n\n         %ssudo rmmod uvcvideo && sudo modprobe uvcvideo%s\n", GREEN, RESET );
      
  if( argc > 1 ){
    printf( "\n %sUsed provided path to list of the video devices:%s %s%s\n", YELLOW, GREEN, argv[1], RESET );    
  } else {
    printf( "\n %sUsed default path to list of the video devices:%s %s%s\n", YELLOW, GREEN, DEV, RESET );    
  }

  auto accessible = videoDevices( argc > 1 ? argv[1] : "/dev" );
  if( accessible.empty() ){
    printf( "\n\n There are no accessible video devices\n" );
    exit( EXIT_FAILURE );
  }

  constexpr const char* TITLE{
    "\n Select video device; in most cases it is first one."
    "\n Note also that single camera can be represented as pair of devices;"
    "\n in such case try first items with even index:"
  };

  //const std::string path{ choice( TITLE, accessible ) };
  const auto path = accessible[ choice( TITLE, accessible ) ];
  printf( "\n Selected: %s\n", path.c_str() );
  printf( "\n [Main  ] Create Ocular instance.." ); fflush( stdout );

  Ocular terminal{ path.c_str() };

  while( terminal.live() ) std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
  printf( "\n [Main  ] Finished\n" );

  return EXIT_SUCCESS;
}
