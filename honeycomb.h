                                                                                                                              /*
 Copyright Mykola Rabchevskiy 2021.
 Distributed under the Boost Software License, Version 1.0.
 (See http://www.boost.org/LICENSE_1_0.txt)
 ______________________________________________________________________________


 About C++20 concepts: https://www.sandordargo.com/categories/dev/

 Hexagonal mesh over video frame

 2021.06.26  Initial version

________________________________________________________________________________________________________________________________
                                                                                                                              */
#ifndef HONEYCOMB_H_INCLUDED
#define HONEYCOMB_H_INCLUDED

#include <concepts>
#include <sstream>

#include "def.h"
#include "planar.h"
#include "timer.h"

namespace CoreAGI::Visual {

  using namespace CoreAGI::Planar;


  struct HexCell: public PixelCoord {
                                                                                                                              /*
    Base class for variety of hexagonal cells.
    PixelCoord is a coordinates of the cell center relative to frame center.
                                                                                                                              */
    unsigned short E[6];  // :edges (edge defined by index of connected cell)
    uint8_t        N;     // :number of actual pixels
    uint8_t        M;     // :number of neighbor cells (number of outbound edges in a connection graph)

    HexCell(                                          ): PixelCoord{      }, E{}, N{ 0              }, M{ 0 } {}
    HexCell( uint8_t numberOfPixels, short X, short Y ): PixelCoord{ X, Y }, E{}, N{ numberOfPixels }, M{ 0 } {}

    unsigned size(          ) const { return N;                         } // :number of actual pixels
    void     excl(          )       { assert( N-- );                    } // :decreases number of actual pixels
    void     edge( int cell )       { assert( M < 6 ); E[ M++ ] = cell; } // :adds edge to list

    virtual ~HexCell(){}

  };//struct Cell


  class Honeycomb {
                                                                                                                              /*
    Keep honeycomb structure of hexagonal cells over visual frame.
                                                                                                                              */
    static constexpr unsigned NIL{ unsigned( -1 ) };

    static const unsigned ROWS{ 10 };   // :number of pixel rows in the hexagonal cell

  public:

    static const unsigned SIZE{ 64 };   // :number of pixels in the hexagonal cell

  private:

    struct Run {                 // :strip of pattern
      int      row;              // :row index
      int      col;              // :first column
      unsigned len;              // :length of strip
    };

    static constexpr Run DEF[ ROWS ]{
      { -5, -1,  2 },
      { -4, -3,  6 },
      { -3, -4,  8 },
      { -2, -4,  8 },
      { -1, -4,  8 },
      {  0, -4,  8 },
      {  1, -4,  8 },
      {  2, -4,  8 },
      {  3, -3,  6 },
      {  4, -1,  2 }
    };


    static constexpr int ROW[ SIZE ]{ // row in the pattern:
                  -5, -5,
          -4, -4, -4, -4, -4, -4,
      -3, -3, -3, -3, -3, -3, -3, -3,
      -2, -2, -2, -2, -2, -2, -2, -2,
      -1, -1, -1, -1, -1, -1, -1, -1,
       0,  0,  0,  0,  0,  0,  0,  0,
       1,  1,  1,  1,  1,  1,  1,  1,
       2,  2,  2,  2,  2,  2,  2,  2,
           3,  3,  3,  3,  3,  3,
                   4,  4
    };

    static constexpr int COL[ SIZE ]{ // column in the pattern:
                  -1,  0,
          -3, -2, -1,  0,  1,  2,
      -4, -3, -2, -1,  0,  1,  2,  3,
      -4, -3, -2, -1,  0,  1,  2,  3,
      -4, -3, -2, -1,  0,  1,  2,  3,
      -4, -3, -2, -1,  0,  1,  2,  3,
      -4, -3, -2, -1,  0,  1,  2,  3,
      -4, -3, -2, -1,  0,  1,  2,  3,
          -3, -2, -1,  0,  1,  2,
                  -1,  0
    };

    const unsigned WIDTH;              // :frame size
    const unsigned HEIGHT;             // :frame size
    const unsigned NUMPIX;             // :frame pixels
    unsigned       NUMBER_OF_CELLS;    // :actual number of cells
    HexCell   *    CELL;               // :array of cells (allocated after first pass)
    uint32_t  *    hexIndex;           // :defines index of call for each frame pixel

  public:

    Honeycomb             ( const Honeycomb& ) = delete;
    Honeycomb& operator = ( const Honeycomb& ) = delete;

    unsigned size() const { return NUMBER_OF_CELLS; }

    const unsigned* mapping() const { return hexIndex; }

    const HexCell& operator[] ( unsigned i ) const {
      assert( i < NUMBER_OF_CELLS );
      return CELL[i];
    }

    Honeycomb( unsigned WIDTH, unsigned HEIGHT ):
      WIDTH           { WIDTH        },
      HEIGHT          { HEIGHT       },
      NUMPIX          { WIDTH*HEIGHT },
      NUMBER_OF_CELLS { 0            },
      CELL            { nullptr      }
    {
      Timer timer;
      printf( "\n [Hnycmb] Frame size: %u x %u, total %u pixels", WIDTH, HEIGHT, NUMPIX ); fflush( stdout );
                                                                                                                              /*
      Allocate map pixel index -> cell index:
                                                                                                                              */
      hexIndex = new unsigned[ NUMPIX ];
      for( unsigned i = 0; i < NUMPIX; i++ ) hexIndex[i] = NIL;
                                                                                                                              /*
      Compose pattern offset array that keeps pixel offset in frame relative to
      central pattern pixel:
                                                                                                                              */
      static int OFFSET[ SIZE ];
      printf( "\n [Hnycmb] Fill up Hexagonal.OFFSET\n" );
      unsigned ord = 0;
      for( unsigned i = 0; i < ROWS; i++ ){
        int col{ DEF[i].col };
        std::stringstream S;
        S << "  " << i;
        for( unsigned j = 0; j < DEF[i].len; j++ ){
          OFFSET[ ord ] = WIDTH*DEF[i].row + col;
          S << std::setw( 6 ) << OFFSET[ ord ];
          col++, ord++;
        }
        printf( "\n %s", S.str().c_str() );
      }
      const unsigned T       { 8 };                                                // :horizontal triangulation step
      const unsigned H       { 7 };                                                // :vertical   triangulation step
      const int      Yo      { int( HEIGHT / 2 )             };                    // :center row
      const int      Xo      { int( WIDTH  / 2 )             };                    // :center column
      const int      MAX_ROW { int( 1 + ( HEIGHT / 2 ) / H ) };                    // :rows    from center to frame edge
      const int      MAX_COL { int( 2 + ( WIDTH  / 2 ) / T ) };                    // :columns from center to frame edge
      printf( "\n\n   Yo      %2d", Yo      );
      printf(   "\n   Xo      %2d", Xo      );
      printf(   "\n   MAX_ROW %2d", MAX_ROW );
      printf(   "\n   MAX_COL %2d", MAX_COL );
                                                                                                                              /*
      Compose cell data in two passes: FIRST count actual cells, SECOND assign cells data.

      Hexagns are overlapped, so some pixels can be attributed to one of two cells;
      as well as pixel must be attributed to exactly one hexagon, used random selection
      of one of two overlapped cells.
                                                                                                                              */
      enum Pass: unsigned{ FIRST=1, SECOND=2 };

      constexpr Pass PASS [2     ]{ FIRST, SECOND };                               // :ara of passes
      unsigned overlaps{ 0 };                                                      // :number of overlapped pixels
      for( auto pass: PASS ){                                                      // :two passes
        printf( "\n\n   Pass %d", pass );
        uint32_t cell{ 0 };                                                        // :cell ordinal
        for( int row = -MAX_ROW; row <= MAX_ROW; row++ ){                          // :cell rows
          const int Yc{ Yo + row * int( H ) };                                     // :base Y coord of cell
          for( int col = -MAX_COL; col <= MAX_COL; col++ ){                        // :cell columns
            const int      evenRow{ int( even( row ) )                         };  // :even or odd row?
            const int      Xc     { Xo + col*int(T) - ( 1 - evenRow )*int(T/2) };  // :base X coord of cell
            const unsigned base{ Yc*WIDTH + Xc };                                  // :base offset for cell
            unsigned n{ 0 };                                                       // :number of actual (inner) pixels
            unsigned pixel[ SIZE ]{};                                              // :list of frame pixels for current cell
            for( unsigned i = 0; i < SIZE; i++ ){                                  // :loop over pattern pixels
              const int  pixCol = Xc + COL[ i ];                                   // :frame col
              const int  pixRow = Yc + ROW[ i ];                                   // :frame row
              const bool in     = ( pixCol >= 0            )                       // :check if pixel inside/outside located
                               && ( pixCol < int( WIDTH  ) )
                               && ( pixRow >= 0            )
                               && ( pixRow < int( HEIGHT ) );
              if( in ){
                int p = base + OFFSET[i];
                if( p >= 0 && p < int( NUMPIX ) ){ pixel[n] = p; n++; }
              }
            }//for i
            if( n > 0 ){ // Non-empty cell:
              if( SECOND == pass ){ // Fill pixel -> cell map:
                CELL[ cell ] = HexCell( n, Xc - Xo, Yo - Yc );
                for( unsigned i = 0; i < n; i++ ){ // Loop over cell` pixels:
                  unsigned p = pixel[i];
                  assert( p < NUMPIX );
                  if( hexIndex[p] == NIL ){ // Cell index is not assigned yet to this pixel:
                    hexIndex[p] = cell;
                  } else { // Cell index already assigned to pixel (overlapped cells);
                    overlaps++;
                                                                                                                              /*
                    Make random choice between this cell and previously assigned:
                                                                                                                              */
                    if( rand() & 0x1 ){ // Exclude pixel from this cell:
                      CELL[ cell ].excl();
                    } else { // Exclude pixel from previously assigned cell:
                      hexIndex[p] = cell;
                      CELL[ hexIndex[p] ].excl();
                    }
                  }
                }//for i
              }//if second pass
              cell += 1;
            }//if Np
          }//for col
        }//for row
        if( pass == FIRST ){ // Allocate cell related arrays:
          NUMBER_OF_CELLS = cell;
          printf( "\n   Total %u hexagonal cells", NUMBER_OF_CELLS );
          assert( NUMBER_OF_CELLS > 0 );
                                                                                                                              /*
          Allocate array of cells:
                                                                                                                              */
          CELL = new HexCell[ NUMBER_OF_CELLS ];
        }
      }//for pass
                                                                                                                              /*
      Recalculate number of cell`s pixels:
                                                                                                                              */
      for( unsigned i = 0; i < NUMBER_OF_CELLS; i++ ) CELL[i].N = 0;
      for( unsigned px = 0; px < NUMPIX; px++ ) CELL[ hexIndex[px] ].N++;
      printf( "\n   Cell fill statistics:\n" );
      printf( "\n     Total overlaps: %u, %.1f%%", overlaps, 100.0*overlaps/NUMPIX );
      printf( "\n     Total cells:    %u",         NUMBER_OF_CELLS                 );

      unsigned cellStat[ SIZE + 1 ];
      for( auto& n: cellStat ) n = 0;
      unsigned totalPixels{ 0 };
      for( unsigned i = 0; i < NUMBER_OF_CELLS; i++ ){
        totalPixels += CELL[i].size();
        cellStat[ CELL[i].size() ] += 1;
      }
      printf( "\n     Total %u pixels:", totalPixels );
      for( unsigned n = SIZE + 1; n > 0; n-- ){
        unsigned f = cellStat[n];
        if( f > 0 ) printf( "\n    %6u cells have %2u pixels", f, n );
      }
      printf( "\n" );
      for( unsigned i = 0; i < NUMPIX; i++ ) assert( hexIndex[i] != NIL );
                                                                                                                              /*
      Make connection (set connection graph edges):
                                                                                                                              */
      printf( "\n [Hnycmb] Make connections.." );
      for( unsigned i = 0; i < NUMBER_OF_CELLS; i++ ){
        HexCell& Ci = CELL[i];
        for( unsigned j = i+1; j < NUMBER_OF_CELLS; j++ ){
          HexCell& Cj = CELL[j];
          int x{ Ci.x - Cj.x };
          int y{ Ci.y - Cj.y };
          int d{ x*x + y*y   };
          if( d < 128 ){
            Ci.edge( j );
            Cj.edge( i );
          }
        }
      }
                                                                                                                              /*
      Calculete max number of neighbours:
                                                                                                                              */
      unsigned Mmax{ 0 };
      for( unsigned i = 0; i < NUMBER_OF_CELLS; i++ ) if( CELL[i].M > Mmax ) Mmax = CELL[i].M;
      printf( "\n [Hnycmb] Max number of neighbours: %u",    Mmax            );
      printf( "\n [Hnycmb] constructor elapsed %.1f msec\n", timer.elapsed() );
    }//Honeycomb constructor

   ~Honeycomb(){
      delete[] CELL;
      delete[] hexIndex;
    }

  };//class Honeycomb

}//namespace CoreAGI::Visual

#endif // HONEYCOMB_H_INCLUDED
