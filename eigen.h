#ifndef EIGEN_H_INCLUDED
#define EIGEN_H_INCLUDED
                                                                                                                              /*

  2020.08.30 Initial version

  2020.09.03 Type of data added to template paramrters

  2020.11.03 Fixed mistyping; Fixed error which took place in the case of equal values on the main diagonal

________________________________________________________________________________________________________________________________
                                                                                                                              */
#include <cmath>
#include <type_traits>

#include "heapsort.h"

namespace CoreAGI {

  template< unsigned N, typename Real = long double > class Eigen {

    static_assert( N > 1                                     );
    static_assert( std::is_floating_point< Real >::value     );
    static_assert( std::numeric_limits< Real >::has_infinity );

    unsigned Nrot;
    Real     A  [N][N];
    Real     V  [N][N];
    Real     D  [N];
    Real     B  [N];
    Real     Z  [N];
    unsigned ord[N];

  public:

    Eigen(): Nrot{ 0 }, A{}, V{}, D{}, B{},Z{}, ord{}{
      for( unsigned i = 0; i < N; i++ ){
        ord[i] = i;
        for( auto& Aij: A[i] ) Aij = 0;
      }
    }

    void clear(){ for( unsigned i = 0; i < N; i++ ) for( int j = 0; j < N; j++ ) A[i][j] = 0.0; }

    void let( unsigned i, unsigned j, Real Aij ){ A[i][j]  = Aij; if( i != j ) A[j][i]  = Aij; }
    void add( unsigned i, unsigned j, Real Aij ){ A[i][j] += Aij; if( i != j ) A[j][i] += Aij; }

    Real eigenValue( unsigned i ) const { return D[ ord[i] ]; }

    bool eigenVector( Real* e, unsigned i ) const {
      if( i >= N ) return false;
      int j = ord[ i ];
      for( unsigned p = 0; p < N; p++ ) e[p] = V[p][j];
      return true;
    }

    Real matrix( unsigned i, unsigned j ) const { return ( i < N and j < N ) ? A[i][j] : 0.0; }

    unsigned rotationNumber() const { return Nrot; }

    void sort(){
      heapSort< unsigned >( ord, N,
       [&]( unsigned i, unsigned j )->int{
                                                                                                                              /*
          Comparator (i,j - elements of ord[]
                                                                                                                              */
          Real Di = std::abs( D[i] );
          Real Dj = std::abs( D[j] );
          return Di == Dj ? 0 : ( Di < Dj ? 1 : -1 );
        }
      );
      // assert( eigenValue( 0 ) >= eigenValue( N-1 ) ); // DEBUG
    }

    unsigned linearSystem( /*out*/ Real* x, const Real* b, Real condition ){
                                                                                                                              /*
      Solution of linear system Ax=b using eigen vectors:
                                                                                                                              */
      if( not run() ) return 0;
      sort();
      Real c[N];
      const unsigned nc = spectral( c, b, condition );
      //for( unsigned i = 0; i < nc; i++ ) printf( "\n   %2u %12.8f", i, double( c[i] ) );     // DEBUG
      Real ek[N];
      for( unsigned i = 0; i < N; i++ ) x[i] = 0;
      for( unsigned k = 0; k < nc; k++ ){
        Real ck = c[ k ];
        eigenVector( ek, k );
        for( unsigned i = 0; i < N; i++ ) x[i] += ck * ek[i];
      }
      return nc;
    }

    int spectral( /*out*/Real* c, const Real* b, Real condition ){
                                                                                                                      /*
      Calculate spectral coefficients c[] for particular right vector b and conditional number:
                                                                                                                      */
      Real limit = eigenValue( 0 ) / condition;
      Real ek[N];
      unsigned n{ 0 };
      for( unsigned k = 0; k < N; k++ ){
        Real lambda_k = eigenValue( k );
        if( std::abs( lambda_k ) < limit ) break;                        // :NB lambda can be negative
        eigenVector( ek, k );                                            // :extract k-th vector
        double ck_ek = 0;
        for( unsigned i = 0; i < N; i++ ) ck_ek += ek[i] * b[i];
        c[n] = ck_ek / lambda_k;
        n++;
      }
      return n;
    }

    bool run(){

      using namespace std;

      constexpr Real EPS{ 1.0E-10 }; // NB can be modified

      Real c, g, h, s, Sm, t, tau, theta, tresh;
      for( unsigned p = 0; p < N; p++ ){
        for( unsigned q = 0; q < N; q++ ) V[p][q] = 0;
        V[p][p] = 1.0;
        ord[p] = p;
      }
      for( unsigned p = 0; p < N; p++ ){
        B[p] = A[p][p];
        D[p] = B[p];
        Z[p] = 0;
      }
      Nrot = 0;
      for( unsigned i = 1; i <= 50; i++ ){ //NB increase 50 to 100?
        Sm = 0.0;
        for( unsigned p = 0; p < N - 1; p++ ) for( unsigned q = p + 1; q < N; q++ ) Sm += std::abs( A[p][q] );
        if( Sm < EPS ) return true;
        tresh = ( i < 4 ) ? 0.2 * Sm / ( N * N ) : 0.0;
        for( unsigned p = 0; p < N - 1; p++ ){
          for( unsigned q = p + 1; q < N; q++ ){
            g = 100 * std::abs( A[p][q] );
            if( i > 4 && ( std::abs( D[p] ) + g ) == std::abs( D[p] ) && ( std::abs( D[q] ) + g ) == std::abs( D[q] ) ){
              A[p][q] = 0;
            } else if( std::abs( A[p][q] ) > tresh ){
              h = D[q] - D[p];
              if( std::abs( h ) + g == g ){
                                                                                                                              /*
                D[p] == D[q] => theta = pi/4:
                                                                                                                              */
                constexpr double SQRT05{ 0.5d };
                c = s = SQRT05;
                tau  = s / ( 1.0 + c );
                h = A[p][q];
              } else {
                theta = 0.5 * h / A[p][q];
                t = 1.0 / ( std::abs( theta ) + sqrt( 1.0 + theta*theta ) );
                if( theta < 0 ) t = -t;
                c    = 1.0 / sqrt( 1.0 + t*t );
                s    = t * c;
                tau  = s / ( 1.0 + c );
                h    = t * A[p][q];
              }
              Z[p] = Z[p] - h;
              Z[q] = Z[q] + h;
              D[p] = D[p] - h;
              D[q] = D[q] + h;
              A[p][q] = 0;
              for( unsigned j = 0; j < p; j++ ){
                g           = A[j][p];
                h           = A[j][q];
                A[j][p] = g - s * ( h + g*tau );
                A[j][q] = h + s * ( g - h*tau );
              }
              for( unsigned j = p + 1; j < q; j++ ){
                g           = A[p][j];
                h           = A[j][q];
                A[p][j] = g - s * ( h + g*tau );
                A[j][q] = h + s * ( g - h*tau );
              }
              for( unsigned j = q + 1; j < N; j++ ){
                g       = A[p][j];
                h       = A[q][j];
                A[p][j] = g - s * ( h + g*tau );
                A[q][j] = h + s * ( g - h*tau );
              }
              for( unsigned j = 0; j < N; j++ ){
                g           = V[j][p];
                h           = V[j][q];
                V[j][p] = g - s * ( h + g*tau );
                V[j][q] = h + s * ( g - h*tau );
              }
              Nrot++;
            }
          }
        }
        for( unsigned p = 0; p < N; p++ ){
          B[p] += Z[p];
          D[p]  = B[p];
          Z[p]  = 0;
        }
      }
      return false;
    }

  }; //class Eigen

} //namespace CoreAGI

#endif // EIGEN_H_INCLUDED
