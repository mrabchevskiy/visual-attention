#ifndef APPROXIMATION_H_INCLUDED
#define APPROXIMATION_H_INCLUDED
                                                                                                                              /*


  2020.08.30 Initial version

  2020.09.03 Type of data added to template parameters

  2020.10.25 Parameter declaration changed for *.point() and operator(); operator[] added

  2021.03.18 Changed constructor and basis function` type

  2021.03.19 Added method `actualBasisSize`

________________________________________________________________________________________________________________________________
                                                                                                                              */
#include <cmath>
#include <functional>
#include <span>

#include "eigen.h"
#include "planar.h"

namespace CoreAGI {

  template< unsigned L, unsigned M, typename Real = long double > class Approximation {
                                                                                                                              /*
    L ~ dimension of functional basis
    M ~ dimension of space;
    i-th basis functions accept Real[L] as argument and return Real value
                                                                                                                              */
    static_assert( L > 1                                 );
    static_assert( M > 0                                 );
    static_assert( std::is_floating_point< Real >::value );

  public:

    using BasisFunction = std::function< double( std::span< double > arg ) >;                                      // [+] 2021.03.18

  protected:

    BasisFunction    f[L];
    Real             b[L];
    Real             c[L];
    Eigen< L, Real > problem;
    unsigned         nc;

  public:

    Approximation( const BasisFunction* basis ): f{}, b{}, c{}, problem{}, nc{ 0 }{                            // [+] 2021.03.18
      for( unsigned i = 0; i < L; i++ ){
        b[i] = c[i] = 0.0;
        assert( f[i] = basis[i] );
      }
    }

    Approximation( std::initializer_list< BasisFunction > basis ): f{}, b{}, c{}, problem{}, nc{ 0 }{          // [+] 2021.03.18
      assert( basis.size() >= L );
      for( unsigned i = 0; auto& Fi: basis ) if( i >= L ) break; else f[ i++ ] = Fi;
      for( unsigned i = 0; i < L; i++ ) b[i] = c[i] = 0.0;
    }

    const Eigen< L, Real >& eigen() const { return problem; }

    void clear(){
      problem.clear();
      for( unsigned i = 0; i < L; i++ ) b[i] = c[i] = 0.0;
    }

    Approximation& point( const std::span< double > arg, double val ){                                              // [+] 2021.03.18
      assert( arg.size() == M );
      Real fr[L]; for( unsigned i = 0; i < L; i++ ) fr[i] = f[i]( arg ); // :calculate values of each basis function
      for( unsigned i = 0; i < L; i++ ) b[i] += fr[i]*val;               // :update right side vector `b`
      for( unsigned i = 0; i < L; i++ ) for( unsigned j = 0; j <= i; j++ ) problem.add( i, j, fr[i]*fr[j] ); // :update matrix
      return *this;
    }

    Approximation& point( const Planar::Cartesian C, double val ){
      assert( M == 2 );
      double P[2]{ Real( C.x ), Real( C.y ) };
//    return point( std::span< Real >{ &C.x, 2 }, val );                                                       // [+] 2021.03.18
      return point( std::span< Real >{ &P[0], 2 }, val );                                                       // [+] 2021.03.18
    }

    unsigned process( Real condition ){
      return nc = problem.linearSystem( /*out*/ c, b, condition );
    }

    unsigned actualBasisSize() const { return nc; }                                                            // [+] 2021.03.19

    Real coeff( unsigned i ) const { return i < nc ? c[i] : 0.0; }

    Real operator[]( unsigned i ) const { return i < nc ? c[i] : 0.0; }                                        // [+] 2020.10.25

    double operator()( std::span< double > arg ) const {                                                                 // [m] 2020.10.25
//      Real r[M];     for( unsigned i = 0; i < M; i++ ) r[i] = arg[i];       // :convert point components to Real
      Real val{0.0}; for( unsigned i = 0; i < L; i++ ) val += c[i]*f[i](arg); // :calculate appoximated value
      return double( val );
    }

    Real operator()( Planar::Cartesian C ) {
      assert( M == 2 );
//    return (*this)( &C.x );
      return (*this)( std::span< Real >( &C.x, 2 ) );
    }

  };//Approximation

}//CoreAGI

#endif
