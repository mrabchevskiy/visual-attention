                                                                                                                              /*
 Planar geometry and related utilities

 2020.10.28  Initial version

 2021.03.23  Added `constexpr` to isPowerOfTwo, zero, degrees, radians; added 'EQ' function

 2021.06.20  PixelCoord modified: `int` replaced by `short`

________________________________________________________________________________________________________________________________
                                                                                                                              */
#ifndef PLANAR_H_INCLUDED
#define PLANAR_H_INCLUDED

#include <cmath>
#include <cstdint>

#include <initializer_list>
#include <limits>
#include <concepts>
#include <span>
#include <tuple>
#include <vector>

#include "heapsort.h"

namespace CoreAGI::Planar {

  using namespace CoreAGI;

  using Real  = double;
  using Angle = int16_t;

  constexpr Angle ANGULAR_RESOLUTION{ 360 };
  constexpr Real  ANGULAR_QUANT     { 2.0*M_PI/ANGULAR_RESOLUTION };

  Real sqr( Real x ){ return x*x; }

  constexpr bool isPowerOfTwo( unsigned N ){
    if( N == 1 ) return true;
    for( unsigned n = 1; n < N + 1; n *= 2 ) if( n == N ) return true;
    return false;
  }

  constexpr bool zero( double x ){
    constexpr double EPS{ 1.0e-6 };
    return fabs( x ) < EPS;
  }

  constexpr bool EQ( const double& u, const double& v ){
    constexpr double EPS{ 1.0e-6 };
    return fabs( u - v ) < EPS;
  }

  constexpr Real degrees( Real radians ){
    constexpr Real FACTOR{ 180.0/M_PI };
    return FACTOR*radians;
  }

  constexpr Real radians( Real degrees ){
    constexpr Real FACTOR{ M_PI/180.0 };
    return FACTOR*degrees;
  }

  constexpr Real NIL{ std::numeric_limits< Real >::quiet_NaN() };

  constexpr std::uint8_t NIHIL  { 0b00000000 };
  constexpr std::uint8_t TESTED { 0b00000001 };
  constexpr std::uint8_t CONTOUR{ 0b00000010 };


  struct Point;      // :forward declaration
  struct Cartesian;  // :forward declaration
  struct Polar;      // :forward declaration
  class  Vector;     // :forward declaration
  class  Line;       // :forward declaration
  struct PixelCoord; // :forward declaration

  struct Cartesian {

    Real x;
    Real y;

    constexpr Cartesian(                     ): x{ NIL }, y{ NIL }{} // :undefined by default
    constexpr Cartesian( Real x, Real y      ): x{  x  }, y{  y  }{}
    constexpr Cartesian( const Cartesian&  C ): x{ C.x }, y{ C.y }{}
    constexpr Cartesian( const PixelCoord& C );                      // :forward declaration
    constexpr Cartesian( const Polar&      P );                      // :forward declaration
    constexpr Cartesian( const Point&      P );                      // :forward declaration

    constexpr explicit operator bool() const{ return not( std::isnan( x ) or std::isnan( y ) ); }

    constexpr Cartesian& operator = ( const Cartesian& C ){ x = C.x; y = C.y; return *this;  }

    constexpr Real operator * ( const Cartesian& C ) const { return x*C.x + y*C.y;                          }
    constexpr Cartesian  operator *  ( Real factor        ) const { return Cartesian{ factor*x, factor*y }; }
              Cartesian& operator *= ( Real factor        )       { x *= factor; y *= factor; return *this; }
    constexpr Cartesian  operator +  ( const Cartesian& C ) const { return Cartesian{ x + C.x, y + C.y };   }
              Cartesian& operator += ( const Cartesian& C )       { x += C.x; y += C.y; return *this;       }
    constexpr Cartesian  operator -  ( const Cartesian& C ) const { return Cartesian{ x - C.x, y - C.y };   }
              Cartesian& operator -= ( const Cartesian& C )       { x -= C.x; y -= C.y; return *this;       }

    constexpr Cartesian operator not() const { return Cartesian{ -y, x }; }

    constexpr Real r() const { return bool( *this ) ? sqrt ( x*x + y*y ) : NIL; }
    constexpr Real a() const { return bool( *this ) ? atan2(  y,    x  ) : NIL; }

    constexpr Cartesian orth() const {
      const double FACTOR{ 1.0/r() };
      return Cartesian{ FACTOR*x, FACTOR*y };
    }

  };//struct Cartesian

  constexpr Cartesian perpendicular( const Cartesian& C ){ return Cartesian{ -C.y, C.x }; }


  struct Polar {

    Real r;
    Real a;

    constexpr Polar(                    ): r{ NIL   }, a{ NIL   }{} // :undefined as default one
    constexpr Polar( Real r, Real a     ): r{  r    }, a{  a    }{}
    constexpr Polar( const Polar&     P ): r{ P.r   }, a{ P.a   }{}
    constexpr Polar( const Cartesian& C ): r{ C.r() }, a{ C.a() }{}

    constexpr explicit operator bool() const{ return not( std::isnan( r ) or std::isnan( a ) ); }

    constexpr Polar& operator =  ( const Polar& P ){ r = P.r; a = P.a; return *this; }

    constexpr Polar  operator *  ( Real factor ){ return bool( *this ) ? Polar{ r*factor, a } : Polar{}; }
              Polar& operator *= ( Real factor ){ if( bool( *this ) )r *= factor; return *this;          }
                                                                                                                              /*
    Angle between Polar`s:
                                                                                                                              */
    constexpr Real   operator / ( const Polar& P ) const { return bool( *this ) ? P.a - a : NIL; }

    constexpr Real x() const { return bool( *this ) ? r*cos( a ) : NIL; }
    constexpr Real y() const { return bool( *this ) ? r*sin( a ) : NIL; }

  };//struct Polar


  constexpr Cartesian::Cartesian( const Polar& P ): x{ NIL }, y{ NIL }{
    if( bool( P ) ) x = P.r*cos( P.a ), y = P.r*sin( P.a );
  }


  struct Vector {

    mutable Cartesian C;
    mutable Polar     P;

    constexpr Vector(                    ): C{          }, P{          }{} // :undefined vector as default one
    constexpr Vector( const Cartesian& C ): C{ C        }, P{ NIL, NIL }{}
    constexpr Vector( const Polar&     P ): C{ NIL, NIL }, P{ P        }{}

    constexpr Vector( const Point& P, const Point& Q );

    constexpr explicit operator Cartesian() const { if( not bool( C ) and bool( P ) ) C = Cartesian( P ); return C; }
    constexpr Cartesian&        cartesian() const { if( not bool( C ) and bool( P ) ) C = Cartesian( P ); return C; }

    constexpr explicit operator Polar() const { if( not bool( P ) and bool( C ) ) P = Polar( C ); return P; }
    constexpr Polar&            polar() const { if( not bool( P ) and bool( C ) ) P = Polar( C ); return P; }

    constexpr Real x() const { if( not bool( C ) and bool( P ) ) C = Cartesian( P ); return C.x; }
    constexpr Real y() const { if( not bool( C ) and bool( P ) ) C = Cartesian( P ); return C.y; }
    constexpr Real r() const { if( not bool( P ) and bool( C ) ) P = Polar    ( C ); return P.r; }
    constexpr Real a() const { if( not bool( P ) and bool( C ) ) P = Polar    ( C ); return P.a; }
                                                                                                                              /*
    Scalar priduct:
                                                                                                                              */
    constexpr Real operator* ( const Vector& V ) const { return cartesian()*V.cartesian(); }

    constexpr Vector operator* ( Real factor ) const {
      return bool( C ) ? Vector{ C*factor } : ( bool( P ) ? Vector{ P*factor } : Vector{} );
    }

    Vector& operator *= ( Real factor ){
      if( bool( P ) ) polar()     *= factor;
      if( bool( C ) ) cartesian() *= factor;
      return *this;
    }

    constexpr Vector operator + ( const Vector& V ) const { return Vector( cartesian() + V.cartesian() ); }
    constexpr Vector operator - ( const Vector& V ) const { return Vector( cartesian() - V.cartesian() ); }

    constexpr Real operator / ( const Vector& V ) const { return polar()/V.polar(); }
                                                                                                                              /*
    Make unit vector (orth):
                                                                                                                              */
    constexpr Vector orth() const {
      const Real factor{ 1.0 / r() };
      if( std::isnan( factor ) ) return Vector{};
      return Vector{ Cartesian( factor*x(), factor*y() ) };
    }

  };//Vector


  struct Point {

    Real x;
    Real y;

    constexpr Point(                    ): x{ NIL }, y{ NIL }{}
    constexpr Point( Real x, Real y     ): x{ x   }, y{ y   }{}
    constexpr Point( const Cartesian& C ): x{ C.x }, y{ C.y }{}

    constexpr Point( const Line& a, const Line& b ); // :forward declaration

    constexpr explicit operator bool() const { return not( std::isnan( x ) or std::isnan( y ) ); }

    constexpr Point  operator +  ( const Cartesian& C ) const { return Point{ x + C.x, y + C.y }; }
    constexpr Point  operator -  ( const Cartesian& C ) const { return Point{ x - C.x, y - C.y }; }

    Point& operator += ( const Cartesian& C ){ x += C.x; y += C.y; return *this; }
    Point& operator -= ( const Cartesian& C ){ x -= C.x, y -= C.y; return *this; }

    constexpr Point operator  +  ( const Vector& V ) const { const Cartesian& C{ V.cartesian() }; return Point{ x + C.x, y + C.y }; }
    constexpr Point operator  -  ( const Vector& V ) const { const Cartesian& C{ V.cartesian() }; return Point{ x - C.x, y - C.y }; }

    Point& operator += ( const Vector& V ){ const Cartesian& C{ V.cartesian() }; x += C.x; y += C.y; return *this; }
    Point& operator -= ( const Vector& V ){ const Cartesian& C{ V.cartesian() }; x -= C.x; y -= C.y; return *this; }

    Point operator*   ( double factor ) const { return Point{ factor*x, factor*y }; }

    Point& operator*= ( double factor ){ x *= factor; y *= factor; return *this; }

    constexpr Real distance( const Point& P ) const {
      const Real dx{ x - P.x };
      const Real dy{ y - P.y };
      return sqrt( dx*dx + dy*dy );
    }

    constexpr Real distance( const Line& L ) const; // :forward declaration

  };//struct Point


  constexpr Vector::Vector( const Point& P, const Point& Q ): C{ Q.x - P.x, Q.y - P.y }, P{}{}

  constexpr Cartesian::Cartesian( const Point& P ): x{ P.x }, y{ P.y }{}


  struct Line {

    Point     P; // :point
    Cartesian T; // :orth

    constexpr Line(                                    ): P{   }, T{          }{}
    constexpr Line( const Point& P, const Cartesian& C ): P{ P }, T{ C        }{}
    constexpr Line( const Point& P, const Vector&    V ): P{ P }, T{ V.orth() }{}

    constexpr explicit operator bool() const { return bool( P ) and bool( T ); }
                                                                                                                              /*
    Point on the line defined by parameter `t`:
                                                                                                                              */
    constexpr Point operator()( Real t ) const { return P + T*t; }

    constexpr Point operator* ( const Line& L ) const { return Point{ *this, L }; }

  };//struct Line

                                                                                                                              /*
  Determinant of 2 x 2 matrix with columns defined by Point struct:
                                                                                                                              */
  constexpr Real det( const Point& C1,    const Point& C2    ){ return C1.x*C2.y - C2.x*C1.y; }
  constexpr Real det( Real Mxx, Real Mxy, Real Myx, Real Myy ){ return Mxx*Myy   - Mxy*Myx;   }


  constexpr Point::Point( const Line& a, const Line& b ):
                                                                                                                              /*
    Intersection point of two lines:
                                                                                                                              */
    x{ std::numeric_limits< Real >::quiet_NaN() },
    y{ std::numeric_limits< Real >::quiet_NaN() }
  {
    constexpr Real EPS{ 1.0e-3 };
                                                                                                                              /*
    Check if lines are parallel:
                                                                                                                              */
    if( ( a.T - b.T ).r() < EPS ) return; // a.orth ==  b.orth
    if( ( a.T + b.T ).r() < EPS ) return; // a.orth == -b.orth
                                                                                                                              /*
    Intersection using equation a( t1 ) = b( t2 )
                                                                                                                              */
    *this = a( det( Point{ b.P.x - a.P.x, b.P.y - a.P.y }, b.T )/det( a.T, b.T ) ); // this <- a( t ) where t = det(.)/det(.)
  }//Point::Point


  constexpr Real Point::distance( const Line& L ) const {
    const Real  t{ Vector{ L.P, *this }*L.T };
    const Point R{ L( t ) };
    return distance( R );
  }//Point::distance


  struct Matrix {
                                                                                                                              /*
            | Mxx | Mxy |
    Matrix  |-----|-----|
            | Myx | Myy |
                                                                                                                              */
    double xx;
    double xy;
    double yx;
    double yy;

    constexpr Matrix(                                            ): xx{ 1.0 }, xy{ 0.0 }, yx{ 0.0 }, yy{ 1.0 }{}
    constexpr Matrix( double xx, double xy, double yx, double yy ): xx{  xx }, xy{  xy }, yx{ yx  }, yy{ yy }{}

    constexpr Point     operator * ( const Point     P ) const { return Point    { xx*P.x + xy*P.y, yx*P.x + yy*P.y }; }
    constexpr Cartesian operator * ( const Cartesian C ) const { return Cartesian{ xx*C.x + xy*C.y, yx*C.x + yy*C.y }; }

  };//struct Matrix

  constexpr Matrix Rot( Real angle, Real scale = 1.0 ){
                                                                                                                              /*
    Composition of Rotation & Scale matrix:
                                                                                                                              */
    const double C{ scale*cos( angle ) };
    const double S{ scale*sin( angle ) };
    return Matrix{ C, -S,  S, C };
  }


  struct Transmute {
                                                                                                                              /*
    Complex transformation that consists of such steps:
      - scaling  around preserved center
      - rotation around this center
      - translation
                                                                                                                              */
    Point     center;
    double    scale;
    Cartesian shift;
    double    angle;

    constexpr Transmute(): center{ 0.0, 0.0 }, scale{ 1.0 }, shift{ 0.0, 0.0 }, angle{ 0.0 }{}

    constexpr Transmute( Point center, double scale, double angle = 0.0, Cartesian shift=Cartesian{ 0.0, 0.0 } ):
      center{ center }, scale{ scale }, shift{ shift }, angle{ angle }{}

    constexpr Point operator ()(const Point& P ) const {
      const Matrix M{ Rot( angle, scale ) };
      return M*Cartesian{ P.x - center.x, P.y - center.y } + shift;
    }

    void apply( /*inout*/ Point* P, unsigned n ) const {
      const Matrix M{ Rot( angle, scale ) };
      for( unsigned i = 0; i < n; i++ ) P[i] = M*Cartesian{ P[i].x - center.x, P[i].y - center.y } + shift;
    }

    void apply( std::span< Point >& P ) const {
      const Matrix M{ Rot( angle, scale ) };
      for( Point& Pi: P ) Pi = M*Cartesian{ Pi.x - center.x, Pi.y - center.y } + shift;
    }

    void apply( std::vector< Point >& P ) const {
      const Matrix M{ Rot( angle, scale ) };
      for( Point& Pi: P ) Pi = M*Cartesian{ Pi.x - center.x, Pi.y - center.y } + shift;
    }

  };//struct Transmute


  class CircumCircle: public Point {

    double r;

    bool contains( const Point& p ) const { return ( sqr( p.x - x ) + sqr( p.y - y ) ) <= sqr( r ); };

    void one( const Point& p ){ x = p.x; y = p.y; r = 0; }

    void two( const Point& p1, const Point& p2 ){
      x = 0.5*( p1.x + p2.x );
      y = 0.5*( p1.y + p2.y );
      r = 0.5*sqrt( sqr( p2.x - p1.x ) + sqr( p2.y - p1.y ) );
    }

    void three( const Point& A, const Point& B, const Point& C ){

      const double EPSILON = 1e-06;

      const Point Eab { B.x - A.x, B.y - A.y };
      const Point Eac { C.x - A.x, C.y - A.y };
                                                                                                                              /*
      Linear system Mu = v:
                                                                                                                              */
      const double M[2][2] { { Eab.x, Eab.y }, { Eac.x, Eac.y }                                            };
      const double v[2]    { 0.5d * ( sqr( Eab.x )+ sqr( Eab.y ) ), 0.5d * ( sqr( Eac.x ) + sqr( Eac.y ) ) };
      const double det     { M[0][0] * M[1][1] - M[0][1] * M[1][0]                                         };
      if( fabs( det ) > EPSILON ){
        const double u[2]  { ( M[1][1]*v[0] - M[0][1]*v[1] ) / det, ( M[0][0]*v[1] - M[1][0]*v[0] ) / det };
        x = A.x + u[0];
        y = A.y + u[1];
        r = sqrt( sqr( u[0] ) + sqr( u[1] ) );
      } else {
        x = y = 0;
        r = std::numeric_limits< double >::max();
      }
    }

    void fixTwo( unsigned n, Point* P, const Point& fixed1, const Point& fixed2 ){
      two( fixed1, fixed2 );
      for( unsigned i = 0; i < n; i++ ) if( !contains( P[i] ) ) three( P[i], fixed1, fixed2 );
    }

    void fixOne( unsigned n, Point* P, const Point& fixed ){
      one( fixed );
      for( unsigned i = 0; i < n; i++ ) if( !contains( P[i] ) ) fixTwo( i, P, P[i], fixed );
    }

  public:

    CircumCircle(): Point{ 0.0, 0.0 }, r{ 0.0 }{}

    CircumCircle( std::vector< Point > points ): Point{ 0.0, 0.0 }, r{ 0.0 }{ // :point array S modified!
      const auto n{ points.size() };
      if( n > 1 ){ // Permutation (0,1,...,n-1):
        std::vector< Point > P( points );
        for( unsigned i  = n - 1; i > 0; i-- ){
          unsigned j = rand() % ( i + 1 );
          if( j != i ){
            const Point t{ P[i] };
            P[i] = P[j];
            P[j] = t;
          }
        }//for i
        one( P[0] );
        for( unsigned i = 1; i < n; i++ ) if( !contains( P[i] ) ) fixOne( i, P.data(), P[i] );
      } else { // n == 1:
        one( points[0] );
      }
    }// CircumCircle constructor

    Point  center() const { return *this; }
    double radius() const { return r;     }

  };//class CircumCircle


  class SmoothCurve {
                                                                                                                              /*
    Smooth curve of equal length fragments; curvature of each fragment defined by
    angle between tangents at begin and and of the fragment
                                                                                                                              */
    std::vector< Angle > data;

  public:

    SmoothCurve(): data{}{};

    SmoothCurve( std::initializer_list< Angle > angle_seq ): data{ angle_seq }{ assert( isPowerOfTwo( data.size() ) ); }
    SmoothCurve( const SmoothCurve& S                     ): data{ S.data    }{ assert( isPowerOfTwo( data.size() ) ); }

    unsigned size() const { return data.size(); }

    Angle operator[]( unsigned i ) const { return data.at( i ); }

    SmoothCurve abridged() const {
      const size_t N{ size() };
      SmoothCurve result{};
      for( unsigned i=0; i < N; i+=2 ) result.data.push_back( data.at( i ) + data.at( i+1 ) );
      return result;
    }

    std::vector< Point > sequence( const Point& location, double radius, double orientation, unsigned length ) const {
                                                                                                                              /*
      Composes sequence of points on the smooth curve.
      Each arc of the curve splitted on the same number of sub-arcs:

      orientation - angle between OX axis and tangent at begin of curve
      location    - location of circumcircle
      diameter    - diameter of circumcircle
      length      - minimal required number of points (actual can be bigger)

      Each arc of the curve splitted on the same number of sub-arcs.
                                                                                                                              */
      unsigned N{ size() };
      unsigned m{ 1      };              // :number of points per segment
      while( N*m + 1 < length ) m++;     // :m is multiple of size
      Point     Po     { 0.0, 0.0 };     // :start point of the current arc
      Point     Pt     {          };     // :end   point of the curent arc
      std::vector< Point > Q( N*m + 1 ); // :resulting sequnce of points
      assert( Q.size() == ( N*m + 1 ) );
      Q[0] = Po;
      double globalAngle{ 0.0 };         // :radians
      unsigned i{ 1 };
      for( const auto& arc: data ){ // Loop over curve arcs, arc ~ arc angle in ANGULAR_QUANTS
        if( arc == 0 ){
                                                                                                                              /*
          Straight line as special case of arc; move along tangent at arc start point:
                                                                                                                              */
          const Cartesian tangent{ cos( globalAngle ), sin( globalAngle ) };
          for( unsigned j = 1; j <= m; j++ ){ // loop aver sub-arcs:
            const double fraction{ double( j )/double( m ) };
            Pt = Po + tangent*fraction;
            assert( i < Q.size() ); // DEBUG
            Q[i++] = Pt;
          }
        } else { // real arc:
                                                                                                                              /*
          Real arc; move along arc with center point `C`:
                                                                                                                              */
          const  Cartesian  normal  { -sin( globalAngle ), cos( globalAngle ) }; // :normal at start of arc
          const  double     arcAngle{ ANGULAR_QUANT*arc                       }; // :arc` central angle
          const  double     radius  { 1.0/arcAngle                            }; // :arc radius
          const  Point      C       { Po + normal*radius                      }; // :center of arc
          for( unsigned j = 1; j <= m; j++ ){  // loop over points on arcs:
            const double    fraction        { double( j )/double( m )       };
            const double    fractionalAngle { arcAngle*fraction             };
            const double    angle           { globalAngle + fractionalAngle };
            const Cartesian fractionalNormal{ -sin( angle ), cos( angle )   }; // :local normal at
            Pt = C - fractionalNormal*radius;
            assert( i < Q.size() ); // DEBUG
            Q[i++] = Pt;
          }
          globalAngle += arcAngle; // :update current angle (radians)
        }
        Po = Pt;                 // :move to begin of the next arc
      }//for arc
      assert( i == N*m + 1 );
                                                                                                                              /*
      Calculate circumcircle:
                                                                                                                              */
      CircumCircle O( Q );
                                                                                                                              /*
      Move to origin and scale:
                                                                                                                              */
      const Point  C     { O.center()     };
      const double factor{ 1.0/O.radius() };
      for( auto& Qi: Q ) Qi = { factor*( Qi.x - C.x ), factor*( Qi.y - C.y ) };
                                                                                                                              /*
      Tramsmute:
                                                                                                                              */
      const Transmute T{ Point{ 0.0, 0.0 }, radius, orientation, Cartesian{ location.x, location.y } };
      T.apply( Q );

      return Q;

    }//sequence

  };//SmoothCurve


//  struct Eigen {
//
//    double    lambda[2]; // :eigen values, lambda[0] >= lambda[1]
//    Cartesian e     [2]; // :eigen vectors
//    double    angle [2];
//                                                                                                                              /*
//    Symmetric matrix assumed: Axy = Ayx, so only 3 values actually used ( Ayx ignored ):
//                                                                                                                              */
//    Eigen( const Matrix& A ): lambda{}, e{}, angle{}{
//      const double det { A.xx*A.yy - A.xy*A.xy };
//      const double tr  { A.xx + A.yy };
//                                                                                                                              /*
//      Quadratic equation for e:  e^2 - tr*e + det = 0
//                                                                                                                              */
//      const     double q = sqrt( tr*tr - 4.0*det );
//      constexpr double FACTOR{ 0.5 };
//      lambda[0] = FACTOR*( tr + q ); // :bogger  eigen value
//      lambda[1] = FACTOR*( tr - q ); // :smaller eigen value
//      assert( lambda[0] >= lambda[1] );                                                                                 // DEBUG
//      assert( zero( lambda[0]*lambda[0] - tr*lambda[0] + det ) );                                                       // DEBUG
//      assert( zero( lambda[1]*lambda[1] - tr*lambda[1] + det ) );                                                       // DEBUG
//      if( zero( A.xx - lambda[0] ) and zero( A.yy ) ) e[0] = Planar::Cartesian{  A.yy - lambda[0], -A.yy }.orth();
//      else                                            e[0] = Planar::Cartesian{ -A.yy,  A.xx - lambda[0] }.orth();
//      e[1] = perpendicular( e[0] );
//      assert( zero( e[0]*e[1] ) );                                                                                      // DEBUG
//      for( int i = 0; i < 2; i++ ) angle[i] = atan2( e[i].y, e[i].x );
//    }
//  };//struct Eigen


  struct PixelCoord {                                                                                          // [m] 2021.06.20
                                                                                                                              /*
    Pixel coordinates on the screen, pixel image and so on;
    NOTE: use conventional coordinate system, OY axis directed bottom up
                                                                                                                              */
    short x;
    short y;

    constexpr PixelCoord(                     ): x{              }, y{               }{}
    constexpr PixelCoord( short x, short y    ): x{          x   }, y{           y   }{}
    constexpr PixelCoord( const PixelCoord& C ): x{        C.x   }, y{         C.y   }{}
    constexpr PixelCoord( const Cartesian&  C ): x{ short( C.x ) }, y{ short( -C.y ) }{}                       // [+] 2020.10.28
                                                                                                                              /*
    PixelCoord O defines coordinate orogin on the rectamgle pixel field:
                                                                                                                              */
    constexpr PixelCoord( const PixelCoord& O, const Point& P ): x{ short( O.x + P.x ) }, y{ short( O.y - P.y ) }{}                                     // [+] 2020.11.04

    constexpr PixelCoord  operator +  ( const PixelCoord& C ) const { return PixelCoord(  x + C.x,  y + C.y ); }
    constexpr PixelCoord  operator -  ( const PixelCoord& C ) const { return PixelCoord(  x - C.x,  y - C.y ); }
    constexpr PixelCoord  operator *  ( int   factor        ) const { return PixelCoord( factor*x, factor*y ); }
    constexpr bool        operator == ( const PixelCoord& C ) const { return x == C.y and y == C.y;            }
    constexpr bool        operator != ( const PixelCoord& C ) const { return x != C.y or  y != C.y;            }
    constexpr PixelCoord& operator =  ( const PixelCoord& C )       { x = C.x; y = C.y; return *this;          }

  };//struct PixelCoord


  constexpr Cartesian::Cartesian( const PixelCoord& C ): x{ double( C.x ) }, y{ double( -C.y ) }{}             // [+] 2020.10.28





}//namespace CoreAGI::Planar

#endif // PLANAR_H_INCLUDED
