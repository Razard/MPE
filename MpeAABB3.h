
// (c) Micelanholies 2015
// Micelanholies Physics Engine
// AABB3 - Axis Aligned Bounding Box 3d

#ifndef	__MPE_AABB3__
#define	__MPE_AABB3__

#include "Mpevec3.h"


namespace Mpe
{

	template <typename type> class AABB3
	{

	public:
		AABB3(void) {};
		AABB3(const type v): l(v), h(v) {};
		AABB3(const Vec3<type>& v): l(v), h(v) {};
		AABB3(const Vec3<type>& a, const Vec3<type>& b): l(a), h(b) {};
		AABB3(const AABB3<type>& aabb) : l(aabb.l), h(aabb.h) {};
		~AABB3(void) {};
	
		Vec3<type> l;
		Vec3<type> h;


		// set this AABB to value
		MPE_FORCE_INLINE AABB3<type>&				operator= (const type val)
		{
			l = val; h = val;
			return (*this);
		}

		// set this AABB to vector
		MPE_FORCE_INLINE AABB3<type>&				operator= (const Vec3<type>& vec)
		{
			l = vec; h = vec;
			return (*this);
		}

		// set this AABB to AABB
		MPE_FORCE_INLINE AABB3<type>&				operator= (const AABB3<type>& aabb)
		{
			l = aabb.l; h = aabb.h;
			return (*this);
		}

		// equality of this AABB to value
		MPE_FORCE_INLINE _b  						operator==(const type val) const
		{
			return (l==val && h==val);
		}
	
		// equality of this AABB to vector
		MPE_FORCE_INLINE _b  						operator==(const Vec3<type>& vec) const
		{
			return (l==vec && h==vec);
		}

		// equality of this AABB to other AABB
		MPE_FORCE_INLINE _b  						operator==(const AABB3<type>& aabb) const
		{
			return (l==aabb.l && h==aabb.h);
		}

		// inequality of this AABB to value
		MPE_FORCE_INLINE _b  						operator!=(const type val) const
		{
			return (l!=val && h!=val);
		}
	
		// inequality of this AABB to vector
		MPE_FORCE_INLINE _b  						operator!=(const Vec3<type>& vec) const
		{
			return (l!=vec && h!=vec);
		}

		// inequality of this AABB to other AABB3
		MPE_FORCE_INLINE _b  						operator!=(const AABB3<type>& aabb) const
		{
			return (l!=aabb.l && h!=aabb.h);
		}

		// get this AABB as is
		MPE_FORCE_INLINE const AABB3<type>&			operator+ (void) const
		{
			return (*this);
		}

		// get this AABB with negated projections
		MPE_FORCE_INLINE const AABB3<type>			operator- (void) const
		{
			return AABB3<type>(-l, -h);
		}

	
		// addition projections of this AABB with value
		MPE_FORCE_INLINE const AABB3<type>			operator+ (const type val) const
		{
			AABB3<type> n(*this);
			n.l -= val;
			n.h += val;
			return n;
		}

		// addition of this AABB with vector
		MPE_FORCE_INLINE const AABB3<type>			operator+ (const Vec3<type>& vec) const
		{
			AABB3<type> n(*this);
			n.l -= vec;
			n.h += vec;
			return n;
		}
	
		// addition of this AABB with AABB
		MPE_FORCE_INLINE const AABB3<type>			operator+ (const AABB3<type>& aabb) const
		{
			AABB3<type> n(*this);
			n += aabb;
			return n;
		}

		// increase projections of this AABB by value
		MPE_FORCE_INLINE void 						operator+=(const type val)
		{
			(*this) += Vec3<type>(val);
		}

		// increase this AABB by vector
		MPE_FORCE_INLINE void  						operator+=(const Vec3<type>& vec)
		{
			(*this).l -= vec;
			(*this).h += vec;
		}
	
		// increase this AABB3 by AABB
		MPE_FORCE_INLINE void  						operator+=(const AABB3<type>& aabb)
		{
			(*this).expand(aabb);
		}

		// subtraction projections of this AABB with value
		MPE_FORCE_INLINE const Vec3<type>			operator- (const type val) const
		{
			AABB3<type> n(*this);
			n.l += val;
			n.h -= val;
			return n;
		}

		// subtraction of this AABB with vector
		MPE_FORCE_INLINE const Vec3<type>			operator- (const Vec3<type>& vec) const
		{
			AABB3<type> n(*this);
			n.l += vec;
			n.h -= vec;
			return n;
		}

		// decrease pojections of this AABB by value
		MPE_FORCE_INLINE void						operator-=(const type val)
		{
			(*this) -= Vec3<type>(val);
		}

		// decrease this AABB by vector
		MPE_FORCE_INLINE void						operator-=(const Vec3<type>& vec)
		{
			(*this).l += vec;
			(*this).h -= vec;
		}

		// magnify this AABB with value
		MPE_FORCE_INLINE const Vec3<type>			operator* (const type val) const
		{
			return AABB3<type>(l*val, h*val);
		}

		// magnify this AABB by value
		MPE_FORCE_INLINE void						operator*=(const type val)
		{
			l *= val; h *= val;
		}

		// reduction of AABB with value
		template <typename type>
		MPE_FORCE_INLINE const AABB3<type>			operator/ (const type val) const
		{
			return AABB3<type>(l/val, h/val);
		}

		// reduction of AABB with float value
		template <>
		MPE_FORCE_INLINE const AABB3<_f>			operator/ <_f>(const _f val) const
		{
			return (*this)*((_f)1/val);
		}

		// reduction of AABB with double value
		template <>
		MPE_FORCE_INLINE const AABB3<_d>			operator/ <_d>(const _d val) const
		{
			return (*this)*((_d)1/val);
		}

		// reduction of this AABB by value
		template <typename type>
		MPE_FORCE_INLINE void						operator/=(const type val)
		{
			l /= val; h /= val;
		}

		// reduction of this AABB by float value
		template <>
		MPE_FORCE_INLINE void						operator/=<_f>(const _f val)
		{
			(*this) *= (_f)1/val;
		}

		// reduction of this AABB by double value
		template <>
		MPE_FORCE_INLINE void						operator/=<_d>(const _d val)
		{
			(*this) *= (_d)1/val;
		}

		// intersection of this AABB with dot
		MPE_FORCE_INLINE _b							operator^ (const Vec3<type>& dot) const
		{
			return (Math::bounde<type>(dot.x, l.x, h.x) &&
					Math::bounde<type>(dot.y, l.y, h.y) &&
					Math::bounde<type>(dot.z, l.z, h.z))
		}

		// intersection of this AABB with other AABB
		MPE_FORCE_INLINE _b							operator^ (const AABB3<type>& aabb) const
		{
			const AABB3<type>& t = *this;
			const AABB3<type>& a =  aabb;
			if (t.h.x < a.l.x) return false;
			if (a.h.x < t.l.x) return false;
			if (t.h.y < a.l.y) return false;
			if (a.h.y < t.l.y) return false;
			if (t.h.z < a.l.z) return false;
			if (a.h.z < t.l.z) return false;
			return true;
		}

		// move this AABB by vector
		MPE_FORCE_INLINE void						move(const Vec3<type>& vec)
		{
			AABB3<type>& t = *this;
			t.l += vec;
			t.h += vec;
		}

		// extend this AABB by new vector
		MPE_FORCE_INLINE void						extend(const Vec3<type>& vec)
		{
			AABB3<type>& t = *this;
			AABB3<type>  n(t);
			n.move(vec);
			t.expand(n);
		}


		// expand this AABB by new vector
		MPE_FORCE_INLINE void						expand(const Vec3<type>& vec)
		{
			AABB3<type>& t = *this;
			t.l.x = Math::min<type>(t.l.x, vec.x);
			t.l.y = Math::min<type>(t.l.y, vec.y);
			t.l.z = Math::min<type>(t.l.z, vec.z);
			t.h.x = Math::max<type>(t.h.x, vec.x);
			t.h.y = Math::max<type>(t.h.y, vec.y);
			t.h.z = Math::max<type>(t.h.z, vec.z);
		}

		// expand this AABB by other AABB
		MPE_FORCE_INLINE void  						expand(const AABB3<type>& aabb)
		{
			AABB3<type>& t = *this;
			t.l.x = Math::min<type>(t.l.x, aabb.l.x);
			t.l.y = Math::min<type>(t.l.y, aabb.l.y);
			t.l.z = Math::min<type>(t.l.z, aabb.l.z);
			t.h.x = Math::max<type>(t.h.x, aabb.h.x);
			t.h.y = Math::max<type>(t.h.y, aabb.h.y);
			t.h.z = Math::max<type>(t.h.z, aabb.h.z);
		}

		// intersection of AABB with dot
		MPE_FORCE_INLINE _b							intersect(const Vec3<type>& dot) const
		{
			return (*this)^dot;
		}

		// intersection of AABB with other AABB3
		MPE_FORCE_INLINE _b							intersect(const AABB3<type>& aabb) const
		{
			return (*this)^aabb;
		}

		// intersection of AABB with axis-aligned plane X (YZ-plane)
		MPE_FORCE_INLINE _b							intersectPlaneX(const type c) const
		{
			const AABB3<type>& t = *this;
			if (t.h.x < c) return false;
			if (t.l.x > c) return false;
			return true;
		}

		// intersection of AABB with axis-aligned plane Y (XZ-plane)
		MPE_FORCE_INLINE _b							intersectPlaneY(const type c) const
		{
			const AABB3<type>& t = *this;
			if (t.h.y < c) return false;
			if (t.l.y > c) return false;
			return true;
		}

		// intersection of AABB with axis-aligned plane Z (XY-plane)
		MPE_FORCE_INLINE _b							intersectPlaneZ(const type c) const
		{
			const AABB3<type>& t = *this;
			if (t.h.z < c) return false;
			if (t.l.z > c) return false;
			return true;
		}


		// set AABB by vertex
		MPE_FORCE_INLINE const AABB3<type>			vertex(const Vec3<type>& dot)
		{
			*this = dot;
			return *this;
		}

		// set AABB by edge
		MPE_FORCE_INLINE const AABB3<type>			edge(const Vec3<type>& dotA, const Vec3<type>& dotB)
		{
			AABB3<type>& t = *this;
			t.l.x = Math::min<type>(dotA.x, dotB.x);
			t.l.y = Math::min<type>(dotA.y, dotB.y);
			t.l.z = Math::min<type>(dotA.z, dotB.z);
			t.h.x = Math::max<type>(dotA.x, dotB.x);
			t.h.y = Math::max<type>(dotA.y, dotB.y);
			t.h.z = Math::max<type>(dotA.z, dotB.z);
			return *this;
		}

		// set AABB by triangle
		MPE_FORCE_INLINE const AABB3<type>			triangle(const Vec3<type>& dotA, const Vec3<type>& dotB, const Vec3<type>& dotC)
		{
			AABB3<type>& t = *this;
			t.l.x = Math::min<type>(dotA.x, Math::min<type>(dotB.x, dotC.x));
			t.l.y = Math::min<type>(dotA.y, Math::min<type>(dotB.y, dotC.y));
			t.l.z = Math::min<type>(dotA.z, Math::min<type>(dotB.z, dotC.z));
			t.h.x = Math::max<type>(dotA.x, Math::max<type>(dotB.x, dotC.x));
			t.h.y = Math::max<type>(dotA.y, Math::max<type>(dotB.y, dotC.y));
			t.h.z = Math::max<type>(dotA.z, Math::max<type>(dotB.z, dotC.z));
			return *this;
		}

		// set AABB by tetrahedron
		MPE_FORCE_INLINE const AABB3<type>			tetrahedron(const Vec3<type>& dotA, const Vec3<type>& dotB, const Vec3<type>& dotC, const Vec3<type>& dotD)
		{
			AABB3<type>& t = *this;
			t.l.x = Math::min<type>(Math::min<type>(dotA.x, dotB.x), Math::min<type>(dotC.x, dotD.x));
			t.l.y = Math::min<type>(Math::min<type>(dotA.y, dotB.y), Math::min<type>(dotC.y, dotD.y));
			t.l.z = Math::min<type>(Math::min<type>(dotA.z, dotB.z), Math::min<type>(dotC.z, dotD.z));
			t.h.x = Math::max<type>(Math::max<type>(dotA.x, dotB.x), Math::max<type>(dotC.x, dotD.x));
			t.h.y = Math::max<type>(Math::max<type>(dotA.y, dotB.y), Math::max<type>(dotC.y, dotD.y));
			t.h.z = Math::max<type>(Math::max<type>(dotA.z, dotB.z), Math::max<type>(dotC.z, dotD.z));
			return *this;
		}

		// set AABB by array of vertices
		MPE_FORCE_INLINE const AABB3<type>			arrayDot(const Vec3<type>* pDot, _ui numDots)
		{
			AABB3<type>& t = *this;
			t = (const type)0;
			for (_ui i = 0; i<numDots; i++)
				t += pDot[i];
			return *this;
		}

		// get middle point of AABB
		MPE_FORCE_INLINE const Vec3<type>			middle(void) const
		{
			return (l+h) / (const type)2;
		}

		// check this AABB completely cover other AABB
		MPE_FORCE_INLINE _b							cover(const AABB3<type>& aabb) const
		{
			const AABB3<type>& t = *this;
			if (t.l.x>aabb.l.x) return false;
			if (t.l.y>aabb.l.y) return false;
			if (t.l.z>aabb.l.z) return false;
			if (t.h.x<aabb.h.x) return false;
			if (t.h.y<aabb.h.y) return false;
			if (t.h.z<aabb.h.z) return false;
			return true;
		}

		// compare this AABB to excluding intersection with plane on axis X (YZ plane)
		MPE_FORCE_INLINE _b							cmpaX(const type c) const
		{
			return (*this).h.x>c;
		}

		// compare this AABB to including intersection with plane on axis X (YZ plane)
		MPE_FORCE_INLINE _b							cmpaeX(const type c) const
		{
			return (*this).h.x>=c;
		}

		// compare this AABB to excluding intersection with plane on axis X (YZ plane)
		MPE_FORCE_INLINE _b							cmpbX(const type c) const
		{
			return (*this).l.x<c;
		}

		// compare this AABB to including intersection with plane on axis X (YZ plane)
		MPE_FORCE_INLINE _b							cmpbeX(const type c) const
		{
			return (*this).l.x<=c;
		}

		// compare this AABB to excluding intersection with plane on axis Y (XZ plane)
		MPE_FORCE_INLINE _b							cmpaY(const type c) const
		{
			return (*this).h.y>c;
		}

		// compare this AABB to including intersection with plane on axis Y (XZ plane)
		MPE_FORCE_INLINE _b							cmpaeY(const type c) const
		{
			return (*this).h.y>=c;
		}

		// compare this AABB to excluding intersection with plane on axis Y (XZ plane)
		MPE_FORCE_INLINE _b							cmpbY(const type c) const
		{
			return (*this).l.y<c;
		}

		// compare this AABB to including intersection with plane on axis Y (XZ plane)
		MPE_FORCE_INLINE _b							cmpbeY(const type c) const
		{
			return (*this).l.y<=c;
		}

		// compare this AABB to excluding intersection with plane on axis Z (XY plane)
		MPE_FORCE_INLINE _b							cmpaZ(const type c) const
		{
			return (*this).h.z>c;
		}

		// compare this AABB to including intersection with plane on axis Z (XY plane)
		MPE_FORCE_INLINE _b							cmpaeZ(const type c) const
		{
			return (*this).h.z>=c;
		}

		// compare this AABB to excluding intersection with plane on axis Z (XY plane)
		MPE_FORCE_INLINE _b							cmpbZ(const type c) const
		{
			return (*this).l.z<c;
		}

		// compare this AABB to including intersection with plane on axis Z (XY plane)
		MPE_FORCE_INLINE _b							cmpbeZ(const type c) const
		{
			return (*this).l.z<=c;
		}

	};	// class AABB3

};	// namespace Mpe

#endif	// __MPE_AABB3__





















