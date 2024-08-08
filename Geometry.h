#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <string>
#include <sstream>

namespace geometry
{
	struct Point
	{
		double x;
		double y;

		bool operator==(const Point & other) const;
		std::string to_wkt() const;
	};
	
	struct Line : std::vector<Point>
	{
		Line() = default;
		Line(const std::vector<Point> & pts) : std::vector<Point>(pts) {}
		Line(std::vector<Point> && pts) : std::vector<Point>(std::move(pts)) {}
		Line(std::initializer_list<Point> pts) : std::vector<Point>(pts) {}
		Line(const_iterator first, const_iterator last) : std::vector<Point>(first, last) {}
		bool is_closed() const;
		Line append(const Line & other);
		std::string to_wkt() const;
		void read_wkt(const std::string & str);
	};

	/// @brief Determines on which side of the segment pi-pj is the point pk located
	/// by computing the cross product of vectors (pj - pi) and (pk - pi).
	///	@return right > 0, left < 0; if all are collinear then result is 0
	double vect_direction(const Point& pi, const Point& pj, const Point& pk);
	double distance(const Point & p1, const Point & p2);
	double distance(const Point & p, const Line & ls);
	Line convex_hull(const Line & line);
	Line fit_linestring(Line ls, double epsilon);
}

#endif
