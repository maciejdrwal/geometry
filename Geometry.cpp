#include "Geometry.h"

#include <algorithm>

namespace geometry
{
	double distance(const Point & p1, const Point & p2)
    {
		return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
	}

	double distance(const Point & p, const Line & ls)
	{
        if (ls.size() == 2)
        {
            const auto p1 = ls.front();
            const auto p2 = ls.back();
            if (p1 == p2)
            {
                return distance(p, p1);
            }
            const double num = std::abs((p2.y - p1.y) * p.x - (p2.x - p1.x) * p.y + p2.x * p1.y - p2.y * p1.x);
            const double den = std::sqrt(std::pow(p2.y - p1.y, 2) + std::pow(p2.x - p1.x, 2));
            return num / den;
        }

		double dmin = std::numeric_limits<double>::max();
        for (size_t i = 0; i < ls.size() - 1; ++i)
        {
			const Line segment{ ls.at(i), ls.at(i + 1) };
            const auto d = distance(p, segment);
			dmin = std::min(dmin, d);
        }

        return dmin;
	}

    Line fit_linestring(Line ls, const double epsilon)
    {
        const auto& first_pt = ls.front();
        const auto& last_pt = ls.back();

        if (ls.size() < 3) return ls;

        double dmax = 0.0;
        size_t index = 0;
        for (size_t i = 1; i < ls.size() - 1; i++)
        {
            const auto d = distance(ls.at(i), Line{ first_pt, last_pt });
            if (d > dmax)
            {
                index = i;
                dmax = d;
            }
        }

        if (dmax > epsilon)
        {
            auto result1 = fit_linestring(Line(ls.begin(), ls.begin() + index + 1), epsilon);
            const auto result2 = fit_linestring(Line(ls.begin() + index, ls.end()), epsilon);

            result1.pop_back();
            result1.append(result2);

            return result1;
        }

        return { first_pt, last_pt };
    }

    Line convex_hull(const Line & line)
    {
        const size_t n = line.size();
        if (n < 4) return line;
        std::vector<Point> points{ line };
        std::vector<Point> result(2 * n);
        std::sort(
            points.begin(), points.end(),
            [](const Point & p1, const Point & p2)
            { return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y); });
        size_t k = 0;
        for (size_t i = 0; i < n; ++i)
        {
            while (k >= 2 && vect_direction(result[k - 2], result[k - 1], points[i]) <= 0)
            {
                k--;
            }
            result[k++] = points[i];
        }
        const auto k0 = k + 1;
        for (size_t i = n - 1; i > 0; --i)
        {
            while (k >= k0 && vect_direction(result[k - 2], result[k - 1], points[i - 1]) <= 0)
            {
                k--;
            }
            result[k++] = points[i - 1];
        }
        result.resize(k - 1);
        return { { result.begin(), result.end() } };
    }

    bool Point::operator==(const Point& other) const
    {
	    return x == other.x && y == other.y;
    }

    std::string Point::to_wkt() const
    {
	    return "POINT(" + std::to_string(x) + ' ' + std::to_string(y) + ')';
    }

    bool Line::is_closed() const
    { return front() == back(); }

    Line Line::append(const Line& other)
    {
	    this->insert(this->end(), other.begin(), other.end());
	    return *this;
    }

    std::string Line::to_wkt() const
    {
	    std::stringstream ss;
	    ss << "LINESTRING(";
	    for (size_t i = 0; i < this->size()-1; ++i)
	    {
		    const auto & pt = this->at(i);
		    ss << std::to_string(pt.x) << ' ' << std::to_string(pt.y) << ',';
	    }
	    ss << std::to_string(this->back().x) << ' ' << std::to_string(this->back().y) << ')';
	    return ss.str();
    }

    void Line::read_wkt(const std::string& str)
    {
	    std::string wkt = str.substr(str.find('(') + 1, str.find(')') - str.find('(') - 1);
	    std::stringstream ss(wkt);
	    std::string token;
	    while (std::getline(ss, token, ','))
	    {
		    std::stringstream ss2(token);
		    std::string x, y;
		    std::getline(ss2, x, ' ');
		    std::getline(ss2, y, ' ');
		    this->push_back({ std::stod(x), std::stod(y) });
	    }
    }

    double vect_direction(const Point& pi, const Point& pj, const Point& pk)
    {
	    const double xi = pi.x, yi = pi.y;
	    const double xj = pj.x, yj = pj.y;
	    const double xk = pk.x, yk = pk.y;
	    return (xk - xi) * (yj - yi) - (xj - xi) * (yk - yi);
    }
}
