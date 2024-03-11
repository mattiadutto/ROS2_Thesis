// Convex-Hull.cpp : Defines the entry point for the console application.
//

#include "nav2_custom_controller/convex_hull.hpp"
#include <algorithm>
#include <iostream>

Coordinate::Coordinate()
{
}

Coordinate::Coordinate(double x, double y):
	x(x),
	y(y)
{}

double Coordinate::GetX() const
{
	return x;
}

double Coordinate::GetY() const
{
	return y;
}


Polygon::Polygon(std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>& polygon) :
	m_polygon(std::move(polygon))
{
}

Polygon::Polygon(const std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>& polygon) :
	m_polygon(polygon)
{
}

Polygon::Polygon()
{}

bool Polygon::CrossProduct(const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointA, const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointB, const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointC)
{
	auto orientation = (pointC.x - pointA.x) * (pointB.y - pointA.y) -
		(pointC.y - pointA.y) * (pointB.x - pointA.x);

	return orientation > 0;
}

//void Polygon::SetConvexHull(std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>&& convexPolygon)
//{
//	m_convexHull = std::move(convexPolygon);
//}

//void Polygon::SetConvexHull(std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> convexPolygon)
//{
//	m_convexHull = convexPolygon;
//}

//std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> Polygon::GetConvexHull()
//{
//	return m_convexHull;
//}


// When sorting the GPS coordinates, we want to give x precidence. We also handle the case where the x coordinate is the same,
// which we then check the y coordinate.
bool Polygon::SortPoints(const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointA, const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointB)
{
	return (pointA.x < pointB.x) || (pointA.x == pointB.x && pointA.y < pointB.y);
}

std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> Polygon::ComputeConvexHull()
{
	// The polygon needs to have at least three points
	if (m_polygon.size() < 3)
	{
		return std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>();
	}

	std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> upperHull;
	std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> lowerHull;
	upperHull.push_back(m_polygon[0]);
	upperHull.push_back(m_polygon[1]);

	/*
	We piecewise construct the convex hull and combine them at the end of the method. Note that this could be
	optimized by combing the while loops.
	*/
	for (size_t i = 2; i < m_polygon.size(); i++)
	{
		while (upperHull.size() > 1 && !CrossProduct(upperHull[upperHull.size() - 2], upperHull[upperHull.size() - 1], m_polygon[i]))
		{
			upperHull.pop_back();
		}
		upperHull.push_back(m_polygon[i]);

		while (lowerHull.size() > 1 && !CrossProduct(lowerHull[lowerHull.size() - 2], lowerHull[lowerHull.size() - 1], m_polygon[m_polygon.size() - i - 1]))
		{
			lowerHull.pop_back();
		}
		lowerHull.push_back(m_polygon[m_polygon.size() - i - 1]);
	}
	upperHull.insert(upperHull.end(), lowerHull.begin(), lowerHull.end());

	return upperHull;
}

/*
int main()
{
	std::vector<Coordinate> polygonPoints = { {1,2}, {0.5, 1}, {0,0} };
	Polygon polygon(polygonPoints);
	auto convexHull = polygon.ComputeConvexHull();
	for (const auto &coordinate: convexHull) {
		std::cout << "(" << coordinate.GetX() << ", " << coordinate.GetY() <<")"<<std::endl;
	}
	return 0;
}


*/