#pragma once
#include <vector>
#include "costmap_converter/costmap_to_polygons.h"

class Coordinate
{
public:
	Coordinate();
	Coordinate(double x, double y);

	double GetX() const;
	double GetY() const;

	void SetX(const double x);
	void SetY(const double y);
	double x;
	double y;

private:
	
};

/*
Class: ConvexHull

Purpose: Holds all of the functions for computing the convex hull. The constructor takes a vector of user supplied coordinates,
		 which is stored internally in m_polygon. ComputeConvexHull takes m_polygon and creates a new vector of coordinates which
		 define the convex hull. 
Use:
		 

*/
class Polygon
{
public:
	Polygon(std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>& polygon);
	Polygon(const std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>& polygon);
	Polygon();

	bool CrossProduct(const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointA, const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointB, const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointC);

	//std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> GetConvexHull();
	std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> ComputeConvexHull();
private:
	//void SetConvexHull(std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>&& convexPolygon);
//	void SetConvexHull(std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> convexPolygon);
	bool SortPoints(const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointA, const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint& pointB);

//	std::vector<Coordinate> m_polygon;
	//std::vector<Coordinate> m_convexHull;
	std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> m_polygon;
	costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint m_convexHull;
};
