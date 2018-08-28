//
// Created by yoni on 20/08/18.
//

#ifndef PATHFINDER_OPTIMALPATH_H
#define PATHFINDER_OPTIMALPATH_H

#include "OptimalPath.h"
#include "Path.h"
#include "PathPlanner.h"
#include "CGAL_defines.h"
#include <CGAL/Origin.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/enum.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_polygon_validation.h>
#include <vector>
#include <list>


class GatePoint
{
public:
    explicit GatePoint();

    GatePoint(Point_2 ent, Point_2 exi);
    Point_2 entrance;
    Point_2 exit;

};
vector<Point_2> findPath(const Point_2 &start,
                         const Point_2 &end,
                         const Polygon_2 &robot,
                         vector<Polygon_2> &obstacles);


vector<int> TSPPlanner(vector<vector<double>> points);

vector<vector<vector<Point_2>>> RoadsPlanner(vector<Point_2> points, const Polygon_2 &robot, vector<Polygon_2> &obstacles);

vector<vector<double>> create_graph(vector<vector<vector<Point_2>>> roads);

void print_2d_mat(vector<vector<double>> mat);

void print_3d_mat(vector<vector<vector<Point_2>>> mat);

void print_vector(vector<int> vec);

void print_vector(vector<double> vec);

void print_path(vector<Point_2> path);

void print_point(Point_2 p);

vector<Point_2> plan_path(vector<Point_2> points, const Polygon_2 &robot, vector<Polygon_2> &obstacles);

vector<Point_2> plan_ski_path(vector<Point_2> gates, const Polygon_2 &robot, double dist_from_gates, Point_2 start_pos);

#endif //PATHFINDER_OPTIMALPATH_H
