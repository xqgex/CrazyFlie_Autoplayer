#include "OptimalPath.h"//
// Created by yoni on 20/08/18.
//

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
#include <sys/socket.h>
#include <stdlib.h>
#include <stdio.h>
#include <boost/timer.hpp>
#include <unistd.h>


//
// Created by gal on 16/08/18.
//
#include "OptimalPath.h"
#include "PathPlanner.h"

using CGAL::Bbox_2;
using std::vector;
using namespace std;
//typedef typename CGAL::Arrangement_2<Traits_2> Arrangement_2;
//typedef typename CGAL::Arr_landmarks_point_location<Arrangement_2> Point_location;


//todo

GatePoint::GatePoint(Point_2 ent, Point_2 exi)
{
       entrance = ent;
       exit = exi;
}


double pointsDistance(Point_2 a_point, Point_2 b_point)
{
    FT distance = (a_point.x() - b_point.x()) * (a_point.x() - b_point.x()) +
                  (a_point.y() - b_point.y()) * (a_point.y() - b_point.y());

    return sqrt(CGAL::to_double(distance));;
}


double path_length(vector<Point_2> path)
{
    if(path.size() == 0)
    {
        return 0.0;
    }
    double length = 0;
    Point_2 temp = path.at(0);
    for(auto point: path)
    {
        length += pointsDistance(temp, point);
        temp = point;
    }
    return length;
}
vector<Point_2> reverse_path(vector<Point_2> path)
{
    vector<Point_2> reversed;
    for(int i = 0; i<path.size(); i++)
    {
        reversed.push_back(path.at(path.size() -1 - i));
    }
    return reversed;
}

vector<Point_2> findPath(const Point_2 &start,
                         const Point_2 &end,
                         const Polygon_2 &robot,
                         vector<Polygon_2> &obstacles) {
    PathPlanner planner(start, end, robot, obstacles);
    return planner.planPath();
}


vector<int> TSPPlanner(vector<vector<double>> graph)
{
    int opt_path_length = 10000000;
    vector<int> opt_path;
    int length = graph.size();
    vector<int> temp_sites;

    for(int i=0; i<length; i++)
    {
        temp_sites.push_back(i);
    }
    do {
        int path_length = 0;
        for(int i=0; i<length-1; i++)
        {
            path_length += graph.at(temp_sites.at(i)).at(temp_sites.at(i+1));
        }


        if(path_length<opt_path_length)
        {
            //todo check
            opt_path_length = path_length;
            opt_path = temp_sites;
        }


    } while(std::next_permutation(temp_sites.begin()+1, temp_sites.end()));

    //print_vector(opt_path);
    return opt_path;
}

vector<vector<vector<Point_2>>> RoadsPlanner(vector<Point_2> points, const Polygon_2 &robot, vector<Polygon_2> &obstacles)
{
    vector<vector<vector<Point_2>>> roads;

    double length = points.size();

    //initialize roads

    for(int i=0; i<length; i++)
    {
        vector<vector<Point_2>> temp;
        for(int j=0; j<length; j++)
        {
            vector<Point_2> temp_j;
            temp.push_back(temp_j);
        }
        roads.push_back(temp);
    }

    for(int i=0; i<length; i++)
    {
        for(int j=i+1; j<length; j++)
        {
            //todo
            //vector<Point_2> path_ij(Path(findPath(points.at(i), points.at(j), robot, obstacles)).get_path()); //using pathplanner - between point i and point j
            vector<Point_2> path_ij(findPath(points.at(i), points.at(j), robot, obstacles)); //using pathplanner - between point i and point j
            roads.at(i).at(j) = path_ij;
            roads.at(j).at(i) = reverse_path(path_ij);
        }
    }
    return roads;
}


vector<vector<vector<Point_2>>> SkiRoadsPlanner(vector<GatePoint> points, const Polygon_2 &robot, vector<Polygon_2> &obstacles)
{
    vector<vector<vector<Point_2>>> roads;

    double length = points.size();

    //initialize roads

    for(int i=0; i<length; i++)
    {
        vector<vector<Point_2>> temp;
        for(int j=0; j<length; j++)
        {
            vector<Point_2> temp_j;
            temp.push_back(temp_j);
        }
        roads.push_back(temp);
    }

    for(int i=0; i<length; i++)
    {
        for(int j=0; j<length; j++)
        {
            //todo check
            if(j==i)
            {
                if(j==length-1)
                {
                    break;
                }
                j++;
            }
            vector<Point_2> path_ij(findPath(points.at(i).exit, points.at(j).entrance, robot, obstacles)); //using pathplanner - between point i and point j
            roads.at(i).at(j) = path_ij;
        }
    }
    return roads;
}


vector<vector<double>> create_ski_graph(vector<vector<vector<Point_2>>> roads)
{

    vector<vector<double>> roads_lengths;
    // initialize graph
    for(int i=0; i<roads.size(); i++)
    {
        vector<double> temp;
        for(int j=0; j<roads.size(); j++)
        {
            temp.push_back(0.0);
        }
        roads_lengths.push_back(temp);
    }


    for(int i=0; i<roads.size(); i++)
    {
        for(int j=0; j<roads.size(); j++)
        {
            double path_ij_length = path_length(roads.at(i).at(j));
            roads_lengths.at(i).at(j) = path_ij_length;
        }
    }

    return roads_lengths;
}


vector<GatePoint> SkiInitSites(vector<Point_2> gates, double dist_from_gate, Point_2 start_pos)
{
    vector<GatePoint> gate_points;
    gate_points.push_back(GatePoint(start_pos,start_pos));
    for(int i=0; i<gates.size()/2; i++)
    {
        Point_2 a = gates.at(2*i);
        Point_2 b = gates.at(2*i+1);
        double dist = CGAL::squared_distance(a,b).to_double();
        Vector_2 vec(a,b);
        Point_2 middle = a + (vec/2);

        Vector_2 perp =vec.perpendicular(CGAL::CLOCKWISE);
        perp = (perp/dist)*dist_from_gate;
        Point_2 entrance = middle + perp;

        perp =vec.perpendicular(CGAL::COUNTERCLOCKWISE);
        perp = (perp/dist)*dist_from_gate;
        Point_2 exit = middle + perp;

        gate_points.push_back(GatePoint(entrance,exit));
    }

    return gate_points;
}


vector<Polygon_2> SkiInitGates(vector<Point_2> gates)
{
    double width = 0.0001;
    vector<Polygon_2> closed_gates;
    for(int i=0; i<gates.size()/2; i++)
    {
        Polygon_2 gate;
        Point_2 a = gates.at(2*i);
        Point_2 b = gates.at(2*i+1);
        double dist = CGAL::squared_distance(a,b).to_double();
        Vector_2 vec(a,b);

        Vector_2 perp =vec.perpendicular(CGAL::CLOCKWISE);
        perp = (perp/dist)*width;
        gate.push_back(a + perp);
        gate.push_back(b + perp);

        perp =vec.perpendicular(CGAL::COUNTERCLOCKWISE);
        perp = (perp/dist)*width;
        gate.push_back(b + perp);
        gate.push_back(a + perp);

        closed_gates.push_back(gate);
    }

    return closed_gates;
}

vector<Point_2> plan_ski_path(vector<Point_2> gates, const Polygon_2 &robot, double dist_from_gates, Point_2 start_pos)
{
    boost::timer timer;

    vector<GatePoint> sites = SkiInitSites(gates, dist_from_gates, start_pos);
    vector<Polygon_2> closed_gates = SkiInitGates(gates);

    cout << "started SkiRoadsPlanner" << endl;
    // remember that the first site in sites should be the starting gate - which will be a GatePoint
    vector < vector < vector < Point_2 >> > ski_roads = SkiRoadsPlanner(sites, robot, closed_gates);
    auto secs = timer.elapsed();
    cout << "SkiRoadsPlanner finished in " << secs << " secs" << endl;
    timer.restart();

    cout << "started create_graph" << endl;
    vector <vector<double>> graph = create_graph(ski_roads);
    secs = timer.elapsed();
    cout << "create_graph finished in " << secs << " secs" << endl;
    timer.restart();


    vector<int> opt_path_indexes = TSPPlanner(graph);
    secs = timer.elapsed();
    cout << "TSPPlanner finished in " << secs << " secs" << endl;
    timer.restart();

    vector <Point_2> opt_path;
    opt_path.push_back(start_pos);

    //todo muad le pooraanut
    for (int i = 0; i < opt_path_indexes.size() - 1; i++)
    {
        vector<Point_2> next_seq = ski_roads.at(opt_path_indexes.at(i)).at(opt_path_indexes.at(i + 1));
        next_seq.push_back(  (sites.at(opt_path_indexes.at(i+1))).exit ); //todo very risky
        opt_path.insert(opt_path.end(), next_seq.begin() + 1, next_seq.end());
    }

    print_path(opt_path);
    return opt_path;
}



vector<vector<double>> create_graph(vector<vector<vector<Point_2>>> roads)
{

    vector<vector<double>> roads_lengths;
    // initialize graph
    for(int i=0; i<roads.size(); i++)
    {
        vector<double> temp;
        for(int j=0; j<roads.size(); j++)
        {
            temp.push_back(0.0);
        }
        roads_lengths.push_back(temp);
    }


    for(int i=0; i<roads.size(); i++)
    {
        for(int j=0; j<roads.size(); j++)
        {
            double path_ij_length = path_length(roads.at(i).at(j));
            roads_lengths.at(i).at(j) = path_ij_length;
            roads_lengths.at(j).at(i) = path_ij_length;
        }
    }


    return roads_lengths;
}


vector<Point_2> plan_path(vector<Point_2> points, const Polygon_2 &robot, vector<Polygon_2> &obstacles)
{
    boost::timer timer;
    cout << "started roads planner" << endl;
    vector<vector<vector<Point_2>>> roads = RoadsPlanner(points, robot, obstacles);
    auto secs = timer.elapsed();
    cout << "RoadsPlanner finished in " << secs << " secs" << endl;
    timer.restart();

    vector <vector<double>> graph = create_graph(roads);
    secs = timer.elapsed();
    cout << "create_graph finished in " << secs << " secs" << endl;
    timer.restart();


    vector<int> opt_path_indexes = TSPPlanner(graph);
    secs = timer.elapsed();
    cout << "TSPPlanner finished in " << secs << " secs" << endl;
    timer.restart();

    vector <Point_2> opt_path;
    opt_path.push_back(points.at(0));

    //todo muad le pooraanut
    for (int i = 0; i < opt_path_indexes.size() - 1; i++)
    {
        vector<Point_2> next_seq = roads.at(opt_path_indexes.at(i)).at(opt_path_indexes.at(i + 1));
        opt_path.insert(opt_path.end(), next_seq.begin() + 1, next_seq.end());
    }

    return opt_path;
}

void print_2d_mat(vector<vector<double>> mat)
{
    cout << "   ";
    for(int j=0; j<mat.size(); j++)
    {
        cout << j << "  ";
    }
    cout << endl;

    for(int i=0; i<mat.size(); i++)
    {
        cout << i << "  ";
        for(int j=0; j<mat.at(i).size(); j++)
        {
            cout << mat.at(i).at(j) << "  ";
        }
        cout << endl;
    }
    cout << endl;
}


void print_3d_mat(vector<vector<vector<Point_2>>> mat)
{
    for(int i =0; i<mat.size(); i++)
    {
        for(int j=0; j<mat.at(i).size(); j++)
        {
            for(int t=0; t<mat.at(i).at(j).size(); t++)
            {
                cout << mat.at(i).at(j).at(t) << " ";
            }
        }
        cout << endl;
    }
}

void print_vector(vector<int> vec)
{
    for(int x : vec)
    {
        cout << x << " ";
    }
    cout << endl;
}

void print_vector(vector<double> vec)
{
    for(int x : vec)
    {
        cout << x << " ";
    }
    cout << endl;
}

void print_point(Point_2 p)
{
    cout << "(" << p.x().to_double() << "," << p.y().to_double() << ") ";
}

void print_path(vector<Point_2> path)
{
    for(Point_2 p: path)
    {
        print_point(p);
    }
    cout << endl;
}



//todo
/*
vector<Point_2> naive_path_optimization(vector<Point_2> oldpath)
{
    vector<Point_2> path(oldpath);
    for(int i =0; i<path.size()-1; i++)
    {
        int best = -1;
        for(int j = i+2; j<path.size()-1; j++) // size()-1 ????? check
        {
            if (legal(i,j))
            {
                best = j;
            }
        }
        if(best != -1)
        {
            path.erase(path.begin() + i +1, path.begin() + best);
        }
    }

    return path;

}


 */
