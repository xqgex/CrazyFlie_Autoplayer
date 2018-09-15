//
// Created by yoni on 27/08/18.
//

#include "RequestHandler.h"
#include "CGAL_defines.h"

#include "OptimalPath.h"
#include "Path.h"
#include "PathPlanner.h"

#define DEFAULT_WORLD_SIZE_X 10.0
#define DEFAULT_WORLD_SIZE_Y 10.0

RequestHandler::RequestHandler() {
    world_size_x = 10.0; // default world size
    world_size_y = 10.0;
    drone_size = 0.1; //default drone size
    set_world_size(DEFAULT_WORLD_SIZE_X, DEFAULT_WORLD_SIZE_Y);
}

void RequestHandler::set_world_size(double x, double y)
{
    world_size_y = y;
    world_size_x = x;
    bounding_box = vector<Polygon_2>();

    Polygon_2 poly;
    poly.push_back(Point_2(0,0));
    poly.push_back(Point_2(0,y));
    poly.push_back(Point_2(-0.1, 0));
    bounding_box.push_back(poly);

    poly = Polygon_2();
    poly.push_back(Point_2(x,0));
    poly.push_back(Point_2(0,0));
    poly.push_back(Point_2(0,-0.1));
    bounding_box.push_back(poly);

    poly = Polygon_2();
    poly.push_back(Point_2(x,y));
    poly.push_back(Point_2(x,0));
    poly.push_back(Point_2(x+0.1,0));
    bounding_box.push_back(poly);

    poly = Polygon_2();
    poly.push_back(Point_2(0,y));
    poly.push_back(Point_2(x,y));
    poly.push_back(Point_2(0,y+0.1));
    bounding_box.push_back(poly);

}

void RequestHandler::set_drone_size(double x)
{
    drone_size = x;
}


std::string path2str(vector <Point_2> path)
{
    std::stringstream ss("");
    std::string str = "";
    for(Point_2 point: path)
    {
        ss << point.x().to_double() << " " << point.y().to_double() << " ";
    }
    return ss.str();
}

void print_square(Polygon_2 poly)
{
    for (int i=0; i<4; i++)
    {
        vector<Point_2> temp;
        temp.push_back((poly.vertex(i)));
        cout << path2str(temp) << "  ";
    }

    cout << endl;
}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}


vector<double> split_into_doubles(const std::string &s, char delim)
{
    vector<string> temp = split(s, delim);
    vector<double> ret;
    for(string str: temp)
    {
        ret.push_back(std::stod(str));
    }
    return ret;
}


void RequestHandler::handle_set_world(string req)
{
    vector<string> params = split(req, ' ');
    double x = std::stod(params.at(0));
    double y = std::stod(params.at(1));

    world_size_y = y;
    world_size_x = x;
    bounding_box = vector<Polygon_2>();

    Polygon_2 poly;
    poly.push_back(Point_2(0,0));
    poly.push_back(Point_2(0,y));
    poly.push_back(Point_2(-0.1, 0));
    bounding_box.push_back(poly);

    poly = Polygon_2();
    poly.push_back(Point_2(x,0));
    poly.push_back(Point_2(0,0));
    poly.push_back(Point_2(0,-0.1));
    bounding_box.push_back(poly);

    poly = Polygon_2();
    poly.push_back(Point_2(x,y));
    poly.push_back(Point_2(x,0));
    poly.push_back(Point_2(x+0.1,0));
    bounding_box.push_back(poly);

    poly = Polygon_2();
    poly.push_back(Point_2(0,y));
    poly.push_back(Point_2(x,y));
    poly.push_back(Point_2(0,y+0.1));
    bounding_box.push_back(poly);
}

void RequestHandler::handle_set_drone_size(string req)
{
    drone_size = std::stod(req);
}

void print_obs(vector<Polygon_2> polys)
{
    for(auto poly: polys)
    {
        print_square(poly);
    }
}


Polygon_2 RequestHandler::position_to_square(double x, double y)
{
    Polygon_2 ret;
    ret.push_back(Point_2(x-drone_size/2, y-drone_size/2));
    ret.push_back(Point_2(x+drone_size/2, y-drone_size/2));
    ret.push_back(Point_2(x+drone_size/2, y+drone_size/2));
    ret.push_back(Point_2(x-drone_size/2, y+drone_size/2));

    CGAL::Orientation orient = ret.orientation();
    if (CGAL::CLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

string RequestHandler::handle_find_path(std::string req)
{
    vector<string> req_params = split(req, '$');
    vector<double> start_pos = split_into_doubles(req_params.at(0), ' ');
    Polygon_2 robot = position_to_square(start_pos.at(0), start_pos.at(1));


    vector<Point_2> sites;
    sites.push_back(Point_2(start_pos.at(0), start_pos.at(1)));

    vector<double> sites_vec = split_into_doubles(req_params.at(1), ' ');
    for (int i = 0; i < sites_vec.size() / 2; i++)
    {
        sites.push_back(Point_2(sites_vec.at(2*i), sites_vec.at(2*i+1)));
    }

    vector<Polygon_2> drones;
    vector<double> temp_obs_drones;
    if(req_params.size()>=3)
    {
        temp_obs_drones = split_into_doubles(req_params.at(2), ' ');
        for (int i = 0; i < temp_obs_drones.size() / 2; i++)
        {
            drones.push_back(position_to_square(temp_obs_drones.at(2 * i), temp_obs_drones.at(2 * i + 1)));
        }

    }

    vector<Polygon_2> obs(drones);
    for(auto bound: bounding_box)
    {
        obs.push_back(bound);
    }

    vector<Point_2> path = plan_path(sites, robot, obs);

    cout << "path is:" << path2str(path) << endl;
    return path2str(path);
}


string RequestHandler::handle_find_ski_path(std::string req)
{
    vector<string> req_params = split(req, '$');
    vector<double> start_pos = split_into_doubles(req_params.at(0), ' ');
    Polygon_2 robot = position_to_square(start_pos.at(0), start_pos.at(1));

    vector<Point_2> gates;

    vector<double> sites_vec = split_into_doubles(req_params.at(1), ' ');
    for (int i = 0; i < sites_vec.size() / 2; i++)
    {
        gates.push_back(Point_2(sites_vec.at(2*i), sites_vec.at(2*i+1)));
    }

    vector<Point_2> path = plan_ski_path(gates, robot, 3*drone_size, Point_2(start_pos.at(0), start_pos.at(1)));

    cout << "path is:" << path2str(path) << endl;
    return path2str(path);
}


string RequestHandler::handle(std::string req)
{
    std::string delimiter = "$";
    size_t pos = req.find(delimiter); 
    std::string req_type = req.substr(0, pos);

    if(req_type == "set_world")
    {
        req.erase(0, pos + 1);
        handle_set_world(req);
        return "world";
    }
    if(req_type == "set_drone_size")
    {
        req.erase(0, pos + 1);
        handle_set_drone_size(req);
        return "drone";
    }

    if(req_type == "find_path")
    {
        req.erase(0, pos + 1);
        return handle_find_path(req);
    }
    if(req_type == "find_ski_path")
    {
        req.erase(0, pos + 1);
        return handle_find_ski_path(req);
    }
    else
    {
        cout << "not good" << endl;
        return "";
    }
}


