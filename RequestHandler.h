//
// Created by yoni on 27/08/18.
//

#ifndef PATHFINDER_REQUESTHANDLER_H
#define PATHFINDER_REQUESTHANDLER_H
#include <vector>
#include <list>
#include <map>
#include <set>
#include <math.h>

#include "CGAL_defines.h"

using namespace std;

class RequestHandler
{
public:
    explicit RequestHandler();
    double world_size_x;
    double world_size_y;
    double drone_size;

    string handle(string request);


private:
    void set_world_size(double x, double y);
    void set_drone_size(double x);
    void handle_set_drone_size(std::string req);
    void handle_set_world(std::string req);
    Polygon_2 position_to_square(double x, double y);
    string handle_find_path(std::string req);
};



#endif //PATHFINDER_REQUESTHANDLER_H
