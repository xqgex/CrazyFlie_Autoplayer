//
// Created by t-idkess on 18-Mar-18.
//

#ifndef INC_2_3_PATH_H
#define INC_2_3_PATH_H

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


using namespace std;

class Path {
private:
    vector<Point_2> _path;
    vector<Point_2> _badPath;
    vector<int> _badObstacles;

    Polygon_2 katamariDamacyRobot(const Segment_2 &segment, const Polygon_2 &robot) const;
    vector<Polygon_2> strechRobotToSegment(const Segment_2 &segment, const Polygon_2 &robot);
    bool verifyLine(const Segment_2 &segment, const Polygon_2 &robot,
                    vector<Polygon_2> &obstacles) ;

public:
    Path(const vector<Point_2> &path);

    bool verify(const Point_2 &start, const Point_2 &end, const Polygon_2 &robot,
                vector<Polygon_2> &obstacles);

    vector<Point_2> get_path();

    friend std::ostream &operator<<(std::ostream &out, Path const &data) {
        out << data._path.size();
        for (auto it = data._path.begin(); it != data._path.end(); ++it) {
            out << " " << it->x().to_double() << " " << it->y().to_double();
        }
        out << endl;
        out << data._badPath.size();
        for (auto it = data._badPath.begin(); it != data._badPath.end(); ++it) {
            out << " " << it->x().to_double() << " " << it->y().to_double();
        }
        out << endl;
        out << data._badObstacles.size();
        for (auto it = data._badObstacles.begin(); it != data._badObstacles.end(); ++it) {
            out << " " << *it ;
        }
        out << endl;
        return out;
    }
};

#endif //INC_2_3_PATH_H
