#include "Path.h"

Path::Path(const vector<Point_2> &path) : _path(path) {}

Polygon_2 Path::katamariDamacyRobot(const Segment_2 &segment, const Polygon_2 &robot) const {
    vector<Point_2> points;
    for (auto it = robot.vertices_begin(); it != robot.vertices_end(); ++it) {
        points.push_back(*it + (segment.source() - CGAL::ORIGIN));
        points.push_back(*it + (segment.target() - CGAL::ORIGIN));
    }

    vector<Point_2> hull;
    CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));
    return Polygon_2(hull.begin(), hull.end());
}

vector<Polygon_2> Path::strechRobotToSegment(const Segment_2 &segment, const Polygon_2 &robot) {
    list<CGAL::Partition_traits_2<Kernel>::Polygon_2> partition_polys;
    Polygon_2 robot1(robot);
    if (robot1.is_clockwise_oriented()) robot1.reverse_orientation();
    CGAL::approx_convex_partition_2(robot1.vertices_begin(),
                                    robot1.vertices_end(),
                                    std::back_inserter(partition_polys));
    vector<Polygon_2> realPartitions;
    for (auto &p:partition_polys) {
        Polygon_2 temp;
        for (auto it = p.vertices_begin(); it != p.vertices_end(); ++it)
            temp.push_back(Point_2(it->x(), it->y()) - (robot[0] - CGAL::ORIGIN));
        realPartitions.push_back(katamariDamacyRobot(segment, temp));
    }
    return realPartitions;
}

bool Path::verifyLine(const Segment_2 &segment, const Polygon_2 &robot,
                      vector<Polygon_2> &obstacles) {
    vector<Polygon_2> wideRobot = strechRobotToSegment(segment, robot);
    bool res = true;
    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (obstacles.at(i).is_clockwise_oriented()) obstacles.at(i).reverse_orientation();
        bool res1 = false;
        for (auto &p :wideRobot) {
            if (p.is_clockwise_oriented()) p.reverse_orientation();
            if (CGAL::do_intersect(p, obstacles.at(i))) {
                res1 = true;
                break;
            }
        }
        if (!obstacles.at(i).is_clockwise_oriented())obstacles.at(i).reverse_orientation();
        if (res1) {
            _badObstacles.push_back((int) i);
            res = false;
        }
    }
    return res;
}

bool Path::verify(const Point_2 &start, const Point_2 &end, const Polygon_2 &robot,
                  vector<Polygon_2> &obstacles) {
    if (_path.empty()) {
        cout << "FAILURE: result path is empty!" << endl;
        return false;
    }
    if (start != _path.front() || end != _path.back()) {
        if (start != _path.front()) cout << "FAILURE: result path start is not equal to the start point!" << endl;
        if (start != _path.front()) cout << "FAILURE: result path end is not equal to the end point!" << endl;
        _path.clear();
        return false;
    }
    for (size_t i = 0; i < _path.size() - 1; ++i)
        if (!verifyLine({_path.at(i), _path.at(i + 1)}, robot, obstacles)) {
            _badPath.push_back(_path.at(i));
            _badPath.push_back(_path.at(i + 1));
            while (_path.size() > i + 1) _path.pop_back();
            return false;
        }
    return true;
}

vector<Point_2> Path::get_path()
{
    vector<Point_2> ret(this->_path);
    cout << "path length" << this->_path.size() << endl;
    cout << "passed get_path" << endl;
    return ret;
}