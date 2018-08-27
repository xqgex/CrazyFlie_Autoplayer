#ifndef PATHFINDER_PATHPLANNER_H
#define PATHFINDER_PATHPLANNER_H

#include <vector>
#include <list>
#include <map>
#include <set>
#include <math.h>

#include "CGAL_defines.h"

using namespace std;

class PointNode
{
public:
    explicit PointNode(FT distance, Point_2 point, PointNode* prev,  Halfedge_handle edge);
    PointNode() = default;

    FT distance;
    Point_2 point;
    PointNode* prev;
    Halfedge_handle edge;
    bool processed = false;
    vector<PointNode*> crossedSegments;
};

struct CmpfaceNodePtrs
{
    bool operator()(const PointNode* lhs, const PointNode* rhs) const;
};

class polygon_split_observer : public CGAL::Arr_observer<Arrangement_2>
{
    void after_split_face(Face_handle f1, Face_handle f2, bool) override;
};

class PathPlanner {
private:
    static const int EDGE_DIVIDING_PARAM;

    const Point_2 &startPoint;
    const Point_2 &endPoint;
    const Polygon_2 &robot;
    vector<Polygon_2> &obstacles;

    Polygon_2 invRobot;
    Polygon_set_2 freeSpace;

    //start and end faces:
    Face_handle start_face;
    Face_handle end_face;

    //bfs maps:
    set<PointNode*, CmpfaceNodePtrs> queue; //use set because need to delete efficiently
    map<Point_2, PointNode> pointsMap;

    map<Halfedge_handle, vector<Point_2>> edgesMap; //use to improve finding all interesting point on edge

    void setInversedRobot();
    void setFreeSpace();
    void addFrame(Arrangement_2 &arr);

    void verticalDecomposition(Arrangement_2 &arr, Kernel &ker);
    void addVerticalSegment(Arrangement_2 &arr, Vertex_handle v, CGAL::Object obj, Kernel &ker);

    Face_handle get_face(Arrangement_2& arr, const Landmarks_pl &pl, const Point_2 &p);

    void setFacesPath(Arrangement_2& arr);
    vector<Point_2> getEdgePoints(Halfedge_handle edge);

    void addFacesToQueue(Arrangement_2 &arr, PointNode* faceNode);
    void addFaceToQueue(Arrangement_2 &arr, PointNode *pointNode, Face_handle face);
    void addPointToQueue(PointNode *pointNode, Point_2 tempPoint, Halfedge_handle tempEdge);

    void tryToImprove(PointNode *pointNode, Point_2 tempPoint);
    void addStartPathToQueue(Arrangement_2 &arr);

    FT pointsDistance(Point_2 a, Point_2 b);

    Segment_2 getSegment(Halfedge_handle edge);
    Segment_2 getSegment(Point_2 a, Point_2 b);

    vector<Point_2> reversedPath(Arrangement_2& arr, Kernel& ker);

public:
    PathPlanner(const Point_2 start, const Point_2 end, const Polygon_2 &robot, vector<Polygon_2> &obstacles);
    vector<Point_2> planPath();
};

#endif //PATHFINDER_PATHPLANNER_H
