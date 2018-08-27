#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

#include "OptimalPath.h"
#include "Path.h"
#include "PathPlanner.h"

//for connection
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8080


using namespace std;

Point_2 loadPoint_2(std::ifstream &is) {
    Kernel::FT x, y;
    is >> x >> y;
    Point_2 point(x, y);
    return point;
}

Polygon_2 loadPolygon(ifstream &is) {
    size_t polygon_size = 0;
    is >> polygon_size;
    Polygon_2 ret;
    while (polygon_size--)
        ret.push_back(loadPoint_2(is));
    CGAL::Orientation orient = ret.orientation();
    if (CGAL::CLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
    size_t number_of_polygons = 0;
    is >> number_of_polygons;
    vector<Polygon_2> ret;
    while (number_of_polygons--)
        ret.push_back(loadPolygon(is));
    return ret;
}


vector<Point_2> loadSites(ifstream &is, vector<Point_2> &ret)
{
    size_t number_of_sites = 0;
    is >> number_of_sites;
    //cout << "num of sites" << number_of_sites << endl;
    //vector<Point_2 > ret;
    while (number_of_sites--)
        ret.push_back(loadPoint_2(is)); //check
    return ret;
}

std::string path2str(vector <Point_2> path)
{
    std::stringstream ss("");
    std::string str = "";
    for(Point_2 point: path)
    {
        ss << point.x().to_double() << " " << point.y().to_double() << "  ";
    }
    ss << endl;
    return ss.str();
}
int main(int argc, char *argv[])
{

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    cout << "listening...";
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                             (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    cout << "connection accepted" << endl;

    int param_read = read(new_socket , buffer, 1024);

    ofstream myfile;
    myfile.open("temp_robot.txt");
    myfile << buffer; //check
    myfile.close();


    //printf("%s\n",buffer );
    send(new_socket , "1" , 2 , 0 );


    param_read = read(new_socket , buffer, 1024);
    myfile.open("temp_sites.txt");
    myfile << buffer; //check
    myfile.close();



    //printf("%s\n",buffer );
    send(new_socket , "1" , 2 , 0 );

    param_read = read(new_socket , buffer, 1024);
    myfile.open("temp_obs.txt");
    myfile << buffer; //check
    myfile.close();

    //printf("%s\n",buffer );
    send(new_socket , "1" , 2 , 0 );

    ifstream inputRobotFile("temp_robot.txt"), inputSitesFile("temp_sites.txt"), inputObstaclesFile("temp_obs.txt");





    /*
    if (argc != 5) {
        cerr << "[USAGE]: inputRobot inputSites inputObstacles outputFile" << endl;
        return 1;
    }


   ifstream inputRobotFile(argv[1]), inputSitesFile(argv[2]), inputObstaclesFile(argv[3]);
    if (!inputRobotFile.is_open() || !inputObstaclesFile.is_open())
    {
        if (!inputRobotFile.is_open())
            cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
        if (!inputObstaclesFile.is_open())
            cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
        if (!inputSitesFile.is_open())
            cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
        return -1;
    }
     */


    if (!inputRobotFile.is_open() || !inputObstaclesFile.is_open())
    {
        if (!inputRobotFile.is_open())
            cerr << "ERROR: Couldn't open file: 1" << endl;
        if (!inputObstaclesFile.is_open())
            cerr << "ERROR: Couldn't open file: 2" << endl;
        if (!inputSitesFile.is_open())
            cerr << "ERROR: Couldn't open file: 3" << endl;
        return -1;
    }

    auto startPoint = loadPoint_2(inputRobotFile);
    auto robot = loadPolygon(inputRobotFile);
    inputRobotFile.close();

    vector <Point_2> points;
    points.push_back(startPoint);
    vector <Point_2> sites = loadSites(inputSitesFile, points);
    //cout << "finished loadSites" << endl;
    inputSitesFile.close();

    //cout << "start loadPolygons" << endl;
    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();
    //cout << "finished loadPolygons" << endl;

    try {
        boost::timer timer;
        //todo

        vector < vector < vector < Point_2 >> > roads = RoadsPlanner(points, robot, obstacles);
        auto secs = timer.elapsed();
        cout << "RoadsPlanner finished in " << secs << " secs" << endl;
        timer.restart();

        //print_3d_mat(roads);
        vector <vector<double>> graph = create_graph(roads);
        secs = timer.elapsed();
        cout << "create_graph finished in " << secs << " secs" << endl;
        timer.restart();

        //print_2d_mat(graph);

        vector<int> opt_path_indexes = TSPPlanner(graph);
        secs = timer.elapsed();
        cout << "TSPPlanner finished in " << secs << " secs" << endl;
        timer.restart();


        vector <Point_2> opt_path;
        opt_path.push_back(startPoint);
        for (int i = 0; i < opt_path_indexes.size() - 1; i++) {
            vector <Point_2> next_seq = roads.at(opt_path_indexes.at(i)).at(opt_path_indexes.at(i + 1));
            opt_path.insert(opt_path.end(), next_seq.begin()+1, next_seq.end());

            //vector<Point_2> *next_seq = &roads.at(opt_path_indexes.at(i)).at(opt_path_indexes.at(i+1));
            //opt_path.insert(opt_path.end(), (*next_seq).begin(), (*next_seq).end() );
        }

        cout << "path: ";
        print_path(opt_path);

        //todo

        cout <<"path length: " << path2str(opt_path).length() << endl;

        send(new_socket , path2str(opt_path).c_str() , path2str(opt_path).length()* sizeof(char) , 0 );

        ofstream outputFile;
        //outputFile.open(argv[4]);
        outputFile.open("./tests/conn_output");
        if (!outputFile.is_open()) {
            cerr << "ERROR: Couldn't open file: " << argv[4] << endl;
            return -1;
        }



        outputFile << opt_path.size() << " ";
        for (Point_2 point: opt_path) {
            outputFile << point.x().to_double() << " ";
            outputFile << point.y().to_double() << " ";
        }

        outputFile.close();
    }
    catch (const char *c) {
        cout << "ERROR: " << c << endl;
        return 0;
    }
    return 0;
}
