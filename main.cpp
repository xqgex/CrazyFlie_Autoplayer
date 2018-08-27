#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

#include "OptimalPath.h"
#include "Path.h"
#include "PathPlanner.h"
#include <stdlib.h>
//for connection
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8080
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

static double world_size_x = 10;
static double world_size_y = 10;
static double drone_size = 1;
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


void handle_set_world(std::string req)
{
    vector<string> params = split(req, ' ');
    world_size_x = std::stod(params.at(0));
    world_size_y = std::stod(params.at(1));
}

void handle_set_drone_size(std::string req)
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


Polygon_2 position_to_square(double x, double y)
{
    //todo tofix
    Polygon_2 ret;
    ret.push_back(Point_2(x-drone_size/2, y-drone_size/2));
    ret.push_back(Point_2(x+drone_size/2, y-drone_size/2));
    ret.push_back(Point_2(x+drone_size/2, y+drone_size/2));
    ret.push_back(Point_2(x-drone_size/2, y+drone_size/2));
    return ret;


    CGAL::Orientation orient = ret.orientation();
    if (CGAL::CLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

string handle_find_path(std::string req)
{
    vector<string> req_params = split(req, '$');
    vector<double> start_pos = split_into_doubles(req_params.at(0), ' ');
    Polygon_2 robot = position_to_square(start_pos.at(0), start_pos.at(1));


    vector<Point_2> sites;
    //todo check below
    sites.push_back(Point_2(start_pos.at(0), start_pos.at(1)));

    vector<double> sites_vec = split_into_doubles(req_params.at(1), ' ');
    for (int i = 0; i < sites_vec.size() / 2; i++)
    {
        sites.push_back(Point_2(sites_vec.at(2*i), sites_vec.at(2*i+1)));
    }


    //cout << path2str(sites) << endl;
    vector<Polygon_2> drones;
    vector<double> temp_obs_drones = split_into_doubles(req_params.at(2), ' ');
    for (int i = 0; i < temp_obs_drones.size() / 2; i++)
    {
        drones.push_back(position_to_square(temp_obs_drones.at(2 * i), temp_obs_drones.at(2 * i + 1)));
    }

    //print_obs(drones);

    vector<Point_2> path = plan_path(sites, robot, drones);
    cout << "path is:" << path2str(path) << endl;
    return path2str(path);
}


string handle_request(std::string req)
{
    std::string delimiter = "$";
    size_t pos = req.find(delimiter); //todo if delimiter is not found
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
    else
        {
        cout << "not good" << endl;
        return "not good";
        //todo illegal request
    }
}


int main(int argc, char *argv[]) {

    //handle_request("find_path$0 0$2 2 4 4 8 8$6 6");
    //sleep(3);
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    cout << "socket is ready for action!" << endl;
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *) &address,
             sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    while (true) {
        if ((new_socket = accept(server_fd, (struct sockaddr *) &address,
                                 (socklen_t *) &addrlen)) < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        cout << "connection accepted" << endl;

        int param_read = read(new_socket, buffer, 1024);

        //cout << "buffer    " << buffer << endl;
        string res = handle_request(buffer);



        // another read for skipping the type of the read
        send(new_socket, res.c_str(), res.length(), 0);

        cout << "handled request! " << endl;

    }
}




    /**

    sleep(10);

    cout << "got response" << endl;
    ofstream myfile;
    myfile.open("temp_robot.txt");
    myfile << buffer; //check
    myfile.close();


    cout << "robot" << endl;
    printf("%s\n",buffer );
    send(new_socket , "1" , 2 , 0 );


    param_read = read(new_socket , buffer, 1024);
    myfile.open("temp_sites.txt");
    myfile << buffer; //check
    myfile.close();


    cout << "got response" << endl;

    printf("%s\n",buffer );
    send(new_socket , "1" , 2 , 0 );

    param_read = read(new_socket , buffer, 1024);
    myfile.open("temp_obs.txt");
    myfile << buffer; //check
    myfile.close();


    printf("%s\n",buffer );
    send(new_socket , "1" , 2 , 0 );



    cout << "got response" << endl;
    ifstream inputRobotFile("temp_robot.txt"), inputSitesFile("temp_sites.txt"), inputObstaclesFile("temp_obs.txt");


    cout << "here" << endl;





    if (argc != 5) {
        cerr << "[USAGE]: inputRobot inputSites inputObstacles outputFile" << endl;
        return 1;
    }

  //////
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
     ///


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

    cout << "here2" << endl;

    auto startPoint = loadPoint_2(inputRobotFile);
    auto robot = loadPolygon(inputRobotFile);
    inputRobotFile.close();

    cout << "here3" << endl;

    vector <Point_2> points;
    points.push_back(startPoint);
    vector <Point_2> sites = loadSites(inputSitesFile, points);
    cout << "finished loadSites" << endl;
    inputSitesFile.close();

    cout << "here" << endl;
    cout << "start loadPolygons" << endl;
    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();
    cout << "finished loadPolygons" << endl;

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
     */

