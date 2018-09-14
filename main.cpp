#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"

#include "Server.h"
#include "RequestHandler.h"
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

using namespace std;

int main(int argc, char *argv[])
{
    Server server = Server();
    while(true)
    {
        RequestHandler handler = RequestHandler();        
        server.waitForConnection();

        while(true)
        {
            string request = server.readRequest();
            if(request.length() == 0)
            {
                break;
            }
            string response = handler.handle(request);
            server.send(response);
        } 
    }
}

































/**
{

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

>>>>>>> be38405da661b9ce97b5cc099da65c8cc3c88954
    }
}
