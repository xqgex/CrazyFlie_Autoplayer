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
    Server server = Server(); //create a server
    while(true)
    {
        RequestHandler handler = RequestHandler(); //create request handler        
        server.waitForConnection(); //wait for connection

        while(true)
        {
            string request = server.readRequest(); //wait for command
            if(request.length() == 0)
            {
                break;
            }
            string response = handler.handle(request); //handle command
            server.send(response); //send response
        } 
    }
}