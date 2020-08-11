/*
 *   C++ sockets on Unix and Windows
 *   Copyright (C) 2002
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "../include/PracticalSocket/PracticalSocket.h"                        // For UDPSocket and SocketException
#include <iostream>                                 // For cout and cerr
#include <cstdlib>                                  // For atoi()
#include <string.h>

using namespace std;

const int ECHOMAX = 1;

int main(int argc, char *argv[]) {

      string servAddress = "192.168.0.74";             // First arg: server address
      // 每个字符代表一个动作 分别为 ”站立“ ”鼓掌“ ”踢腿“ ”平举“ ”挥手“
      char echoString[5][1] = {{'s'}, {'c'}, {'k'}, {'h'}, {'w'}};
      int echoStringLen = 1;                            // Length of string to echo
      if (echoStringLen > ECHOMAX) {                    // Check input length
        cerr << "Echo string too long" << endl;
        exit(1);
      }
        //先将消息发送至ROS过渡模块
      unsigned short echoServPort = 33177;
      try {
        UDPSocket sock;
        // Send the string to the server
        sock.sendTo(echoString[0], echoStringLen, servAddress, echoServPort);
        // Receive a response
    //    char echoBuffer[ECHOMAX + 1];                   // Buffer for echoed string + \0
    //    int respStringLen;                              // Length of received response
        // if ((respStringLen = sock.recv(echoBuffer, ECHOMAX)) != echoStringLen) {
        //   cerr << "Unable to receive" << endl;
        //   exit(1);
        // }
         cout << "333333333333\n";
        // echoBuffer[respStringLen] = '\0';             // Terminate the string!
        // cout << "Received: " << echoBuffer << endl;   // Print the echoed arg

        // Destructor closes the socket

      } catch (SocketException &e) {
        cerr << e.what() << endl;
        exit(1);
      }
      return 0;
}
