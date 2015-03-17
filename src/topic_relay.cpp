/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Institute for Artificial Intelligence,
 *  Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Jan Winkler */


#include <iostream>
#include <cstdlib>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <ros/message.h>
#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <ros/message_traits.h>


int g_argc;
char** g_argv;
int nFDWrite;
std::string strMD5;
std::string strDataType;
char* lastmsg;
int lastlength;


std::string exec(std::string cmd) {
  FILE* pipe = popen(cmd.c_str(), "r");
  
  if(!pipe) {
    return "ERROR";
  }
  
  char buffer[128];
  std::string result = "";
  
  while(!feof(pipe)) {
    if(fgets(buffer, 128, pipe) != NULL) {
      result += buffer;
    }
  }
  
  pclose(pipe);
  
  return result;
}


template <class ContainerAllocator>
struct FakeMessage_ {
  int nLength;
  char* cData;
  
  FakeMessage_() {}
  
  void deserialize(uint8_t* uData, int nLength) {
    std::stringstream sts;
    
    if(lastlength == nLength) {
      if(lastmsg) {
	if(memcmp(lastmsg, uData, nLength) == 0) {
	  return;
	}
      }
    }
    
    for(int nI = 0; nI < nLength; nI++) {
      sts << (int)uData[nI];
      sts << " ";
    }
    
    sts << "\0";
    write(nFDWrite, sts.str().c_str(), sts.str().length());
  }
};


typedef FakeMessage_<std::allocator<void> > FakeMessage;


ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(FakeMessage, strMD5.c_str(), strDataType.c_str(), "")


namespace ros {
  namespace serialization {
    template<class ContainerAllocator>
    struct Serializer< ::FakeMessage_<ContainerAllocator> > {
      template<typename Stream>
      inline static void read(Stream& stream, ::FakeMessage_<ContainerAllocator> t) {
	t.deserialize(stream.getData(), stream.getLength());
      }
      
      inline static void write(Stream& stream, ::FakeMessage t) {
	memcpy(stream.advance(sizeof(t.nLength)), t.cData, t.nLength);
      }
      
      inline static uint32_t serializedLength(const ::FakeMessage t) {
	return t.nLength;
      }
    };
  }
}

void _callback(FakeMessage fmMessage) {} // Dummy

typedef void pfunc_t (int rfd, int wfd);

void childHandler(int rfd, int wfd) {
  nFDWrite = wfd;
  
  int nRead;
  char buffer[1024];
  
  std::string strTopic;
  std::string strMaster;
  
  // Read parameters
  while(true) {
    nRead = read(rfd, buffer, 1024);
    
    if(nRead  > 0) {
      buffer[nRead] = 0;
      
      char cTopic[80];
      char cMaster[80];
      
      memset(cTopic, 0, 80);
      memset(cMaster, 0, 80);
      
      sscanf(buffer, "%s %s", cTopic, cMaster);
      strTopic = cTopic;
      strMaster = cMaster;
      
      break;
    }
  }
  
  int argc = g_argc + 1;
  char** argv = new char*[argc];
  
  for(int nI = 0; nI < g_argc; nI++) {
    argv[nI] = g_argv[nI];
  }
  
  argv[g_argc] = (char*)(std::string("__master:=") + strMaster).c_str();
  
  ros::init(argc, argv, "topic_relay");
  ros::NodeHandle nh;
  ros::SubscribeOptions ops;
  ops.topic = strTopic;
  
  typedef typename ros::ParameterAdapter<FakeMessage>::Message MessageType;
  ros::SubscriptionCallbackHelperT<FakeMessage>* sch = new ros::SubscriptionCallbackHelperT<FakeMessage>(_callback, ros::DefaultMessageCreator<FakeMessage>());
  ops.helper = ros::SubscriptionCallbackHelperPtr(sch);
  
  ros::master::V_TopicInfo topics;
  bool bHasTopic = false;
  
  while(!bHasTopic) {
    ros::master::getTopics(topics);
    
    for(ros::master::TopicInfo ti : topics) {
      if(ti.name == ops.topic) {
	ops.datatype = ti.datatype;
	strDataType = ops.datatype;
	strMD5 = exec("rosmsg md5 " + ops.datatype);
	ops.md5sum = strMD5.substr(0, strMD5.length() - 1);
	strMD5 = ops.md5sum;
	bHasTopic = true;
	
	break;
      }
    }
  }
  
  ros::Subscriber subTest = nh.subscribe(ops);
  
  // Now initialize the publisher
  ros::SubscriberCallbacksPtr scp;
  ros::Publisher pubTopic = nh.advertise<FakeMessage>(strTopic, 1);
  
  int flags = fcntl(rfd, F_GETFL, 0);
  fcntl(rfd, F_SETFL, flags | O_NONBLOCK);
  
  while(true) {
    ros::spinOnce();
    nRead = read(rfd, buffer, 1024);
    
    if(nRead > 0) {
      buffer[nRead] = 0;
      std::string strContent = buffer;
      
      size_t szStart = 0;
      size_t szCurrent = 0;
      memset(buffer, 0, 1024);
      int nIndex = 0;
      
      while(true) {
	szCurrent = strContent.find(" ", szStart);
	if(szCurrent == std::string::npos) {
	  break;
	}
	
	std::string strSub = strContent.substr(szStart, szCurrent - szStart);
	int nCurrent;
	sscanf(strSub.c_str(), "%d ", &nCurrent);
	
	buffer[nIndex] = (char)nCurrent;
	nIndex++;
	
	szStart = szCurrent + 1;
      }
      
      FakeMessage fmMessage;
      fmMessage.nLength = nIndex;
      fmMessage.cData = new char[nIndex];
      memcpy(fmMessage.cData, buffer, nIndex);
      
      if(lastmsg) {
	delete[] lastmsg;
      }
      
      lastmsg = new char[nIndex];
      lastlength = nIndex;
      memcpy(lastmsg, fmMessage.cData, nIndex);
      
      pubTopic.publish(fmMessage);
      
      delete[] fmMessage.cData;
    }
  }
  
  delete[] argv;
}

pid_t pcreate(int fds[2], pfunc_t pfunc) {
  /* Spawn a process from pfunc, returning it's pid. The fds array passed will
   * be filled with two descriptors: fds[0] will read from the child process,
   * and fds[1] will write to it.
   * Similarly, the child process will receive a reading/writing fd set (in
   * that same order) as arguments.
   */
  pid_t pid;
  int pipes[4];

  /* Warning: I'm not handling possible errors in pipe/fork */

  pipe(&pipes[0]); /* Parent read/child write pipe */
  pipe(&pipes[2]); /* Child read/parent write pipe */

  if ((pid = fork()) > 0) {
    /* Parent process */
    fds[0] = pipes[0];
    fds[1] = pipes[3];

    close(pipes[1]);
    close(pipes[2]);

    return pid;
  } else {
    close(pipes[0]);
    close(pipes[3]);

    pfunc(pipes[2], pipes[1]);
    
    exit(0);
  }

  return -1; /* ? */
}

int main(int argc, char** argv) {
  lastmsg = NULL;
  lastlength = 0;
  
  int fdschild1[2];
  int fdschild2[2];
  
  g_argc = argc;
  g_argv = argv;
  
  if(pcreate(fdschild1, childHandler) == 0) {
    // Child
  } else {
    // Parent
    pcreate(fdschild2, childHandler);
    
    std::string strParams1 = std::string(argv[1]) + " " + std::string(argv[2]);
    std::string strParams2 = std::string(argv[1]) + " " + std::string(argv[3]);
    
    write(fdschild1[1], strParams1.c_str(), strParams1.length());
    write(fdschild2[1], strParams2.c_str(), strParams2.length());
    
    int nRead;
    char buffer[1024];
    
    int flags = fcntl(fdschild1[0], F_GETFL, 0);
    fcntl(fdschild1[0], F_SETFL, flags | O_NONBLOCK);
    
    flags = fcntl(fdschild2[0], F_GETFL, 0);
    fcntl(fdschild2[0], F_SETFL, flags | O_NONBLOCK);
    
    while(true) {
      nRead = read(fdschild1[0], buffer, sizeof(buffer));
      
      if(nRead > 0) {
	buffer[nRead] = 0;
	std::string strt = std::string(buffer);
	write(fdschild2[1], buffer, nRead);
      }
      
      nRead = read(fdschild2[0], buffer, sizeof(buffer));
      
      if(nRead > 0) {
	buffer[nRead] = 0;
	std::string strt = std::string(buffer);
	write(fdschild1[1], buffer, nRead);
      }
    }
  }
  
  return EXIT_SUCCESS;
}
