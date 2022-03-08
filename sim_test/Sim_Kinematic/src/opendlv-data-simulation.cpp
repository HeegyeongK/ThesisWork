//Simulation test_1

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <cmath>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
       (0 == commandlineArguments.count("freq")) ||
       (0 == commandlineArguments.count("numDrones")) ||
       (0 == commandlineArguments.count("startX")) || 
       (0 == commandlineArguments.count("startY")) ) {
      std::cerr << argv[0] << "tests drone visualisation by sending dummy flight data" << std::endl;
      std::cerr << "         --cid:  CID of the OD4Session to send and receive messages" << std::endl;
      std::cerr << "         --freq:  map update frequency" << std::endl;
      std::cerr << "         --numDrones:  number of drones" << std::endl;
      std::cerr << "         --startX:  starting x coordinate" << std::endl;
      std::cerr << "         --startY:  starting y coordinate" << std::endl; 
      std::cerr << "Example: " << argv[0] << "--cid=111 --freq=15 --numDrones=2 --startX=0.0 --startY=0.0 --verbose" << std::endl;
      retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    const uint32_t FREQ{static_cast<uint32_t>(std::stoi(commandlineArguments["freq"]))};
    const uint32_t NUM_DRONES{static_cast<uint32_t>(std::stoi(commandlineArguments["numDrones"]))};
    const float START_X{static_cast<float>(std::stof(commandlineArguments["startX"]))};
    const float START_Y{static_cast<float>(std::stof(commandlineArguments["startY"]))};
    
    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
    cluon::OD4Session od4{CID};

    //Starting Position
    opendlv::sim::Frame startFrame;
    startFrame.x(START_X);
    startFrame.y(START_Y);
    startFrame.z(0);
    startFrame.roll(0);
    startFrame.pitch(0);
    startFrame.yaw(0);

/*
    opendlv::sim::Frame startFrame_Drone;
    startFrame_Drone.x(START_X);
    startFrame_Drone.y(START_Y);
    startFrame_Drone.z(0);
    startFrame_Drone.roll(0);
    startFrame_Drone.pitch(0);
    startFrame_Drone.yaw(0);
*/

    //Initialize
    std::vector<opendlv::sim::Frame> droneFrames(NUM_DRONES, startFrame);
    opendlv::sim::Frame boatFrame(startFrame);
    uint32_t senderStamp;

    //Main loop
    float t = 0.0f;
    float dt = 0.5f;
    float tMax = 1000.0f;
    //float dv = 0.0001f;
    
    float c = 0.0f;
    float costh = 0.0f;
    float sinth = 0.0f;
    

    while (t<tMax){
      t=t+dt;
      
      //update position
      boatFrame.x(START_X+t);
      boatFrame.y(START_Y+t);
      
      c = std::sqrt(std::pow(boatFrame.x(),2)+ std::pow(boatFrame.y(),2));
      costh = (boatFrame.x()/c);
      sinth = (boatFrame.y()/c);

      droneFrames[0].x( c * costh - 30 * sinth);
      droneFrames[0].y( c * sinth + 30 * costh);
      droneFrames[1].x( c * costh + 30 * sinth);
      droneFrames[1].y( c * sinth - 30 * costh);


      /*
      droneFrames[0].x((boatFrame.x()+2+c)*costh - (boatFrame.y()-2)*sinth);
      droneFrames[0].y((boatFrame.x()+2+c)*sinth + (boatFrame.y()-2)*costh);
      
      droneFrames[1].x((boatFrame.x()+2+c)*costh - (boatFrame.y()+2)*sinth);
      droneFrames[1].y((boatFrame.x()+2+c)*sinth + (boatFrame.y()+2)*costh);
      */

      //send boat data
      cluon::data::TimeStamp sampleTime;
      senderStamp = 0;
      od4.send(boatFrame, sampleTime, senderStamp);
      if (VERBOSE) {
        std::cout << "Sending position, boat : x ="<<boatFrame.x() <<" ,y= "<< boatFrame.y() << std::endl;
      }

      //send dron data 
      for (uint32_t iDrone = 0; iDrone < NUM_DRONES; iDrone++){
        senderStamp = iDrone + 1; // +1 since boat has senderStamp 0
        od4.send(droneFrames[iDrone], sampleTime, senderStamp);
        if (VERBOSE){
          std::cout << "Sending positions, drone " << iDrone << ": x = "<< droneFrames[iDrone].x() << ", y = "<<droneFrames[iDrone].y()<<std::endl;

        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1000/FREQ));

    }
  }
  return retCode;
}  

    