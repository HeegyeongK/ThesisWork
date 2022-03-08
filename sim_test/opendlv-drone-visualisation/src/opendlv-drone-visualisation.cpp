#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>

//Rescales from longitude/latitude coordinates to pixel positions in image (using start location as middle of the map)

/*
cv::Point getpixelPosition(float xCoordinate, float yCoordinate, float startX, float startY, float latSpan, float longSpan, uint32_t width, uint32_t height) {
  float relPosX = 0.5f + (xCoordinate - startX) / latSpan; 
  float relPosY = 0.5f + (yCoordinate - startY) / longSpan;
  uint32_t imgX = round(relPosX * width);
  uint32_t imgY = round(height - relPosY * height); //Redefined so positive y direction is up/north
  return cv::Point(imgX, imgY);
}
*/

//Make it start from the center of the frame
cv::Point getpixelPosition(float xCoordinate, float yCoordinate, float START_X, float START_Y, uint32_t width, uint32_t height){
  float relPosX = (xCoordinate - START_X) + width / 2;
  float relPosY = height - ((yCoordinate - START_Y) + height / 2);
  uint32_t imgX = round (relPosX);
  uint32_t imgY = round (relPosY);
  return cv::Point(imgX, imgY);
}
int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("freq")) ||
         (0 == commandlineArguments.count("numDrones")) ||
         (0 == commandlineArguments.count("startX")) ||
         (0 == commandlineArguments.count("startY")) ||
         (0 == commandlineArguments.count("latSpan")) || 
         (0 == commandlineArguments.count("longSpan")) ||        
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << "         --cid:  CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --freq:  map update frequency" << std::endl;
        std::cerr << "         --numDrones:  number of drones" << std::endl;
        std::cerr << "         --startX:  starting x coordinate" << std::endl;
        std::cerr << "         --numDrones:  starting y coordinate" << std::endl; 
        std::cerr << "         --latSpan:  how many latitudes the map spans" << std::endl;        
        std::cerr << "         --longSpan:  how many longitudes the map spans" << std::endl; 
        std::cerr << "         --width:  width of the plot window" << std::endl;
        std::cerr << "         --height:  height of the plot window" << std::endl;
        std::cerr << "Example: " << argv[0] << "--cid=111 --freq=30 --numDrones=2 --startX=20.0 --startY=60.0 --longSpan=0.1 --latSpan=0.1 --width=800 --height=800 --verbose" << std::endl;
    }
    else {
        uint16_t const CID = std::stoi(commandlineArguments["cid"]);
        const uint32_t FREQ{static_cast<uint32_t>(std::stoi(commandlineArguments["freq"]))};
        const uint32_t NUM_DRONES{static_cast<uint32_t>(std::stoi(commandlineArguments["numDrones"]))};
        const float START_X{static_cast<float>(std::stof(commandlineArguments["startX"]))};
        const float START_Y{static_cast<float>(std::stof(commandlineArguments["startY"]))};
        const float LATSPAN{static_cast<float>(std::stof(commandlineArguments["latSpan"]))};
        const float LONGSPAN{static_cast<float>(std::stof(commandlineArguments["longSpan"]))}; 
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};       
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        
        // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
        cluon::OD4Session od4{CID};
        
        //Starting position
        opendlv::sim::Frame startFrame;
        startFrame.x(START_X); //Latitude
        startFrame.y(START_Y); //Longitude
        startFrame.z(0); //Altitude
        startFrame.roll(0);
        startFrame.pitch(0);
        startFrame.yaw(0);
        
        //Initialization
        std::vector<opendlv::sim::Frame> droneFrames(NUM_DRONES, startFrame);
        opendlv::sim::Frame boatFrame(startFrame);
        std::mutex frameMutex;
        
        //Handle frame updates
        auto onFrameUpdate{[&boatFrame, &droneFrames, &frameMutex, &VERBOSE](
            cluon::data::Envelope &&envelope)            
          {
            std::lock_guard<std::mutex> const lock(frameMutex);
            uint32_t const senderStamp = envelope.senderStamp();
            auto frame = 
              cluon::extractMessage<opendlv::sim::Frame>(
                  std::move(envelope));
            
            if (senderStamp == 0) {
              boatFrame = frame;
              if (VERBOSE) {
                std::cout<< "Let's see how it works"<<std::endl;
                //std::cout << "Received positions, boat: x = " << boatFrame.x() << ", y = " << boatFrame.y() << std::endl;
               }   
            } else {      
              uint32_t iDrone = senderStamp - 1; //-1 since first senderStamp is for the boat
              droneFrames[iDrone] = frame;
              if (VERBOSE) {
                std::cout << "Received positions, drone " << iDrone << ": x = " << droneFrames[iDrone].x() << ", y = " << droneFrames[iDrone].y() << std::endl;
              } 
            }
          }};

        // Register lambda for the message identifier for opendlv::sim::Frame
        od4.dataTrigger(opendlv::sim::Frame::ID(), onFrameUpdate);       
          
        //Create map grid
        cv::Mat imgGrid(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        for (uint32_t i = 0; i < 10; i++){
          cv::line(imgGrid, cv::Point(0, i*(HEIGHT/10)), cv::Point(WIDTH, i*(HEIGHT/10)), cv::Scalar(30,30,30, 1), 1);
          cv::line(imgGrid, cv::Point(i*(WIDTH/10), 0), cv::Point(i*(WIDTH/10), HEIGHT), cv::Scalar(30, 30, 30, 1), 1);
        }
        
        //Display long./lat. edge values in NW/SE map corners
        float latMin = START_X - LATSPAN/2;
        float latMax = START_X + LATSPAN/2;
        float longMin = START_Y - LONGSPAN/2;
        float longMax = START_Y + LONGSPAN/2;
        std::string nwStr = "NW edge: Long. " + std::to_string(longMax) + ", Lat. " + std::to_string(latMin);
        std::string seStr = "SE edge: Long. " + std::to_string(longMin) + ", Lat. " + std::to_string(latMax);
        cv::putText(imgGrid, nwStr, cv::Point(5,10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0, 1), 1); 
        cv::putText(imgGrid, seStr, cv::Point(WIDTH-290, HEIGHT-5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0, 1), 1); 
        
        //Create blank canvas for plotting drone explored area
        cv::Mat imgExplored(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0)); //cumulating the results
        
        //Main loop
        while (od4.isRunning()) {
          cv::Mat imgPositions(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0)); //updating the results
          
          //Draw boat position
          cv::Point pixelPos = getpixelPosition(boatFrame.x(), boatFrame.y(), START_X, START_Y, WIDTH, HEIGHT);
          std::cout<<"Boat poistion on the map"<<pixelPos.x<<","<<pixelPos.y<<std::endl;
          cv::circle(imgPositions, pixelPos, 8, cv::Scalar(255, 0, 0, 1), -1);  
          
          //Draw drone positions and colorize explored area
          for (uint32_t iDrone = 0; iDrone < NUM_DRONES; iDrone++) {
            pixelPos = getpixelPosition(droneFrames[iDrone].x(), droneFrames[iDrone].y(), START_X, START_Y, WIDTH, HEIGHT);
            std::cout<<"Drone Position on the map"<<pixelPos.x<<","<<pixelPos.y<<std::endl;
            //cv::circle(imgExplored, pixelPos, 10, cv::Scalar(100,100,100,1), -1); 
            cv::circle(imgPositions, pixelPos, 3, cv::Scalar(255,255,255,1), -1);
            cv::putText(imgPositions, std::to_string(iDrone), cv::Point(pixelPos.x+3, pixelPos.y+3), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 0, 255, 1), 2); //Display drone number
          }
                  
          //Combine into single map and display on screen
          cv::Mat imgMap(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
          imgExplored.copyTo(imgMap, imgExplored);
          imgGrid.copyTo(imgMap, imgGrid);
          imgPositions.copyTo(imgMap, imgPositions);
          cv::imshow("Map", imgMap);
          cv::waitKey(1);
          
          std::this_thread::sleep_for(std::chrono::milliseconds(1000/FREQ));
        }     
        retCode = 0;
    }
    return retCode;
}
