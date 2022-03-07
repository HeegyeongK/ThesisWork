# OpenDLV microservice for visualising drone positions
Sends out drone coordinates using the opendlv standard message set in order to test/showcase the opendlv-drone-visualisation microservice.

## Running the microservice
If not done already, allow GUI access with the command `xhost +`. Then download and run together with the visualisation microservice through standing in the opendlv-drone-visualisation folder and using the yml-file via `docker-compose -f run-simulation.yml up`

In order to change drone behaviour/pathing, make changes to the "Update positions" section of opendlv-data-simulation.cpp. Don't forget to change the `numDrone` commandline argument if adding/removing behaviour patterns.

Commandline arguments:
`--cid` = Client ID for UDP messages
`--freq` = Update frequency
`--numDrones` = Number of flying drones
`--verbose` = Extra output in form of the sent drone coordinates
