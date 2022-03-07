# OpenDLV microservice for visualising drone positions
Displays the drone positions on a map. The big blue circle represents the marine platform and the small white circles represents the flying drones together with an index number for each of them. The greyed out area shows where the drone has explored (currently just set as a set pixel radius around the drone for demonstration purposes rather than actual area based on real coordinates). 

## Running the microservice
If not done already, allow GUI access with the command `xhost +`

Run together with the opendlv-data-simulation microservice using the yml-file: `docker-compose -f run-simulation.yml up`

Or run inidivudally through `docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY opendlv-drone-visualisation --cid=111 --freq=80 --numDrones=3 --startX=20.0 --startY=60.0 --longSpan=0.1 --latSpan=0.1 --width=800 --height=800 --verbose`

Commandline arguments:
`--cid` = Client ID for UDP messages
`--freq` = Update frequency
`--numDrones` = Number of flying drones
`--startX / startY` = The drones' starting position, given in latitude/longitude coordinates respectively (used as map center)
`--latSpan / longSpan` = How many degrees of latitude/longitude the map spans
`--width / height` = The width/height of the image displayed on screen
`--verbose` = Extra output in form of received drone coordinates
