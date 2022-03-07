FROM ubuntu:20.04 as builder
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    cmake \
    software-properties-common \
    libopencv-apps-dev

RUN add-apt-repository -y ppa:chrberger/libcluon
RUN apt-get update
RUN apt-get install -y libcluon

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install


FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get install -y \	
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libopencv-core4.2 \
    libopencv-imgproc4.2 \
    libopencv-video4.2 \
    libopencv-calib3d4.2 \
    libopencv-features2d4.2 \
    libopencv-objdetect4.2 \
    libopencv-highgui4.2 \
    libopencv-videoio4.2 \
    libopencv-flann4.2 

WORKDIR /usr/bin
COPY --from=builder /tmp/dest /usr
ENTRYPOINT ["/usr/bin/opendlv-drone-visualisation"]
