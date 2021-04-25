FROM ubuntu:focal

ENV DEBIAN_FRONTEND noninteractive

LABEL Author="Adam Wojasinski"
LABEL description="Image for building stm32 projects"

RUN apt-get update \
    && apt-get install -y make gcc cmake git wget g++ curl\
    && mkdir -p /home/stm32/temp

# Download and install CMake
RUN curl -L https://github.com/Kitware/CMake/releases/download/v3.19.4/cmake-3.19.4-Linux-x86_64.tar.gz -o /home/stm32/temp/cmake.tar.gz \
    && tar -xzf /home/stm32/temp/cmake.tar.gz -C /opt

# Download and install ARM Toolchain
RUN curl -L https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -o /home/stm32/temp/arm.tar.bz2 \
    && tar -xjf /home/stm32/temp/arm.tar.bz2 -C /opt

# Add to Path
ENV PATH="/opt/cmake-3.19.4-Linux-x86_64/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:${PATH}"

# Check ARM Toolchain version
RUN arm-none-eabi-gcc --version \
    && arm-none-eabi-g++ --version \
    && arm-none-eabi-size --version

# Delete not needed packages and folders
RUN apt-get remove -y wget \
    && rm -rf /home/stm32/temp

# Create working directory for sharing with repository
RUN mkdir /home/stm32/repo
WORKDIR /home/stm32/repo