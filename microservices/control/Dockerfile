FROM alpine:3.7 as builder
MAINTAINER Yue Kang yuek@chalmers.se
RUN apk update && \
    apk --no-cache add \
        ca-certificates \
        cmake \
        g++ \
        make
ADD . /opt/sources
WORKDIR /opt/sources
RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && \
    make example && cp example /tmp

# Deploy.
FROM alpine:3.7
MAINTAINER Yue Kang yuek@chalmers.se
RUN apk update && \
    apk --no-cache add \
        libstdc++ libgcc && \
    mkdir /opt
WORKDIR /opt
COPY --from=builder /tmp/example .
CMD ["/opt/example --cid=112"]
