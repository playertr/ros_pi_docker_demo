# define global arguments accessible to all stages
ARG ROSDISTRO=noetic
ARG WORK_DIR=/ros_ws

# build stage #########################################
# start from ros-base, which already has several build dependencies
FROM ros:${ROSDISTRO}-ros-base as build
ARG ROSDISTRO
ARG WORK_DIR

WORKDIR ${WORK_DIR}

# install catkin, removing apt list afterwards
RUN apt-get update && apt-get install -y \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

# copy our source code into /ros_ws/src/talker within the container
RUN mkdir src
COPY talker/ src/talker/

# build the code -- need to source ROS within this RUN shell
RUN . /opt/ros/${ROSDISTRO}/setup.sh && \
    catkin build

# run stage ###########################################
FROM ros:${ROSDISTRO}-ros-core
ARG ROSDISTRO
ARG WORK_DIR
ENV WORK_DIR=${WORK_DIR}
ENV ROSDISTRO=${ROSDISTRO}

# copy the compiled code from build stage into this new "run" container
COPY --from=build ${WORK_DIR}/ ${WORK_DIR}/

# add an entrypoint that gets executed before our command,
# to source the appropriate ROS underlay and workspace overlay
ADD docker/entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh
ENTRYPOINT [ "./entrypoint.sh" ]