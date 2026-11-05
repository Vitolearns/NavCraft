FROM osrf/ros:jazzy-desktop
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions python3-rosdep && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY . /workspace
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y && rm -rf /var/lib/apt/lists/*
RUN bash scripts/build_ws.sh
