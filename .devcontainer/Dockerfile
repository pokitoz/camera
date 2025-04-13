ARG DOCKER_TAG=latest
FROM espressif/idf:${DOCKER_TAG}


# From https://github.com/micro-ROS/micro_ros_espidf_component/blob/jazzy/docker/Dockerfile

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

ENV IDF_PATH=/opt/esp/idf/
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update -q && \
    apt install -yq lsb-release gosu nano && \
    rm -rf /var/lib/apt/lists/*

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

COPY ./install_micro_ros_deps_script.sh /install_micro_ros_deps_script.sh

RUN mkdir -p /tmp/install_micro_ros_deps_script && \
    mv /install_micro_ros_deps_script.sh /tmp/install_micro_ros_deps_script/ && \
    IDF_EXPORT_QUIET=1 /tmp/install_micro_ros_deps_script/install_micro_ros_deps_script.sh && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update -y && apt-get install udev -y

ARG USER_ID=espidf
RUN apt-get update && apt-get install sudo
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USER_ID

RUN echo "source $IDF_PATH/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]