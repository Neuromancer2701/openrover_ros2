FROM ros:rolling

# Distribution of ROS2
ENV ROS_DISTRO=rolling
#ENV ROS_DOMAIN_ID=100

RUN apt update \
  && apt upgrade -y \
  && apt install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      cmake \
      rsync \
      autogen autoconf build-essential cmake graphviz \
      libboost-dev libboost-test-dev libgtest-dev libtool \
      python3-sip-dev doxygen python3-sphinx pkg-config \
      python3-sphinx-rtd-theme \
      neovim \
      ros-rolling-joy-linux \
      tar \
      wget \
      libserial-dev \
  && apt-get clean

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# Change the password 'password' to something more secure
RUN useradd -m user && yes password | passwd user

# Setup scripts
RUN echo ". /opt/ros/$ROS_DISTRO/setup.sh" >> /home/user/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]