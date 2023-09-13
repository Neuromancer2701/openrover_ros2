FROM ros:rolling

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      cmake \
      rsync \
      neovim \
      ros-rolling-joy-linux \
      tar \
      wget \
      git autogen autoconf build-essential cmake graphviz \
      libboost-dev libboost-test-dev libgtest-dev libtool \
      python3-sip-dev doxygen python3-sphinx pkg-config \
      python3-sphinx-rtd-theme \
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

CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]
