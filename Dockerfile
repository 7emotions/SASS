FROM ros:humble AS base

SHELL [ "/bin/bash", "-c" ]

# Install some tools and libraries.
RUN apt-get update && apt-get -y install \
    vim wget curl unzip \
    build-essential \
    cmake \
    make ninja-build \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

FROM base AS develop

# Install develop tools (ssh/clangd/zsh/etc...)
RUN apt-get update && apt-get -y install \
    openssh-client \
    lsb-release software-properties-common gnupg zsh sudo \
    libcanberra-gtk-module libcanberra-gtk3-module && \
    rm -rf /var/lib/apt/lists/* \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)" && \
    ln -s /usr/bin/clangd-* /usr/bin/clangd

# Add user
RUN useradd -m developer --shell /bin/zsh && echo "developer:developer" | chpasswd && adduser developer sudo && \
    echo "developer ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add developer dialout
WORKDIR /home/developer
ENV USER=developer
ENV WORKDIR=/home/developer

USER developer

# Install oh my zsh & change theme to af-magic
RUN set -eo pipefail && \
    curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' .zshrc


ENTRYPOINT ["/bin/zsh"]