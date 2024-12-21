FROM carlasim/carla:0.9.14

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH /opt/conda/bin:$PATH

USER root

# Install apt dependencies from apt_dependencies.txt
COPY apt_dependencies.txt .
RUN apt-get update --fix-missing && \
    xargs -a apt_dependencies.txt apt-get install -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

## the following two lines are required to make tzdata configuration non-interactive
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update alternatives to use Python 3.8 as the default version
RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo add-apt-repository ppa:deadsnakes/ppa

RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 && \
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 2 && \
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 3

# Run all commands as carla
RUN usermod -aG sudo carla \
    && echo "carla ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER carla

ENV HOME="/home/carla"
ENV PATH="$HOME/.local/bin:$PATH"
WORKDIR $HOME

# Install pip dependencies from pip_dependencies.txt
COPY pip_dependencies.txt .
RUN python3 -m pip install --upgrade pip setuptools && \
    xargs -a pip_dependencies.txt python3 -m pip install

CMD ["/bin/bash"]
