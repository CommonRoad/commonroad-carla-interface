FROM carlasim/carla:latest
# Install miniconda3
## Copied from https://hub.docker.cocomm/r/continuumio/miniconda3/dockerfile
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH /opt/conda/bin:$PATH
USER root
RUN apt-get update --fix-missing && \
    apt-get install -y wget bzip2 ca-certificates curl git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    /opt/conda/bin/conda clean -tipsy && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    conda update -n base -c defaults conda

CMD [ "/bin/bash" ]

## the following two lines are required to make tzdata configuration non-interactive
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# Run all commands as cruser
RUN apt-get update && apt-get install -y sudo
RUN useradd --create-home --shell /bin/bash cruser \
    && usermod -aG sudo cruser \
    && echo "cruser ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER cruser
ENV HOME="/home/cruser"
ARG PROFILE="$HOME/.profile"
ARG BASHRC="$HOME/.bashrc"
ENV PYENV_ROOT="$HOME/pyenv"
WORKDIR $HOME

RUN sudo apt-get update \
	&& sudo apt-get -y upgrade \
	&& sudo apt-get -y install git gcc g++ cmake make xdg-utils

SHELL ["/bin/bash", "-c"]

# Install pip
RUN sudo apt-get -y install python3-pip

# activate conda environment
RUN conda create --name cr37 python=3.7
RUN source activate cr37 \
    && echo "Successfully installed Python version:" \
    && python --version \


# update path to include packages installed by conda
ENV PATH $HOME/.local/bin:$PATH
