FROM python:3.13.0

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=dev-user
ARG GROUPNAME=dev-user
ARG UID=1000
ARG GID=1000
ARG WORKDIR=/home/dev-user/workspace

RUN groupadd -g $GID $GROUPNAME && \
    useradd -m -s /bin/bash -u $UID -g $GID $USERNAME

RUN apt update && apt install -y \
git \
vim \
wget \
sudo \
graphviz \
graphviz-dev \
&& apt clean \
&& rm -rf /var/lib/apt/lists/*

RUN mkdir -p $WORKDIR
RUN chown -R $UID:$GID $WORKDIR

USER $USERNAME
WORKDIR $WORKDIR

RUN python -m pip install --upgrade --user pip
RUN python -m pip install --upgrade --user setuptools

COPY requirements.txt $WORKDIR
RUN python -m pip install --user -r requirements.txt