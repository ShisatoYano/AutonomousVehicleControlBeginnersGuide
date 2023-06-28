FROM python:3.11.4

ARG DEBIAN_FRONTEND=noninteractive

ARG UID=2000
ARG GID=2000

RUN apt update && apt upgrade
RUN apt install -y \
git \
vim \
wget \
&& apt clean \
&& rm -rf /var/lib/apt/lists/* \
&& groupadd -g $GID dev-user \
&& useradd -m -d /home/dev-user -s /bin/bash -u $UID -g $GID dev-user

RUN mkdir -p /home/dev-user/workspace
WORKDIR /home/dev-user/workspace
COPY ../requirements.txt /home/dev-user/workspace

RUN python -m pip intall --upgrade pip
RUN pip install -r requirements.txt

USER dev-user