FROM ubuntu:noble

ARG USERNAME
ARG USER_UID
ARG USER_GID=$USER_UID
ARG TZ
ENV TZ=$TZ
ARG LANG
ENV LANG=$LANG

RUN groupadd --gid $USER_GID $USERNAME && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
RUN apt-get update && apt-get install -y sudo
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME

ENV DEBIAN_FRONTEND=noninteractive

# Install locales
RUN apt-get install -qqy --no-install-recommends locales && \
    ln -snf "/usr/share/zoneinfo/$TZ" /etc/localtime && \
    echo "$TZ" >/etc/timezone && \
    echo "en_US.UTF-8 UTF-8" >/etc/locale.gen && \
    locale-gen && \
    update-locale LANG=${LANG} LC_ALL=${LANG} LANGUAGE=

RUN apt-get install -y \
    bash \
    git

USER $USERNAME