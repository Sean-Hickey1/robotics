FROM tuwrobotics/mr2024-base
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO=jazzy


# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN mkdir /home/$USERNAME/projects \
    && chown -R  $USERNAME:$USERNAME /home/$USERNAME/projects 
RUN apt-get update && apt-get upgrade -y
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
# enable mouse in tmux
RUN echo 'set -g mouse on' >> ~/.tmux.conf
# defines your projects base
RUN echo 'export PROJECTS_DIR=${HOME}/projects' >> ~/.bashrc 
# forces your bash to source env.sh if it exists
RUN echo "if [ -f "env.sh" ]; then source env.sh; fi" >> ~/.bashrc 
# change bash colour in container
RUN sed -i 's|\\\[\\033\[01;32m\\\]\\u@\\h|\\\[\\033\[01;43m\\\]\\u@\\h|g' ~/.bashrc 
CMD ["/bin/bash"]
