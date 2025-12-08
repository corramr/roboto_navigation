# pull from base image
FROM corra09/nav2_docker:dev


# copy application code
COPY ./app/nav2_ws/src /root/nav2_ws/src
COPY ./app/initialize /root/initialize


# set working directory
WORKDIR /root/nav2_ws

