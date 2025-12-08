# WORKFLOW

## DEV BRANCH
### SVILUPPO
* Pullare le modifiche
* Buildare l'immagine (nel caso in cui la base image sia cambiata)
* Runnare l'immagine (montando i volumi necessari)

### RILASCIO
* Buildare l'immagine (in modo tale da copiare /app nell'immagine)
* Pushare l'immagine su dockerhub con l'apposito tag

### COMANDI
* build image: docker build -t corra09/nav2_docker:dev .
* run image (with volumes): docker run -it --privileged --net=host   --env="DISPLAY=$DISPLAY"   --env="QT_X11_NO_MITSHM=1"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --volume="./app/nav2_ws/src:/root/nav2_ws/src" --volume="./app/initialize:/root/initialize" --device /dev/dri:/dev/dri  corra09/nav2_docker:dev
* push image: docker push corra09/nav2_docker:dev

## MASTER BRANCH
### SVILUPPO
* Pullare le modifiche
* Buildare l'immagine (nel caso in cui la base image sia cambiata)
* Runnare l'immagine (montando i volumi necessari)

### RILASCIO
* Buildare l'immagine (in modo tale da copiare /app nell'immagine)
* Pushare l'immagine su dockerhub con l'apposito tag

### COMANDI
* build image: docker build -t corra09/nav2_docker:master .
* run image (with volumes): docker run -it --privileged --net=host   --env="DISPLAY=$DISPLAY"   --env="QT_X11_NO_MITSHM=1"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --volume="./app/nav2_ws/src:/root/nav2_ws/src" --volume="./app/initialize:/root/initialize" --device /dev/dri:/dev/dri  corra09/nav2_docker:master
* push image: docker push corra09/nav2_docker:master



