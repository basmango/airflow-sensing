# run basmango/waypoints image using docker, mount current directory

sudo docker run --net=host -it --volume="./:/airflow" basmango/waypoints /bin/bash  