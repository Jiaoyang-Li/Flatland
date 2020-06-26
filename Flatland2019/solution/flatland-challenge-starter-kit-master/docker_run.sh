#!/bin/bash


if [ -e environ_secret.sh ]
then
    echo "Note: Gathering environment variables from environ_secret.sh"
    source environ_secret.sh
else
    echo "Note: Gathering environment variables from environ.sh"
    source environ.sh
fi

# Expected Env variables : in environ.sh
sudo docker run \
    --net=host \
    -v ./scratch/test-envs:/flatland_envs:z \
    -it ${IMAGE_NAME}:${IMAGE_TAG} \
    /home/aicrowd/run.sh
