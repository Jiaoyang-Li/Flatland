# Frequently Asked Questions

A non-exhaustive list of Frequently Asked Questions for the [Flatland Challenge](https://www.aicrowd.com/challenges/flatland-challenge).

### How do I locally build a docker image out of my submission ?

* Install Dependencies
- **docker** : By following the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
- **aicrowd-repo2docker**

```sh
pip install aicrowd-repo2docker
```

* Build Image
```
sudo aicrowd-repo2docker --no-run \
  --user-id 1001 \
  --user-name aicrowd \
  --image-name my-random-agent \
  --debug .
```

### Debugging the packaged software environment

If you have issues with your submission because of your software environment and dependencies, you can debug them, by first building the docker image, and then getting a shell inside the image by :

```
# After ensuring that you have build the docker image by following the 
# instructions here : https://github.com/AIcrowd/flatland-challenge-starter-kit/blob/master/FAQ.md#how-do-i-locally-build-a-docker-image-out-of-my-submission-

docker run --net=host -it my-random-agent /bin/bash
```

and then exploring to find the cause of the issue.

# Author
Sharada Mohanty <https://twitter.com/MeMohanty>