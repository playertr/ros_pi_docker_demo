# ros_pi_docker_demo
A demo of using Docker to put ROS on a Raspberry Pi -- while compiling on your desktop!


## 1. Write the ROS node.

I made a simple ROS Noetic [package](talker) with a node that prints the current timestamp. To make the node I ran `catkin create pkg talker` from this repo directory. Then I filled in the auto-generated `package.xml` file and made a `talker` node loosely following the basic [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).

## 2. Set up Docker.

Our example node is simple to compile, but [some](https://github.com/ros-planning/moveit) ROS packages would take a long time to compile on a Raspberry Pi. Therefore, we want to _cross-compile_ the code on a beefy desktop machine and simply run it on the Pi. This is called "cross" compiling because the desktop is producing code with different instructions than its processor can execute: the desktop uses the `x86` instruction set architecture, while the Pi uses `ARM`. We'll use Docker to cross-compile.

Docker has some big benefits: when we produce a Docker "image", we allow people to run our code without installing dependencies by hand using `apt`, `git`, and their ilk. This is convenient, but it's also [reproducible](https://www.thoughtworks.com/en-us/insights/blog/reproducible-work-environments-using-docker): even if dependency versions in remote channels and repositories change, the versions that we bake into our Docker images are there forever. Using Docker is also very scalable: you only need to execute the `docker run` command to turn an image into a running container, which means that deploying code to robots can as simple as placing an image in a folder on the robot.

There are some reasons to [avoid](https://ubuntu.com/blog/ros-docker) using Docker with ROS as well: it is difficult to provide a container with access to privileged resources such as GPIO and networking without increasing your container's attack surface, and Docker alone is not well suited for over-the-air updates in production. However, for this simple example (and many research robotics applications), Docker will do.

Here's how we'll set it up:

### 2.1 Install Docker on the desktop computer.
Follow Docker's [installation instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository), including any post-installation steps. For Ubuntu:
```
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o \
    /usr/share/keyrings/docker-archive-keyring.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker $USER
```

### 2.2 Write a Dockerfile.
The Dockerfile is the recipe for installing your code in a container, and baking that into a reproducible image. I made a Dockerfile in [`docker/Dockerfile`](docker/Dockerfile) -- have a look. I followed several best practices:
* Because each `RUN` command in a Dockerfile [adds another layer to the image](https://medium.com/@gdiener/how-to-build-a-smaller-docker-image-76779e18d48a), I minimized layer size by adding little data as possible with each command. That means removing the apt lists after every `apt update` command.
* To keep the final image size small for deployment on the Pi, I used a multi-stage build, putting the build dependencies into an intermediate container called `build` and keeping only the compiled code in the final `run` container. This brought the image size from 1.2 GB to 0.7 GB. There's a helpful example of this on ROS' official Docker Hub [page](https://hub.docker.com/_/ros#:~:text=Creating%20a%20Dockerfile%20to%20build%20ROS%20packages).

To develop the Dockerfile, I started with [examples](https://github.com/playertr/rosbridge_demo) I had seen before and modified them. My workflow involves adding a layer, building what I have so far, and then dropping into a shell on the container to verify that the layer was successful. It is slow to build a Dockerfile that way, but things are sped up somewhat by Docker's automatic caching of the layers it builds. To build and run containers, I used:

```
docker build -f docker/Dockerfile . -t playertr/talker
docker run -it --net host playertr/talker bash
``` 

## 3. Make an ARM Docker image.
To cross-compile to `ARM`, we change the command to use the `buildx` multi-platform functionality:
```
docker buildx build --platform linux/arm64 -f docker/Dockerfile -t playertr/talker:arm64 --output type=docker .
```

This creates a Docker image called `playertr/talker:arm64` that I can view when I run `docker image ls`, getting an output like:
```
REPOSITORY          TAG         IMAGE ID        CREATED             SIZE
playertr/talker     arm64       4bb5f491d286    8 minutes ago       740MB
```
I can't run this container on my local desktop, because it is meant for an `ARM64` device.

> **NOTE**: It is possible to compile for a 32-bit ARM OS, but I have not been able to do this. Luckily, all recent Pi devices (starting with the Raspberry Pi 3 and Pi Zero 2) support 64-bit operating systems, which must be loaded for this demo to work.
>
> When attempting to compile for a 32-bit OS, you need to ensure your desktop has QEMU binary format emulation installed, and you need to set up your Docker `buildx` container to use this emulation as demonstrated [here](https://stackoverflow.com/questions/65365797/docker-buildx-exec-user-process-caused-exec-format-error). However, many common applications do not support `ARM32` very well; I could not build this demo in `ARM32` because a recently fixed [bug](https://gitlab.kitware.com/cmake/cmake/-/issues/20568) in CMake was still present in the upstream `ros:noetic-ros-base` image used in this demo, causing `catkin build` to fail. The writing was on the wall; no 32-bit ARM for this demo.

## 4. Set up the Pi.
I set up the Raspberry Pi according to time-honored tradition: I used the [Pi Imager](https://www.raspberrypi.com/software/) to flash 64-bit Raspberry Pi OS; I spirited `wpa_supplicant.conf` and `ssh` into the cryptic `/boot` partition for [headless setup](https://www.raspberrypi.com/documentation/computers/remote-access.html#ssh); and I logged in via `ssh`.

Then I installed Docker on the Pi using the [convenience script](https://docs.docker.com/engine/install/debian/#install-using-the-convenience-script) recommended for Raspbian. It ended up looking like:
```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER # log back in after this
```

## 5. Transfer the image onto the Pi.
Remember how we saw our beautiful `playertr/talker:arm64` image when we ran `docker image ls` in Step 3? There are just three steps to see the same output on the Raspberry Pi.

First, turn the Docker image into a `.tar` file. On the desktop:
```
docker save -o talker-arm64.tar playertr/talker:arm64
```

Then, get the file onto the Pi with your favorite file transfer method, such as `rsync`, `ftp`, carrier pigeon USB, or semaphore. We'll use `rsync`. On the desktop:
```
rsync -v --info=progress2 talker-arm64.tar pi@raspberrypi.local:~/
```

With the file on the Pi, load it into Docker. From the Pi terminal:
```
docker load < ~/talker-arm64.tar
```

## 6. Enjoy ROS on your Pi
If "enjoying ROS on your Pi" means "executing the most minimal working example possible", then we're in for the ride of our lives. On the Pi, run:
```
docker run -it --net host playertr/talker:arm64 roslaunch talker talker.launch
```
and observe that sweet, sweet roscore output streaming to `stdout`.

You can also listen to the `talker` from your local computer by setting the ROS networking configuration appropriately. On the Pi: 
```
pi@raspberrypi:~ $ docker run -it --net host --env ROS_MASTER_URI=http://localhost:11311 --env ROS_HOSTNAME=10.0.0.133 playertr/talker:arm64 roslaunch talker talker.launch
```
And on the desktop:
```
tim@tim-UBUNTU:~ $ ROS_MASTER_URI=http://pi@raspberrypi.local:11311 ROS_HOSTNAME=10.0.0.189 rostopic echo /time
```
(I determined my internal IP addresses by parsing them from the commands `ping raspberrypi.local` and `ping tim-UBUNTU.local`)

Congrats! You've successfully:
* Made your code more reproducible with a Dockerfile that anyone can build on almost any computer.
* Created an immutable image with all of your code's dependencies baked in, future-proofing your system.
* Compiled ARM code on your desktop computer, saving your Pi from doing the heavy lifting.
* Gotten your code to work a barebones Pi with only five terminal commands, reducing the per-robot effort to deploy code.

# Extensions
There are some more clever things you can do:
* Push your image to Docker Hub (or deploy your own registry server) so you can simply `docker pull` the images to the Pi, gaining automatic layer caching, versioning, and faster downloads.
* Add a Github CI runner to automatically build your Docker image whenever you push to `main`.
* Use `docker-compose` to manage a community of microservices on the Pi: one for ROS, one for checking for updates, etc.
* Add a `systemd` daemon to start your Docker container as soon as the Pi powers on.
* Bind-mount a configuration file to the container so it can see device-specific configuration such as the preferred hostname.

---
# References
* Headless Pi setup
    * https://www.raspberrypi.com/documentation/computers/remote-access.html#ssh
* Saving Docker image and sending over `ssh`
    * https://stackoverflow.com/questions/23935141/how-to-copy-docker-images-from-one-host-to-another-without-using-a-repository
* Docker `buildx` cross-compilation
    * https://stackoverflow.com/questions/65365797/docker-buildx-exec-user-process-caused-exec-format-error
    * https://collabnix.com/building-arm-based-docker-images-on-docker-desktop-made-possible-using-buildx/
    * https://github.com/docker/buildx/issues/166
* Installing docker on Pi
    * https://docs.docker.com/engine/install/debian/#install-using-the-convenience-script
    * https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user
* Reasons NOT to user Docker with ROS
    * https://ubuntu.com/blog/ros-docker#:~:text=Docker's%20most%20significant%20flaw%20in,to%20host%20resources%20or%20permissions.