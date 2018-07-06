# Docker catkin packager

This image can be used to generate a single binary `deb` archive from the contents of a catkin workspace.

The image expects the source directory to be mounted onto `/root/catkin_ws/src` in the container, and the generated `deb` will be written onto `/debout`.

Before building, the included script will fetch the contents of the `.rosinstall` file in the root of the workspace if there is one.

The process can be configured by setting the following environment variables in the container:

- `CUSTOM_DISTRO_NAME` (defaults to `kinetic-custom`): is the name of the _custom distro_ into which the products will be installed (e.g. by default insalls into `/opt/ros/kinetic-custom`)
- `PACKAGE_NAME` (defaults to `CUSTOM_DISTRO_NAME`): the name of the package to be genrated
- `VERSION_NUMBER` (defaults to `1.0.0`): the version number of the package to be generated
- `SSH_PRIVATE_KEY`: optional private SSH key if required to fetch private repositories in `.rosinstall`
- `SSH_SERVER_HOSTKEYS`: optional host keys of the server hosting the private repositories in `.rosinstall` (can be obtained with `ssh-keyscan SERVER.URL`)

If successful, the generated file will be written to `/debout/${PACKAGE_NAME}_${VERSION_NUMBER}_amd64.deb` inside the container.
Mount the desired folder in your host into `/debout` to be able to retrieve the file.

## Example

The following command will generate a `deb` archive from the packages in `/home/user/my_catkin_ws/src` and put the result in `/home/user/my_catkin_ws/deb`.

```
docker run --rm -e SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" \
                -e SSH_SERVER_HOSTKEYS="$(ssh-keyscan git.server.com)" \
                -e CUSTOM_DISTRO_NAME="cool-distro" \
                -e PACKAGE_NAME="cool-package" \
                -e VERSION_NUMBER="0.0.1" \
                -v /home/user/my_catkin_ws/src:/root/catkin_ws/src:ro \
                -v /home/user/my_catkin_ws/deb:/debout \
                tecnaliarobotics/docker-catkin-packager:kinetic
```
