ZSTT ROS workspace
==================

Environment: Ubuntu Linux 16.04LTS 64-bit with 4.13.0-43-generic kernel


Prerequisite
------------

To work correctly, you need to intilize submodules.
For this, run the script in root of workspace:

```console
$ cd ~/zstt-ros
$ ./scripts/git-init-submodules.sh
```

If there are no problems, it shows like this:

```text
TASK [git : Initialize git submodules]
[OK]

TASK [git : Update git submodules]
[OK]
```


Build
-----

Run following command in `zstt-ros` workspace:

```console
$ catkin_make
```

The output directories are `build` and `devel`.


Load Configuration
------------------

```console
$ source devel/setup.bash
```


RUN
---

in progress...


Trouble Shooting
----------------

* RQT Plugin does not listed...
  - Remove rqt caching and retry
  ```shell
  $ rm ~/.config/ros.org/rqt_gui.ini
  $ rqt
  ```
