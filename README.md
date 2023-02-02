# K4R Data Provider

The **K4R Data Provider** forwards data from the Digital Twin API (dt-api) of the Knowledge4Retail platform, a local files or knowrob (like shelves or products) to ROS.

The K4R data provider was initiated developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

![](doc/images/DFKI_Logo_e_schrift.jpg)

## Motivation
The **K4R Data Provider** allows you to use data from the K4R Platform in ROS-Services. 
You can either use the k4r_data_provider as command line interface (CLI) to get the data from the Services and save as json files or use the provided ROS Services to publish the data to ROS directly (for Product and Shelf)

## Installation
### Command Line Interface
**k4r-data-provider** is written in python, so its CLI can be installed for the current user using pip/pip3:

```bash
# clone and cd into the mapdesc-directory
pip3 install .
```

or for all users system-wide:
```bash
# clone and cd into the mapdesc-directory
sudo pip3 install .
```

### ROS-environment
The ROS environment can be build using catkin.
For certificate authentification with the K4R platform you need to add your certificates to the k4r_data-folder

### Dependencies
To use the ROS-Services you need to install ROS 1 noetic.
To just import/export K4R-data using the CLI you just need 

## Usage
Depending on what you like to do you have different options.

### Usage for the CLI
To download data from the platform and store it for later use you can utilize the CLI. To see the usage for the CLI type `k4r_data_provider -h` after installation.

### Run as part of the K4R Platform
The K4R-data-provider is used provide the state of the environment to the knowledge base of the planning system in the K4R-planning docker container.

## Contributing

Please use the [issue tracker](map_desc/issues) to submit bug reports and feature requests. Please use merge requests as described [here](/CONTRIBUTING.md) to add/adapt functionality. 

## License

K4R-data-provider is distributed under the [MIT license](https://opensource.org/licenses/MIT).

## Maintainer / Authors / Contributers

Andreas Bresser, andreas.bresser@dfki.de

Copyright 2022/2023, DFKI GmbH / Robotics Innovation Center
