# Beaglebone Ansible System Configurations

In this directory, we include a set of Ansible configurations for creating a
development environment on a beaglebone to set it up to act as a jetson for testing.

To use one of these Ansible playbooks on Debian release stretch, make sure Ansible is installed:

```sh
$ sudo apt install ansible
```

Then run the playbook
```sh
$ ansible-playbook -K -i "localhost," -c local beaglebone.yml
```
For this script you'll probably need extra variables for assigning the beaglebone's \
ethernet ip (10.3.0.X) so add --extra-vars "<variable definition>"
example
```
$ ansible-playbook -K -i "localhost," -c local beaglebone.yml --extra-vars "num = 2"
```

## Usage
clone embedded-testbench onto your beaglebone \
```git clone https://github.com/umrover/embedded-testbench``` \
checkout out to this branch and run the ansible playbook  \
```cd embedded-testbench``` \
```git checkout beaglebone-ansible-setup``` \
```ansible-playbook -K -i "localhost," -c local beaglebone.yml --extra-vars "num = 2" ```

This should set up you beaglebone to have all the relevant apt packages and network configurations to be on the rover network and function as a stand in jetson for testing. 

clone mrover-workspace onto your beaglebone \ 
```git clone https://github.com/umrover/mrover-workspace``` \
Add any necessary pip requirements to pip_deps/beaglebone_requirements.txt \
Run jarvis: \
```cd ~/mrover-workspace``` \
```./jarvis exec jetson/science-bridge``` \
This will bootstrap jarvis for you and build a common package we've used for testing jetson programs onto your beaglebone. You probably will need to add ```numpy==1.16.2``` to the beaglebone_requirements.txt folder if it fails to build the first time due to numpy issues. 

## Production MRover systems

The configuration in `jetson.yml` in [the mrover-workspace repo](https://github.com/umrover/mrover-workspace/tree/main/ansible) may be used to set up the main on-board
computer on an actual rover. This script makes several assumptions about the
system architecture and is not guaranteed to work outside of the beaglebone black with Debian/stretch installed. This is only for using beaglebones as stand in jetson's/linux devices during code development since we have limited jetson boards. 

`beaglebone.yml` executes roles that configure `systemd` services, `udev` rules,
and environment files for the jetson components. It is intended that running
this playbook will configure the beaglebone-black-series board we are using such that
all necessary processes will run on startup.

No services are necessary to run on a beaglebone however this script sets up the \
```rover-beaglebone-rgb.service``` for testing. 
