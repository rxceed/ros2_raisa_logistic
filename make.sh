#!/bin/bash

colcon build --executor parallel --parallel $(nproc)