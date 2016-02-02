#!/bin/bash
until yarp name --list; do
    sleep 10
done

yarp read /voice | espeak -ves &
