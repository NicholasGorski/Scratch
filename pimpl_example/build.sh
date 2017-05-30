#!/bin/bash -e

g++ -std=c++14 -Wall -c foo.cpp -o foo.o
g++ -std=c++14 -Wall -c main.cpp -o main.o
g++ -std=c++14 -Wall foo.o main.o -o pimpl
