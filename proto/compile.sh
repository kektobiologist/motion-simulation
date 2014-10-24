#!/bin/sh
rm -rf cpp
mkdir cpp
protoc --cpp_out=cpp *.proto

