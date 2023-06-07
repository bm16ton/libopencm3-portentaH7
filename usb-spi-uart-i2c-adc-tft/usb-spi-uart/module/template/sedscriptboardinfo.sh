#!/bin/bash

cp board-info-template  part4-board-info.txt
sed -i "s/bivar1/$1/g" part4-board-info.txt
sed -i "s/bivar2/$2/g" part4-board-info.txt
sed -i "s/bivar3/$3/g" part4-board-info.txt
sed -i "s/bivar4/$4/g" part4-board-info.txt
