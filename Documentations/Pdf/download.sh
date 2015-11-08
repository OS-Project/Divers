#!/bin/bash

for i in $(seq 9)
do
	wget https://static.lwn.net/images/pdf/LDD3/ch0$i.pdf
done;

for i in $(seq 9)
do
	let "n = i + 9"
	wget https://static.lwn.net/images/pdf/LDD3/ch$n.pdf
done;
