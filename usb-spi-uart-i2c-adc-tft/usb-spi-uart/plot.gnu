set terminal png
set output ofilename
plot for[col=2:4] filename using 1:col title columnheader(col) with lines
