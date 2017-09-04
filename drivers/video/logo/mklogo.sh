#!/bin/bash
rm logo_linux_clut224.o
pngtopnm logo_linux_clut224.png > logo_linux_clut224.pnm
pnmquant 224 logo_linux_clut224.pnm > logo_linux_clut224_tmp.pnm
pnmtoplainpnm logo_linux_clut224_tmp.pnm > logo_linux_clut224.ppm
