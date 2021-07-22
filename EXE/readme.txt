2021/07/22 By Dr Chenlei Lv
Personal Website: https://aliexken.github.io/

How to run the AIVS-based simplification:

Using "run.cmd" to simplified the input point cloud.
Instruction of input parameters:

AIVS_Pure.exe Bunny.obj 10000 Bunny_Sim.obj

Bunny.obj: input point cloud
10000: simplification number
Bunny_Sim.obj: output point cloud

The error information is stored in "errorLog.txt"

If you want to simplify your 3D object, change the input and outout file name. 
(Our project supports different kinds of input format, including off, ply, and obj.)

The source code is stored in AIVS_Pure.