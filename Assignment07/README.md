Assignment07
========================================

Team Members
------------

Charles Coulton

Chris Forkner

Niki Silveria

Building This Example
---------------------

To build this example just 

>$ cd build 
>$ make

The excutable will be put in bin

the EXE requires 3 arguments: vertex shader, fragment shader, and solar system file

EXAMPLE:

For solar_system.txt (if in bin directory):

>$ ./Assign7 shadervs.cg shaderfs.cg solar_system.txt

Usage
-----

You can look at the planets with the 0-9 keys (0 is Sun, 1 is Mercury...)

Time scale can be increased with 'r' and decreased with 'e'

You can zoom in with '=' and zoom out with '-'

Other
-----

There are provided models with a material files in bin/textures

Using assimp for model Loading

Using Magick++ for textures

