# My MD5 Stuff

This is a sample implementation of MD5 support in Quake using the MD5 models from the Quake 2021 release.

The codebase is FitzQuake 0.85; I lightly modified it to compile in Visual C++ 2008 Express, and run acceptably on a number of test machines, but have otherwise resisted the temptation to change any behaviours other than adding MD5 support.  You should be able to do a diff with the original FitzQuake source and clearly see what was done.  The project should also upconvert (reasonably) clean to more recent versions of Visual C++ or Visual Studio.

The MD5 code was adapted from http://tfc.duke.free.fr/coding/md5-specs-en.html and that remains under it's original license.

Portions of the drawing code were adapted from ID Software's original modelgen.c code at https://github.com/id-Software/Quake-Tools/blob/master/qutils/MODELGEN/MODELGEN.C

This implementation is relatively feature-complete so far as the FitzQuake renderer is concerned, so in addition to MD5 model drawing, it also handles them in r_shadows and r_showtris mode.  For reasons of simplicity and clarity I didn't do a multitextured path for fullbrights, but otherwise it should be all there.

The MD5 drawing code uses OpenGL 1.1 calls *only*, so there are no vertex buffers, shaders or other features which you might expect to see in a more modern implementation.  It does however use vertex arrays, but limited to the OpenGL 1.1 interfaces, and it should be easy enough to convert that to glBegin/glEnd code if you wish.

Finally, it should go without saying that this is a techdemo sample implementation, so you shouldn't have expectations around it being usable for general-case Quake play, especially with modern maps.

