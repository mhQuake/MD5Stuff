# My MD5 Stuff

This is a sample implementation of MD5 support in Quake using the MD5 models from the Quake 2021 release.

The codebase is FitzQuake 0.85; I lightly modified it to compile in Visual C++ 2008 Express, and run acceptably on a number of test machines, but have otherwise resisted the temptation to change any behaviours other than adding MD5 support.  You should be able to do a diff with the original FitzQuake source and clearly see what was done.  The project should also upconvert (reasonably) clean to more recent versions of Visual C++ or Visual Studio.

The MD5 code was adapted from http://tfc.duke.free.fr/coding/md5-specs-en.html and that remains under it's original license.

Portions of the drawing code were adapted from ID Software's original modelgen.c code at https://github.com/id-Software/Quake-Tools/blob/master/qutils/MODELGEN/MODELGEN.C

