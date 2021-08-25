# My MD5 Stuff

This is a public release of the MD5 stuff from my (unreleased) Quake 1 engine.  The intent is to provide a sample working implementation of MD5 loading and drawing, specifically using the new MD5 replacement models from the Quake 2021 reissue.  This code, as it stands, is not going to be immediately usable by anybody, and you can certainly not just drop-n-go it into any other source port.  Instead it's usable as a reference for what problems I encountered and how I solved them, and for assistance with writing your own implementation.

Much of this was based on the sample code at http://tfc.duke.free.fr/coding/md5-specs-en.html and the Quake 2021 MD5s are fully compatible with that code.  That code also provides an OpenGL 1.1 animation and rendering path, and I'd encourage you to use mine in conjunction with it.

So as to not clutter the code with my inane ramblings, I'm going to put them here instead.  These may be of interest in understanding why certain things are the way that they are.

## License

My original code is GPLv2 licensed, and the original sample base reamins under it's own license.  This should ensure the broadest licensing compatibility for any derivatives.

## Animation

MD5 is a skeletal format, but my code doesn't do skeletal animation.  Instead I pre-bake keyframes from the skeleton at load time, then just do regular keyframe interpolation at draw time.  This was done to allow my MD5 renderer to share 95% of the rendering codepath with MDLs.  Visually it looks just fine to me, but if anyone identifies any bugs with this approach I'd be interested to know.  The base sample at http://tfc.duke.free.fr/coding/md5-specs-en.html does skeletal animation on the CPU into an OpenGL 1.1 vertex array.

## Generating Normals

The source MD5s don't contain normals, so I grabbed some code from modelgen.c (https://github.com/id-Software/Quake-Tools/blob/master/qutils/MODELGEN/MODELGEN.C) to generate them from the position data.  This code should be generally usable for regenerating normals for MDLs too.  The weird bit where I rotate the position before generating the normal, then rotate the normal back afterwards, is in the original modelgen.c code and I don't know why ID did that, but I decided it was best to keep it for consistency.

## Vertex Compression

Because I do keyframes my vertex data can get large, so I compress them down to a 10 bit 0..1 range per component, then scale and offset them back up at draw time.  Using 10 bits per component does cause some precision loss, but it passes my own visual comparison tests.  (Yes, this is essentially the same as how MDLs work, but using 10 bits instead of 8, and with the scale and offset being per-frame rather than global to the model, which gives a few extra bits of precision - this is sufficient that they don't suffer from the same vertex wobble as MDLs.  You're free to use full floats if you wish, of course.)

## Textures

The Quake 2021 MD5 textures are .lmp files and I haven't bothered implementing support for replacement textures in other formats.  

## PAK File Loading

The base sample uses stdio for loading it's MD5s, and in particular depends heavily on feof, which makes it incompatible when used for loading from PAK files.  I changed that to using COM_LoadHunkFile and parsing lines from the resulting text stream (see discussion of COM_ParseLine, below), and that works.

## COM_ParseLine

This is a companion to COM_Parse and just reads a single line out of a text stream.  I wrote it quickly, and bashed about with it until it worked with the source MD5s, but I don't know how robust it is for general-case use.  Parsing plain text always gives me the heebeejeebies and I'd welcome any feedback or bugfixes from the community.
