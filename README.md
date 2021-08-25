# MD5Stuff

This is a public release of the MD5 stuff from my (unreleased) Quake 1 engine.  The intent is to provide a sample working implementation of MD5 loading and drawing, specifically using the new MD5 replacement models from the Quake 2021 reissue.  This code, as it stands, is not going to be immediately usable by anybody, and you can certainly not just drop-n-go it into any other source port.  Instead it's usable as a reference for what problems I encountered and how I solved them, and for assistance with writing your own implementation.

Much of this was based on the sample code at http://tfc.duke.free.fr/coding/md5-specs-en.html and the Quake 2021 MD5s are fully compatible with that code.  That code also provides an OpenGL 1.1 animation and rendering path, and I'd encourage you to use mine in conjunction with it.

So as to not clutter the code with my inane ramblings, I'm going to put them here instead.  These may be of interest in understanding why certain things are the way that they are.

## COM_ParseLine

This is a companion to COM_Parse and just reads a single line out of a text stream.  I wrote it quickly, and bashed about with it until it worked with the source MD5s, but I don't know how robust it is for general-case use.  Parsing plain text always gives me the heebeejeebies and I'd welcome any feedback or bugfixes from the community.
