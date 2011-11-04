GCC = gcc -Wall -g -std=gnu99 -D_REENTRANT -fPIC -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -Wno-unused-parameter -Wno-format-zero-length

JNI_INCLUDES = -I/usr/lib/jvm/java-6-sun/include/ -I/usr/lib/jvm/java-6-sun/include/linux -I/usr/lib/jvm/java-6-openjdk/include/
SHARED_EXT = so
LIBJGL_COMMAND = ld
SHARED_FLAG = --shared
LD_FLAGS = -lX11 -lGL

###############
# OS X Specific
###############
ifeq "$(shell uname)" "Darwin"
# The --shared option isn't supported on OS X and causes all kinds of havoc. A couple other OS X no-ops were removed.
GCC = gcc -Wall -g -std=gnu99 -fPIC -Wno-unused-parameter -Wno-format-zero-length
# Different location of Java headers. Also, <GL/gl(x).h> need to be included to override the non-X11 OpenGL
JNI_INCLUDES = -I/System/Library/Frameworks/JavaVM.framework/Headers/ -I/usr/X11/include/
# OS X understands jnilib or dylib for JNI libraries.
SHARED_EXT = jnilib
# gcc without the -c still performs linking. ld on OS X with JNI projects causes trouble.
LIBJGL_COMMAND = gcc
# -shared flag not supported on OS X.
SHARED_FLAG = -dynamiclib
# Classic OS X compilation flags. The X11 library search path needs to be included for the -lX11 option.
LD_FLAGS = -lX11 -lGL -framework OpenGL -framework JavaVM -L/usr/X11/lib/
endif


all:    gltest libjgl.$(SHARED_EXT)

#############################################################
LIBJGL_OBJS = glcontext.o glcontext-x11.o gl_jni.o lphash.o parray.o
libjgl.$(SHARED_EXT): april_vis_GL.h $(LIBJGL_OBJS)
	$(LIBJGL_COMMAND) $(SHARED_FLAG) $(LIBJGL_OBJS) -o libjgl.$(SHARED_EXT) $(LD_FLAGS)

#############################################################
GLTEST_OBJS = glcontext.o glcontext-x11.o gltest.o libjgl.$(SHARED_EXT)
gltest: $(GLTEST_OBJS)
	$(GCC) -o $@ $(GLTEST_OBJS) $(LD_FLAGS)

#############################################################
%.o: %.c
	$(GCC) -O2 -c -fno-omit-frame-pointer -fno-stack-protector $< $(JNI_INCLUDES)

april_vis_GL.h: ../src/april/vis/GL.java
	echo "Rebuilding JNI headers. Ensure java file has been recently built."
	javah -classpath ../april.jar  april.vis.GL

clean:
	rm -rf *.o *~ gltest libjgl.$(SHARED_EXT)
