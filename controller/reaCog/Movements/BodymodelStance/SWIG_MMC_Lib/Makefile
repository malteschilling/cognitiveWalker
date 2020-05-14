# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)
# The c++ compiler that will be used to compile the source files.
CXX=clang++

# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -Wno-unused-parameter additionally 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -g produces debugging information in the operating system's native format.
CXXFLAGS=-std=c++11 -stdlib=libc++ -DNDEBUG -fPIC -O3

# Preprocessor flags
# For Mac OS X you might have to change this towards your version and location of python3.
CPPFLAGS=-I/Library/Frameworks/Python.framework/Versions/3.6/include/python3.6m
SPECIALFLAGS=-undefined suppress -flat_namespace

ifneq ($(UNAME), Darwin)
# The c++ compiler that will be used to compile the source files.
CXX=g++

# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -Wno-unused-parameter additionally 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -g produces debugging information in the operating system's native format.
CXXFLAGS=-std=c++11 -DNDEBUG -fPIC -O3

# Preprocessor flags
CPPFLAGS=-I/usr/include/python3.5m
SPECIALFLAGS=
endif

all: body leg

body:
	swig -python -py3 -c++ mmcBody.i
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c mmcBody_wrap.cxx 
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c mmcBody.cxx
	$(CXX) $(CXXFLAGS) $(SPECIALFLAGS) -shared -o _mmcBody.so mmcBody.o mmcBody_wrap.o -L/usr/local/lib/

leg:
	swig -python -py3 -c++ mmcLeg.i
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c mmcLeg_wrap.cxx
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c mmcLeg.cxx 
	$(CXX) $(CXXFLAGS) $(SPECIALFLAGS) -shared -o _mmcLeg.so mmcLeg.o mmcLeg_wrap.o

all: body leg

clean:
	rm -f *.o
	rm -f *.so
