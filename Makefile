Sources=main.cpp Agent.cpp Includes.cpp Dubins.cpp
Target = dubins 

# general compiler settings
CXXFLAGS = -O3 -ffast-math 

all:
	$(CXX) $(CXXFLAGS) $(Sources) -o $(Target) $(LDFLAGS)

clean:
	@$(RM) $(Target)

.PHONY: clean
