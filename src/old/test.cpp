
#include <sys/stat.h>
#include <string>
#include <stdio.h>


	inline bool file_exists (const std::string& name) {
		struct stat buffer;   
		return (stat (name.c_str(), &buffer) == 0);
	}
	
	
	int main(int argc, char** argv){
		printf("File alal exists? %d\n", file_exists("./old/old_homog.cpp") );
	}
