cmake_minimum_required(VERSION 2.8.3)

find_package(catkin)
project(checker)

if(catkin_FOUND)
  catkin_package()
endif()



include_directories(include)

#cerca i package Bison e Flex
find_package(BISON)
find_package(FLEX)

#genera i parser .cpp dai sorgenti flex e Bison
BISON_TARGET(PredParser  ${PROJECT_SOURCE_DIR}/src/predgram.y 
			 ${PROJECT_SOURCE_DIR}/src/predgram.tab.cpp 
			 COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/predgram.tab.h" 
			 COMPILE_FLAGS "-p pred")
FLEX_TARGET(PredScanner  ${PROJECT_SOURCE_DIR}/src/predflex.l 
			 ${PROJECT_SOURCE_DIR}/src/predflex.cpp 
			 COMPILE_FLAGS "-Ppred")
BISON_TARGET(RulesParser ${PROJECT_SOURCE_DIR}/src/rulesgram.y 
			 ${PROJECT_SOURCE_DIR}/src/rulesgram.tab.cpp 
			 COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/rulesgram.tab.h"  
			 COMPILE_FLAGS "-p rules")
FLEX_TARGET(RulesScanner ${PROJECT_SOURCE_DIR}/src/rulesflex.l 
			 ${PROJECT_SOURCE_DIR}/src/rulesflex.cpp 
			 COMPILE_FLAGS "-Prules")
BISON_TARGET(ShapeParser ${PROJECT_SOURCE_DIR}/src/shapegram.y 
			 ${PROJECT_SOURCE_DIR}/src/shapegram.tab.cpp 
			 COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/shapegram.tab.h" 
			 COMPILE_FLAGS "-p shape")
FLEX_TARGET(ShapeScanner ${PROJECT_SOURCE_DIR}/src/shapeflex.l 
			 ${PROJECT_SOURCE_DIR}/src/shapeflex.cpp 
			 COMPILE_FLAGS "-Pshape")
BISON_TARGET(AssocParser ${PROJECT_SOURCE_DIR}/src/assgram.y 
			 ${PROJECT_SOURCE_DIR}/src/assgram.tab.cpp 
			 COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/assgram.tab.h" 
			 COMPILE_FLAGS "-p ass")
FLEX_TARGET(AssocScanner ${PROJECT_SOURCE_DIR}/src/assflex.l 
			 ${PROJECT_SOURCE_DIR}/src/assflex.cpp 
			 COMPILE_FLAGS "-Pass")

#aggiunge le dipendenze tra parser e scanner (.l e .y)
ADD_FLEX_BISON_DEPENDENCY(PredScanner PredParser)
ADD_FLEX_BISON_DEPENDENCY(RulesScanner RulesParser)
ADD_FLEX_BISON_DEPENDENCY(ShapeScanner ShapeParser)
ADD_FLEX_BISON_DEPENDENCY(AssocScanner AssocParser)

#common commands for building c++ executables and libraries
add_executable( ${PROJECT_NAME}
		src/check.cpp 
		${BISON_PredParser_OUTPUTS}  
		${FLEX_PredScanner_OUTPUTS}
		${BISON_RulesParser_OUTPUTS}  
		${FLEX_RulesScanner_OUTPUTS}
		${BISON_ShapeParser_OUTPUTS}  
		${FLEX_ShapeScanner_OUTPUTS}
		${BISON_AssocParser_OUTPUTS}  
		${FLEX_AssocScanner_OUTPUTS})