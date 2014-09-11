cmake_minimum_required(VERSION 2.8.3)

find_package(catkin)
project(mr_brian)

if(catkin_FOUND)
catkin_package(INCLUDE_DIRS fuzzy/include brian/include
	       LIBRARIES fuzzy brian)
endif()

include_directories(fuzzy/include
                    brian/include)


#search for Bison and Flex packages
find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

#set the folder of lib_fuzzy and lib_tree_classifier
set(LIB_FUZZY_SOURCE_DIR ${PROJECT_SOURCE_DIR}/fuzzy/src)
set(LIB_BRIAN_SOURCE_DIR ${PROJECT_SOURCE_DIR}/brian/src)


#generate parsers and lexers from flex and Bison sources
#for fuzzy library
BISON_TARGET(ShapeParser ${LIB_FUZZY_SOURCE_DIR}/shapegram.y 
                         ${LIB_FUZZY_SOURCE_DIR}/shapegram.tab.cpp 
                         COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/fuzzy/include/shapegram.tab.h" 
                         COMPILE_FLAGS "-p shape")
FLEX_TARGET(ShapeScanner ${LIB_FUZZY_SOURCE_DIR}/shapeflex.l 
                         ${LIB_FUZZY_SOURCE_DIR}/shapeflex.cpp 
                         COMPILE_FLAGS "-Pshape")
BISON_TARGET(AssocParser ${LIB_FUZZY_SOURCE_DIR}/assocgram.y 
                         ${LIB_FUZZY_SOURCE_DIR}/assocgram.tab.cpp 
                         COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/fuzzy/include/assocgram.tab.h" 
                         COMPILE_FLAGS "-p assoc")
FLEX_TARGET(AssocScanner ${LIB_FUZZY_SOURCE_DIR}/assocflex.l 
                         ${LIB_FUZZY_SOURCE_DIR}/assocflex.cpp 
                         COMPILE_FLAGS "-Passoc")
#for brian library
BISON_TARGET(PredParser ${LIB_BRIAN_SOURCE_DIR}/predgram.y 
                        ${LIB_BRIAN_SOURCE_DIR}/predgram.tab.cpp 
                        COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/brian/include/predgram.tab.h" 
                        COMPILE_FLAGS "-p pred")
FLEX_TARGET(PredScanner ${LIB_BRIAN_SOURCE_DIR}/predflex.l 
                        ${LIB_BRIAN_SOURCE_DIR}/predflex.cpp 
                        COMPILE_FLAGS "-Ppred")
BISON_TARGET(RulesParser ${LIB_BRIAN_SOURCE_DIR}/rulesgram.y 
                         ${LIB_BRIAN_SOURCE_DIR}/rulesgram.tab.cpp 
                         COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/brian/include/rulesgram.tab.h" 
                         COMPILE_FLAGS "-p rules")
FLEX_TARGET(RulesScanner ${LIB_BRIAN_SOURCE_DIR}/rulesflex.l  
                         ${LIB_BRIAN_SOURCE_DIR}/rulesflex.cpp 
                         COMPILE_FLAGS "-Prules")

#add dependencies between parser and scanner (.l e .y)
#for fuzzy
ADD_FLEX_BISON_DEPENDENCY(ShapeScanner ShapeParser)
ADD_FLEX_BISON_DEPENDENCY(AssocScanner AssocParser)
#for brian
ADD_FLEX_BISON_DEPENDENCY(PredScanner PredParser)
ADD_FLEX_BISON_DEPENDENCY(RulesScanner RulesParser)


#build the libraries
add_library(fuzzy STATIC
			${LIB_FUZZY_SOURCE_DIR}/action.cpp
			${LIB_FUZZY_SOURCE_DIR}/action_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/action_list_debug.cpp
			${LIB_FUZZY_SOURCE_DIR}/assoc_file_parser.cpp
			${LIB_FUZZY_SOURCE_DIR}/association.cpp
			${LIB_FUZZY_SOURCE_DIR}/association_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/association_set_multimap.cpp
			${LIB_FUZZY_SOURCE_DIR}/barycentre_defuzzyfier.cpp
			${LIB_FUZZY_SOURCE_DIR}/command.cpp
			${LIB_FUZZY_SOURCE_DIR}/command_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/command_list_debug.cpp
			${LIB_FUZZY_SOURCE_DIR}/crisp_data.cpp
			${LIB_FUZZY_SOURCE_DIR}/crisp_data_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/crisp_data_list_debug.cpp
			${LIB_FUZZY_SOURCE_DIR}/data.cpp
			${LIB_FUZZY_SOURCE_DIR}/defuzzyfier.cpp
			${LIB_FUZZY_SOURCE_DIR}/div_triangle.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzy_crisp_rel.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzy_data.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzy_data_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzy_data_list_debug.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzyfier.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzy_set.cpp
			${LIB_FUZZY_SOURCE_DIR}/fuzzy_set_multimap.cpp
			${LIB_FUZZY_SOURCE_DIR}/getFuzzy.cpp
			${LIB_FUZZY_SOURCE_DIR}/point.cpp
			${LIB_FUZZY_SOURCE_DIR}/point_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/point_multimap.cpp
			${LIB_FUZZY_SOURCE_DIR}/rectangle.cpp
			${LIB_FUZZY_SOURCE_DIR}/shape.cpp
			${LIB_FUZZY_SOURCE_DIR}/shape_file_parser.cpp
			${LIB_FUZZY_SOURCE_DIR}/shape_multimap.cpp
			${LIB_FUZZY_SOURCE_DIR}/shape_point.cpp
			${LIB_FUZZY_SOURCE_DIR}/shapes_list.cpp
			${LIB_FUZZY_SOURCE_DIR}/singleton.cpp
			${LIB_FUZZY_SOURCE_DIR}/singleton_defuzzyfier.cpp
			${LIB_FUZZY_SOURCE_DIR}/trapezium.cpp
			${LIB_FUZZY_SOURCE_DIR}/triangle.cpp
			${LIB_FUZZY_SOURCE_DIR}/triangle_ol.cpp
			${LIB_FUZZY_SOURCE_DIR}/triangle_or.cpp
			${BISON_ShapeParser_OUTPUTS}  
                        ${FLEX_ShapeScanner_OUTPUTS}
                        ${BISON_AssocParser_OUTPUTS}  
                        ${FLEX_AssocScanner_OUTPUTS}
			)
			
add_library(brian STATIC
			${LIB_BRIAN_SOURCE_DIR}/brian.cpp 
			${LIB_BRIAN_SOURCE_DIR}/behavior_eng.cpp 
			${LIB_BRIAN_SOURCE_DIR}/can_doer.cpp 
			${LIB_BRIAN_SOURCE_DIR}/compose.cpp 	
			${LIB_BRIAN_SOURCE_DIR}/preacher.cpp 
			${LIB_BRIAN_SOURCE_DIR}/predicate.cpp 
			${LIB_BRIAN_SOURCE_DIR}/predicate_list.cpp 
			${LIB_BRIAN_SOURCE_DIR}/predicate_list_debug.cpp
			${LIB_BRIAN_SOURCE_DIR}/proposed_action_list.cpp
			${LIB_BRIAN_SOURCE_DIR}/proposed_action_list_debug.cpp
			${LIB_BRIAN_SOURCE_DIR}/wanter_eng.cpp
			${LIB_BRIAN_SOURCE_DIR}/weight_want.cpp
			${LIB_BRIAN_SOURCE_DIR}/weight_want_list.cpp
			${LIB_BRIAN_SOURCE_DIR}/weight_want_list_debug.cpp
			${LIB_BRIAN_SOURCE_DIR}/engine_objects.cpp 
			${LIB_BRIAN_SOURCE_DIR}/rules_behav.cpp
			${FLEX_PredScanner_OUTPUTS}
			${BISON_PredParser_OUTPUTS}  
			${BISON_RulesParser_OUTPUTS}
		 	${FLEX_RulesScanner_OUTPUTS}
			${FLEX_RulesScanner_OUTPUTS}
           )

target_link_libraries(brian fuzzy)


if(catkin_FOUND)
install(TARGETS brian fuzzy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()