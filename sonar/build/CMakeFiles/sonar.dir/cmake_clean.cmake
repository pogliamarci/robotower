FILE(REMOVE_RECURSE
  "../src/sonar/msg"
  "../src/sonar/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/sonar.dir/src/sonar.o"
  "CMakeFiles/sonar.dir/src/ReadSonar.o"
  "CMakeFiles/sonar.dir/src/ReadSonarBase.o"
  "CMakeFiles/sonar.dir/src/SerialCommunication.o"
  "CMakeFiles/sonar.dir/src/CharCircularBuffer.o"
  "CMakeFiles/sonar.dir/src/LedParser.o"
  "../bin/sonar.pdb"
  "../bin/sonar"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/sonar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
