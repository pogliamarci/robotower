FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/Echoes/msg"
  "src/Echoes/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/Echoes/msg/__init__.py"
  "src/Echoes/msg/_Sonar.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
