FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/Echoes/msg"
  "src/Echoes/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/Echoes/srv/__init__.py"
  "src/Echoes/srv/_Led.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
