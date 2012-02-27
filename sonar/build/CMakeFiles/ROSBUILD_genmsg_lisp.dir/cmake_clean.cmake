FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sonar/msg"
  "../src/sonar/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Led.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Led.lisp"
  "../msg_gen/lisp/Sonar.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Sonar.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
