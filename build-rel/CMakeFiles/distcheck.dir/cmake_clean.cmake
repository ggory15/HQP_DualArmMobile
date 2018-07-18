file(REMOVE_RECURSE
  "doc/HQP.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
  "CMakeFiles/distcheck"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/distcheck.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
