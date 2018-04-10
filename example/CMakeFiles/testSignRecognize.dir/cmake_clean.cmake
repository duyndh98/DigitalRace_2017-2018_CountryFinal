file(REMOVE_RECURSE
  "bin/Release/testSignRecognize.pdb"
  "bin/Release/testSignRecognize"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/testSignRecognize.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
