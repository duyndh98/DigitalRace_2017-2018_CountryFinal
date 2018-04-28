file(REMOVE_RECURSE
  "bin/Release/test-steering-coef.pdb"
  "bin/Release/test-steering-coef"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/test-steering-coef.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
