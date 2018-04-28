file(REMOVE_RECURSE
  "bin/Release/vision-test.pdb"
  "bin/Release/vision-test"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/vision-test.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
