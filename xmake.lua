add_rules("mode.debug", "mode.release")
add_requires("glm", "doctest")
add_requires("glslang", {configs = {binaryonly = true}})
target("sqt")
do
  add_rules("utils.glsl2spv")
  add_files("src/x.comp.glsl")
  add_files("src/main.cpp")
  add_packages("glm", "doctest")
  add_packages("glslang")
  set_languages("c++latest")
end
