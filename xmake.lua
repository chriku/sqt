add_rules("mode.debug", "mode.release")
set_policy("package.requires_lock", true)
add_requires("glm", "doctest", "zlib", "etl", "nlohmann_json", "asio", "restinio")
-- add_requires("glslang", {configs = {binaryonly = true}})
add_requires("protobuf-cpp")
set_toolchains("clang", "llvm")
target("sqt")
do
  -- add_rules("utils.glsl2spv")
  -- add_files("src/x.comp.glsl")
  add_files("src/*.cpp")
  add_packages("glm", "doctest", "zlib", "etl", "nlohmann_json", "asio", "restinio")
  -- add_packages("glslang")
  set_languages("c++latest")
  add_defines("GLM_ENABLE_EXPERIMENTAL")
  add_defines("ETL_VERBOSE_ERRORS")
  if is_mode("debug") then
    add_defines("ETL_THROW_EXCEPTIONS", "ETL_CHECK_PUSH_POP")
  end
  add_packages("protobuf-cpp", {public = true})
  add_rules("protobuf.cpp")
  add_files("fileformat.proto", "osmformat.proto", {proto_public = true})
  add_ldflags("-fuse-ld=lld")
  if is_mode("debug") then
    set_optimize("fast")
    set_optimize("fastest")
    add_cxflags("-march=native")
    add_ldflags("-march=native")
    add_cxflags("-fstack-protector-all", {force = true})
    add_ldflags("-fstack-protector-all", {force = true})
  else
    set_optimize("fastest")
    add_cxflags("-march=native")
    add_ldflags("-march=native")
    add_defines("NDEBUG")
  end
end
