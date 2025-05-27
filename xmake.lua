add_rules("mode.debug", "mode.release")
add_requires("glm", "doctest", "zlib", "etl", "nlohmann_json")
add_requires("glslang", {configs = {binaryonly = true}})
add_requires("protobuf-cpp")
set_toolchains("clang", "llvm")
if not is_mode("debug") then
  set_policy("build.optimization.lto", true)
  add_ldflags("-fuse-ld=lld")
end
target("sqt")
do
  add_rules("utils.glsl2spv")
  -- add_files("src/x.comp.glsl")
  add_files("src/*.cpp")
  add_packages("glm", "doctest", "zlib", "etl", "nlohmann_json")
  add_packages("glslang")
  set_languages("c++latest")
  add_defines("GLM_ENABLE_EXPERIMENTAL")
  add_defines("ETL_THROW_EXCEPTIONS", "ETL_VERBOSE_ERRORS")
  if is_mode("debug") then
    add_defines("ETL_CHECK_PUSH_POP")
  end
  add_packages("protobuf-cpp", {public = true})
  add_rules("protobuf.cpp")
  add_files("fileformat.proto", "osmformat.proto", {proto_public = true})
  if is_mode("debug") then
    -- set_optimize("fast")
    set_optimize("fastest")
    add_cxflags("-march=native")
    add_ldflags("-march=native")
    if false then
      add_cxflags("-fsanitize=address")
      add_ldflags("-fsanitize=address")
    end
    -- add_cxflags("-fstack-protector-all", {force = true})
    -- add_ldflags("-fstack-protector-all", {force = true})
  else
    set_optimize("fastest")
    add_cxflags("-march=native")
    add_ldflags("-march=native")
    add_defines("NDEBUG")
  end
end
