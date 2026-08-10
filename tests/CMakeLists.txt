cmake_minimum_required(VERSION 3.13)
project(DSpicoHostTests C)

enable_testing()

add_executable(rom_cache_consistency_test
  rom_cache_consistency_test.c
  ../src/romCache.c
)

target_compile_definitions(rom_cache_consistency_test PRIVATE
  ENABLE_PSRAM_CACHE
  ENABLE_UART_LOG
)

target_include_directories(rom_cache_consistency_test PRIVATE
  stubs
  ../src
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rom_cache_consistency_test PRIVATE
    -Wall
    -Wextra
    -Werror
  )
endif()

add_test(NAME rom_cache_consistency COMMAND rom_cache_consistency_test)
