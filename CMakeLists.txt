# =======================================================
# CMakeLists.txt
# Location: ~/ns-3.45/src/hybrid-aqm/CMakeLists.txt
# =======================================================

build_lib(
  LIBNAME hybrid-aqm
  SOURCE_FILES
    model/mpc-queue-disc.cc
  HEADER_FILES
    model/mpc-queue-disc.h
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libtraffic-control}
)
