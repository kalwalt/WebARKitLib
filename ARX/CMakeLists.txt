cmake_minimum_required( VERSION 3.4 )

set(ARX_VERSION_MAJOR 1)
set(ARX_VERSION_MINOR 0)
set(ARX_VERSION_TINY  5)
set(ARX_VERSION_DEV   1)
set(ARX_VERSION_STRING "${ARX_VERSION_MAJOR}.${ARX_VERSION_MINOR}.${ARX_VERSION_TINY}")
math(EXPR ARX_VERSION_NUMBER "${ARX_VERSION_MAJOR}*1000000+${ARX_VERSION_MINOR}*10000+${ARX_VERSION_TINY}*100+${ARX_VERSION_DEV}")
set(HAVE_INTEL_SIMD 0)
# Create ARX/AR/config.h and allow subdirectories to find it.
configure_file(AR/include/ARX/AR/config.h.in AR/include/ARX/AR/config.h @ONLY)