# =============================================================================
# Root Makefile — delegates to code/Makefile
# Run `make` or `make clean` directly from the project root.
# =============================================================================

.PHONY: all clean flash flash-openocd flash-stlink help

all clean flash flash-openocd flash-stlink help:
	"$(MAKE)" -C code $@
