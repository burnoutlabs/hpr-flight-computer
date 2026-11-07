set shell := ["bash", "-euo", "pipefail", "-c"]

# PlatformIO projects, one per firmware image.
projects := "firmware/flightcomp firmware/gndstation"

# Override with PIO=/path/to/pio when pio is not on PATH.
pio := env("PIO", "pio")

# .clang-format is written for clang-format 19; CI installs that major and sets
# CLANG_FORMAT=clang-format-19. Another major can format the same file differently.
clang_format := env("CLANG_FORMAT", "clang-format")

[doc("List the recipes")]
default:
    @just --list

[doc("Build every env of both firmware projects")]
build:
    for p in {{ projects }}; do {{ pio }} run -d "$p"; done

[doc("CI equivalent: check formatting, build every env and run the host tests; changes no tracked file")]
check: fmt-check build test

[doc("Format the firmware sources in place with clang-format")]
fmt:
    for f in firmware/*/src/*.cpp firmware/*/include/*.h firmware/common/lib/*/src/*; do {{ clang_format }} -i --assume-filename=x.cpp "$f"; done

[doc("Report firmware sources that clang-format would change, without changing them")]
fmt-check:
    rc=0; for f in firmware/*/src/*.cpp firmware/*/include/*.h firmware/common/lib/*/src/*; do {{ clang_format }} --dry-run --Werror --assume-filename=x.cpp "$f" || rc=1; done; exit $rc

[doc("Run the host unit tests of both projects (PlatformIO native env)")]
test:
    for p in {{ projects }}; do {{ pio }} test -e native -d "$p"; done
