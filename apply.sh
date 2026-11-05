#!/bin/sh
# Add the PICO 4 driver to a Monado source tree (tested on v25.1.0).
#   ./apply.sh /path/to/monado
set -eu
here=$(cd "$(dirname "$0")" && pwd)
m=${1:?usage: apply.sh <monado source dir>}

mkdir -p "$m/src/xrt/drivers/pico4"
cp "$here"/drivers/pico4/* "$m/src/xrt/drivers/pico4/"

# Patches to Monado itself (patches/*.patch, -p1 against the Monado tree),
# each applied once.
for p in "$here"/patches/*.patch; do
	[ -e "$p" ] || continue
	if (cd "$m" && patch -R -p1 --dry-run -s <"$p") >/dev/null 2>&1; then
		continue # already applied
	fi
	(cd "$m" && patch -p1 -s <"$p") || { echo "apply.sh: $p does not apply" >&2; exit 1; }
	echo "applied $(basename "$p")"
done

if grep -q XRT_BUILD_DRIVER_PICO4 "$m/CMakeLists.txt"; then
	# Already added: only bring the source list up to date.
	grep -q pico4_ctrl_eskf.c "$m/src/xrt/drivers/CMakeLists.txt" ||
		sed -i 's|pico4/pico4.c pico4/pico4_interface.h|pico4/pico4.c pico4/pico4_ctrl_eskf.c pico4/pico4_interface.h|' \
			"$m/src/xrt/drivers/CMakeLists.txt"
	grep -q pico4_head.c "$m/src/xrt/drivers/CMakeLists.txt" ||
		sed -i 's|pico4/pico4_ctrl_eskf.c pico4/pico4_interface.h|pico4/pico4_ctrl_eskf.c pico4/pico4_head.c pico4/pico4_interface.h|' \
			"$m/src/xrt/drivers/CMakeLists.txt"
	exit 0
fi

# Option + driver list (feeds xrt_config_drivers.h).
sed -i 's|^option(XRT_BUILD_DRIVER_SIMULATED "Enable simulated driver" ON)|&\noption_with_deps(XRT_BUILD_DRIVER_PICO4 "Enable PICO 4 (mainline Linux) driver" DEPENDS XRT_HAVE_LINUX)|' \
	"$m/CMakeLists.txt"
# Insert after the first match only; busybox sed has no 0,/re/ form.
insert_after_first() { # file regex text
	awk -v re="$2" -v txt="$3" '{ print } !done && $0 ~ re { print txt; done = 1 }' "$1" >"$1.tmp"
	mv "$1.tmp" "$1"
}
insert_before_first() { # file regex text
	awk -v re="$2" -v txt="$3" '!done && $0 ~ re { print txt; done = 1 } { print }' "$1" >"$1.tmp"
	mv "$1.tmp" "$1"
}
insert_after_first "$m/CMakeLists.txt" '^\t"SIMULATED"$' '	"PICO4"'

insert_before_first "$m/src/xrt/drivers/CMakeLists.txt" '^if[(]ENABLED_HEADSET_DRIVERS[)]' 'if(XRT_BUILD_DRIVER_PICO4)
	add_library(drv_pico4 STATIC pico4/pico4.c pico4/pico4_ctrl_eskf.c pico4/pico4_head.c pico4/pico4_interface.h)
	target_link_libraries(drv_pico4 PRIVATE xrt-interfaces aux_util aux_math aux_os)
	list(APPEND ENABLED_HEADSET_DRIVERS pico4)
endif()
'

cat >> "$m/src/xrt/targets/common/CMakeLists.txt" <<'CMAKE'

if(XRT_BUILD_DRIVER_PICO4)
	target_link_libraries(target_lists PRIVATE drv_pico4)
endif()
CMAKE

tl="$m/src/xrt/targets/common/target_lists.c"
# Include next to the first (header) RIFT_S block, builder before the simulated one.
insert_before_first "$tl" '^#ifdef XRT_BUILD_DRIVER_RIFT_S$' '#ifdef XRT_BUILD_DRIVER_PICO4
#include "pico4/pico4_interface.h"
#endif
'
insert_before_first "$tl" '^#ifdef T_BUILDER_SIMULATED // High up' '#ifdef XRT_BUILD_DRIVER_PICO4
    pico4_builder_create,
#endif // XRT_BUILD_DRIVER_PICO4
'
echo "PICO 4 driver added to $m"
