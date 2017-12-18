# root overlay configfs directory
CONFIGFS_PATH=/sys/kernel/config/device-tree/overlays/
# basename of the dtbo file
OVERLAY_BASE=encoders

# overlay directory in the configfs
OVERLAY_DIR=$(CONFIGFS_PATH)$(OVERLAY_BASE)

build: $(OVERLAY_BASE).dtbo

%.dtbo: %.dts
	@# According to various sources, the unit_address_vs_reg warning is a false
	@# positive for overlays, so disable it.
	dtc -W no-unit_address_vs_reg -@ "$<" -o "$@"

# We load the overlay using the configfs API
# (see Documentation/devicetree/configfs-overlays.txt).
# This is to avoid dependencies (eg. the `dtoverlay` script could be used).
load: build
	@echo "Loading $(OVERLAY_BASE).dtbo into $(OVERLAY_DIR)"
	@mkdir $(OVERLAY_DIR)
	@cat $(OVERLAY_BASE).dtbo > $(OVERLAY_DIR)/dtbo

unload:
	rmdir $(OVERLAY_DIR)

# TODO: Install target, cp dtbo to /boot/overlays
