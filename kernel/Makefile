# root overlay configfs directory
CONFIGFS_PATH=/sys/kernel/config/device-tree/overlays/
# basename of the dts/dtbo files
OVERLAY_BASE=kurt-encoders
# installation directory for the dtbo
INSTALL_DIR=/boot/overlays

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

install: build
	@echo "Installing $(OVERLAY_BASE).dtbo to $(INSTALL_DIR) (this needs root privileges)"
	@cp "$(OVERLAY_BASE).dtbo" "$(INSTALL_DIR)/$(OVERLAY_BASE).dtbo"
	@echo "Done. You can load $(OVERLAY_BASE).dtbo on boot by putting this into config.txt:"
	@echo "    dtoverlay=$(OVERLAY_BASE)"

uninstall:
	@echo "Removing $(INSTALL_DIR)/$(OVERLAY_BASE).dtbo"
	@rm "$(INSTALL_DIR)/$(OVERLAY_BASE).dtbo"

clean:
	@rm $(OVERLAY_BASE).dtbo
