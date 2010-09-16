all:
	@echo "invoking opensource mm-video make"
	$(MAKE) -C vidc/vdec
	$(MAKE) -C vidc/venc

install:
	$(MAKE) -C vidc/vdec install
	$(MAKE) -C vidc/venc install
