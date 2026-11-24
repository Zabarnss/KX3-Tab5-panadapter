idf_component_register(
    SRCS "panadapter_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash usb usb_host_uac usb_host usb_host_uac2 usb_host_ch34x_vcp usb_host_cdc_acm usb_host_ftdi_vcp m5stack-tab5 display lvgl logger task format
)
