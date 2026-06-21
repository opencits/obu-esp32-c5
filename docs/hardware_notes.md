
## Resources
 - Waveshare https://docs.waveshare.com/ESP32-C5-WIFI6-KIT/Resources-And-Documents
 - ESP32-C5: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/get-started/index.html
 - Opentrafficmap: https://codeberg.org/opentrafficmap
 

## USB UART
```log
Jun 20 10:32:40 thritter-tux kernel: usb 7-1.3: new full-speed USB device number 8 using xhci_hcd
Jun 20 10:32:41 thritter-tux kernel: usb 7-1.3: New USB device found, idVendor=1a86, idProduct=55d3, bcdDevice= 4.45
Jun 20 10:32:41 thritter-tux kernel: usb 7-1.3: New USB device strings: Mfr=0, Product=2, SerialNumber=3
Jun 20 10:32:41 thritter-tux kernel: usb 7-1.3: Product: USB Single Serial
Jun 20 10:32:41 thritter-tux kernel: usb 7-1.3: SerialNumber: 5B7B033092
Jun 20 10:32:41 thritter-tux kernel: cdc_acm 7-1.3:1.0: ttyACM0: USB ACM device
Jun 20 10:32:41 thritter-tux mtp-probe[61704]: checking bus 7, device 8: "/sys/devices/pci0000:00/0000:00:08.3/0000:68:00.4/usb7/7-1/7-1.3"
Jun 20 10:32:41 thritter-tux mtp-probe[61704]: bus: 7, device: 8 was not an MTP device
Jun 20 10:32:41 thritter-tux boltd[2392]: probing: started [1000]
Jun 20 10:32:41 thritter-tux snapd[2203]: hotplug.go:206: hotplug device add event ignored, enable experimental.hotplug
Jun 20 10:32:41 thritter-tux mtp-probe[61718]: checking bus 7, device 8: "/sys/devices/pci0000:00/0000:00:08.3/0000:68:00.4/usb7/7-1/7-1.3"
Jun 20 10:32:41 thritter-tux mtp-probe[61718]: bus: 7, device: 8 was not an MTP device
```



## USB JTAG
```log
Jun 20 10:30:27 thritter-tux kernel: usb 7-1.3: new full-speed USB device number 7 using xhci_hcd
Jun 20 10:30:27 thritter-tux kernel: usb 7-1.3: New USB device found, idVendor=303a, idProduct=1001, bcdDevice= 1.02
Jun 20 10:30:27 thritter-tux kernel: usb 7-1.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
Jun 20 10:30:27 thritter-tux kernel: usb 7-1.3: Product: USB JTAG/serial debug unit
Jun 20 10:30:27 thritter-tux kernel: usb 7-1.3: Manufacturer: Espressif
Jun 20 10:30:27 thritter-tux kernel: usb 7-1.3: SerialNumber: D0:CF:13:E2:77:90
Jun 20 10:30:27 thritter-tux mtp-probe[60857]: checking bus 7, device 7: "/sys/devices/pci0000:00/0000:00:08.3/0000:68:00.4/usb7/7-1/7-1.3"
Jun 20 10:30:27 thritter-tux mtp-probe[60857]: bus: 7, device: 7 was not an MTP device
Jun 20 10:30:27 thritter-tux boltd[2392]: probing: started [1000]
Jun 20 10:30:27 thritter-tux kernel: cdc_acm 7-1.3:1.0: ttyACM0: USB ACM device
Jun 20 10:30:27 thritter-tux kernel: usbcore: registered new interface driver cdc_acm
Jun 20 10:30:27 thritter-tux kernel: cdc_acm: USB Abstract Control Model driver for USB modems and ISDN adapters
Jun 20 10:30:27 thritter-tux mtp-probe[60877]: checking bus 7, device 7: "/sys/devices/pci0000:00/0000:00:08.3/0000:68:00.4/usb7/7-1/7-1.3"
Jun 20 10:30:27 thritter-tux mtp-probe[60877]: bus: 7, device: 7 was not an MTP device
Jun 20 10:30:27 thritter-tux snapd[2203]: hotplug.go:206: hotplug device add event ignored, enable experimental.hotplug
```
